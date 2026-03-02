#include "sws_analog_service.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include "pmu.hpp"
#include "rgb_led.hpp"
#include "nrfx_saadc.h"

extern RGBLed *status_led;

#ifndef CPPUTEST
#include "crc16.h"
#else
#define crc16_compute(x, y, z) 0xFFFF
#endif

// ADC constants
#define ADC_REFERENCE_V 0.6f           // 0.6V internal reference
#define ADC_GAIN_1_4 (1.0f/4.0f)       // Gain 1/4 for SWS channel

// ADC validation limits for 14-bit ADC (0-16383)
#define ADC_INVALID_MIN 0              // Air reads ~0 on LinkIt V4 hardware (open circuit)
#define ADC_INVALID_MAX 16383          // Maximum valid ADC value (14-bit)

// Default values if configuration is invalid (work for both resolutions)
#define DEFAULT_THRESHOLD_MIN 0
#define DEFAULT_THRESHOLD_MAX 8000
#define DEFAULT_HYSTERESIS_PERCENT 10     // Reduced: faster transition, rapid detection handles noise
#define DEFAULT_CALIB_INTERVAL_SEC 3600
#define DEFAULT_MAX_DIVE_TIME_SEC 7200
#define DEFAULT_MIN_SURFACE_TIME_SEC 10

// Optimized parameters for FAST SURFACE DETECTION (priority: surface over underwater)
#define DEFAULT_THRESHOLD_RATIO_PERCENT 32  // Balanced: fast surface via tiers, stable underwater via threshold
#define DEFAULT_ALPHA_PERCENT 19            // 0.19 - fast EMA adaptation
#define DEFAULT_MAX_SAMPLES 1               // Immediate dive detection
#define DEFAULT_MIN_DRY_SAMPLES 2           // Ultra-fast surface confirmation (2 samples = 2s)

// Biofouling detection thresholds
#define HYSTERESIS_STUCK_TIMEOUT_SEC 30     // Recalibrate if stuck in zone for 30s
#define SURFACE_ADAPT_THRESHOLD 1.2f        // Trigger air recalib if readings 20% above baseline (faster biofouling adapt)
#define MIN_SURFACE_TIME_FOR_ADAPT 8        // Minimum surface time before adapting air baseline
#define EXTENDED_DIVE_RECALIB_START_SEC 30  // Start checking for biofouling after 30s underwater (was 60)
#define EXTENDED_DIVE_RECALIB_INTERVAL_SEC 15  // Check every 15s during extended dive (was 30)

// Water baseline protection - prevents biofouling surface readings from corrupting calibration
#define ABSOLUTE_MIN_WATER_ADC 2000         // True seawater on 14-bit ADC is typically > 2000
#define MIN_WATER_AIR_RATIO 5               // Water reading must be at least 5x air baseline

// Trend-based surface detection - detects surface by ADC decreasing trend (drying)
// OPTIMIZED FOR FAST SURFACE DETECTION
#define TREND_DECREASE_THRESHOLD_PERCENT 1  // 1% decrease threshold (was 2%) - catches slow drying better
#define TREND_DECREASE_ABSOLUTE_MIN 20      // 20 ADC counts min (raised from 8 to reject noise, lowered % compensates)
#define TREND_CONSECUTIVE_DECREASE_MIN 2    // 2 consecutive decreases for faster trend detection
#define TREND_TOTAL_DROP_PERCENT 8          // 8% total drop in trend window (was 10%)
#define CUMULATIVE_DROP_PERCENT 12          // 12% cumulative drop from peak (was 15%)

// Variance-based surface detection - high variance = drying surface, low variance = stable underwater
#define VARIANCE_HIGH_THRESHOLD 10000       // Variance above this suggests surface (drying)
#define VARIANCE_LOW_THRESHOLD 2000         // Variance below this suggests stable underwater

// Surface lockout - prevent immediate re-submersion after max dive time
#define SURFACE_LOCKOUT_DURATION_SEC 30     // 30s lockout after max dive time forces surface

// === RAPID TRANSITION DETECTION thresholds ===
// When a turtle surfaces, the electrode loses water contact instantly, causing
// a large ADC drop. Even with heavy biofouling (salt/biofilm), the RELATIVE
// drop is always significant because the conductivity path through water is broken.
// These thresholds enable <2s surface detection in all conditions.
#define RAPID_DROP_TIER1_PERCENT 18         // Single sample: >18% drop → immediate surface
#define RAPID_DROP_TIER1_ABSOLUTE 200       // AND >200 ADC absolute drop
#define RAPID_DROP_TIER2_PERCENT 7          // Two consecutive: >7% drop → surface (was 9%)
#define RAPID_DROP_TIER2_ABSOLUTE 80        // AND >80 ADC absolute drop (was 100)
#define RAPID_DROP_TIER3_PERCENT 5          // Sustained trend: >5% with trend → surface
#define RAPID_DROP_TIER3_ABSOLUTE 60        // AND >60 ADC absolute drop
// TIER 4: Sliding window slope detection for gradual biofouling drying
// With heavy biofouling, salt/biofilm retains moisture → the ADC drop is gradual
// (3-4% per sample) rather than a sharp cliff. T1-T3 check single-sample drops
// and miss this pattern. T4 accumulates the drop over a window of samples.
#define RAPID_DROP_TIER4_PERCENT 8          // >8% total drop over window → surface (was 10%)
#define RAPID_DROP_TIER4_ABSOLUTE 200       // AND >200 ADC absolute drop over window (raised for safety without midpoint guard)
#define RAPID_DROP_TIER4_WINDOW 4           // Look back 4 samples (4s at 1s gap)
#define BIOFOULING_OVERRIDE_MIN_TIME_SEC 3  // Minimum time underwater before biofouling override (was 15)

// Initialize static members in noinit section
SWSAnalogService::CalibrationData SWSAnalogService::m_calib;
uint16_t SWSAnalogService::m_calib_crc;
SWSAnalogService::Status SWSAnalogService::m_status = {};
SWSAnalogService* SWSAnalogService::s_instance = nullptr;
bool SWSAnalogService::m_test_mode = false;

SWSAnalogService::Status SWSAnalogService::get_status() {
    return m_status;
}

void SWSAnalogService::start_test_mode() {
    m_test_mode = true;
    if (s_instance) {
        DEBUG_INFO("SWSAnalog: Test mode started");
        s_instance->start();
    }
}

void SWSAnalogService::stop_test_mode() {
    if (s_instance) {
        s_instance->stop();
        DEBUG_INFO("SWSAnalog: Test mode stopped");
    }
    if (status_led) {
        status_led->off();
    }
    m_test_mode = false;
}

bool SWSAnalogService::is_test_running() {
    return m_test_mode;
}

// SAADC event handler (required but not used for blocking mode)
static void nrfx_saadc_event_handler_sws(nrfx_saadc_evt_t const *p_event)
{
    (void)p_event;
}

void SWSAnalogService::service_init() {
    // Call parent initialization
    UWDetectorService::service_init();

    // Load configuration parameters
    m_threshold_min = service_read_param<unsigned int>(ParamID::SWS_ANALOG_THRESHOLD_MIN);
    m_threshold_max = service_read_param<unsigned int>(ParamID::SWS_ANALOG_THRESHOLD_MAX);
    m_hysteresis_percent = service_read_param<unsigned int>(ParamID::SWS_ANALOG_HYSTERESIS);
    m_calib_interval_sec = service_read_param<unsigned int>(ParamID::SWS_ANALOG_CALIB_INTERVAL);
    m_max_dive_time_sec = service_read_param<unsigned int>(ParamID::UW_MAX_DIVE_TIME);
    m_min_surface_time_sec = service_read_param<unsigned int>(ParamID::UW_MIN_SURFACE_TIME);

    // Validate configuration parameters
    if (m_threshold_min >= m_threshold_max) {
        DEBUG_WARN("SWSAnalog: Invalid threshold_min | using default");
        m_threshold_min = DEFAULT_THRESHOLD_MIN;
    }
    if (m_threshold_max > ADC_INVALID_MAX || m_threshold_max <= m_threshold_min) {
        DEBUG_WARN("SWSAnalog: Invalid threshold_max | using default");
        m_threshold_max = DEFAULT_THRESHOLD_MAX;
    }
    if (m_hysteresis_percent > 50) {
        DEBUG_WARN("SWSAnalog: Invalid hysteresis | using default");
        m_hysteresis_percent = DEFAULT_HYSTERESIS_PERCENT;
    }

    // Load new optimized parameters (use defaults if not configured)
    m_threshold_ratio_percent = DEFAULT_THRESHOLD_RATIO_PERCENT;
    m_alpha_percent = DEFAULT_ALPHA_PERCENT;
    m_max_samples = DEFAULT_MAX_SAMPLES;
    m_min_dry_samples = DEFAULT_MIN_DRY_SAMPLES;

    // Initialize or validate calibration data
    if (!validate_calibration_data()) {
        // CRITICAL: Zero out calibration struct to prevent garbage noinit values
        // (e.g., threshold_water=8000 from random RAM) from surviving into calibrate_air_baseline()
        memset(&m_calib, 0, sizeof(m_calib));
        DEBUG_INFO("SWSAnalog: Performing initial air calibration");
        calibrate_air_baseline();
    } else {
        DEBUG_INFO("SWSAnalog: Calibration data valid - air=%u water=%u thresh=%u",
                   m_calib.threshold_air, m_calib.threshold_water, m_calib.threshold_current);
    }

    // Initialize timing and state tracking
    m_last_state_change_time = 0;
    m_time_in_current_state = 0;
    m_consecutive_samples = 0;
    m_min_adc_during_dive = 0xFFFF;
    m_hysteresis_start_time = 0;

    // Initialize surface readings buffer
    m_surface_readings_idx = 0;
    m_surface_readings_count = 0;
    for (int i = 0; i < SURFACE_BUFFER_SIZE; i++) {
        m_surface_readings[i] = 0;
    }

    // Initialize trend detection buffer
    m_trend_buffer_idx = 0;
    m_trend_buffer_count = 0;
    m_decreasing_trend_count = 0;
    for (int i = 0; i < TREND_BUFFER_SIZE; i++) {
        m_trend_buffer[i] = 0;
    }

    // Initialize cumulative drop tracking
    m_peak_adc_since_underwater = 0;
    m_cumulative_drop_percent = 0;

    // Initialize surface lockout
    m_surface_lockout_remaining = 0;

    // Initialize variance tracking
    m_variance_sum_sq = 0;
    m_variance_mean = 0;
    m_rapid_surface_confirmed = false;

    DEBUG_INFO("SWSAnalog: Initialized - min=%u max=%u hyst=%u%% ratio=%u%% alpha=%u%%",
               m_threshold_min, m_threshold_max, m_hysteresis_percent,
               m_threshold_ratio_percent, m_alpha_percent);
}

bool SWSAnalogService::service_is_enabled() {
    if (m_test_mode) return true;
    bool enabled = service_read_param<bool>(ParamID::UNDERWATER_EN);
    BaseUnderwaterDetectSource src = service_read_param<BaseUnderwaterDetectSource>(ParamID::UNDERWATER_DETECT_SOURCE);
    return enabled && (src == BaseUnderwaterDetectSource::SWS || src == BaseUnderwaterDetectSource::SWS_GNSS);
}

uint16_t SWSAnalogService::read_analog_sws() {
#ifdef SWS_ADC
    nrf_saadc_value_t raw = 0;

    // Enable SWS sender pin (apply voltage to electrode)
    GPIOPins::set(SWS_ENABLE_PIN);

    // Wait for signal stabilization (capacitance settling)
    PMU::delay_ms(m_enable_sample_delay);

    // Initialize SAADC peripheral
    nrfx_saadc_init(&BSP::ADC_Inits.config, nrfx_saadc_event_handler_sws);
    nrfx_saadc_channel_init(SWS_ADC, &BSP::ADC_Inits.channel_config[SWS_ADC]);

    // Perform ADC conversion
    nrfx_err_t err = nrfx_saadc_sample_convert(SWS_ADC, &raw);
    if (err != NRFX_SUCCESS) {
        DEBUG_ERROR("SWSAnalog: ADC conversion failed with error %d", err);
        raw = 0;
    }

    // Uninitialize SAADC to save power
    nrfx_saadc_uninit();

    // Disable SWS sender pin
    GPIOPins::clear(SWS_ENABLE_PIN);

    DEBUG_TRACE("SWSAnalog: Raw ADC value = %d", raw);

    return (uint16_t)(raw < 0 ? 0 : raw);
#else
    // No SWS ADC channel on this board — always report dry
    return 0;
#endif
}

bool SWSAnalogService::validate_calibration_data() {
    // Calculate CRC of calibration data
    uint16_t calculated_crc = crc16_compute((const uint8_t *)&m_calib,
                                             sizeof(m_calib) - sizeof(m_calib.crc),
                                             nullptr);

    // Check if CRC matches and data is reasonable
    if (calculated_crc != m_calib.crc) {
        DEBUG_WARN("SWSAnalog: Calibration CRC mismatch (calc=%u stored=%u)", calculated_crc, m_calib.crc);
        return false;
    }

    // Validate calibration values are within expected ranges
    if (m_calib.threshold_air < m_threshold_min || m_calib.threshold_air > m_threshold_max) {
        DEBUG_WARN("SWSAnalog: Invalid air threshold %u", m_calib.threshold_air);
        return false;
    }

    if (m_calib.threshold_water < m_threshold_min || m_calib.threshold_water > (m_threshold_max * 2)) {
        DEBUG_WARN("SWSAnalog: Invalid water threshold %u", m_calib.threshold_water);
        return false;
    }

    // Air should be less than water (lower conductivity = lower ADC reading)
    if (m_calib.threshold_air >= m_calib.threshold_water) {
        DEBUG_WARN("SWSAnalog: Air threshold >= water threshold");
        return false;
    }

    return m_calib.is_calibrated;
}

void SWSAnalogService::calibrate_air_baseline() {
    const int NUM_SAMPLES = 10;
    uint32_t sum = 0;
    int valid_count = 0;

    DEBUG_INFO("SWSAnalog: Starting air baseline calibration (%d samples)", NUM_SAMPLES);

    // Take multiple samples and average
    for (int i = 0; i < NUM_SAMPLES; i++) {
        uint16_t value = read_analog_sws();
        if (is_value_valid(value)) {
            sum += value;
            valid_count++;
        } else {
            DEBUG_WARN("SWSAnalog: Invalid sample %u during air calibration", value);
        }
        PMU::delay_ms(100);  // Small delay between samples
    }

    if (valid_count == 0) {
        DEBUG_ERROR("SWSAnalog: No valid samples during air calibration | keeping previous baseline");
        return;
    }

    m_calib.threshold_air = (uint16_t)(sum / valid_count);

    // If water threshold not yet set, use a reasonable default
    if (m_calib.threshold_water == 0 || m_calib.threshold_water <= m_calib.threshold_air) {
        // Assume water will be ~3x air conductivity as initial estimate
        m_calib.threshold_water = m_calib.threshold_air * 3;
        if (m_calib.threshold_water > m_threshold_max) {
            m_calib.threshold_water = m_threshold_max;
        }
    }

    // Update dynamic threshold
    update_dynamic_threshold();

    m_calib.is_calibrated = true;
    m_calib.last_calibration_time = PMU::get_timestamp_ms() / 1000;

    // Calculate and store CRC
    m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                 sizeof(m_calib) - sizeof(m_calib.crc),
                                 nullptr);

    DEBUG_INFO("SWSAnalog: Air calibration complete - air=%u water=%u thresh=%u",
               m_calib.threshold_air, m_calib.threshold_water, m_calib.threshold_current);
}

void SWSAnalogService::calibrate_water_baseline(uint16_t value) {
    // Use exponential moving average to update water baseline
    // This allows gradual adaptation to changing salinity
    // Optimized: configurable alpha (default 0.13 for faster adaptation)
    float alpha = m_alpha_percent / 100.0f;

    if (!is_value_valid(value)) {
        return;
    }

    // STRICT CONDITIONS to prevent corruption from biofouling surface readings:
    // Problem: With biofouling, surface readings are elevated (e.g., 800-1500 instead of 200)
    // If we update water baseline with these values, threshold drops and we can't detect surface
    //
    // Solution: Only update water baseline if ALL conditions are met:
    // 1. Value is clearly above current threshold (truly underwater, not in hysteresis zone)
    // 2. Value is significantly above air baseline (at least 5x - true saltwater is 10x+)
    // 3. Value meets ABSOLUTE minimum for seawater (typical seawater ADC > 2000 on 14-bit)
    // 4. AND value is within expected water range OR higher (salinity increase is OK)

    uint16_t threshold_with_margin = m_calib.threshold_current + m_calib.hysteresis_value;
    uint16_t min_expected_water = (uint16_t)(m_calib.threshold_water * 0.95f);  // Allow 5% drop max (tighter to prevent drying curve corruption)

    // CRITICAL: These absolute thresholds prevent biofouling corruption
    uint16_t absolute_min_water = ABSOLUTE_MIN_WATER_ADC;
    uint16_t min_above_air = (uint16_t)(m_calib.threshold_air * MIN_WATER_AIR_RATIO);

    bool clearly_underwater = value > threshold_with_margin;
    bool above_absolute_min = value >= absolute_min_water;
    bool significantly_above_air = value >= min_above_air;
    bool within_expected_range = value >= min_expected_water;
    bool salinity_increase = value > m_calib.threshold_water;
    // Guard: do NOT update water baseline when ADC is trending down (drying curve)
    // During electrode drying, values like 3920 still pass all checks but pull water baseline down
    bool not_in_drying_trend = (m_decreasing_trend_count < 2);

    // Detect if water baseline needs fast convergence:
    // Case 1: Initial estimate (< ABSOLUTE_MIN_WATER_ADC, e.g. air*3=12)
    // Case 2: Stale/wrong baseline — reading is >50% above current water (e.g. water=2109, reading=4350)
    bool water_at_initial_estimate = (m_calib.threshold_water < absolute_min_water);
    bool water_far_below_reading = (value > (uint16_t)(m_calib.threshold_water * 1.5f));

    // ALL conditions must be met to update water baseline
    if ((water_at_initial_estimate || water_far_below_reading) &&
        above_absolute_min && significantly_above_air && clearly_underwater) {
        // Fast bootstrap: accept reading directly (or with high alpha for large jumps)
        if (water_at_initial_estimate) {
            // First real reading ever — accept directly
            DEBUG_INFO("SWSAnalog: Bootstrap water baseline %u -> %u (first real reading)",
                       m_calib.threshold_water, value);
            m_calib.threshold_water = value;
        } else {
            // Stale baseline — use high alpha (0.5) for fast convergence
            uint16_t new_water = (uint16_t)(0.5f * value + 0.5f * m_calib.threshold_water);
            DEBUG_INFO("SWSAnalog: Fast water convergence %u -> %u (value=%u, alpha=0.5)",
                       m_calib.threshold_water, new_water, value);
            m_calib.threshold_water = new_water;
        }
        update_dynamic_threshold();

        m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                     sizeof(m_calib) - sizeof(m_calib.crc),
                                     nullptr);
    } else if (clearly_underwater && above_absolute_min && significantly_above_air &&
        not_in_drying_trend && (within_expected_range || salinity_increase)) {

        uint16_t new_water = (uint16_t)(alpha * value + (1.0f - alpha) * m_calib.threshold_water);

        DEBUG_TRACE("SWSAnalog: Updating water baseline %u -> %u (alpha=%.2f)",
                    m_calib.threshold_water, new_water, (double)alpha);

        m_calib.threshold_water = new_water;
        update_dynamic_threshold();

        // Update CRC
        m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                     sizeof(m_calib) - sizeof(m_calib.crc),
                                     nullptr);
    } else {
        DEBUG_TRACE("SWSAnalog: Skipping water update (val=%u absMin=%u airX5=%u)",
                    value, absolute_min_water, min_above_air);
    }
}

void SWSAnalogService::update_dynamic_threshold() {
    // Calculate threshold using configurable ratio
    // Lower ratio = threshold closer to air = faster surface detection
    // Optimized: 37% instead of 40% for faster surface detection with biofouling
    float ratio = m_threshold_ratio_percent / 100.0f;
    uint16_t range = m_calib.threshold_water - m_calib.threshold_air;

    m_calib.threshold_current = m_calib.threshold_air + (uint16_t)(range * ratio);

    // Calculate hysteresis value
    m_calib.hysteresis_value = (uint16_t)(m_calib.threshold_current * m_hysteresis_percent / 100.0f);

    // Ensure minimum hysteresis of 10 ADC counts
    if (m_calib.hysteresis_value < 10) {
        m_calib.hysteresis_value = 10;
    }

    DEBUG_TRACE("SWSAnalog: Updated threshold=%u (ratio=%u%%) hysteresis=%u",
                m_calib.threshold_current, m_threshold_ratio_percent, m_calib.hysteresis_value);
}

uint16_t SWSAnalogService::add_to_history_and_filter(uint16_t value) {
    // Add to circular buffer
    m_adc_history[m_adc_history_idx] = value;
    m_adc_history_idx = (m_adc_history_idx + 1) % ADC_HISTORY_SIZE;

    // Calculate moving average
    uint32_t sum = 0;
    int count = 0;
    for (int i = 0; i < ADC_HISTORY_SIZE; i++) {
        if (m_adc_history[i] != 0) {  // Skip uninitialized values
            sum += m_adc_history[i];
            count++;
        }
    }

    return count > 0 ? (uint16_t)(sum / count) : value;
}

bool SWSAnalogService::is_value_valid(uint16_t value) const {
    // Check if value is within valid ADC range (not saturated)
    return (value <= ADC_INVALID_MAX);
}

bool SWSAnalogService::should_recalibrate() const {
    if (m_calib_interval_sec == 0) {
        return false;  // Auto-recalibration disabled
    }

    uint64_t current_time_sec = PMU::get_timestamp_ms() / 1000;
    uint64_t elapsed = current_time_sec - m_calib.last_calibration_time;

    return (elapsed >= m_calib_interval_sec);
}

bool SWSAnalogService::check_safety_timeouts(bool current_state) {
    uint64_t current_time_sec = PMU::get_timestamp_ms() / 1000;

    // Track time in current state
    if (m_last_state_change_time == 0) {
        m_last_state_change_time = current_time_sec;
        m_time_in_current_state = 0;
    } else {
        m_time_in_current_state = current_time_sec - m_last_state_change_time;
    }

    // Check max dive time (force surface if underwater too long)
    if (current_state && m_max_dive_time_sec > 0) {
        if (m_time_in_current_state >= m_max_dive_time_sec) {
            DEBUG_WARN("SWSAnalog: Max dive time exceeded (%us) | forcing surface detection",
                       m_time_in_current_state);
            return true;  // Override to surface
        }
    }

    // Check min surface time (ignore underwater detections if just surfaced)
    if (!current_state && m_min_surface_time_sec > 0) {
        if (m_time_in_current_state < m_min_surface_time_sec) {
            DEBUG_TRACE("SWSAnalog: Min surface time not met (%us < %us) | ignoring transient",
                        m_time_in_current_state, m_min_surface_time_sec);
        }
    }

    return false;  // No override needed
}

bool SWSAnalogService::detector_state() {
    // Detection method tracking for IHM diagnostics
    uint8_t detect_method = SWSAnalogService::DETECT_NONE;
    uint8_t detect_drop_percent = 0;
    uint16_t detect_drop_absolute = 0;

    // Read ADC value
    uint16_t raw_value = read_analog_sws();

    // Validate reading
    if (!is_value_valid(raw_value)) {
        DEBUG_WARN("SWSAnalog: Invalid ADC reading %u | using previous state", raw_value);
        return m_current_state;  // Keep previous state if reading is invalid
    }

    // Apply moving average filter
    uint16_t filtered_value = add_to_history_and_filter(raw_value);

    DEBUG_TRACE("SWSAnalog: raw=%u filtered=%u thresh=%u hyst=%u",
                raw_value, filtered_value, m_calib.threshold_current, m_calib.hysteresis_value);

    // === TREND DETECTION (for biofouling surface detection) ===
    // Track ADC values to detect decreasing trend (= drying = surface)
    uint16_t prev_trend_value = 0;
    if (m_trend_buffer_count > 0) {
        uint8_t prev_idx = (m_trend_buffer_idx + TREND_BUFFER_SIZE - 1) % TREND_BUFFER_SIZE;
        prev_trend_value = m_trend_buffer[prev_idx];
    }

    // Add to trend buffer - use RAW value for sharper drop detection
    // The moving average dampens transitions, making rapid drops appear smaller
    m_trend_buffer[m_trend_buffer_idx] = raw_value;
    m_trend_buffer_idx = (m_trend_buffer_idx + 1) % TREND_BUFFER_SIZE;
    if (m_trend_buffer_count < TREND_BUFFER_SIZE) {
        m_trend_buffer_count++;
    }

    // Track peak ADC while "underwater" (for cumulative drop detection)
    if (m_current_state) {
        if (filtered_value > m_peak_adc_since_underwater) {
            m_peak_adc_since_underwater = filtered_value;
        }
        // Calculate cumulative drop from peak
        if (m_peak_adc_since_underwater > 0) {
            m_cumulative_drop_percent = (uint16_t)(
                (m_peak_adc_since_underwater - filtered_value) * 100 / m_peak_adc_since_underwater
            );
        }
    }

    // Check if value is decreasing (more sensitive: 2% or 8 ADC counts minimum for slow drying)
    // Compare RAW vs RAW (both from trend buffer) for consistent drop detection
    bool is_decreasing = false;
    if (prev_trend_value > 0 && prev_trend_value > raw_value) {
        uint16_t percent_threshold = (uint16_t)(prev_trend_value * TREND_DECREASE_THRESHOLD_PERCENT / 100);
        uint16_t decrease_threshold = (percent_threshold > TREND_DECREASE_ABSOLUTE_MIN) ?
                                       percent_threshold : TREND_DECREASE_ABSOLUTE_MIN;
        is_decreasing = (prev_trend_value > raw_value + decrease_threshold);
    }

    // More tolerant: decrement by 1 instead of reset to 0
    // This allows recovery from single noisy samples
    if (is_decreasing) {
        m_decreasing_trend_count++;
    } else if (m_decreasing_trend_count > 0) {
        m_decreasing_trend_count--;
    }

    // Calculate max and min in trend buffer for total drop calculation
    uint16_t trend_max = 0, trend_min = 0xFFFF;
    for (int i = 0; i < m_trend_buffer_count; i++) {
        if (m_trend_buffer[i] > trend_max) trend_max = m_trend_buffer[i];
        if (m_trend_buffer[i] < trend_min) trend_min = m_trend_buffer[i];
    }

    // Calculate total drop percentage
    uint16_t total_drop_percent = 0;
    if (trend_max > 0 && trend_max > trend_min) {
        total_drop_percent = (uint16_t)((trend_max - trend_min) * 100 / trend_max);
    }

    // === VARIANCE DETECTION ===
    // Calculate variance using running mean and sum of squares
    // High variance = unstable (drying surface), Low variance = stable (underwater)
    if (m_trend_buffer_count >= 4) {
        uint32_t sum = 0;
        for (int i = 0; i < m_trend_buffer_count; i++) {
            sum += m_trend_buffer[i];
        }
        m_variance_mean = (uint16_t)(sum / m_trend_buffer_count);

        m_variance_sum_sq = 0;
        for (int i = 0; i < m_trend_buffer_count; i++) {
            int32_t diff = (int32_t)m_trend_buffer[i] - (int32_t)m_variance_mean;
            m_variance_sum_sq += (uint32_t)(diff * diff);
        }
        m_variance_sum_sq /= m_trend_buffer_count;  // This is the variance
    }

    // Trend-based surface detection flag (consecutive OR cumulative drop)
    bool consecutive_trend = (m_decreasing_trend_count >= TREND_CONSECUTIVE_DECREASE_MIN &&
                              total_drop_percent >= TREND_TOTAL_DROP_PERCENT);
    bool cumulative_trend = (m_cumulative_drop_percent >= CUMULATIVE_DROP_PERCENT &&
                             m_time_in_current_state > 5);  // 5s: gentle biofouling slopes need fast cumul detection
    bool trend_suggests_surface = consecutive_trend || cumulative_trend;

    // Variance-based surface detection flag
    bool variance_suggests_surface = (m_variance_sum_sq >= VARIANCE_HIGH_THRESHOLD);

    DEBUG_TRACE("SWSAnalog: trend_dec=%u drop=%u%% cumul=%u%% var=%u trend_surf=%u var_surf=%u",
                m_decreasing_trend_count, total_drop_percent, m_cumulative_drop_percent,
                m_variance_sum_sq, trend_suggests_surface, variance_suggests_surface);

    // === CONTINUOUS AIR BASELINE TRACKING ===
    // While at surface, accumulate ADC readings into a circular buffer.
    // This replaces the old blocking 10-sample burst recalibration:
    //   - No blocking delays (uses the readings we already take during normal sampling)
    //   - Works regardless of how long the animal stays at surface
    //   - When calib interval expires, recompute air baseline from accumulated readings
    //   - Also detects biofouling (elevated surface readings) for immediate adaptation
    if (!m_current_state && m_time_in_current_state > MIN_SURFACE_TIME_FOR_ADAPT) {
        // Add to surface readings buffer (circular, overwrites oldest)
        m_surface_readings[m_surface_readings_idx] = filtered_value;
        m_surface_readings_idx = (m_surface_readings_idx + 1) % SURFACE_BUFFER_SIZE;
        if (m_surface_readings_count < SURFACE_BUFFER_SIZE) {
            m_surface_readings_count++;
        }

        // Compute average of accumulated surface readings
        if (m_surface_readings_count >= SURFACE_BUFFER_SIZE / 2) {
            uint32_t sum = 0;
            for (int i = 0; i < m_surface_readings_count; i++) {
                sum += m_surface_readings[i];
            }
            uint16_t avg_surface = (uint16_t)(sum / m_surface_readings_count);

            // === PERIODIC RECALIBRATION (non-blocking) ===
            // When calib interval has elapsed, update air baseline from accumulated readings
            if (should_recalibrate()) {
                uint16_t old_air = m_calib.threshold_air;
                m_calib.threshold_air = avg_surface;
                update_dynamic_threshold();

                // Update CRC and reset calibration timer
                m_calib.last_calibration_time = PMU::get_timestamp_ms() / 1000;
                m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                             sizeof(m_calib) - sizeof(m_calib.crc),
                                             nullptr);

                DEBUG_INFO("SWSAnalog: Air recalib from %u surface samples: %u -> %u",
                           m_surface_readings_count, old_air, m_calib.threshold_air);

                // Reset buffer for next interval
                m_surface_readings_count = 0;
                m_surface_readings_idx = 0;
            }
            // === BIOFOULING DETECTION (immediate adaptation) ===
            // If surface readings are significantly elevated, adapt gradually
            else if (avg_surface > (uint16_t)(m_calib.threshold_air * SURFACE_ADAPT_THRESHOLD)) {
                uint16_t old_air = m_calib.threshold_air;
                // Gradual adaptation (10% per update) to avoid overcorrection
                m_calib.threshold_air = (uint16_t)(m_calib.threshold_air * 0.9f + avg_surface * 0.1f);
                update_dynamic_threshold();
                DEBUG_INFO("SWSAnalog: Adaptive air recalib %u -> %u (biofouling)", old_air, m_calib.threshold_air);
            }
        }
    } else if (m_current_state) {
        // Reset surface buffer when underwater
        m_surface_readings_count = 0;
        m_surface_readings_idx = 0;
    }

    // Determine state with hysteresis and sample confirmation
    bool new_state = m_current_state;
    uint16_t threshold_high = m_calib.threshold_current + m_calib.hysteresis_value;
    uint16_t threshold_low = m_calib.threshold_current - m_calib.hysteresis_value;
    // Midpoint of air-water range: guard for sensitive rapid detection tiers (T3/T4)
    // Prevents false surface triggers from normal underwater ADC fluctuations
    uint16_t midpoint_air_water = (m_calib.threshold_air + m_calib.threshold_water) / 2;

    // === RAPID TRANSITION DETECTION (immediate surface on significant ADC drop) ===
    // When a turtle surfaces, the electrode loses water contact instantly.
    // Even with heavy biofouling, the RELATIVE drop is always large because
    // the conductivity path through water is broken.
    // This enables <2s surface detection in ALL conditions including heavy biofouling.
    //
    // Four tiers of detection (optimized for fastest surface detection):
    // TIER 1: Single significant drop (>18% AND >200 abs) → 1 sample = 1s detection
    // TIER 2: Moderate confirmed drop (>9% x2 samples) → 2 samples = 2s detection
    // TIER 3: Small sustained drop with trend (>5% with trend>=2) → backup detection
    // TIER 4: Sliding window (>10% over 4 samples) → 4s detection for gradual biofouling drying
    bool fast_drop_detected = false;
    if (m_current_state && prev_trend_value > 0 && raw_value < prev_trend_value) {
        // Use RAW value (not filtered) for drop calculation.
        // The moving average filter dampens transitions, but we need to detect
        // the instant change when the electrode loses water contact.
        uint16_t drop_percent = (uint16_t)((prev_trend_value - raw_value) * 100 / prev_trend_value);
        uint16_t absolute_drop = prev_trend_value - raw_value;

        // Safety check for TIER 2: raw value should be below water baseline
        // Relaxed to 0.92 (was 0.85) for biofouling: water baseline may drift down
        bool below_water_baseline = (raw_value < (uint16_t)(m_calib.threshold_water * 0.92f));

        // TIER 1: Massive single-sample drop → immediate surface (1s detection)
        // below_water_baseline guard: prevents false trigger from large underwater fluctuations
        if (drop_percent >= RAPID_DROP_TIER1_PERCENT &&
            absolute_drop >= RAPID_DROP_TIER1_ABSOLUTE &&
            below_water_baseline) {
            fast_drop_detected = true;
            detect_method = DETECT_RAPID_T1;
            detect_drop_percent = (uint8_t)drop_percent;
            detect_drop_absolute = absolute_drop;
            DEBUG_INFO("SWSAnalog: RAPID T1! %u%% drop (raw=%u prev=%u) SURFACE",
                       drop_percent, raw_value, prev_trend_value);
        }
        // TIER 2: Moderate drop confirmed over 2 consecutive decreasing samples (2s detection)
        else if (drop_percent >= RAPID_DROP_TIER2_PERCENT &&
                 absolute_drop >= RAPID_DROP_TIER2_ABSOLUTE &&
                 m_decreasing_trend_count >= 1 &&
                 below_water_baseline) {
            fast_drop_detected = true;
            detect_method = DETECT_RAPID_T2;
            detect_drop_percent = (uint8_t)drop_percent;
            detect_drop_absolute = absolute_drop;
            DEBUG_INFO("SWSAnalog: RAPID T2! %u%% x2 (raw=%u prev=%u) trend=%u SURFACE",
                       drop_percent, raw_value, prev_trend_value, m_decreasing_trend_count);
        }
        // TIER 3: Smaller drop with sustained decreasing trend (backup)
        // midpoint guard: only fire when value is in surface half of air-water range
        else if (drop_percent >= RAPID_DROP_TIER3_PERCENT &&
                 absolute_drop >= RAPID_DROP_TIER3_ABSOLUTE &&
                 m_decreasing_trend_count >= 2 &&
                 raw_value < midpoint_air_water) {
            fast_drop_detected = true;
            detect_method = DETECT_RAPID_T3;
            detect_drop_percent = (uint8_t)drop_percent;
            detect_drop_absolute = absolute_drop;
            DEBUG_INFO("SWSAnalog: RAPID T3! %u%% trend=%u (raw=%u prev=%u) SURFACE",
                       drop_percent, m_decreasing_trend_count, raw_value, prev_trend_value);
        }
    }

    // TIER 4: Sliding window slope detection (gradual biofouling drying)
    // With heavy biofouling, salt/biofilm retains moisture → ADC drops gradually
    // (3-4% per sample) instead of sharply. T1-T3 check single-sample drops and
    // miss this gentle slope. T4 accumulates the total drop over a window.
    // Outside the raw < prev check: noisy samples may briefly increase but the
    // overall window trend is still downward.
    if (!fast_drop_detected && m_current_state &&
        m_trend_buffer_count >= (RAPID_DROP_TIER4_WINDOW + 1)) {
        // trend_buffer stores raw values; idx was already incremented after adding current
        uint8_t window_start_idx = (m_trend_buffer_idx + TREND_BUFFER_SIZE
                                     - (RAPID_DROP_TIER4_WINDOW + 1)) % TREND_BUFFER_SIZE;
        uint16_t window_start_value = m_trend_buffer[window_start_idx];
        if (window_start_value > raw_value) {
            uint16_t window_drop_pct = (uint16_t)(
                (window_start_value - raw_value) * 100 / window_start_value);
            uint16_t window_abs_drop = window_start_value - raw_value;
            if (window_drop_pct >= RAPID_DROP_TIER4_PERCENT &&
                window_abs_drop >= RAPID_DROP_TIER4_ABSOLUTE &&
                m_decreasing_trend_count >= 2) {
                fast_drop_detected = true;
                detect_method = DETECT_RAPID_T4;
                detect_drop_percent = (uint8_t)window_drop_pct;
                detect_drop_absolute = window_abs_drop;
                DEBUG_INFO("SWSAnalog: RAPID T4! %u%% over %u samples (raw=%u start=%u trend=%u) SURFACE",
                           window_drop_pct, RAPID_DROP_TIER4_WINDOW, raw_value,
                           window_start_value, m_decreasing_trend_count);
            }
        }
    }

    // === TREND/VARIANCE BASED SURFACE DETECTION (Biofouling override) ===
    // Even if ADC is above threshold, detect surface if:
    // 1. We're currently "underwater" and
    // 2. ADC shows consistent decreasing trend (drying) OR high variance (unstable)
    // 3. OR rapid transition was detected (bypass all time/trend checks)
    bool biofouling_surface_override = false;
    if (m_current_state && (m_time_in_current_state > BIOFOULING_OVERRIDE_MIN_TIME_SEC || fast_drop_detected)) {
        if (fast_drop_detected) {
            // Rapid transition detected: the rate of change IS the evidence
            // No need for absolute threshold checks - the drop itself is unambiguous
            biofouling_surface_override = true;
            DEBUG_INFO("SWSAnalog: RAPID surface override (fast_drop ADC=%u)", filtered_value);

            // CRITICAL: Reset ADC history buffer to the new raw value.
            // Without this, the moving average filter retains old underwater values,
            // producing a high filtered_value on the next call. Since m_current_state
            // is only updated by the parent at end-of-batch, the next detector_state()
            // call still sees m_current_state=true and the high filtered_value causes
            // it to return true (underwater), undoing the rapid detection.
            for (int i = 0; i < ADC_HISTORY_SIZE; i++) {
                m_adc_history[i] = raw_value;
            }

            // Recalibrate air baseline using raw value (represents actual air reading)
            // Guard: new air must stay below threshold_current and below water/2
            if (raw_value > m_calib.threshold_air &&
                raw_value < m_calib.threshold_current &&
                raw_value < (m_calib.threshold_water / 2)) {
                uint16_t old_air = m_calib.threshold_air;
                m_calib.threshold_air = (uint16_t)(raw_value * 0.8f);
                update_dynamic_threshold();
                DEBUG_INFO("SWSAnalog: Air recalib after rapid drop %u -> %u", old_air, m_calib.threshold_air);
            }
        } else if ((trend_suggests_surface || variance_suggests_surface) &&
                   filtered_value < (uint16_t)(m_calib.threshold_water * 0.85f)) {
            // Trend/variance based detection: adaptive check using water baseline
            // instead of fixed ABSOLUTE_MIN_WATER_ADC. With heavy biofouling the
            // water baseline drifts down (e.g., 2200) and surface readings can stay
            // above the old fixed 2000 cutoff, blocking detection. Using 85% of
            // water baseline adapts to actual conditions.
            biofouling_surface_override = true;
            if (detect_method == DETECT_NONE) {
                detect_method = DETECT_TREND;
            }
            DEBUG_INFO("SWSAnalog: Trend/var surface override (trend=%u var=%u ADC=%u water85=%u)",
                       trend_suggests_surface, variance_suggests_surface, filtered_value,
                       (uint16_t)(m_calib.threshold_water * 0.85f));

            // Recalibrate air baseline to current trend minimum
            // Guards: trend_min must be (1) above current air, (2) below current threshold
            // (prevents drying-curve values from pushing air into the water range),
            // and (3) below 50% of water baseline (sanity: air should be well below water)
            if (trend_min > m_calib.threshold_air &&
                trend_min < m_calib.threshold_current &&
                trend_min < (m_calib.threshold_water / 2)) {
                uint16_t old_air = m_calib.threshold_air;
                m_calib.threshold_air = (uint16_t)(trend_min * 0.9f);
                update_dynamic_threshold();
                DEBUG_INFO("SWSAnalog: Trend-based air recalib %u -> %u", old_air, m_calib.threshold_air);
            }
        }
    }

    if (biofouling_surface_override) {
        // Force surface detection based on trend/variance
        new_state = false;
        m_consecutive_samples = 0;
        m_hysteresis_start_time = 0;
        // Flag: rapid detection confirmed - bypass confirmation on subsequent samples
        // within the same parent batch (m_current_state hasn't been updated yet)
        m_rapid_surface_confirmed = true;
    } else if (m_rapid_surface_confirmed && filtered_value < threshold_high) {
        // Rapid detection was confirmed in a previous sample of this batch.
        // m_current_state is still true (parent updates at end-of-batch), but we know
        // we're at surface. Return false immediately without confirmation delay.
        new_state = false;
        m_consecutive_samples = 0;
        m_hysteresis_start_time = 0;
    } else if (filtered_value > threshold_high) {
        // Above threshold - potential underwater
        m_consecutive_samples++;
        m_hysteresis_start_time = 0;  // Left hysteresis zone
        m_rapid_surface_confirmed = false;  // Clear rapid flag if going back underwater

        if (m_consecutive_samples >= m_max_samples) {
            new_state = true;  // UNDERWATER confirmed
            if (detect_method == DETECT_NONE) detect_method = DETECT_THRESHOLD;
            calibrate_water_baseline(filtered_value);
        }
    } else if (filtered_value < threshold_low) {
        // Below threshold - potential surface
        m_hysteresis_start_time = 0;  // Left hysteresis zone

        if (m_current_state) {
            // Currently underwater, need confirmation for surface
            m_consecutive_samples++;
            if (m_consecutive_samples >= m_min_dry_samples) {
                new_state = false;  // SURFACE confirmed
                if (detect_method == DETECT_NONE) detect_method = DETECT_THRESHOLD;
            }
        } else {
            // Already at surface, stay there
            new_state = false;
            m_consecutive_samples = 0;
        }
    } else {
        // === IN HYSTERESIS ZONE ===
        m_consecutive_samples = 0;
        uint64_t now_sec = PMU::get_timestamp_ms() / 1000;

        // Record when we entered the hysteresis zone
        if (m_hysteresis_start_time == 0) {
            m_hysteresis_start_time = now_sec;
        }

        uint32_t time_in_hysteresis_sec = (uint32_t)(now_sec - m_hysteresis_start_time);

        // If stuck in hysteresis zone too long, recalibrate air baseline
        if (time_in_hysteresis_sec > HYSTERESIS_STUCK_TIMEOUT_SEC &&
            (time_in_hysteresis_sec % 15) == 0) {  // Check every 15 seconds
            uint16_t old_air = m_calib.threshold_air;
            // Move air baseline up toward current reading
            uint16_t new_air = (uint16_t)(filtered_value * 0.75f);
            if (new_air > (uint16_t)(old_air * 1.2f)) {
                m_calib.threshold_air = new_air;
                update_dynamic_threshold();
                DEBUG_WARN("SWSAnalog: Hysteresis stuck recalib %u -> %u", old_air, new_air);
            }
        }

        DEBUG_TRACE("SWSAnalog: In hysteresis zone (%us) | maintaining state=%u",
                    time_in_hysteresis_sec, new_state);
    }

    // Track minimum ADC during dive for biofouling detection
    if (m_current_state) {  // Currently underwater
        if (filtered_value < m_min_adc_during_dive) {
            m_min_adc_during_dive = filtered_value;
        }

        // === EXTENDED DIVE RECALIBRATION (Proactive biofouling detection) ===
        // After EXTENDED_DIVE_RECALIB_START_SEC underwater, periodically check if readings
        // suggest biofouling. This catches cases where the device thinks it's underwater
        // but is actually at surface with elevated baseline from biofouling/salt deposits
        if (m_time_in_current_state > EXTENDED_DIVE_RECALIB_START_SEC &&
            (m_time_in_current_state % EXTENDED_DIVE_RECALIB_INTERVAL_SEC) == 0) {
            // If minimum ADC is way above air threshold but below water threshold,
            // this indicates the air baseline has shifted up significantly (biofouling)
            if (m_min_adc_during_dive != 0xFFFF &&
                m_min_adc_during_dive > (uint16_t)(m_calib.threshold_air * 2) &&
                m_min_adc_during_dive < (uint16_t)(m_calib.threshold_water * 0.7f)) {

                uint16_t new_air = (uint16_t)(m_min_adc_during_dive * 0.85f);

                // Only apply if significant shift (>30% increase, was 50%)
                if (new_air > (uint16_t)(m_calib.threshold_air * 1.3f)) {
                    uint16_t old_air = m_calib.threshold_air;
                    m_calib.threshold_air = new_air;
                    update_dynamic_threshold();
                    m_consecutive_samples = 0;  // Force re-evaluation
                    DEBUG_WARN("SWSAnalog: Extended dive recalib %u -> %u (biofouling detected)",
                               old_air, new_air);
                }
            }
        }
    }

    // Decrement surface lockout timer
    if (m_surface_lockout_remaining > 0) {
        m_surface_lockout_remaining--;
    }

    // Check safety timeouts
    bool timeout_override = check_safety_timeouts(new_state);
    if (timeout_override) {
        // === FORCED RECALIBRATION ON MAX DIVE TIMEOUT ===
        // If we hit max dive time, likely biofouling issue - recalibrate air baseline
        if (m_min_adc_during_dive != 0xFFFF &&
            m_min_adc_during_dive > (uint16_t)(m_calib.threshold_air * 2)) {
            uint16_t old_air = m_calib.threshold_air;
            m_calib.threshold_air = (uint16_t)(m_min_adc_during_dive * 0.8f);
            update_dynamic_threshold();
            DEBUG_WARN("SWSAnalog: Max dive recalib %u -> %u (biofouling suspected)", old_air, m_calib.threshold_air);
        }
        new_state = false;  // Force surface state
        detect_method = DETECT_SAFETY;

        // CRITICAL: Set surface lockout to prevent immediate re-submersion
        m_surface_lockout_remaining = SURFACE_LOCKOUT_DURATION_SEC;
        DEBUG_INFO("SWSAnalog: Surface lockout activated for %us", SURFACE_LOCKOUT_DURATION_SEC);
    }

    // Surface lockout: prevent return to underwater during lockout period
    if (m_surface_lockout_remaining > 0 && new_state) {
        DEBUG_TRACE("SWSAnalog: Surface lockout active (%us remaining) | staying at surface",
                    m_surface_lockout_remaining);
        new_state = false;
    }

    // Detect state changes for timing tracking
    if (new_state != m_current_state) {
        uint64_t current_time_sec = PMU::get_timestamp_ms() / 1000;
        m_last_state_change_time = current_time_sec;
        m_time_in_current_state = 0;
        m_consecutive_samples = 0;
        m_min_adc_during_dive = 0xFFFF;  // Reset min ADC tracking
        m_hysteresis_start_time = 0;

        // Reset trend and variance tracking on state change
        m_trend_buffer_count = 0;
        m_trend_buffer_idx = 0;
        m_decreasing_trend_count = 0;
        m_variance_sum_sq = 0;
        m_variance_mean = 0;

        // Reset cumulative drop tracking on state change
        m_peak_adc_since_underwater = 0;
        m_cumulative_drop_percent = 0;

        // Reset rapid surface confirmation flag for clean state
        m_rapid_surface_confirmed = false;

        DEBUG_INFO("SWSAnalog: State change %u -> %u (value=%u | thresh=%u)",
                   m_current_state, new_state, filtered_value, m_calib.threshold_current);

        // LED feedback in test mode only: BLUE=underwater, YELLOW=surface
        if (m_test_mode && status_led) {
            status_led->set(new_state ? RGBLedColor::BLUE : RGBLedColor::YELLOW);
        }
    }

    // NOTE: Periodic air recalibration is now handled non-blocking in the
    // "CONTINUOUS AIR BASELINE TRACKING" section above, using accumulated
    // surface readings from normal sampling. No blocking burst needed here.

    // Update status snapshot for DTE diagnostic readout
    m_status.threshold_air = m_calib.threshold_air;
    m_status.threshold_water = m_calib.threshold_water;
    m_status.threshold_current = m_calib.threshold_current;
    m_status.hysteresis = m_calib.hysteresis_value;
    m_status.last_raw_adc = raw_value;
    m_status.last_filtered_adc = filtered_value;
    m_status.is_calibrated = m_calib.is_calibrated;
    m_status.is_underwater = new_state;
    m_status.time_in_state_sec = (uint32_t)m_time_in_current_state;
    // Detection diagnostics for IHM
    if (detect_method != DETECT_NONE) {
        m_status.last_detect_method = detect_method;
        m_status.last_drop_percent = detect_drop_percent;
        m_status.last_drop_absolute = detect_drop_absolute;
    }
    m_status.trend_count = m_decreasing_trend_count;
    m_status.consecutive_samples = m_consecutive_samples;
    m_status.midpoint = (m_calib.threshold_air + m_calib.threshold_water) / 2;
    m_status.contrast_ratio_x10 = (m_calib.threshold_air > 0) ?
        (uint16_t)(m_calib.threshold_water * 10 / m_calib.threshold_air) : 0;

    return new_state;
}

uint16_t SWSAnalogService::calculate_calibration_crc() const {
    return crc16_compute((const uint8_t *)&m_calib,
                         sizeof(m_calib) - sizeof(m_calib.crc),
                         nullptr);
}
