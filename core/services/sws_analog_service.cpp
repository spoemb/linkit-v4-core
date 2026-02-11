#include "sws_analog_service.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include "pmu.hpp"
#include "nrfx_saadc.h"

#ifndef CPPUTEST
#include "crc16.h"
#else
#define crc16_compute(x, y, z) 0xFFFF
#endif

// ADC constants
#define ADC_REFERENCE_V 0.6f           // 0.6V internal reference
#define ADC_GAIN_1_4 (1.0f/4.0f)       // Gain 1/4 for SWS channel

// ADC validation limits for 14-bit ADC (0-16383)
#define ADC_INVALID_MIN 50             // Minimum valid ADC value
// Note: ADC_INVALID_MAX is checked against configured THRESHOLD_MAX parameter

// Default values if configuration is invalid (work for both resolutions)
#define DEFAULT_THRESHOLD_MIN 100
#define DEFAULT_THRESHOLD_MAX 3000
#define DEFAULT_HYSTERESIS_PERCENT 14     // Optimized from Monte Carlo: balance transitions/stability
#define DEFAULT_CALIB_INTERVAL_SEC 3600
#define DEFAULT_MAX_DIVE_TIME_SEC 7200
#define DEFAULT_MIN_SURFACE_TIME_SEC 10

// Optimized parameters for FAST SURFACE DETECTION (priority: surface over underwater)
#define DEFAULT_THRESHOLD_RATIO_PERCENT 35  // Lower: threshold closer to air = faster surface detect
#define DEFAULT_ALPHA_PERCENT 19            // 0.19 - fast EMA adaptation
#define DEFAULT_MAX_SAMPLES 1               // Immediate dive detection
#define DEFAULT_MIN_DRY_SAMPLES 2           // Ultra-fast surface confirmation (2 samples = 2s)

// Biofouling detection thresholds
#define HYSTERESIS_STUCK_TIMEOUT_SEC 30     // Recalibrate if stuck in zone for 30s
#define SURFACE_ADAPT_THRESHOLD 1.3f        // Trigger air recalib if readings 30% above baseline
#define MIN_SURFACE_TIME_FOR_ADAPT 10       // Minimum surface time before adapting air baseline
#define EXTENDED_DIVE_RECALIB_START_SEC 60  // Start checking for biofouling after 60s underwater
#define EXTENDED_DIVE_RECALIB_INTERVAL_SEC 30  // Check every 30s during extended dive

// Water baseline protection - prevents biofouling surface readings from corrupting calibration
#define ABSOLUTE_MIN_WATER_ADC 2000         // True seawater on 14-bit ADC is typically > 2000
#define MIN_WATER_AIR_RATIO 5               // Water reading must be at least 5x air baseline

// Trend-based surface detection - detects surface by ADC decreasing trend (drying)
// OPTIMIZED FOR FAST SURFACE DETECTION
#define TREND_DECREASE_THRESHOLD_PERCENT 2  // 2% decrease threshold
#define TREND_DECREASE_ABSOLUTE_MIN 8       // Lowered: 8 ADC counts min for faster detection
#define TREND_CONSECUTIVE_DECREASE_MIN 3    // Reduced: 3 consecutive decreases (was 4)
#define TREND_TOTAL_DROP_PERCENT 10         // Reduced: 10% total drop (was 15%)
#define CUMULATIVE_DROP_PERCENT 15          // Reduced: 15% cumulative drop (was 20%)

// Variance-based surface detection - high variance = drying surface, low variance = stable underwater
#define VARIANCE_HIGH_THRESHOLD 10000       // Variance above this suggests surface (drying)
#define VARIANCE_LOW_THRESHOLD 2000         // Variance below this suggests stable underwater

// Surface lockout - prevent immediate re-submersion after max dive time
#define SURFACE_LOCKOUT_DURATION_SEC 30     // 30s lockout after max dive time forces surface

// Initialize static members in noinit section
SWSAnalogService::CalibrationData SWSAnalogService::m_calib;
uint16_t SWSAnalogService::m_calib_crc;

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
    if (m_threshold_min < ADC_INVALID_MIN || m_threshold_min >= m_threshold_max) {
        DEBUG_WARN("SWSAnalog: Invalid threshold_min, using default");
        m_threshold_min = DEFAULT_THRESHOLD_MIN;
    }
    if (m_threshold_max > ADC_INVALID_MAX || m_threshold_max <= m_threshold_min) {
        DEBUG_WARN("SWSAnalog: Invalid threshold_max, using default");
        m_threshold_max = DEFAULT_THRESHOLD_MAX;
    }
    if (m_hysteresis_percent > 50) {
        DEBUG_WARN("SWSAnalog: Invalid hysteresis, using default");
        m_hysteresis_percent = DEFAULT_HYSTERESIS_PERCENT;
    }

    // Load new optimized parameters (use defaults if not configured)
    m_threshold_ratio_percent = DEFAULT_THRESHOLD_RATIO_PERCENT;
    m_alpha_percent = DEFAULT_ALPHA_PERCENT;
    m_max_samples = DEFAULT_MAX_SAMPLES;
    m_min_dry_samples = DEFAULT_MIN_DRY_SAMPLES;

    // Initialize or validate calibration data
    if (!validate_calibration_data()) {
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
    m_time_in_hysteresis = 0;

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

    DEBUG_INFO("SWSAnalog: Initialized - min=%u max=%u hyst=%u%% ratio=%u%% alpha=%u%%",
               m_threshold_min, m_threshold_max, m_hysteresis_percent,
               m_threshold_ratio_percent, m_alpha_percent);
}

bool SWSAnalogService::service_is_enabled() {
    bool enabled = service_read_param<bool>(ParamID::UNDERWATER_EN);
    BaseUnderwaterDetectSource src = service_read_param<BaseUnderwaterDetectSource>(ParamID::UNDERWATER_DETECT_SOURCE);
    // Enable if source is SWS or SWS_GNSS (we'll assume SWS uses analog for now)
    return enabled && (src == BaseUnderwaterDetectSource::SWS || src == BaseUnderwaterDetectSource::SWS_GNSS);
}

uint16_t SWSAnalogService::read_analog_sws() {
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

    DEBUG_INFO("SWSAnalog: Starting air baseline calibration (%d samples)", NUM_SAMPLES);

    // Take multiple samples and average
    for (int i = 0; i < NUM_SAMPLES; i++) {
        uint16_t value = read_analog_sws();
        if (is_value_valid(value)) {
            sum += value;
        } else {
            DEBUG_WARN("SWSAnalog: Invalid sample %u during air calibration", value);
        }
        PMU::delay_ms(100);  // Small delay between samples
    }

    m_calib.threshold_air = (uint16_t)(sum / NUM_SAMPLES);

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
    uint16_t min_expected_water = (uint16_t)(m_calib.threshold_water * 0.85f);  // Allow 15% drop max

    // CRITICAL: These absolute thresholds prevent biofouling corruption
    uint16_t absolute_min_water = ABSOLUTE_MIN_WATER_ADC;
    uint16_t min_above_air = (uint16_t)(m_calib.threshold_air * MIN_WATER_AIR_RATIO);

    bool clearly_underwater = value > threshold_with_margin;
    bool above_absolute_min = value >= absolute_min_water;
    bool significantly_above_air = value >= min_above_air;
    bool within_expected_range = value >= min_expected_water;
    bool salinity_increase = value > m_calib.threshold_water;

    // ALL conditions must be met to update water baseline
    if (clearly_underwater && above_absolute_min && significantly_above_air &&
        (within_expected_range || salinity_increase)) {
        uint16_t new_water = (uint16_t)(alpha * value + (1.0f - alpha) * m_calib.threshold_water);

        DEBUG_TRACE("SWSAnalog: Updating water baseline %u -> %u (alpha=%.2f)",
                    m_calib.threshold_water, new_water, alpha);

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
    // Check if value is within valid range (not saturated or zero)
    // Upper limit is the configured maximum threshold (works for both 12-bit and 14-bit ADC)
    return (value >= ADC_INVALID_MIN && value <= m_threshold_max);
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
            DEBUG_WARN("SWSAnalog: Max dive time exceeded (%us), forcing surface detection",
                       m_time_in_current_state);
            return true;  // Override to surface
        }
    }

    // Check min surface time (ignore underwater detections if just surfaced)
    if (!current_state && m_min_surface_time_sec > 0) {
        if (m_time_in_current_state < m_min_surface_time_sec) {
            DEBUG_TRACE("SWSAnalog: Min surface time not met (%us < %us), ignoring transient",
                        m_time_in_current_state, m_min_surface_time_sec);
            // Don't override, but this info can be used by caller
        }
    }

    return false;  // No override needed
}

bool SWSAnalogService::detector_state() {
    // Read ADC value
    uint16_t raw_value = read_analog_sws();

    // Validate reading
    if (!is_value_valid(raw_value)) {
        DEBUG_WARN("SWSAnalog: Invalid ADC reading %u, using previous state", raw_value);
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

    // Add to trend buffer
    m_trend_buffer[m_trend_buffer_idx] = filtered_value;
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

    // Check if value is decreasing (more sensitive: 2% or 10 ADC counts minimum for slow drying)
    bool is_decreasing = false;
    if (prev_trend_value > 0) {
        // Lowered threshold for better sensitivity with slow biofouling drying
        uint16_t percent_threshold = (uint16_t)(prev_trend_value * TREND_DECREASE_THRESHOLD_PERCENT / 100);
        uint16_t decrease_threshold = (percent_threshold > TREND_DECREASE_ABSOLUTE_MIN) ?
                                       percent_threshold : TREND_DECREASE_ABSOLUTE_MIN;
        is_decreasing = (prev_trend_value > filtered_value + decrease_threshold);
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
                             m_time_in_current_state > 10);  // Reduced: 10s (was 20s) for faster detection
    bool trend_suggests_surface = consecutive_trend || cumulative_trend;

    // Variance-based surface detection flag
    bool variance_suggests_surface = (m_variance_sum_sq >= VARIANCE_HIGH_THRESHOLD);

    DEBUG_TRACE("SWSAnalog: trend_dec=%u drop=%u%% cumul=%u%% var=%u trend_surf=%u var_surf=%u",
                m_decreasing_trend_count, total_drop_percent, m_cumulative_drop_percent,
                m_variance_sum_sq, trend_suggests_surface, variance_suggests_surface);

    // === ADAPTIVE AIR BASELINE RECALIBRATION (Biofouling compensation) ===
    // When at surface for extended time with elevated readings, adapt air baseline
    if (!m_current_state && m_time_in_current_state > MIN_SURFACE_TIME_FOR_ADAPT) {
        // Add to surface readings buffer
        m_surface_readings[m_surface_readings_idx] = filtered_value;
        m_surface_readings_idx = (m_surface_readings_idx + 1) % SURFACE_BUFFER_SIZE;
        if (m_surface_readings_count < SURFACE_BUFFER_SIZE) {
            m_surface_readings_count++;
        }

        // Check if readings are elevated (biofouling detected)
        if (m_surface_readings_count >= SURFACE_BUFFER_SIZE / 2 &&
            m_time_in_current_state % 10 == 0) {  // Check every 10 seconds

            uint32_t sum = 0;
            for (int i = 0; i < m_surface_readings_count; i++) {
                sum += m_surface_readings[i];
            }
            uint16_t avg_surface = (uint16_t)(sum / m_surface_readings_count);

            // If surface readings significantly above air baseline, adapt
            if (avg_surface > (uint16_t)(m_calib.threshold_air * SURFACE_ADAPT_THRESHOLD)) {
                uint16_t old_air = m_calib.threshold_air;
                // Gradual adaptation (10% per update)
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

    // === FAST DROP DETECTION (immediate surface on rapid ADC decrease) ===
    // If ADC drops significantly in a short time, bypass all delays and trigger surface
    // This catches rapid transitions even with biofouling (no 15s wait needed)
    bool fast_drop_detected = false;
    if (m_current_state && m_trend_buffer_count >= 3 && m_time_in_current_state > 3) {
        // Find max ADC in last 3-4 samples
        uint16_t recent_max = 0;
        for (int i = 0; i < m_trend_buffer_count && i < 4; i++) {
            if (m_trend_buffer[i] > recent_max) {
                recent_max = m_trend_buffer[i];
            }
        }
        // Calculate drop percentage from recent max
        if (recent_max > 0 && filtered_value < recent_max) {
            uint16_t fast_drop_percent = (recent_max - filtered_value) * 100 / recent_max;
            // If >8% drop in ~3s AND below true seawater range → immediate surface
            if (fast_drop_percent >= 8 && filtered_value < ABSOLUTE_MIN_WATER_ADC) {
                fast_drop_detected = true;
                DEBUG_INFO("SWSAnalog: FAST DROP detected! %u%% drop (max=%u curr=%u) → SURFACE",
                           fast_drop_percent, recent_max, filtered_value);
            }
        }
    }

    // === TREND/VARIANCE BASED SURFACE DETECTION (Biofouling override) ===
    // Even if ADC is above threshold, detect surface if:
    // 1. We're currently "underwater" and
    // 2. ADC shows consistent decreasing trend (drying) OR high variance (unstable)
    // 3. AND we've been "underwater" for a while (to avoid false triggers)
    bool biofouling_surface_override = false;
    if (m_current_state && (m_time_in_current_state > 15 || fast_drop_detected)) {
        // Check if trend AND/OR variance suggest we're actually at surface
        // OR fast drop was detected (bypass trend/variance check)
        if (trend_suggests_surface || variance_suggests_surface || fast_drop_detected) {
            // Additional check: ADC should be in "suspicious" range (not true seawater)
            // True seawater is typically > 2000 ADC on 14-bit
            if (filtered_value < ABSOLUTE_MIN_WATER_ADC) {
                biofouling_surface_override = true;
                DEBUG_INFO("SWSAnalog: Surface override triggered (trend=%u var=%u fast=%u ADC=%u)",
                           trend_suggests_surface, variance_suggests_surface, fast_drop_detected, filtered_value);

                // Recalibrate air baseline to current trend minimum
                if (trend_min > m_calib.threshold_air && trend_min < ABSOLUTE_MIN_WATER_ADC) {
                    uint16_t old_air = m_calib.threshold_air;
                    m_calib.threshold_air = (uint16_t)(trend_min * 0.9f);  // Set air to 90% of min
                    update_dynamic_threshold();
                    DEBUG_INFO("SWSAnalog: Trend-based air recalib %u -> %u", old_air, m_calib.threshold_air);
                }
            }
        }
    }

    if (biofouling_surface_override) {
        // Force surface detection based on trend/variance
        new_state = false;
        m_consecutive_samples = 0;
        m_time_in_hysteresis = 0;
    } else if (filtered_value > threshold_high) {
        // Above threshold - potential underwater
        m_consecutive_samples++;
        m_time_in_hysteresis = 0;  // Reset hysteresis counter

        if (m_consecutive_samples >= m_max_samples) {
            new_state = true;  // UNDERWATER confirmed
            calibrate_water_baseline(filtered_value);
        }
    } else if (filtered_value < threshold_low) {
        // Below threshold - potential surface
        m_time_in_hysteresis = 0;  // Reset hysteresis counter

        if (m_current_state) {
            // Currently underwater, need confirmation for surface
            m_consecutive_samples++;
            if (m_consecutive_samples >= m_min_dry_samples) {
                new_state = false;  // SURFACE confirmed
            }
        } else {
            // Already at surface, stay there
            new_state = false;
            m_consecutive_samples = 0;
        }
    } else {
        // === IN HYSTERESIS ZONE ===
        m_consecutive_samples = 0;
        m_time_in_hysteresis++;

        // If stuck in hysteresis zone too long, recalibrate air baseline
        if (m_time_in_hysteresis > HYSTERESIS_STUCK_TIMEOUT_SEC &&
            m_time_in_hysteresis % 15 == 0) {  // Check every 15 seconds
            uint16_t old_air = m_calib.threshold_air;
            // Move air baseline up toward current reading
            uint16_t new_air = (uint16_t)(filtered_value * 0.75f);
            if (new_air > (uint16_t)(old_air * 1.2f)) {
                m_calib.threshold_air = new_air;
                update_dynamic_threshold();
                DEBUG_WARN("SWSAnalog: Hysteresis stuck recalib %u -> %u", old_air, new_air);
            }
        }

        DEBUG_TRACE("SWSAnalog: In hysteresis zone (%us), maintaining state=%u",
                    m_time_in_hysteresis, new_state);
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
                m_min_adc_during_dive > (uint16_t)(m_calib.threshold_air * 3) &&
                m_min_adc_during_dive < (uint16_t)(m_calib.threshold_water * 0.7f)) {

                uint16_t new_air = (uint16_t)(m_min_adc_during_dive * 0.85f);

                // Only apply if significant shift (>50% increase)
                if (new_air > (uint16_t)(m_calib.threshold_air * 1.5f)) {
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

        // CRITICAL: Set surface lockout to prevent immediate re-submersion
        m_surface_lockout_remaining = SURFACE_LOCKOUT_DURATION_SEC;
        DEBUG_INFO("SWSAnalog: Surface lockout activated for %us", SURFACE_LOCKOUT_DURATION_SEC);
    }

    // Surface lockout: prevent return to underwater during lockout period
    if (m_surface_lockout_remaining > 0 && new_state) {
        DEBUG_TRACE("SWSAnalog: Surface lockout active (%us remaining), staying at surface",
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
        m_time_in_hysteresis = 0;

        // Reset trend and variance tracking on state change
        m_trend_buffer_count = 0;
        m_trend_buffer_idx = 0;
        m_decreasing_trend_count = 0;
        m_variance_sum_sq = 0;
        m_variance_mean = 0;

        // Reset cumulative drop tracking on state change
        m_peak_adc_since_underwater = 0;
        m_cumulative_drop_percent = 0;

        DEBUG_INFO("SWSAnalog: State change %u -> %u (value=%u, thresh=%u)",
                   m_current_state, new_state, filtered_value, m_calib.threshold_current);
    }

    // Periodic recalibration check (only when at surface)
    if (!new_state && should_recalibrate()) {
        DEBUG_INFO("SWSAnalog: Periodic recalibration triggered");
        calibrate_air_baseline();
    }

    return new_state;
}

uint16_t SWSAnalogService::calculate_calibration_crc() const {
    return crc16_compute((const uint8_t *)&m_calib,
                         sizeof(m_calib) - sizeof(m_calib.crc),
                         nullptr);
}
