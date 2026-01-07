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

// ADC validation limits (conservative values work for both 12-bit and 14-bit)
// Gentracker: 12-bit (0-4095), Horizon: 14-bit (0-16383)
#define ADC_INVALID_MIN 50             // Minimum valid ADC value
// Note: ADC_INVALID_MAX is checked against configured THRESHOLD_MAX parameter

// Default values if configuration is invalid (work for both resolutions)
#define DEFAULT_THRESHOLD_MIN 100
#define DEFAULT_THRESHOLD_MAX 3000
#define DEFAULT_HYSTERESIS_PERCENT 10
#define DEFAULT_CALIB_INTERVAL_SEC 3600
#define DEFAULT_MAX_DIVE_TIME_SEC 7200
#define DEFAULT_MIN_SURFACE_TIME_SEC 10

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

    // Initialize or validate calibration data
    if (!validate_calibration_data()) {
        DEBUG_INFO("SWSAnalog: Performing initial air calibration");
        calibrate_air_baseline();
    } else {
        DEBUG_INFO("SWSAnalog: Calibration data valid - air=%u water=%u thresh=%u",
                   m_calib.threshold_air, m_calib.threshold_water, m_calib.threshold_current);
    }

    // Initialize timing
    m_last_state_change_time = 0;
    m_time_in_current_state = 0;

    DEBUG_INFO("SWSAnalog: Initialized - min=%u max=%u hyst=%u%% calib_int=%us",
               m_threshold_min, m_threshold_max, m_hysteresis_percent, m_calib_interval_sec);
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
    const float ALPHA = 0.1f;  // Smoothing factor (10% new value, 90% old value)

    if (!is_value_valid(value)) {
        return;
    }

    // Only update if value is significantly higher than air threshold
    if (value > (m_calib.threshold_air * 1.5f)) {
        uint16_t new_water = (uint16_t)(ALPHA * value + (1.0f - ALPHA) * m_calib.threshold_water);

        DEBUG_TRACE("SWSAnalog: Updating water baseline %u -> %u", m_calib.threshold_water, new_water);

        m_calib.threshold_water = new_water;
        update_dynamic_threshold();

        // Update CRC
        m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                     sizeof(m_calib) - sizeof(m_calib.crc),
                                     nullptr);
    }
}

void SWSAnalogService::update_dynamic_threshold() {
    // Calculate midpoint between air and water thresholds
    uint16_t midpoint = (m_calib.threshold_air + m_calib.threshold_water) / 2;

    // Apply slight bias toward water to reduce false positives
    // (40% of range above air instead of 50%)
    m_calib.threshold_current = m_calib.threshold_air +
                                (uint16_t)((m_calib.threshold_water - m_calib.threshold_air) * 0.4f);

    // Calculate hysteresis value
    uint16_t range = m_calib.threshold_water - m_calib.threshold_air;
    m_calib.hysteresis_value = (uint16_t)(range * m_hysteresis_percent / 100.0f);

    // Ensure minimum hysteresis of 10 ADC counts
    if (m_calib.hysteresis_value < 10) {
        m_calib.hysteresis_value = 10;
    }

    DEBUG_TRACE("SWSAnalog: Updated threshold=%u hysteresis=%u",
                m_calib.threshold_current, m_calib.hysteresis_value);
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

    // Determine state with hysteresis
    bool new_state;
    uint16_t threshold_high = m_calib.threshold_current + m_calib.hysteresis_value;
    uint16_t threshold_low = m_calib.threshold_current - m_calib.hysteresis_value;

    if (filtered_value > threshold_high) {
        // Definitely underwater (high conductivity)
        new_state = true;

        // Update water baseline for continuous adaptation
        calibrate_water_baseline(filtered_value);
    } else if (filtered_value < threshold_low) {
        // Definitely at surface (low conductivity)
        new_state = false;
    } else {
        // In hysteresis zone - maintain previous state
        new_state = m_current_state;
        DEBUG_TRACE("SWSAnalog: In hysteresis zone, maintaining state=%u", new_state);
    }

    // Check safety timeouts
    bool timeout_override = check_safety_timeouts(new_state);
    if (timeout_override) {
        new_state = false;  // Force surface state
    }

    // Detect state changes for timing tracking
    if (new_state != m_current_state) {
        uint64_t current_time_sec = PMU::get_timestamp_ms() / 1000;
        m_last_state_change_time = current_time_sec;
        m_time_in_current_state = 0;
        DEBUG_INFO("SWSAnalog: State change detected - new_state=%u value=%u", new_state, filtered_value);
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
