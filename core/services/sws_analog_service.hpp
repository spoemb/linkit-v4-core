#pragma once

#include "bsp.hpp"
#include "uwdetector_service.hpp"

/**
 * @brief SWS Analog Detection Service with Dynamic Threshold
 *
 * This service provides robust saltwater detection using analog ADC readings
 * with auto-calibration and dynamic thresholding to handle varying salinity
 * conditions and environmental drift.
 *
 * Key Features:
 * - Auto-calibration: Learns "air" and "water" baseline values
 * - Dynamic threshold: Adapts to salinity changes over time
 * - Hysteresis: Prevents oscillation at threshold boundary
 * - Temporal filtering: Requires N consecutive samples for confirmation
 * - Safety timeouts: Max dive time and min surface time protection
 * - Persistent calibration: Survives device resets (stored in noinit RAM)
 */
class SWSAnalogService : public UWDetectorService {
public:
    SWSAnalogService() : UWDetectorService("SWSAnalog") {
        m_adc_history_idx = 0;
        m_last_state_change_time = 0;
        m_time_in_current_state = 0;
        for (int i = 0; i < ADC_HISTORY_SIZE; i++) {
            m_adc_history[i] = 0;
        }
    }

private:
    // Calibration data structure (stored in noinit RAM to survive resets)
    struct CalibrationData {
        uint16_t threshold_air;          // Baseline ADC value in air (low conductivity)
        uint16_t threshold_water;        // Baseline ADC value in water (high conductivity)
        uint16_t threshold_current;      // Current active threshold
        uint16_t hysteresis_value;       // Hysteresis margin (in ADC counts)
        uint64_t last_calibration_time;  // Timestamp of last calibration
        bool is_calibrated;              // Whether initial calibration is complete
        uint16_t crc;                    // CRC for data integrity
    };

    // Static calibration data in noinit RAM section (survives reset)
    static CalibrationData m_calib __attribute__((section(".noinit")));
    static uint16_t m_calib_crc __attribute__((section(".noinit")));

    // History buffer for moving average filter
    // Optimized: smaller buffer = faster response
    static constexpr int ADC_HISTORY_SIZE = 2;  // Was 5, now 2 for fast response
    uint16_t m_adc_history[ADC_HISTORY_SIZE];
    uint8_t m_adc_history_idx;

    // Timing tracking for safety timeouts
    uint64_t m_last_state_change_time;
    uint64_t m_time_in_current_state;

    // Sample confirmation counters (for robust detection)
    uint8_t m_consecutive_samples;      // Consecutive samples in same direction
    uint16_t m_min_adc_during_dive;     // Track min ADC for biofouling detection
    uint32_t m_time_in_hysteresis;      // Time stuck in hysteresis zone

    // Surface readings buffer for adaptive air baseline
    static constexpr int SURFACE_BUFFER_SIZE = 10;
    uint16_t m_surface_readings[SURFACE_BUFFER_SIZE];
    uint8_t m_surface_readings_idx;
    uint8_t m_surface_readings_count;

    // Trend detection buffer (for derivative-based surface detection)
    static constexpr int TREND_BUFFER_SIZE = 8;  // Track last 8 readings for trend
    uint16_t m_trend_buffer[TREND_BUFFER_SIZE];
    uint8_t m_trend_buffer_idx;
    uint8_t m_trend_buffer_count;
    uint8_t m_decreasing_trend_count;  // Consecutive decreasing samples

    // Cumulative drop tracking (more robust than consecutive-only for slow biofouling drying)
    uint16_t m_peak_adc_since_underwater;  // Track peak ADC since going "underwater"
    uint16_t m_cumulative_drop_percent;    // Drop from peak to current

    // Surface lockout after max dive time (prevent immediate re-submersion)
    uint32_t m_surface_lockout_remaining;  // Seconds remaining in lockout

    // Variance detection (high variance = surface drying, low variance = stable underwater)
    uint32_t m_variance_sum_sq;  // Sum of squared differences for variance calc
    uint16_t m_variance_mean;    // Running mean for variance calc

    // Configuration parameters (loaded from config store)
    uint16_t m_threshold_min;
    uint16_t m_threshold_max;
    uint16_t m_hysteresis_percent;
    uint32_t m_calib_interval_sec;
    uint32_t m_max_dive_time_sec;
    uint32_t m_min_surface_time_sec;

    // New optimized parameters for fast surface detection
    uint8_t m_threshold_ratio_percent;  // Position of threshold (37% = closer to air)
    uint8_t m_alpha_percent;            // EMA factor for water baseline (13 = 0.13)
    uint8_t m_max_samples;              // Samples to confirm dive (1 = immediate)
    uint8_t m_min_dry_samples;          // Samples to confirm surface (5 = robust)

    /**
     * @brief Read analog ADC value from SWS channel
     * @return Raw ADC value (0-4095 for 12-bit, 0-16383 for 14-bit)
     */
    uint16_t read_analog_sws();

    /**
     * @brief Initialize or validate calibration data from noinit RAM
     * @return true if calibration data is valid, false if needs initialization
     */
    bool validate_calibration_data();

    /**
     * @brief Perform initial calibration in air (baseline)
     * Called on first boot or after calibration data corruption
     */
    void calibrate_air_baseline();

    /**
     * @brief Update water baseline when underwater is confirmed
     * @param value Current ADC reading
     */
    void calibrate_water_baseline(uint16_t value);

    /**
     * @brief Update dynamic threshold based on current calibration
     * Uses midpoint between air and water with hysteresis
     */
    void update_dynamic_threshold();

    /**
     * @brief Add value to history buffer and get filtered result
     * @param value New ADC reading to add
     * @return Filtered value (moving average)
     */
    uint16_t add_to_history_and_filter(uint16_t value);

    /**
     * @brief Check if ADC value is within valid range
     * @param value ADC reading to validate
     * @return true if value is valid (not saturated or out of range)
     */
    bool is_value_valid(uint16_t value) const;

    /**
     * @brief Check if recalibration is needed based on time interval
     * @return true if calibration interval has elapsed
     */
    bool should_recalibrate() const;

    /**
     * @brief Check safety timeout conditions
     * @param current_state Current underwater state
     * @return true if timeout override is needed
     */
    bool check_safety_timeouts(bool current_state);

    /**
     * @brief Calculate CRC16 for calibration data integrity
     * @return CRC16 value
     */
    uint16_t calculate_calibration_crc() const;

protected:
    /**
     * @brief Detect current state (underwater or surface)
     * @return true if underwater detected, false if at surface
     */
    bool detector_state() override;

    /**
     * @brief Check if service is enabled in configuration
     * @return true if SWS analog detection is enabled
     */
    bool service_is_enabled() override;

    /**
     * @brief Service initialization - load config and validate calibration
     */
    void service_init() override;
};
