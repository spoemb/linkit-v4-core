#pragma once

#include "bsp.hpp"
#include "uwdetector_service.hpp"

/**
 * @brief SWS Analog Surface Detection — fast, adaptive, biofouling-resistant
 *
 * RC charge discrimination via 100nF cap + adaptive delay (100-5000µs).
 * 5-level surface override (L1-L5) for fast detection even with degraded electrodes.
 * Auto-calibration of air/water baselines with noinit RAM persistence.
 *
 * Test mode (DTE SWSTST): LED BLUE=underwater, YELLOW=surface. Async status push.
 */
class SWSAnalogService : public UWDetectorService {
public:
    // Detection method IDs (for IHM visualization)
    enum DetectMethod : uint8_t {
        DETECT_NONE      = 0,  // No transition this sample
        DETECT_THRESHOLD = 1,  // Simple threshold crossing
        DETECT_RAPID_T1  = 2,  // Rapid Tier 1 - single sharp drop
        DETECT_RAPID_T2  = 3,  // Rapid Tier 2 - confirmed moderate drop
        DETECT_RAPID_T3  = 4,  // Rapid Tier 3 - trend-based small drop
        DETECT_RAPID_T4  = 5,  // Rapid Tier 4 - sliding window gradual drop
        DETECT_TREND     = 6,  // Trend/variance biofouling override
        DETECT_SAFETY    = 7,  // Safety timeout override
    };

    // Status snapshot for DTE SWSST command (read-only diagnostic)
    struct Status {
        uint16_t threshold_air;       // Current air baseline ADC
        uint16_t threshold_water;     // Current water baseline ADC
        uint16_t threshold_current;   // Active threshold ADC
        uint16_t hysteresis;          // Hysteresis value (ADC counts)
        uint16_t last_raw_adc;        // Last raw ADC reading
        uint16_t last_filtered_adc;   // Last filtered ADC reading
        bool     is_calibrated;       // Calibration valid
        bool     is_underwater;       // Current state (true=underwater)
        uint32_t time_in_state_sec;   // Seconds in current state
        uint8_t  surface_level;       // Last detection level (0=none, 1-5=L1-L5)
        uint16_t contrast_x10;       // Water/air contrast ratio x10 (e.g. 47 = 4.7x)
        uint16_t observed_peak;      // Highest ADC value actually observed
        uint32_t sample_delay_us;    // Current adaptive RC charge delay (µs)
    };

    static Status get_status();

    /**
     * @brief Test mode API: start/stop SWS independently of config (DTE SWSTST command)
     *
     * start_test_mode() forces the service enabled and begins sampling.
     * stop_test_mode() halts sampling and turns off the status LED.
     * During test mode, detector_state() sets the RGB LED on state transitions:
     * BLUE = underwater, YELLOW = surface.
     */
    static void start_test_mode();
    static void stop_test_mode();
    static bool is_test_running();

    // Async notify: push SWSST status on each sample during test mode
    static void set_status_notify(std::function<void(const Status&)> fn);
    static void clear_status_notify();

    // Callback invoked when test mode stops (to restore normal LED behavior)
    static void set_on_test_stop(std::function<void()> fn);
    static void clear_on_test_stop();

#ifdef CPPUTEST
    // Reset static calibration data for test isolation
    static void reset_noinit_data() {
        memset(&m_calib, 0, sizeof(m_calib));
        m_observed_peak_adc = 0;
        m_observed_peak_crc = 0;
        m_status = {};
        s_instance = nullptr;
        m_test_mode = false;
        m_status_notify = nullptr;
    }
#endif

    SWSAnalogService() : UWDetectorService("SWSAnalog") {
        s_instance = this;
        m_adc_history_idx = 0;
        m_adc_history_count = 0;
        m_last_state_change_time = 0;
        m_time_in_current_state = 0;
        m_consecutive_samples = 0;
        m_surface_readings_idx = 0;
        m_surface_readings_count = 0;
        m_prev_raw = 0;
        m_drop_reference = 0;
        m_consecutive_raw_drops = 0;
        m_trend_buffer_idx = 0;
        m_trend_buffer_count = 0;
        m_prev_ma3 = 0;
        m_ma3_trend_start = 0;
        m_ma3_trend_count = 0;
        m_peak_adc_since_underwater = 0;
        m_recent_peak = 0;
        m_surface_lockout_remaining = 0;
        m_first_sample_done = false;
        for (int i = 0; i < ADC_HISTORY_SIZE; i++) {
            m_adc_history[i] = 0;
        }
        for (int i = 0; i < TREND_MA_SIZE; i++) {
            m_trend_buffer[i] = 0;
        }
        for (int i = 0; i < SURFACE_BUFFER_SIZE; i++) {
            m_surface_readings[i] = 0;
        }
    }

private:
    // Live status snapshot (updated each detector_state() call)
    static Status m_status;
    // Test mode support
    static SWSAnalogService* s_instance;
    static bool m_test_mode;
    static std::function<void(const Status&)> m_status_notify;
    static std::function<void()> m_on_test_stop;
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

    // Dynamic peak ADC tracker: highest ADC value actually observed (noinit for persistence)
    // Used to cap water baseline estimates when calibrating from air with wet electrodes
    static uint16_t m_observed_peak_adc __attribute__((section(".noinit")));
    static uint16_t m_observed_peak_crc __attribute__((section(".noinit")));

    // History buffer for moving average filter
    // Optimized: smaller buffer = faster response
    static constexpr int ADC_HISTORY_SIZE = 2;  // Was 5, now 2 for fast response
    uint16_t m_adc_history[ADC_HISTORY_SIZE];
    uint8_t m_adc_history_idx;
    uint8_t m_adc_history_count;

    // Timing tracking for safety timeouts
    uint64_t m_last_state_change_time;
    uint64_t m_time_in_current_state;

    // Sample confirmation counters (for robust detection)
    uint8_t m_consecutive_samples;      // Consecutive samples in same direction

    // Surface readings buffer for adaptive air baseline
    static constexpr int SURFACE_BUFFER_SIZE = 10;
    uint16_t m_surface_readings[SURFACE_BUFFER_SIZE];
    uint8_t m_surface_readings_idx;
    uint8_t m_surface_readings_count;

    // === MULTI-LEVEL SURFACE DETECTION STATE ===

    // Level 1 & 2: Fast raw drop detection
    uint16_t m_prev_raw;               // Previous raw ADC value
    uint16_t m_drop_reference;         // Raw value when consecutive drops started
    uint16_t m_consecutive_raw_drops;  // Count of consecutive raw drops

    // Level 3: Trend MA3 detection
    static constexpr int TREND_MA_SIZE = 3;
    uint16_t m_trend_buffer[TREND_MA_SIZE]; // Ring buffer for MA3 computation
    uint8_t m_trend_buffer_idx;
    uint8_t m_trend_buffer_count;
    uint16_t m_prev_ma3;               // Previous 3-sample moving average
    uint16_t m_ma3_trend_start;        // MA3 value when trend started
    uint16_t m_ma3_trend_count;        // Consecutive MA3 decreases

    // Level 4 & 5: Relative and cumulative drop
    uint16_t m_peak_adc_since_underwater;

    // Level 1: Recent peak for fast drop detection (decays with drift)
    uint16_t m_recent_peak;              // Tracks recent max, slowly decays to follow drift

    // Safety
    uint32_t m_surface_lockout_remaining;

    // First-sample coherence check
    bool m_first_sample_done;

    // Configuration parameters (loaded from config store)
    uint16_t m_threshold_min;
    uint16_t m_threshold_max;
    uint16_t m_hysteresis_percent;
    uint32_t m_calib_interval_sec;
    uint32_t m_max_dive_time_sec;
    uint32_t m_min_surface_time_sec;

    // New optimized parameters for fast surface detection
    uint8_t m_threshold_ratio_percent;  // Position of threshold (35% = closer to air)
    uint8_t m_alpha_percent;            // EMA factor for water baseline (19 = 0.19)

    // Adaptive sample delay (in µs) — auto-adjusted based on contrast
    uint32_t m_sample_delay_us;         // Current delay (100-5000 µs)

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
     * @brief Adjust sample delay based on current contrast ratio.
     * Reduces delay when biofouling degrades contrast, increases when clean.
     */
    void adjust_sample_delay();

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
