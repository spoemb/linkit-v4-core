#include "sws_analog_service.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include "pmu.hpp"
#include "nrfx_saadc.h"
#include "rgb_led.hpp"

#ifndef CPPUTEST
#include "crc16.h"
#else
// Simple CRC16-CCITT for test builds (replaces Nordic SDK crc16.h)
static inline uint16_t crc16_compute(const uint8_t *data, uint16_t length, const uint16_t *) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}
#endif

// LED for test mode visual feedback
extern RGBLed *status_led;

// ADC constants
#define ADC_REFERENCE_V 0.6f
#define ADC_GAIN_1_6 (1.0f/6.0f)
#define ADC_INVALID_MIN 0
#define ADC_INVALID_MAX 16383          // 14-bit ADC

// Default configuration values
#define DEFAULT_HYSTERESIS_PERCENT 4
#define DEFAULT_CALIB_INTERVAL_SEC 3600
#define DEFAULT_MAX_DIVE_TIME_SEC 7200

// Detection tuning
#define DEFAULT_THRESHOLD_RATIO_PERCENT 35
#define DEFAULT_ALPHA_PERCENT 19
#define DEFAULT_MIN_DRY_SAMPLES 1        // Immediate surface on threshold crossing

// Water baseline protection
#define ABSOLUTE_MIN_WATER_ADC 2000
#define MIN_WATER_AIR_RATIO 3

// Minimum air baseline floor: prevents adaptive DOWN from collapsing to near-zero.
// Even a clean dry electrode reads ~20-50 ADC with RC circuit.
#define AIR_BASELINE_FLOOR 20

// SWS.CAL offsets for Calibration class persistence
// 0 = manual water hint (SCALW/SWSCAL)
// 1 = manual air hint (SCALW/SWSCAL)
// 2 = running water baseline (auto-updated on state transitions)
// 3 = running air baseline (auto-updated on state transitions)
// 4 = observed peak ADC
#define CAL_OFFSET_HINT_WATER 0
#define CAL_OFFSET_HINT_AIR   1
#define CAL_OFFSET_RUN_WATER  2
#define CAL_OFFSET_RUN_AIR    3
#define CAL_OFFSET_PEAK       4

// Surface baseline adaptation
#define SURFACE_ADAPT_THRESHOLD 1.3f
#define MIN_SURFACE_TIME_FOR_ADAPT 10

// ═══════════════════════════════════════════════════════
//  MULTI-LEVEL SURFACE DETECTION
//
//  All levels require: underwater ≥ 1s AND proximity guard OK
//
//  Level 1 - INSTANT (1 sample)
//    Drop > 4% from previous raw → immediate surface
//    Use case: sharp water exit, any electrode condition
//
//  Level 2 - FAST (2 samples)
//    2 consecutive raw drops, cumulative > 3% → surface
//    Use case: gradual exit where individual drops < 3% each
//
//  Level 3 - TREND (3+ MA3 decreases)
//    MA3 decreasing 3+ times, total MA3 drop > 4% → surface
//    Use case: heavy biofouling, very slow drying, noisy signal
//
//  Level 4 - ABSOLUTE (variable)
//    filtered < water_baseline × 92% → surface
//    Use case: moderate biofouling, well-calibrated baselines
//
//  Level 5 - SAFETY NET (>10s underwater)
//    Drop from dive peak > 10% → surface
//    Use case: fallback when baselines have drifted
// ═══════════════════════════════════════════════════════

// Level 1
#define L1_DROP_PERCENT 4              // Drop from recent peak threshold (%)

// Level 2
#define L2_DROP_PERCENT 3              // Cumulative 2-sample raw drop (%)
#define L2_MIN_CONSECUTIVE 2           // Minimum consecutive raw drops
#define L2_MIN_STEP_PERCENT 2          // Each individual step must be ≥ 2% (filters drift)

// Level 3
#define L3_MIN_CONSECUTIVE 3           // Consecutive MA3 decreases
#define L3_DROP_PERCENT 4              // Total MA3 drop from trend start (%)

// Level 4
#define L4_DROP_PERCENT 8              // Drop from water baseline (%)

// Level 5
#define L5_DROP_PERCENT 10             // Cumulative drop from peak (%)
#define L5_MIN_TIME_SEC 10             // Minimum time underwater before L5

// Safety
#define OVERRIDE_MIN_TIME_SEC 1        // Minimum underwater time before any override
#define SURFACE_LOCKOUT_DURATION_SEC 30
#define MAX_CONSECUTIVE_DIVE_TIMEOUTS 3 // Force surface after N timeouts without any surface detection

// Adaptive sample delay (µs)
#define SAMPLE_DELAY_MIN_US     100    // Floor: below this, ADC values approach noise
#define SAMPLE_DELAY_MAX_US     5000   // Ceiling: above this, biofouled signals converge
#define SAMPLE_DELAY_DEFAULT_US 1000   // Default: 1ms (good balance for clean electrode)
#define CONTRAST_LOW_THRESHOLD  50     // contrast_x10 < 5.0x → reduce delay
#define CONTRAST_HIGH_THRESHOLD 100    // contrast_x10 > 10.0x → increase delay

// Proximity guard: L-overrides blocked if value > water_baseline * guard%
// Adaptive: relaxes when contrast is low (biofouling) to allow small-gap detection
#define PROXIMITY_GUARD_PERCENT 95     // Default: must drop below 95% of peak
#define PROXIMITY_GUARD_BIOFOULING 99  // Relaxed: when contrast < 5x, allow 1% gap

// Air recalibration on L-override: conservative EMA for gradual adaptation.
// At water exit, filtered value lags (MA2) and is near water baseline → small weight
// prevents ratcheting air upward on repeated rapid transitions.
#define AIR_RECALIB_EMA_WEIGHT 0.15f
// Hard safety cap: air must never exceed 70% of water.
// Guarantees threshold_high stays below ~86% of water baseline,
// so actual water readings (even 10-15% below baseline) still cross the threshold.
#define AIR_RECALIB_MAX_RATIO 0.70f

// Initialize static members
SWSAnalogService::CalibrationData SWSAnalogService::m_calib;
uint16_t SWSAnalogService::m_observed_peak_adc;
uint16_t SWSAnalogService::m_observed_peak_crc;
SWSAnalogService::Status SWSAnalogService::m_status = {};
SWSAnalogService* SWSAnalogService::s_instance = nullptr;
bool SWSAnalogService::m_test_mode = false;
SWSAnalogService::CalibPhase SWSAnalogService::m_calib_phase = SWSAnalogService::CalibPhase::IDLE;
uint32_t SWSAnalogService::m_calib_sum = 0;
uint8_t SWSAnalogService::m_calib_count = 0;
uint16_t SWSAnalogService::m_calib_air_result = 0;
uint16_t SWSAnalogService::m_calib_water_result = 0;
uint8_t SWSAnalogService::m_calib_stable_count = 0;
uint16_t SWSAnalogService::m_calib_prev_value = 0;
std::function<void(const SWSAnalogService::CalibResult&)> SWSAnalogService::m_calib_notify;
std::function<void(const SWSAnalogService::Status&)> SWSAnalogService::m_status_notify;
std::function<void()> SWSAnalogService::m_on_test_stop;
#if ENABLE_SWS_LOG
Logger *SWSAnalogService::m_sws_logger = nullptr;
#endif

SWSAnalogService::Status SWSAnalogService::get_status() {
    return m_status;
}

void SWSAnalogService::start_test_mode() {
    m_test_mode = true;
    if (s_instance) {
        DEBUG_INFO("SWSAnalog: Test mode started");
        s_instance->start();
        // Set initial LED to reflect current state
        if (status_led) {
            if (s_instance->m_current_state)
                status_led->set(RGBLedColor::BLUE);    // UNDERWATER
            else
                status_led->set(RGBLedColor::YELLOW);  // SURFACE
        }
    }
}

void SWSAnalogService::stop_test_mode() {
    if (s_instance) {
        s_instance->stop();
        DEBUG_INFO("SWSAnalog: Test mode stopped");
    }
    m_test_mode = false;
    // Restore normal LED behavior
    if (m_on_test_stop)
        m_on_test_stop();
}

bool SWSAnalogService::is_test_running() {
    return m_test_mode;
}

// ═══════════════════════════════════════════════════════
//  GUIDED CALIBRATION — LED-assisted air/water measurement
//
//  GREEN flashing  → place in AIR  → samples → GREEN solid
//  BLUE  flashing  → place in WATER → samples → BLUE solid
//  WHITE flash     → done (success)
//  RED flash       → done (failure)
// ═══════════════════════════════════════════════════════

#define CALIB_NUM_SAMPLES 5            // samples per phase (air/water)
#define CALIB_STABILITY_THRESHOLD 3    // consecutive stable readings to start sampling
#define CALIB_STABILITY_TOLERANCE 200  // ADC counts variation allowed for "stable"
#define CALIB_SAMPLE_INTERVAL_MS 1000  // 1s sampling during guided calibration

void SWSAnalogService::start_guided_calibration() {
    if (!s_instance) return;

    m_calib_phase = CalibPhase::AIR_WAITING;
    m_calib_sum = 0;
    m_calib_count = 0;
    m_calib_air_result = 0;
    m_calib_water_result = 0;
    m_calib_stable_count = 0;
    m_calib_prev_value = 0;

    // Start the SWS service if not already running
    m_test_mode = true;
    s_instance->start();

    DEBUG_INFO("SWSAnalog: Guided calibration started — place device in AIR");
    if (status_led) status_led->flash(RGBLedColor::GREEN, 500);
}

void SWSAnalogService::cancel_guided_calibration() {
    if (m_calib_phase == CalibPhase::IDLE) return;

    m_calib_phase = CalibPhase::IDLE;
    m_test_mode = false;
    if (s_instance) s_instance->stop();

    if (status_led) status_led->flash(RGBLedColor::RED, 200);
    DEBUG_INFO("SWSAnalog: Guided calibration cancelled");

    if (m_calib_notify) {
        CalibResult r = {3, 0, 0};  // status=cancelled
        m_calib_notify(r);
    }
}

bool SWSAnalogService::is_guided_calibration_running() {
    return m_calib_phase != CalibPhase::IDLE;
}

SWSAnalogService::CalibResult SWSAnalogService::get_guided_calibration_result() {
    if (m_calib_phase == CalibPhase::DONE) {
        return {1, m_calib_air_result, m_calib_water_result};  // success
    } else if (m_calib_phase == CalibPhase::IDLE && m_calib_air_result > 0) {
        return {1, m_calib_air_result, m_calib_water_result};  // already done
    }
    return {0, 0, 0};  // in progress
}

void SWSAnalogService::set_guided_calib_notify(std::function<void(const CalibResult&)> fn) {
    m_calib_notify = fn;
}

void SWSAnalogService::clear_guided_calib_notify() {
    m_calib_notify = nullptr;
}

void SWSAnalogService::set_status_notify(std::function<void(const Status&)> fn) {
    m_status_notify = fn;
}

void SWSAnalogService::clear_status_notify() {
    m_status_notify = nullptr;
}

void SWSAnalogService::set_on_test_stop(std::function<void()> fn) {
    m_on_test_stop = fn;
}

void SWSAnalogService::clear_on_test_stop() {
    m_on_test_stop = nullptr;
}

// SAADC event handler (required but not used for blocking mode)
static void nrfx_saadc_event_handler_sws(nrfx_saadc_evt_t const *p_event)
{
    (void)p_event;
}

// ═══════════════════════════════════════════════════════
//  MANUAL CALIBRATION (via DTE $SCALW/$SCALR, persisted in SWS.CAL)
//  Offset 0 = expected water ADC
//  Offset 1 = expected air ADC
// ═══════════════════════════════════════════════════════

void SWSAnalogService::calibration_write(const double value, const unsigned int offset) {
    m_manual_calib.write(offset, value);
    m_manual_calib.save(true);
    DEBUG_INFO("SWSAnalog: Manual calib write offset=%u value=%.0f (saved)", offset, value);
}

void SWSAnalogService::calibration_read(double &value, const unsigned int offset) {
    try {
        value = m_manual_calib.read(offset);
    } catch (...) {
        value = 0.0;
    }
}

void SWSAnalogService::calibration_save(bool force) {
    m_manual_calib.save(force);
}

// ═══════════════════════════════════════════════════════
//  FLASH PERSISTENCE — via SWS.CAL (Calibration class)
//  Running calibration stored at offsets 2-4, survives hard resets.
// ═══════════════════════════════════════════════════════

void SWSAnalogService::save_calibration_to_flash() {
    if (!s_instance) return;

    s_instance->m_manual_calib.write(CAL_OFFSET_RUN_WATER, (double)m_calib.threshold_water);
    s_instance->m_manual_calib.write(CAL_OFFSET_RUN_AIR, (double)m_calib.threshold_air);
    s_instance->m_manual_calib.write(CAL_OFFSET_PEAK, (double)m_observed_peak_adc);
    s_instance->m_manual_calib.save(true);
    DEBUG_TRACE("SWSAnalog: Calibration saved to SWS.CAL (air=%u water=%u peak=%u)",
                m_calib.threshold_air, m_calib.threshold_water, m_observed_peak_adc);
}

bool SWSAnalogService::load_calibration_from_flash() {
    if (!s_instance) return false;

    double water = 0, air = 0, peak = 0;
    try {
        water = s_instance->m_manual_calib.read(CAL_OFFSET_RUN_WATER);
        air = s_instance->m_manual_calib.read(CAL_OFFSET_RUN_AIR);
        peak = s_instance->m_manual_calib.read(CAL_OFFSET_PEAK);
    } catch (...) {
        return false;
    }

    // Sanity check
    if (air <= 0 || water <= 0 || water <= air || water > ADC_INVALID_MAX) {
        return false;
    }

    m_calib.threshold_air = (uint16_t)air;
    m_calib.threshold_water = (uint16_t)water;
    update_dynamic_threshold();
    m_calib.is_calibrated = true;
    m_calib.last_calibration_time = PMU::get_timestamp_ms() / 1000;
    m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                 sizeof(m_calib) - sizeof(m_calib.crc), nullptr);

    if (peak > 0 && peak <= ADC_INVALID_MAX) {
        m_observed_peak_adc = (uint16_t)peak;
        m_observed_peak_crc = crc16_compute((const uint8_t *)&m_observed_peak_adc,
                                             sizeof(m_observed_peak_adc), nullptr);
    }

    DEBUG_INFO("SWSAnalog: Calibration restored from SWS.CAL - air=%u water=%u thresh=%u peak=%u",
               m_calib.threshold_air, m_calib.threshold_water,
               m_calib.threshold_current, m_observed_peak_adc);
    return true;
}

void SWSAnalogService::service_init() {
    UWDetectorService::service_init();

    // SWS analog has its own internal filtering (MA2, 5-level detection, consecutive
    // samples). Multi-sample batching is redundant and wastes CPU in sample_gap waits.
    m_max_samples = 1;

    // Load configuration
    m_hysteresis_percent = service_read_param<unsigned int>(ParamID::SWS_ANALOG_HYSTERESIS);
    m_calib_interval_sec = service_read_param<unsigned int>(ParamID::SWS_ANALOG_CALIB_INTERVAL);
    m_max_dive_time_sec = service_read_param<unsigned int>(ParamID::UW_MAX_DIVE_TIME);
    m_min_surface_time_sec = service_read_param<unsigned int>(ParamID::UW_MIN_SURFACE_TIME);

    if (m_hysteresis_percent > 50) {
        m_hysteresis_percent = DEFAULT_HYSTERESIS_PERCENT;
    }

    m_threshold_ratio_percent = DEFAULT_THRESHOLD_RATIO_PERCENT;
    m_alpha_percent = DEFAULT_ALPHA_PERCENT;

    // Initialize adaptive sample delay from config (value in ms → convert to µs)
    m_sample_delay_us = m_enable_sample_delay * 1000;
    if (m_sample_delay_us < SAMPLE_DELAY_MIN_US) m_sample_delay_us = SAMPLE_DELAY_MIN_US;
    if (m_sample_delay_us > SAMPLE_DELAY_MAX_US) m_sample_delay_us = SAMPLE_DELAY_MAX_US;

    // Validate observed peak ADC from noinit RAM
    uint16_t peak_crc = crc16_compute((const uint8_t *)&m_observed_peak_adc,
                                       sizeof(m_observed_peak_adc), nullptr);
    if (peak_crc != m_observed_peak_crc || m_observed_peak_adc > ADC_INVALID_MAX) {
        m_observed_peak_adc = 0;  // Will be learned from live readings
        DEBUG_INFO("SWSAnalog: Observed peak ADC reset (invalid noinit)");
    } else {
        DEBUG_INFO("SWSAnalog: Observed peak ADC=%u (from noinit)", m_observed_peak_adc);
    }

    // Validate or initialize calibration from noinit RAM
    if (!validate_calibration_data()) {
        DEBUG_INFO("SWSAnalog: Calibration invalid | trying flash backup");
        memset(&m_calib, 0, sizeof(m_calib));
        if (!load_calibration_from_flash()) {
            DEBUG_INFO("SWSAnalog: No flash backup | clearing and recalibrating");
            calibrate_air_baseline();
        }
    } else {
        DEBUG_INFO("SWSAnalog: Calibration valid - air=%u water=%u thresh=%u",
                   m_calib.threshold_air, m_calib.threshold_water, m_calib.threshold_current);
    }

    // Initialize state tracking
    m_last_state_change_time = 0;
    m_time_in_current_state = 0;
    m_consecutive_samples = 0;
    // Surface readings buffer
    m_surface_readings_idx = 0;
    m_surface_readings_count = 0;
    for (int i = 0; i < SURFACE_BUFFER_SIZE; i++)
        m_surface_readings[i] = 0;

    // Level 1 & 2: fast drop
    m_prev_raw = 0;
    m_drop_reference = 0;
    m_consecutive_raw_drops = 0;

    // Level 3: trend MA3
    m_trend_buffer_idx = 0;
    m_trend_buffer_count = 0;
    m_prev_ma3 = 0;
    m_ma3_trend_start = 0;
    m_ma3_trend_count = 0;
    for (int i = 0; i < TREND_MA_SIZE; i++)
        m_trend_buffer[i] = 0;

    // Level 4 & 5
    m_peak_adc_since_underwater = 0;

    // Safety
    m_surface_lockout_remaining = 0;
    m_consecutive_dive_timeouts = 0;

    // First-sample coherence check
    m_first_sample_done = false;

    DEBUG_INFO("SWSAnalog: Init - hyst=%u%% ratio=%u%%",
               m_hysteresis_percent, m_threshold_ratio_percent);
}

bool SWSAnalogService::service_is_enabled() {
    if (m_test_mode) return true;
    return service_read_param<bool>(ParamID::UNDERWATER_EN);
}

unsigned int SWSAnalogService::service_next_schedule_in_ms() {
    // Fast 1s sampling during guided calibration for responsive user experience
    if (m_calib_phase != CalibPhase::IDLE && m_calib_phase != CalibPhase::DONE) {
        return CALIB_SAMPLE_INTERVAL_MS;
    }
    // Normal: read configured surface/underwater period
    unsigned int period = m_current_state ?
        service_read_param<unsigned int>(ParamID::SAMPLING_UNDER_FREQ) * 1000 :
        service_read_param<unsigned int>(ParamID::SAMPLING_SURF_FREQ) * 1000;
    return period > 0 ? period : 1000;
}

uint16_t SWSAnalogService::read_analog_sws() {
    nrf_saadc_value_t raw = 0;

    // RC discrimination: 100nF cap charges through water resistance.
    // Short delay → better contrast between water (τ≈1ms) and wet film (τ≈5-10ms).
    GPIOPins::set(SWS_ENABLE_PIN);
    PMU::delay_us(m_sample_delay_us);

    nrfx_err_t init_err = nrfx_saadc_init(&BSP::ADC_Inits.config, nrfx_saadc_event_handler_sws);
    if (init_err == NRFX_ERROR_INVALID_STATE) {
        nrfx_saadc_uninit();
        init_err = nrfx_saadc_init(&BSP::ADC_Inits.config, nrfx_saadc_event_handler_sws);
    }
    if (init_err != NRFX_SUCCESS) {
        GPIOPins::clear(SWS_ENABLE_PIN);
        return 0;
    }
    nrfx_saadc_channel_init(SWS_ADC, &BSP::ADC_Inits.channel_config[SWS_ADC]);

    nrfx_err_t err = nrfx_saadc_sample_convert(SWS_ADC, &raw);
    if (err != NRFX_SUCCESS) {
        DEBUG_ERROR("SWSAnalog: ADC conversion failed %d", err);
        raw = 0;
    }

    nrfx_saadc_uninit();
    GPIOPins::clear(SWS_ENABLE_PIN);

    return (uint16_t)(raw < 0 ? 0 : raw);
}

bool SWSAnalogService::validate_calibration_data() {
    uint16_t calculated_crc = crc16_compute((const uint8_t *)&m_calib,
                                             sizeof(m_calib) - sizeof(m_calib.crc), nullptr);
    if (calculated_crc != m_calib.crc) {
        DEBUG_WARN("SWSAnalog: CRC mismatch (calc=%u stored=%u)", calculated_crc, m_calib.crc);
        return false;
    }
    if (m_calib.threshold_air > ADC_INVALID_MAX) {
        return false;
    }
    if (m_calib.threshold_water > ADC_INVALID_MAX) {
        return false;
    }
    if (m_calib.threshold_air >= m_calib.threshold_water) {
        return false;
    }
    return m_calib.is_calibrated;
}

void SWSAnalogService::calibrate_air_baseline() {
    // Check for manual calibration from SWS.CAL (set via $SCALW,8,offset,value)
    // Offset 0 = expected water ADC, Offset 1 = expected air ADC
    double hint_water = 0.0, hint_air = 0.0;
    calibration_read(hint_water, 0);
    calibration_read(hint_air, 1);

    if (hint_water > 0 && hint_air > 0 && hint_water > hint_air) {
        m_calib.threshold_air = (uint16_t)hint_air;
        m_calib.threshold_water = (uint16_t)hint_water;

        update_dynamic_threshold();
        m_calib.is_calibrated = true;
        m_calib.last_calibration_time = PMU::get_timestamp_ms() / 1000;
        m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                     sizeof(m_calib) - sizeof(m_calib.crc), nullptr);

        DEBUG_INFO("SWSAnalog: Calib - air=%u water=%u thresh=%u (from SWS.CAL)",
                   m_calib.threshold_air, m_calib.threshold_water, m_calib.threshold_current);
        save_calibration_to_flash();
        return;
    }

    // Auto-calibration: sample the ADC
    const int NUM_SAMPLES = 10;
    uint32_t sum = 0;
    int valid_count = 0;
    uint16_t min_val = 0xFFFF, max_val = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        uint16_t value = read_analog_sws();
        if (is_value_valid(value)) {
            sum += value;
            valid_count++;
            if (value < min_val) min_val = value;
            if (value > max_val) max_val = value;
        }
        PMU::delay_ms(100);
    }

    if (valid_count == 0) {
        DEBUG_ERROR("SWSAnalog: No valid samples during calibration");
        return;
    }

    uint16_t avg = (uint16_t)(sum / valid_count);

    // Heuristic: if avg > WATER_DETECT_HEURISTIC, we're likely in water already.
    // A dry electrode typically reads < 2000 ADC counts. If reading is high,
    // treat it as water baseline and estimate air as avg/3.
    const uint16_t WATER_DETECT_HEURISTIC = 2500;

    if (avg > WATER_DETECT_HEURISTIC) {
        // Started calibration IN WATER — swap logic
        DEBUG_WARN("SWSAnalog: Calibration in water detected (avg=%u > %u)", avg, WATER_DETECT_HEURISTIC);
        m_calib.threshold_water = avg;
        // Estimate air as 1/3 of water (conservative)
        m_calib.threshold_air = avg / 3;
        if (m_calib.threshold_air < AIR_BASELINE_FLOOR)
            m_calib.threshold_air = AIR_BASELINE_FLOOR;
    } else {
        // Normal: started in air
        m_calib.threshold_air = avg;
        if (m_calib.threshold_water == 0 || m_calib.threshold_water <= m_calib.threshold_air) {
            // Priority order for water estimate:
            // 1. Manual hint from SWS.CAL (only water provided)
            // 2. Observed peak from noinit (previous session)
            // 3. Heuristic: air×3 (fallback)
            if (hint_water > m_calib.threshold_air) {
                m_calib.threshold_water = (uint16_t)hint_water;
                DEBUG_INFO("SWSAnalog: Water from manual hint %u", m_calib.threshold_water);
            } else if (m_observed_peak_adc > m_calib.threshold_air * 2 &&
                       m_observed_peak_adc >= (uint16_t)(m_calib.threshold_air * MIN_WATER_AIR_RATIO)) {
                m_calib.threshold_water = m_observed_peak_adc;
                DEBUG_INFO("SWSAnalog: Water from observed peak %u", m_observed_peak_adc);
            } else {
                // Heuristic: water ≈ air × 3, capped at air × 5 to avoid
                // unreachable thresholds when air is high (wet electrode startup).
                m_calib.threshold_water = m_calib.threshold_air * 3;
                uint32_t max_water_estimate = (uint32_t)m_calib.threshold_air * 5;
                if (max_water_estimate > ADC_INVALID_MAX)
                    max_water_estimate = ADC_INVALID_MAX;
                if (m_calib.threshold_water > (uint16_t)max_water_estimate)
                    m_calib.threshold_water = (uint16_t)max_water_estimate;
            }
            if (m_calib.threshold_water > ADC_INVALID_MAX)
                m_calib.threshold_water = ADC_INVALID_MAX;
        }
    }

    update_dynamic_threshold();
    m_calib.is_calibrated = true;
    m_calib.last_calibration_time = PMU::get_timestamp_ms() / 1000;
    m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                 sizeof(m_calib) - sizeof(m_calib.crc), nullptr);

    DEBUG_INFO("SWSAnalog: Calib - air=%u water=%u thresh=%u (started_in_%s)",
               m_calib.threshold_air, m_calib.threshold_water, m_calib.threshold_current,
               avg > WATER_DETECT_HEURISTIC ? "WATER" : "AIR");

    save_calibration_to_flash();
}

void SWSAnalogService::calibrate_water_baseline(uint16_t value) {
    float alpha = m_alpha_percent / 100.0f;
    if (!is_value_valid(value)) return;

    uint16_t threshold_with_margin = m_calib.threshold_current + m_calib.hysteresis_value;

    // Relaxed guard when water baseline is still estimated (no real peaks observed yet).
    // Estimated water (air*3) can be wildly off with dirty pins → allow full adaptation.
    bool water_is_estimated = (m_observed_peak_adc == 0);
    uint16_t min_expected_water = water_is_estimated ?
        (uint16_t)(m_calib.threshold_air * MIN_WATER_AIR_RATIO) :
        (uint16_t)(m_calib.threshold_water * 0.85f);

    // Only apply air ratio guard when air baseline is reasonable (< 1000 ADC).
    // If air is high (e.g. from corrupted calibration), skip this check to allow recovery.
    bool air_ratio_ok = (m_calib.threshold_air >= 1000) ||
                        (value >= (uint16_t)(m_calib.threshold_air * MIN_WATER_AIR_RATIO));

    // Water must exceed threshold and maintain minimum ratio to air.
    // No absolute ADC floor: allows operation in fresh/brackish water where
    // ADC values can be well below 2000.
    bool ok = (value > threshold_with_margin) &&
              air_ratio_ok &&
              (value >= min_expected_water || value > m_calib.threshold_water);

    if (ok) {
        uint16_t new_water = (uint16_t)(alpha * value + (1.0f - alpha) * m_calib.threshold_water);
        // Cap at observed peak so water baseline never exceeds actual readings
        if (m_observed_peak_adc > 0 && new_water > m_observed_peak_adc)
            new_water = m_observed_peak_adc;
        m_calib.threshold_water = new_water;
        update_dynamic_threshold();
        m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                     sizeof(m_calib) - sizeof(m_calib.crc), nullptr);
    }
}

void SWSAnalogService::update_dynamic_threshold() {
    // Dynamic ratio: low contrast (biofouling) → threshold closer to water
    float contrast = (m_calib.threshold_air > 0) ?
        (float)m_calib.threshold_water / (float)m_calib.threshold_air : 10.0f;
    m_contrast_x10 = (uint16_t)(contrast * 10.0f);

    float ratio;
    if (contrast >= 8.0f) {
        ratio = m_threshold_ratio_percent / 100.0f;  // Clean: 35%
    } else if (contrast >= 4.0f) {
        ratio = 0.50f;  // Moderate biofouling: midpoint
    } else {
        // Very low contrast (wet electrodes or severe biofouling):
        // threshold closer to air to ensure water readings can cross it.
        // Multi-level detection handles false surface prevention.
        ratio = 0.40f;
    }

    uint16_t range = m_calib.threshold_water - m_calib.threshold_air;
    m_calib.threshold_current = m_calib.threshold_air + (uint16_t)(range * ratio);

    m_calib.hysteresis_value = (uint16_t)(m_calib.threshold_current * m_hysteresis_percent / 100.0f);
    if (m_calib.hysteresis_value < 10)
        m_calib.hysteresis_value = 10;

    // Cap threshold+hysteresis at observed peak ADC so we never exceed actual readings
    if (m_observed_peak_adc > 0) {
        uint16_t max_thresh_high = m_observed_peak_adc;
        if (m_calib.threshold_current + m_calib.hysteresis_value > max_thresh_high) {
            // Adjust: keep threshold and reduce hysteresis, or lower threshold
            if (m_calib.threshold_current >= max_thresh_high) {
                m_calib.threshold_current = (uint16_t)(max_thresh_high * 0.90f);
                m_calib.hysteresis_value = (uint16_t)(max_thresh_high * 0.05f);
            } else {
                m_calib.hysteresis_value = max_thresh_high - m_calib.threshold_current;
            }
            if (m_calib.hysteresis_value < 10)
                m_calib.hysteresis_value = 10;
        }
    }

    DEBUG_TRACE("SWSAnalog: thresh=%u contrast=%.1f ratio=%.0f%% hyst=%u peak=%u",
                m_calib.threshold_current, (double)contrast, (double)(ratio*100),
                m_calib.hysteresis_value, m_observed_peak_adc);
}

uint16_t SWSAnalogService::add_to_history_and_filter(uint16_t value) {
    m_adc_history[m_adc_history_idx] = value;
    m_adc_history_idx = (m_adc_history_idx + 1) % ADC_HISTORY_SIZE;
    if (m_adc_history_count < ADC_HISTORY_SIZE)
        m_adc_history_count++;

    uint32_t sum = 0;
    for (int i = 0; i < m_adc_history_count; i++) {
        sum += m_adc_history[i];
    }
    return (uint16_t)(sum / m_adc_history_count);
}

bool SWSAnalogService::is_value_valid(uint16_t value) const {
    return (value <= ADC_INVALID_MAX);
}

void SWSAnalogService::adjust_sample_delay() {
    if (m_calib.threshold_air == 0) return;
    uint16_t contrast = m_contrast_x10;
    uint32_t old_delay = m_sample_delay_us;
    if (contrast < CONTRAST_LOW_THRESHOLD && m_sample_delay_us > SAMPLE_DELAY_MIN_US) {
        m_sample_delay_us = m_sample_delay_us * 3 / 4;  // -25%
        if (m_sample_delay_us < SAMPLE_DELAY_MIN_US) m_sample_delay_us = SAMPLE_DELAY_MIN_US;
    } else if (contrast > CONTRAST_HIGH_THRESHOLD && m_sample_delay_us < SAMPLE_DELAY_MAX_US) {
        m_sample_delay_us = m_sample_delay_us * 11 / 10;  // +10%
        if (m_sample_delay_us > SAMPLE_DELAY_MAX_US) m_sample_delay_us = SAMPLE_DELAY_MAX_US;
    }
    if (old_delay != m_sample_delay_us) {
        DEBUG_INFO("SWSAnalog: Adaptive delay %uus -> %uus (contrast=%u.%u)",
                   old_delay, m_sample_delay_us, contrast/10, contrast%10);
    }
}

bool SWSAnalogService::should_recalibrate() const {
    if (m_calib_interval_sec == 0) return false;
    uint64_t elapsed = (PMU::get_timestamp_ms() / 1000) - m_calib.last_calibration_time;
    return (elapsed >= m_calib_interval_sec);
}

bool SWSAnalogService::check_safety_timeouts(bool current_state) {
    uint64_t now = PMU::get_timestamp_ms() / 1000;
    if (m_last_state_change_time == 0) {
        m_last_state_change_time = now;
        m_time_in_current_state = 0;
    } else {
        m_time_in_current_state = now - m_last_state_change_time;
    }

    if (current_state && m_max_dive_time_sec > 0 &&
        m_time_in_current_state >= m_max_dive_time_sec) {
        DEBUG_WARN("SWSAnalog: Max dive time %us | recalibrating", m_time_in_current_state);
        return true;
    }
    return false;
}

// ═══════════════════════════════════════════════════════
//  MAIN DETECTION: 5-LEVEL SURFACE DETECTION
// ═══════════════════════════════════════════════════════

bool SWSAnalogService::detector_state() {

    // === 1. READ ADC ===
    uint16_t raw_value = read_analog_sws();
    if (!is_value_valid(raw_value)) {
        DEBUG_WARN("SWSAnalog: Invalid ADC %u", raw_value);
        return m_current_state;
    }

    // === 1b. COHERENCE CHECK (first sample + continuous) ===
    // Detects calibration mismatch vs actual ADC readings.
    // First sample: catches stale noinit data.
    // Continuous: catches environment changes (e.g. device moved from indoor to outdoor).
    {
        bool calib_incoherent = false;

        if (!m_first_sample_done) {
            m_first_sample_done = true;

            // Case 1: Stored air is low, but reading is way above water → wrong medium
            if (raw_value > m_calib.threshold_current + m_calib.hysteresis_value * 3 &&
                raw_value > m_calib.threshold_water * 1.3f) {
                DEBUG_WARN("SWSAnalog: Coherence fail - raw=%u >> water=%u, recalibrating",
                           raw_value, m_calib.threshold_water);
                calib_incoherent = true;
            }
            // Case 2: Stored air is high (calibrated in water), but reading is low → in air
            else if (raw_value < m_calib.threshold_air * 0.5f && m_calib.threshold_air > 2000) {
                DEBUG_WARN("SWSAnalog: Coherence fail - raw=%u << air=%u, recalibrating",
                           raw_value, m_calib.threshold_air);
                calib_incoherent = true;
            }
        }

        // Continuous coherence: if on surface and raw reading exceeds water baseline
        // by a large margin, the environment has changed (e.g. moved to real water
        // after indoor calibration). Adapt water baseline immediately instead of
        // waiting for the slow EMA convergence.
        if (!calib_incoherent && !m_current_state && m_calib.threshold_water > 0 &&
            raw_value > m_calib.threshold_water * 2 &&
            m_time_in_current_state > 2) {
            DEBUG_WARN("SWSAnalog: Continuous coherence - raw=%u >> water=%u, adapting water",
                       raw_value, m_calib.threshold_water);
            m_calib.threshold_water = raw_value;
            update_dynamic_threshold();
            m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                         sizeof(m_calib) - sizeof(m_calib.crc), nullptr);
            // Flash save deferred to next state transition (section 10) to avoid
            // excessive writes during rapid convergence
        }

        if (calib_incoherent) {
            memset(&m_calib, 0, sizeof(m_calib));
            if (raw_value > 2500) {
                m_calib.threshold_water = raw_value;
                m_calib.threshold_air = raw_value / 3;
            } else {
                m_calib.threshold_air = raw_value;
                m_calib.threshold_water = raw_value * 3;
                if (m_observed_peak_adc > 0 && m_calib.threshold_water > m_observed_peak_adc)
                    m_calib.threshold_water = m_observed_peak_adc;
                if (m_calib.threshold_water > ADC_INVALID_MAX)
                    m_calib.threshold_water = ADC_INVALID_MAX;
            }
            update_dynamic_threshold();
            m_calib.is_calibrated = true;
            m_calib.last_calibration_time = PMU::get_timestamp_ms() / 1000;
            m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                         sizeof(m_calib) - sizeof(m_calib.crc), nullptr);
            DEBUG_INFO("SWSAnalog: Recalib from coherence - air=%u water=%u thresh=%u",
                       m_calib.threshold_air, m_calib.threshold_water, m_calib.threshold_current);
            save_calibration_to_flash();
        }
    }

    // Save prev_raw BEFORE filtering overwrites history
    uint16_t prev_raw = m_prev_raw;
    m_prev_raw = raw_value;

    uint16_t filtered_value = add_to_history_and_filter(raw_value);

    // === 2. TIME TRACKING ===
    bool timeout_override = check_safety_timeouts(m_current_state);

    // === 3. LEVEL 1 & 2: FAST RAW DROP (uses raw, not filtered, for speed) ===
    uint8_t surface_level = 0;

    // Proximity guard: blocks L-overrides if still close to water peak.
    // Adaptive: relaxes to 99% when contrast < 5x (biofouling).
    uint16_t proximity_ref = m_calib.threshold_water;
    if (m_peak_adc_since_underwater > proximity_ref)
        proximity_ref = m_peak_adc_since_underwater;
    uint8_t guard_pct = (m_contrast_x10 < 50) ? PROXIMITY_GUARD_BIOFOULING : PROXIMITY_GUARD_PERCENT;
    bool proximity_ok = (proximity_ref == 0) ||
        (filtered_value < (uint16_t)(proximity_ref * guard_pct / 100.0f));

    if (m_current_state && prev_raw > 0 && m_time_in_current_state >= OVERRIDE_MIN_TIME_SEC
        && proximity_ok) {

        // LEVEL 1: Sudden single-sample drop from previous raw
        // Uses prev_raw (not recent_peak) to avoid false triggers from gradual drift
        if (prev_raw > 0 && raw_value < prev_raw) {
            uint16_t single_drop_pct = (uint16_t)((uint32_t)(prev_raw - raw_value) * 100 / prev_raw);
            if (single_drop_pct >= L1_DROP_PERCENT) {
                surface_level = 1;
            }
        }

        // LEVEL 2: Two consecutive significant raw drops with cumulative threshold
        // Each step must exceed L2_MIN_STEP_PERCENT to filter gradual drift/salinity changes
        if (surface_level == 0) {
            if (raw_value < prev_raw) {
                uint16_t step_pct = (uint16_t)((uint32_t)(prev_raw - raw_value) * 100 / prev_raw);
                if (step_pct >= L2_MIN_STEP_PERCENT) {
                    if (m_consecutive_raw_drops == 0) {
                        m_drop_reference = prev_raw;
                    }
                    m_consecutive_raw_drops++;

                    uint16_t cumul_pct = (uint16_t)((uint32_t)(m_drop_reference - raw_value) * 100 / m_drop_reference);
                    if (m_consecutive_raw_drops >= L2_MIN_CONSECUTIVE && cumul_pct >= L2_DROP_PERCENT) {
                        surface_level = 2;
                    }
                } else {
                    m_consecutive_raw_drops = 0;  // Drift, not a real drop
                }
            } else {
                m_consecutive_raw_drops = 0;
            }
        }
    } else if (!m_current_state) {
        m_consecutive_raw_drops = 0;
    }

    // === 4. LEVEL 3: TREND MA3 (consecutive MA3 decreases → electrode drying) ===
    m_trend_buffer[m_trend_buffer_idx] = filtered_value;
    m_trend_buffer_idx = (m_trend_buffer_idx + 1) % TREND_MA_SIZE;
    if (m_trend_buffer_count < TREND_MA_SIZE)
        m_trend_buffer_count++;

    uint16_t current_ma3 = filtered_value;
    if (m_trend_buffer_count >= TREND_MA_SIZE) {
        uint32_t ma_sum = 0;
        for (int i = 0; i < TREND_MA_SIZE; i++)
            ma_sum += m_trend_buffer[i];
        current_ma3 = (uint16_t)(ma_sum / TREND_MA_SIZE);
    }

    if (surface_level == 0 && m_current_state && m_prev_ma3 > 0 &&
        m_trend_buffer_count >= TREND_MA_SIZE &&
        m_time_in_current_state >= OVERRIDE_MIN_TIME_SEC && proximity_ok) {

        if (current_ma3 < m_prev_ma3) {
            // MA3 decreased
            if (m_ma3_trend_count == 0) {
                m_ma3_trend_start = m_prev_ma3;  // record start of trend
            }
            m_ma3_trend_count++;
        } else {
            // Allow 1 flat/increase without full reset (noise tolerance)
            if (m_ma3_trend_count > 0)
                m_ma3_trend_count--;
        }

        if (m_ma3_trend_count >= L3_MIN_CONSECUTIVE && m_ma3_trend_start > 0) {
            uint16_t ma3_drop = (uint16_t)((uint32_t)(m_ma3_trend_start - current_ma3) * 100 / m_ma3_trend_start);
            if (ma3_drop >= L3_DROP_PERCENT) {
                surface_level = 3;
            }
        }
    } else if (!m_current_state) {
        m_ma3_trend_count = 0;
        m_ma3_trend_start = 0;
    }

    m_prev_ma3 = current_ma3;

    // === 5. LEVEL 4 & 5: RELATIVE AND CUMULATIVE DROP ===
    if (m_current_state) {
        if (filtered_value > m_peak_adc_since_underwater)
            m_peak_adc_since_underwater = filtered_value;

        // Recent peak: decays 2%/sample toward reading (prevents drift false-trigger)
        if (raw_value > m_recent_peak || m_recent_peak == 0) {
            m_recent_peak = raw_value;
        } else {
            // Decay: recent_peak moves 2% toward current reading each sample
            // Slow decay preserves sensitivity with long sampling periods (10s)
            m_recent_peak = (uint16_t)(m_recent_peak * 0.98f + raw_value * 0.02f);
        }
    }

    if (surface_level == 0 && m_current_state &&
        m_time_in_current_state >= OVERRIDE_MIN_TIME_SEC && proximity_ok) {

        // LEVEL 4: Drop relative to water baseline
        if (m_calib.threshold_water > 0) {
            uint16_t water_thresh = (uint16_t)(m_calib.threshold_water *
                                               (1.0f - L4_DROP_PERCENT / 100.0f));
            if (filtered_value < water_thresh) {
                surface_level = 4;
            }
        }

        // LEVEL 5: Cumulative drop from peak during this dive
        if (surface_level == 0 && m_peak_adc_since_underwater > 0 &&
            m_time_in_current_state > L5_MIN_TIME_SEC) {
            uint16_t drop = (uint16_t)((uint32_t)(m_peak_adc_since_underwater - filtered_value) * 100 /
                                        m_peak_adc_since_underwater);
            if (drop >= L5_DROP_PERCENT) {
                surface_level = 5;
            }
        }
    }

    // === 6. SURFACE BASELINE TRACKING (blocked during lockout) ===
    if (!m_current_state && m_time_in_current_state > MIN_SURFACE_TIME_FOR_ADAPT
        && m_surface_lockout_remaining == 0) {
        if (filtered_value < m_calib.threshold_current) {
            m_surface_readings[m_surface_readings_idx] = filtered_value;
            m_surface_readings_idx = (m_surface_readings_idx + 1) % SURFACE_BUFFER_SIZE;
            if (m_surface_readings_count < SURFACE_BUFFER_SIZE)
                m_surface_readings_count++;
        }

        if (m_surface_readings_count >= SURFACE_BUFFER_SIZE / 2) {
            uint32_t sum = 0;
            for (int i = 0; i < m_surface_readings_count; i++)
                sum += m_surface_readings[i];
            uint16_t avg = (uint16_t)(sum / m_surface_readings_count);

            if (should_recalibrate()) {
                uint16_t old = m_calib.threshold_air;
                m_calib.threshold_air = avg;
                update_dynamic_threshold();
                m_calib.last_calibration_time = PMU::get_timestamp_ms() / 1000;
                m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                             sizeof(m_calib) - sizeof(m_calib.crc), nullptr);
                m_surface_readings_count = 0;
                m_surface_readings_idx = 0;
                DEBUG_INFO("SWSAnalog: Air recalib %u -> %u", old, m_calib.threshold_air);
                adjust_sample_delay();
            }
            else if (avg > (uint16_t)(m_calib.threshold_air * SURFACE_ADAPT_THRESHOLD) &&
                     avg < m_calib.threshold_current) {
                // Upward adaptation: air drifting higher (biofouling)
                uint16_t old = m_calib.threshold_air;
                m_calib.threshold_air = (uint16_t)(m_calib.threshold_air * 0.9f + avg * 0.1f);
                update_dynamic_threshold();
                m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                             sizeof(m_calib) - sizeof(m_calib.crc), nullptr);
                DEBUG_INFO("SWSAnalog: Adaptive air UP %u -> %u", old, m_calib.threshold_air);
                adjust_sample_delay();
            }
            else if (avg < (uint16_t)(m_calib.threshold_air * 0.70f)) {
                // Downward adaptation: air was too high (wet electrode calibration)
                //
                // Guard against runaway drift toward zero: block if air baseline is
                // already close to actual surface readings (< 2× avg). This replaces
                // the old absolute contrast gate which permanently blocked adaptation
                // once water baseline was established.
                bool air_already_low = (m_calib.threshold_air < avg * 2);
                if (air_already_low) {
                    DEBUG_TRACE("SWSAnalog: Adaptive air DOWN blocked (air=%u already close to avg=%u)",
                                m_calib.threshold_air, avg);
                } else {
                    uint16_t old = m_calib.threshold_air;
                    uint16_t new_air = (uint16_t)(m_calib.threshold_air * 0.80f + avg * 0.20f);
                    if (new_air < AIR_BASELINE_FLOOR) new_air = AIR_BASELINE_FLOOR;
                    m_calib.threshold_air = new_air;
                    update_dynamic_threshold();
                    m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                                 sizeof(m_calib) - sizeof(m_calib.crc), nullptr);
                    m_surface_readings_count = 0;
                    m_surface_readings_idx = 0;
                    DEBUG_INFO("SWSAnalog: Adaptive air DOWN %u -> %u", old, m_calib.threshold_air);
                    adjust_sample_delay();
                }
            }
        }
    } else if (m_current_state) {
        m_surface_readings_count = 0;
        m_surface_readings_idx = 0;
    }

    // === 7. STATE DETERMINATION ===
    bool new_state = m_current_state;
    uint16_t threshold_high = m_calib.threshold_current + m_calib.hysteresis_value;
    uint16_t threshold_low = (m_calib.hysteresis_value >= m_calib.threshold_current)
        ? 0 : m_calib.threshold_current - m_calib.hysteresis_value;
    // Prevent underflow: ensure threshold_low stays above air baseline
    if (threshold_low <= m_calib.threshold_air && m_calib.threshold_current > m_calib.threshold_air) {
        threshold_low = m_calib.threshold_air + 1;
    }

    // Basic threshold detection
    if (filtered_value > threshold_high) {
        new_state = true;
        m_consecutive_samples = 0;

        // Don't update water baseline during surface lockout (readings are surface, not water)
        if (m_surface_lockout_remaining == 0) {
            calibrate_water_baseline(filtered_value);
        }
    } else if (filtered_value < threshold_low) {

        if (m_current_state) {
            m_consecutive_samples++;
            if (m_consecutive_samples >= DEFAULT_MIN_DRY_SAMPLES)
                new_state = false;
        } else {
            new_state = false;
            m_consecutive_samples = 0;
        }
    } else {
        // Hysteresis zone — maintain current state
        m_consecutive_samples = 0;
    }

    // === 7b. UPDATE OBSERVED PEAK ADC ===
    // Placed after calibrate_water_baseline() so water_is_estimated (peak==0)
    // is correctly evaluated on the first underwater samples.
    if (raw_value > m_observed_peak_adc) {
        m_observed_peak_adc = raw_value;
        m_observed_peak_crc = crc16_compute((const uint8_t *)&m_observed_peak_adc,
                                             sizeof(m_observed_peak_adc), nullptr);
    }

    // Apply multi-level surface override
    if (surface_level > 0 && m_current_state && new_state) {
        new_state = false;
        m_consecutive_samples = 0;

        // Recalibrate air: gentle EMA toward filtered value (prevents corruption
        // from transitional readings at water exit where filtered is still near water).
        // Hard cap at 90% of water ensures threshold_high never exceeds water ADC.
        uint16_t old_air = m_calib.threshold_air;
        uint16_t new_air = (uint16_t)(m_calib.threshold_air * (1.0f - AIR_RECALIB_EMA_WEIGHT)
                                       + filtered_value * AIR_RECALIB_EMA_WEIGHT);
        uint16_t hard_cap = (uint16_t)(m_calib.threshold_water * AIR_RECALIB_MAX_RATIO);
        if (new_air > hard_cap) new_air = hard_cap;
        if (new_air < AIR_BASELINE_FLOOR) new_air = AIR_BASELINE_FLOOR;
        if (new_air != m_calib.threshold_air && new_air < m_calib.threshold_water) {
            m_calib.threshold_air = new_air;
            update_dynamic_threshold();
            m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                         sizeof(m_calib) - sizeof(m_calib.crc), nullptr);
            DEBUG_INFO("SWSAnalog: SURFACE L%u | air recalib %u -> %u | thresh=%u",
                       surface_level, old_air, m_calib.threshold_air, m_calib.threshold_current);

            adjust_sample_delay();
        }

        // Enforce minimum surface time to prevent immediate re-trigger
        if (m_min_surface_time_sec > 0) {
            m_surface_lockout_remaining = m_min_surface_time_sec;
        }

        DEBUG_INFO("SWSAnalog: SURFACE L%u | raw=%u filt=%u ma3=%u air=%u water=%u lockout=%us",
                   surface_level, raw_value, filtered_value, current_ma3,
                   m_calib.threshold_air, m_calib.threshold_water, m_surface_lockout_remaining);
    }

    // === 8. MAX DIVE TIMEOUT — escalating response ===
    // First timeouts: recalibrate water baseline (legitimate long dives).
    // After MAX_CONSECUTIVE_DIVE_TIMEOUTS without surface: force surface + lockout
    // to give GPS/Argos a TX window (biofouling safety net).
    if (timeout_override) {
        m_consecutive_dive_timeouts++;
        uint16_t old_water = m_calib.threshold_water;

        // Always recalibrate water baseline from current reading
        if (is_value_valid(filtered_value) && filtered_value > m_calib.threshold_air * 2) {
            m_calib.threshold_water = filtered_value;
            update_dynamic_threshold();
            m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                         sizeof(m_calib) - sizeof(m_calib.crc), nullptr);
            save_calibration_to_flash();
        }

        if (m_consecutive_dive_timeouts >= MAX_CONSECUTIVE_DIVE_TIMEOUTS) {
            // Escalation: force surface after repeated timeouts (biofouling safety net)
            new_state = false;
            m_surface_lockout_remaining = SURFACE_LOCKOUT_DURATION_SEC;
            m_consecutive_dive_timeouts = 0;
            DEBUG_WARN("SWSAnalog: Dive timeout escalation — forcing surface | water %u -> %u | air=%u",
                       old_water, m_calib.threshold_water, m_calib.threshold_air);
        } else {
            // Reset timer for next interval
            m_last_state_change_time = PMU::get_timestamp_ms() / 1000;
            DEBUG_WARN("SWSAnalog: Dive timeout %u/%u — recalib | water %u -> %u | air=%u",
                       m_consecutive_dive_timeouts, MAX_CONSECUTIVE_DIVE_TIMEOUTS,
                       old_water, m_calib.threshold_water, m_calib.threshold_air);
        }
    }

    // === 9. SURFACE LOCKOUT (time-based, not sample-count-based) ===
    // Lockout duration is in seconds; compared against actual time in surface state.
    // !m_current_state guards against premature clear on the L-override call
    // (where m_time_in_current_state still reflects the old UW state).
    if (m_surface_lockout_remaining > 0) {
        if (!m_current_state && m_time_in_current_state >= m_surface_lockout_remaining) {
            m_surface_lockout_remaining = 0;
        } else if (new_state) {
            new_state = false;
        }
    }

    // === 10. STATE CHANGE ===
    if (new_state != m_current_state) {
        m_last_state_change_time = PMU::get_timestamp_ms() / 1000;
        m_time_in_current_state = 0;
        m_consecutive_samples = 0;
        m_peak_adc_since_underwater = 0;
        m_recent_peak = 0;
        m_consecutive_dive_timeouts = 0;  // Reset escalation on any state change

        // Reset fast drop tracking
        m_consecutive_raw_drops = 0;
        m_drop_reference = 0;

        // Reset trend tracking
        m_trend_buffer_count = 0;
        m_trend_buffer_idx = 0;
        m_ma3_trend_count = 0;
        m_ma3_trend_start = 0;
        m_prev_ma3 = 0;

        DEBUG_INFO("SWSAnalog: %s -> %s | raw=%u filt=%u thresh=%u air=%u water=%u",
                   m_current_state ? "UW" : "SURF",
                   new_state ? "UW" : "SURF",
                   raw_value, filtered_value,
                   m_calib.threshold_current, m_calib.threshold_air, m_calib.threshold_water);

        // Persist calibration to flash on state transitions (infrequent, important)
        save_calibration_to_flash();

        // Test mode: override LED to show SWS state
        if (m_test_mode && status_led) {
            if (new_state)
                status_led->set(RGBLedColor::BLUE);    // UNDERWATER
            else
                status_led->set(RGBLedColor::YELLOW);  // SURFACE
        }
    }

    // === 11. STATUS PUSH ===
    m_status.threshold_air = m_calib.threshold_air;
    m_status.threshold_water = m_calib.threshold_water;
    m_status.threshold_current = m_calib.threshold_current;
    m_status.hysteresis = m_calib.hysteresis_value;
    m_status.last_raw_adc = raw_value;
    m_status.last_filtered_adc = filtered_value;
    m_status.is_calibrated = m_calib.is_calibrated;
    m_status.is_underwater = new_state;
    m_status.time_in_state_sec = (uint32_t)m_time_in_current_state;
    m_status.surface_level = surface_level;
    m_status.contrast_x10 = m_contrast_x10;
    m_status.observed_peak = m_observed_peak_adc;
    m_status.sample_delay_us = m_sample_delay_us;

    if (m_test_mode && m_status_notify) {
        m_status_notify(m_status);
    }

#if ENABLE_SWS_LOG
    // === 12. PERSISTENT LOG ===
    if (m_sws_logger) {
        SWSLogEntry sws_entry = {};
        service_set_log_header_time(sws_entry.header, service_current_time());
        sws_entry.header.log_type = LOG_UNDERWATER;
        sws_entry.header.payload_size = sizeof(SWSLogEntry) - sizeof(LogHeader);
        sws_entry.raw_adc = raw_value;
        sws_entry.filtered_adc = filtered_value;
        sws_entry.threshold = m_calib.threshold_current;
        sws_entry.hysteresis = m_calib.hysteresis_value;
        sws_entry.air = m_calib.threshold_air;
        sws_entry.water = m_calib.threshold_water;
        sws_entry.calibrated = m_calib.is_calibrated ? 1 : 0;
        sws_entry.underwater = new_state ? 1 : 0;
        sws_entry.time_in_state = (uint16_t)m_time_in_current_state;
        sws_entry.surface_level = surface_level;
        sws_entry.contrast_x10 = m_contrast_x10;
        sws_entry.observed_peak = m_observed_peak_adc;
        sws_entry.sample_delay_us = (uint16_t)m_sample_delay_us;
        m_sws_logger->write(&sws_entry);
    }
#endif

    // === 13. GUIDED CALIBRATION STATE MACHINE ===
    if (m_calib_phase != CalibPhase::IDLE) {
        switch (m_calib_phase) {
        case CalibPhase::AIR_WAITING:
            // Waiting for stable readings in air before sampling
            if (m_calib_prev_value > 0 &&
                ((raw_value > m_calib_prev_value) ?
                 (raw_value - m_calib_prev_value) : (m_calib_prev_value - raw_value))
                < CALIB_STABILITY_TOLERANCE) {
                m_calib_stable_count++;
            } else {
                m_calib_stable_count = 0;
            }
            m_calib_prev_value = raw_value;
            if (m_calib_stable_count >= CALIB_STABILITY_THRESHOLD) {
                m_calib_phase = CalibPhase::AIR_SAMPLING;
                m_calib_sum = 0;
                m_calib_count = 0;
                if (status_led) status_led->flash(RGBLedColor::GREEN, 150);
                DEBUG_INFO("SWSAnalog: Guided calib — sampling AIR");
            }
            break;

        case CalibPhase::AIR_SAMPLING:
            m_calib_sum += raw_value;
            m_calib_count++;
            if (m_calib_count >= CALIB_NUM_SAMPLES) {
                m_calib_air_result = (uint16_t)(m_calib_sum / m_calib_count);
                if (status_led) status_led->set(RGBLedColor::GREEN);
                DEBUG_INFO("SWSAnalog: Guided calib — AIR=%u — now place in WATER",
                           m_calib_air_result);
                // Transition to water phase after brief pause
                m_calib_phase = CalibPhase::WATER_WAITING;
                m_calib_stable_count = 0;
                m_calib_prev_value = 0;
                // Short delay then switch LED
                PMU::delay_ms(1000);
                if (status_led) status_led->flash(RGBLedColor::BLUE, 500);
            }
            break;

        case CalibPhase::WATER_WAITING:
            // Wait for readings significantly above air baseline
            if (raw_value > m_calib_air_result * 2) {
                if (m_calib_prev_value > 0 &&
                    ((raw_value > m_calib_prev_value) ?
                     (raw_value - m_calib_prev_value) : (m_calib_prev_value - raw_value))
                    < CALIB_STABILITY_TOLERANCE) {
                    m_calib_stable_count++;
                } else {
                    m_calib_stable_count = 0;
                }
            } else {
                m_calib_stable_count = 0;
            }
            m_calib_prev_value = raw_value;
            if (m_calib_stable_count >= CALIB_STABILITY_THRESHOLD) {
                m_calib_phase = CalibPhase::WATER_SAMPLING;
                m_calib_sum = 0;
                m_calib_count = 0;
                if (status_led) status_led->flash(RGBLedColor::BLUE, 150);
                DEBUG_INFO("SWSAnalog: Guided calib — sampling WATER");
            }
            break;

        case CalibPhase::WATER_SAMPLING:
            m_calib_sum += raw_value;
            m_calib_count++;
            if (m_calib_count >= CALIB_NUM_SAMPLES) {
                m_calib_water_result = (uint16_t)(m_calib_sum / m_calib_count);
                DEBUG_INFO("SWSAnalog: Guided calib — WATER=%u", m_calib_water_result);

                // Validate: water must be significantly above air
                bool success = (m_calib_water_result > m_calib_air_result * 2);
                if (success) {
                    // Save to SWS.CAL (batch writes, single flash serialize)
                    m_manual_calib.write(CAL_OFFSET_HINT_WATER, (double)m_calib_water_result);
                    m_manual_calib.write(CAL_OFFSET_HINT_AIR, (double)m_calib_air_result);
                    m_manual_calib.save(true);

                    // Apply immediately
                    m_calib.threshold_air = m_calib_air_result;
                    m_calib.threshold_water = m_calib_water_result;
                    update_dynamic_threshold();
                    m_calib.is_calibrated = true;
                    m_calib.last_calibration_time = PMU::get_timestamp_ms() / 1000;
                    m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                                 sizeof(m_calib) - sizeof(m_calib.crc), nullptr);
                    save_calibration_to_flash();

                    if (status_led) status_led->flash(RGBLedColor::WHITE, 200);
                    DEBUG_INFO("SWSAnalog: Guided calib SUCCESS — air=%u water=%u thresh=%u",
                               m_calib_air_result, m_calib_water_result, m_calib.threshold_current);
                } else {
                    if (status_led) status_led->flash(RGBLedColor::RED, 200);
                    DEBUG_WARN("SWSAnalog: Guided calib FAILED — water=%u not >> air=%u",
                               m_calib_water_result, m_calib_air_result);
                }

                m_calib_phase = CalibPhase::DONE;
                if (m_calib_notify) {
                    CalibResult r = {(uint8_t)(success ? 1 : 2),
                                     m_calib_air_result, m_calib_water_result};
                    m_calib_notify(r);
                }

                // Stop test mode after short delay for LED feedback
                PMU::delay_ms(2000);
                m_test_mode = false;
                m_calib_phase = CalibPhase::IDLE;
                if (s_instance) s_instance->stop();
                if (m_on_test_stop) m_on_test_stop();
            }
            break;

        case CalibPhase::IDLE:
        case CalibPhase::DONE:
        default:
            break;
        }
    }

    return new_state;
}

