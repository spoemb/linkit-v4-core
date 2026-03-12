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
#define ADC_REFERENCE_V 0.6f
#define ADC_GAIN_1_6 (1.0f/6.0f)
#define ADC_INVALID_MIN 0
#define ADC_INVALID_MAX 16383          // 14-bit ADC

// Default configuration values
#define DEFAULT_THRESHOLD_MIN 0
#define DEFAULT_THRESHOLD_MAX 8000
#define DEFAULT_HYSTERESIS_PERCENT 4
#define DEFAULT_CALIB_INTERVAL_SEC 3600
#define DEFAULT_MAX_DIVE_TIME_SEC 7200
#define DEFAULT_MIN_SURFACE_TIME_SEC 10

// Detection tuning
#define DEFAULT_THRESHOLD_RATIO_PERCENT 35
#define DEFAULT_ALPHA_PERCENT 19
#define DEFAULT_MAX_SAMPLES 1
#define DEFAULT_MIN_DRY_SAMPLES 1

// Water baseline protection
#define ABSOLUTE_MIN_WATER_ADC 2000
#define MIN_WATER_AIR_RATIO 3

// Surface baseline adaptation
#define SURFACE_ADAPT_THRESHOLD 1.3f
#define MIN_SURFACE_TIME_FOR_ADAPT 10

// ═══════════════════════════════════════════════════════
//  MULTI-LEVEL SURFACE DETECTION
//
//  All levels require: underwater ≥ 1s AND proximity guard OK
//
//  Level 1 - INSTANT (1 sample)
//    Drop > 5% from recent peak (decaying) → immediate surface
//    Use case: clean/moderate electrode, sharp water exit
//
//  Level 2 - FAST (2 samples)
//    2 consecutive raw drops, cumulative > 3% → surface
//    Use case: gradual exit where individual drops < 5%
//
//  Level 3 - TREND (3+ MA3 decreases)
//    MA3 decreasing 3+ times, total MA3 drop > 8% → surface
//    Use case: heavy biofouling, very slow drying, noisy signal
//
//  Level 4 - ABSOLUTE (variable)
//    filtered < water_baseline × 85% → surface
//    Use case: reliable when water baseline is well-calibrated
//
//  Level 5 - SAFETY NET (>10s underwater)
//    Drop from dive peak > 15% → surface
//    Use case: fallback when baselines have drifted
// ═══════════════════════════════════════════════════════

// Level 1
#define L1_DROP_PERCENT 5              // Drop from recent peak threshold (%)

// Level 2
#define L2_DROP_PERCENT 3              // Cumulative 2-sample raw drop (%)
#define L2_MIN_CONSECUTIVE 2           // Minimum consecutive raw drops

// Level 3
#define L3_MIN_CONSECUTIVE 3           // Consecutive MA3 decreases
#define L3_DROP_PERCENT 5              // Total MA3 drop from trend start (%)

// Level 4
#define L4_DROP_PERCENT 15             // Drop from water baseline (%)

// Level 5
#define L5_DROP_PERCENT 15             // Cumulative drop from peak (%)
#define L5_MIN_TIME_SEC 10             // Minimum time underwater before L5

// Safety
#define OVERRIDE_MIN_TIME_SEC 1        // Minimum underwater time before any override
#define SURFACE_LOCKOUT_DURATION_SEC 30

// Proximity guard: L-overrides blocked if value > water_baseline * this%
// Prevents false surface detection from normal underwater ADC drift
#define PROXIMITY_GUARD_PERCENT 95     // Must drop below 95% of peak to allow override

// Initialize static members
SWSAnalogService::CalibrationData SWSAnalogService::m_calib;
uint16_t SWSAnalogService::m_observed_peak_adc;
uint16_t SWSAnalogService::m_observed_peak_crc;
SWSAnalogService::Status SWSAnalogService::m_status = {};
SWSAnalogService* SWSAnalogService::s_instance = nullptr;
bool SWSAnalogService::m_test_mode = false;
std::function<void(const SWSAnalogService::Status&)> SWSAnalogService::m_status_notify;

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
    m_test_mode = false;
}

bool SWSAnalogService::is_test_running() {
    return m_test_mode;
}

void SWSAnalogService::set_status_notify(std::function<void(const Status&)> fn) {
    m_status_notify = fn;
}

void SWSAnalogService::clear_status_notify() {
    m_status_notify = nullptr;
}

// SAADC event handler (required but not used for blocking mode)
static void nrfx_saadc_event_handler_sws(nrfx_saadc_evt_t const *p_event)
{
    (void)p_event;
}

void SWSAnalogService::service_init() {
    UWDetectorService::service_init();

    // Load configuration
    m_threshold_min = service_read_param<unsigned int>(ParamID::SWS_ANALOG_THRESHOLD_MIN);
    m_threshold_max = service_read_param<unsigned int>(ParamID::SWS_ANALOG_THRESHOLD_MAX);
    m_hysteresis_percent = service_read_param<unsigned int>(ParamID::SWS_ANALOG_HYSTERESIS);
    m_calib_interval_sec = service_read_param<unsigned int>(ParamID::SWS_ANALOG_CALIB_INTERVAL);
    m_max_dive_time_sec = service_read_param<unsigned int>(ParamID::UW_MAX_DIVE_TIME);
    m_min_surface_time_sec = service_read_param<unsigned int>(ParamID::UW_MIN_SURFACE_TIME);

    if (m_threshold_min >= m_threshold_max) {
        m_threshold_min = DEFAULT_THRESHOLD_MIN;
    }
    if (m_threshold_max > ADC_INVALID_MAX || m_threshold_max <= m_threshold_min) {
        m_threshold_max = DEFAULT_THRESHOLD_MAX;
    }
    if (m_hysteresis_percent > 50) {
        m_hysteresis_percent = DEFAULT_HYSTERESIS_PERCENT;
    }

    m_threshold_ratio_percent = DEFAULT_THRESHOLD_RATIO_PERCENT;
    m_alpha_percent = DEFAULT_ALPHA_PERCENT;
    m_max_samples = DEFAULT_MAX_SAMPLES;
    m_min_dry_samples = DEFAULT_MIN_DRY_SAMPLES;

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
        DEBUG_INFO("SWSAnalog: Calibration invalid | clearing and recalibrating");
        memset(&m_calib, 0, sizeof(m_calib));
        calibrate_air_baseline();
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

    // First-sample coherence check
    m_first_sample_done = false;

    DEBUG_INFO("SWSAnalog: Init - min=%u max=%u hyst=%u%% ratio=%u%%",
               m_threshold_min, m_threshold_max, m_hysteresis_percent, m_threshold_ratio_percent);
}

bool SWSAnalogService::service_is_enabled() {
    if (m_test_mode) return true;
    bool enabled = service_read_param<bool>(ParamID::UNDERWATER_EN);
    BaseUnderwaterDetectSource src = service_read_param<BaseUnderwaterDetectSource>(ParamID::UNDERWATER_DETECT_SOURCE);
    return enabled && (src == BaseUnderwaterDetectSource::SWS || src == BaseUnderwaterDetectSource::SWS_GNSS);
}

uint16_t SWSAnalogService::read_analog_sws() {
    nrf_saadc_value_t raw = 0;

    // === FAST-CHARGE DISCRIMINATION ===
    // The 100nF cap at the ADC pin charges through water resistance:
    //   Water (R≈10kΩ):  τ = 1ms  → at 2ms: 86% charged → ~2700 ADC
    //   Wet film (R≈50-100kΩ): τ = 5-10ms → at 2ms: 18-33% → ~570-1000 ADC
    //   Air (R=∞):       τ = ∞   → at 2ms: 0% → ~0 ADC
    //
    // Cap must be discharged before reading. Between samples (1s interval),
    // the 1MΩ pull-down discharges the cap: τ=100ms, 5τ=500ms → fully discharged.

    // Enable SWS electrode excitation
    GPIOPins::set(SWS_ENABLE_PIN);

    // Short delay: exploits RC time constant difference between
    // active water contact (fast charge) and residual wet film (slow charge).
    // This is the KEY to discriminating wet electrode from submerged.
    PMU::delay_ms(m_enable_sample_delay);

    nrfx_saadc_init(&BSP::ADC_Inits.config, nrfx_saadc_event_handler_sws);
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
        if (m_calib.threshold_air < m_threshold_min)
            m_calib.threshold_air = m_threshold_min;
    } else {
        // Normal: started in air
        m_calib.threshold_air = avg;
        if (m_calib.threshold_water == 0 || m_calib.threshold_water <= m_calib.threshold_air) {
            m_calib.threshold_water = m_calib.threshold_air * 3;
            // Cap at observed peak if available (prevents overshoot with wet electrodes)
            if (m_observed_peak_adc > 0 && m_calib.threshold_water > m_observed_peak_adc) {
                m_calib.threshold_water = m_observed_peak_adc;
                DEBUG_INFO("SWSAnalog: Water capped at observed peak %u", m_observed_peak_adc);
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
}

void SWSAnalogService::calibrate_water_baseline(uint16_t value) {
    float alpha = m_alpha_percent / 100.0f;
    if (!is_value_valid(value)) return;

    uint16_t threshold_with_margin = m_calib.threshold_current + m_calib.hysteresis_value;
    uint16_t min_expected_water = (uint16_t)(m_calib.threshold_water * 0.85f);

    // Only apply air ratio guard when air baseline is reasonable (< 1000 ADC).
    // If air is high (e.g. from corrupted calibration), skip this check to allow recovery.
    bool air_ratio_ok = (m_calib.threshold_air >= 1000) ||
                        (value >= (uint16_t)(m_calib.threshold_air * MIN_WATER_AIR_RATIO));

    bool ok = (value > threshold_with_margin) &&
              (value >= ABSOLUTE_MIN_WATER_ADC) &&
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
        DEBUG_WARN("SWSAnalog: Max dive time %us | forcing surface", m_time_in_current_state);
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

    // === 1b. FIRST-SAMPLE COHERENCE CHECK ===
    // If stored calibration says air=1000 but we read 4800, the calibration
    // is stale or was done in the wrong medium. Force recalibration.
    if (!m_first_sample_done) {
        m_first_sample_done = true;
        bool calib_incoherent = false;

        // Case 1: Stored air is low, but actual reading is very high → we're in water
        //         and stored threshold is way below current reading → can't detect water
        if (raw_value > m_calib.threshold_current + m_calib.hysteresis_value * 3 &&
            raw_value > m_calib.threshold_water * 1.3f) {
            DEBUG_WARN("SWSAnalog: Coherence fail - raw=%u >> water=%u, recalibrating",
                       raw_value, m_calib.threshold_water);
            calib_incoherent = true;
        }
        // Case 2: Stored air is high (calibrated in water), but actual reading is low → in air
        //         with inflated thresholds
        else if (raw_value < m_calib.threshold_air * 0.5f && m_calib.threshold_air > 2000) {
            DEBUG_WARN("SWSAnalog: Coherence fail - raw=%u << air=%u, recalibrating",
                       raw_value, m_calib.threshold_air);
            calib_incoherent = true;
        }

        if (calib_incoherent) {
            memset(&m_calib, 0, sizeof(m_calib));
            // Use current reading to decide: high = water, low = air
            if (raw_value > 2500) {
                m_calib.threshold_water = raw_value;
                m_calib.threshold_air = raw_value / 3;
            } else {
                m_calib.threshold_air = raw_value;
                m_calib.threshold_water = raw_value * 3;
                // Cap at observed peak if available
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
        }
    }

    // === 1c. UPDATE OBSERVED PEAK ADC ===
    // Track the highest ADC value actually seen. Used to cap water baseline
    // estimates so threshold never exceeds what the ADC actually produces.
    if (raw_value > m_observed_peak_adc) {
        m_observed_peak_adc = raw_value;
        m_observed_peak_crc = crc16_compute((const uint8_t *)&m_observed_peak_adc,
                                             sizeof(m_observed_peak_adc), nullptr);
    }

    // Save prev_raw BEFORE filtering overwrites history
    uint16_t prev_raw = m_prev_raw;
    m_prev_raw = raw_value;

    uint16_t filtered_value = add_to_history_and_filter(raw_value);

    // === 2. TIME TRACKING ===
    bool timeout_override = check_safety_timeouts(m_current_state);

    // === 3. LEVEL 1 & 2: FAST RAW DROP DETECTION ===
    // Uses raw values (not filtered) for maximum speed.
    // The filter dampens transitions — we need to see the instant change.
    uint8_t surface_level = 0;  // 0 = no override, 1-5 = detection level

    // Proximity guard: if filtered value is still very close to water level,
    // any drops are just normal underwater noise/drift, NOT a real surface event.
    // Use max(water_baseline, peak_during_dive) as reference to handle EMA drift.
    uint16_t proximity_ref = m_calib.threshold_water;
    if (m_peak_adc_since_underwater > proximity_ref)
        proximity_ref = m_peak_adc_since_underwater;
    bool proximity_ok = (proximity_ref == 0) ||
        (filtered_value < (uint16_t)(proximity_ref * PROXIMITY_GUARD_PERCENT / 100.0f));

    if (m_current_state && prev_raw > 0 && m_time_in_current_state >= OVERRIDE_MIN_TIME_SEC
        && proximity_ok) {

        // LEVEL 1: Drop from recent peak (no consecutive requirement)
        // Uses m_recent_peak which decays with drift, so only a SUDDEN drop triggers.
        // Drift: peak decays 5%/sample toward reading → no false trigger.
        // Real exit: reading drops 5%+ instantly → triggers immediately.
        if (m_recent_peak > 0 && raw_value < m_recent_peak) {
            uint16_t peak_drop_pct = (uint16_t)((uint32_t)(m_recent_peak - raw_value) * 100 / m_recent_peak);
            if (peak_drop_pct >= L1_DROP_PERCENT) {
                surface_level = 1;
            }
        }

        // LEVEL 2: Two consecutive raw drops with cumulative threshold
        if (surface_level == 0) {
            if (raw_value < prev_raw) {
                if (m_consecutive_raw_drops == 0) {
                    m_drop_reference = prev_raw;
                }
                m_consecutive_raw_drops++;

                uint16_t cumul_pct = (uint16_t)((uint32_t)(m_drop_reference - raw_value) * 100 / m_drop_reference);
                if (m_consecutive_raw_drops >= L2_MIN_CONSECUTIVE && cumul_pct >= L2_DROP_PERCENT) {
                    surface_level = 2;
                }
            } else {
                m_consecutive_raw_drops = 0;
            }
        }
    } else if (!m_current_state) {
        m_consecutive_raw_drops = 0;
    }

    // === 4. LEVEL 3: TREND MA3 ===
    // Computes 3-sample moving average. If each new MA3 is lower than the
    // previous one → electrode is drying → surface. Smooths noise while
    // preserving trend information. Optimal for slow biofouling transitions.
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

        // Recent peak: tracks the last high point, slowly decays to follow drift.
        // If reading > recent_peak → update immediately (new peak)
        // If reading < recent_peak → decay toward reading (alpha=5% per sample)
        // This prevents underwater drift from accumulating into a false L1 trigger.
        if (raw_value > m_recent_peak || m_recent_peak == 0) {
            m_recent_peak = raw_value;
        } else {
            // Decay: recent_peak moves 5% toward current reading each sample
            m_recent_peak = (uint16_t)(m_recent_peak * 0.95f + raw_value * 0.05f);
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

    // === 6. SURFACE BASELINE TRACKING ===
    if (!m_current_state && m_time_in_current_state > MIN_SURFACE_TIME_FOR_ADAPT) {
        // Only accept readings below threshold (safety: prevents underwater
        // readings from corrupting air baseline when detection is wrong)
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
            }
            else if (avg < (uint16_t)(m_calib.threshold_air * 0.70f)) {
                // Downward adaptation: air was too high (wet electrode calibration)
                // Adapt faster (20%) since readings are far below stored air
                uint16_t old = m_calib.threshold_air;
                uint16_t new_air = (uint16_t)(m_calib.threshold_air * 0.80f + avg * 0.20f);
                // Enforce minimum air baseline to prevent collapse over long deployment
                if (new_air < 5) new_air = 5;
                m_calib.threshold_air = new_air;
                update_dynamic_threshold();
                m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                             sizeof(m_calib) - sizeof(m_calib.crc), nullptr);
                m_surface_readings_count = 0;
                m_surface_readings_idx = 0;
                DEBUG_INFO("SWSAnalog: Adaptive air DOWN %u -> %u", old, m_calib.threshold_air);
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
    // Prevent underflow deadlock: with very low ADC values (e.g. air=5, water=15),
    // hysteresis can exceed the air-to-threshold gap, making threshold_low ≤ air baseline.
    // This means no ADC reading at air level can ever cross below threshold_low.
    // Fix: ensure threshold_low is always above the air baseline.
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
            if (m_consecutive_samples >= m_min_dry_samples)
                new_state = false;
        } else {
            new_state = false;
            m_consecutive_samples = 0;
        }
    } else {
        // Hysteresis zone — maintain current state
        m_consecutive_samples = 0;
    }

    // Apply multi-level surface override
    if (surface_level > 0 && m_current_state && new_state) {
        new_state = false;
        m_consecutive_samples = 0;

        // CRITICAL: Recalibrate air baseline to current reading.
        // When electrode exits water, it's WET and reads much higher than dry air.
        // Without this, threshold stays too low and next sample re-triggers underwater.
        // GUARD: Only recalibrate if it won't destroy contrast (new air must be < 80% of water)
        uint16_t old_air = m_calib.threshold_air;
        uint16_t max_air_for_contrast = (uint16_t)(m_calib.threshold_water * 0.80f);
        if (filtered_value > m_calib.threshold_air * 2 &&
            filtered_value < max_air_for_contrast) {
            // Filtered value creates reasonable contrast with water → good surface estimate
            m_calib.threshold_air = filtered_value;
            update_dynamic_threshold();
            m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                         sizeof(m_calib) - sizeof(m_calib.crc), nullptr);
            DEBUG_INFO("SWSAnalog: SURFACE L%u | air recalib %u -> %u | thresh=%u",
                       surface_level, old_air, m_calib.threshold_air, m_calib.threshold_current);
        }

        // Enforce minimum surface time to prevent immediate re-trigger
        if (m_min_surface_time_sec > 0) {
            m_surface_lockout_remaining = m_min_surface_time_sec;
        }

        DEBUG_INFO("SWSAnalog: SURFACE L%u | raw=%u filt=%u ma3=%u air=%u water=%u lockout=%us",
                   surface_level, raw_value, filtered_value, current_ma3,
                   m_calib.threshold_air, m_calib.threshold_water, m_surface_lockout_remaining);
    }

    // === 8. MAX DIVE TIMEOUT ===
    if (timeout_override) {
        new_state = false;
        // Do NOT recalibrate air from underwater values — it corrupts calibration.
        // Just force surface and let normal surface baseline tracking fix things.
        m_surface_lockout_remaining = SURFACE_LOCKOUT_DURATION_SEC;
        DEBUG_WARN("SWSAnalog: Max dive timeout | air=%u water=%u thresh=%u",
                   m_calib.threshold_air, m_calib.threshold_water, m_calib.threshold_current);
    }

    // === 9. SURFACE LOCKOUT ===
    if (m_surface_lockout_remaining > 0) {
        m_surface_lockout_remaining--;
        if (new_state) new_state = false;
    }

    // === 10. STATE CHANGE ===
    if (new_state != m_current_state) {
        m_last_state_change_time = PMU::get_timestamp_ms() / 1000;
        m_time_in_current_state = 0;
        m_consecutive_samples = 0;
        m_peak_adc_since_underwater = 0;
        m_recent_peak = 0;

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
    m_status.contrast_x10 = (m_calib.threshold_air > 0) ?
        (uint16_t)((float)m_calib.threshold_water / (float)m_calib.threshold_air * 10.0f) : 0;
    m_status.observed_peak = m_observed_peak_adc;

    if (m_test_mode && m_status_notify) {
        m_status_notify(m_status);
    }

    return new_state;
}

