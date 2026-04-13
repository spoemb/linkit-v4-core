/**
 * @file sws_analog_detection.cpp
 * @brief SWS analog — multi-level surface detection, safety timeouts, detector_state.
 */

#include <cstdint>
#include "sws_analog_service.hpp"
#include "sws_analog_constants.hpp"
#include "debug.hpp"
#include "pmu.hpp"
#include "gpio.hpp"
#include "rgb_led.hpp"

extern RGBLed *status_led;

/// @brief Check if calibration interval has elapsed and recalibration is needed.
/// @return true if m_calib_interval_sec has passed since last calibration.
bool SWSAnalogService::should_recalibrate() const {
    if (m_calib_interval_sec == 0) return false;
    uint64_t elapsed = (PMU::get_timestamp_ms() / 1000) - m_calib.last_calibration_time;
    return (elapsed >= m_calib_interval_sec);
}

/// @brief Check safety timeout — force surface if max dive time exceeded.
/// @param current_state  Current underwater state (true=submerged).
/// @return true if timeout override forces a state change.
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

/// @brief Main detection logic — read ADC, filter, 5-level surface detection, calibrate, log.
/// @return true if underwater, false if at surface.
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
            else if (raw_value < m_calib.threshold_air * 0.5f && m_calib.threshold_air > 5000) {
                DEBUG_WARN("SWSAnalog: Coherence fail - raw=%u << air=%u, recalibrating",
                           raw_value, m_calib.threshold_air);
                calib_incoherent = true;
            }
        }

        // EC-5: Continuous coherence — require 3 consecutive samples > water×2
        // before adapting. Prevents splash/wave from corrupting water baseline.
        if (!calib_incoherent && !m_current_state && m_calib.threshold_water > 0 &&
            raw_value > m_calib.threshold_water * 2 &&
            m_time_in_current_state > 2) {
            m_coherence_high_count++;
            if (m_coherence_high_count >= 3) {
                DEBUG_WARN("SWSAnalog: Continuous coherence - raw=%u >> water=%u (%u consecutive), adapting water",
                           raw_value, m_calib.threshold_water, m_coherence_high_count);
                // Cap at observed peak to prevent runaway water baseline
                uint16_t new_water = raw_value;
                if (m_observed_peak_adc > 0 && new_water > m_observed_peak_adc)
                    new_water = m_observed_peak_adc;
                m_calib.threshold_water = new_water;
                update_dynamic_threshold();
                m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                             sizeof(m_calib) - sizeof(m_calib.crc), nullptr);
                m_coherence_high_count = 0;
                // Flash save deferred to next state transition (section 10) to avoid
                // excessive writes during rapid convergence
            }
        } else {
            m_coherence_high_count = 0;
        }

        if (calib_incoherent) {
            memset(&m_calib, 0, sizeof(m_calib));
            if (raw_value > 7000) {
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

                    uint16_t cumul_pct = m_drop_reference ? static_cast<uint16_t>(static_cast<uint32_t>(m_drop_reference - raw_value) * 100 / m_drop_reference) : 0;
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
        // EC-6: Guard against underflow if filtered_value > peak (shouldn't happen, but defensive)
        if (surface_level == 0 && m_peak_adc_since_underwater > 0 &&
            filtered_value < m_peak_adc_since_underwater &&
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
                // Guard against runaway drift toward zero:
                // 1. Block if avg itself is implausibly low (< 2× floor) — likely
                //    ADC noise or uncharged RC circuit, not real air readings.
                // 2. Block if air baseline is already close to actual readings.
                bool avg_too_low = (avg < AIR_BASELINE_FLOOR * 2);
                bool air_already_low = (m_calib.threshold_air < avg * 2);
                if (avg_too_low || air_already_low) {
                    DEBUG_TRACE("SWSAnalog: Adaptive air DOWN blocked (avg=%u air=%u floor=%u)",
                                avg, m_calib.threshold_air, AIR_BASELINE_FLOOR);
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
    //
    // EC-3: Anti-spike filter + slow decay + coherence guard.
    // - Reject isolated spikes: new peak must be within 120% of current peak (once established)
    // - Slow EMA decay (0.999) allows recovery from corrupted peaks
    // - Coherence guard: reset peak if it exceeds water baseline × 3
    {
        bool peak_updated = false;

        if (raw_value > m_observed_peak_adc) {
            // Anti-spike: accept unconditionally during first water contact (peak < water baseline)
            // or if within 120% of current established peak.
            // This allows rapid peak convergence on first immersion (air=150 → water=12000)
            // while still rejecting isolated spikes once the peak is established in water.
            bool first_water_contact = (m_observed_peak_adc < m_calib.threshold_current);
            if (m_observed_peak_adc == 0 || first_water_contact ||
                raw_value <= (uint32_t)m_observed_peak_adc * 6 / 5) {
                m_observed_peak_adc = raw_value;
                peak_updated = true;
                m_consecutive_spike_rejects = 0;
            } else {
                m_consecutive_spike_rejects++;
                if (m_consecutive_spike_rejects >= 10) {
                    // Peak is stuck far below actual readings — force reset
                    DEBUG_WARN("SWSAnalog: Peak stuck (10 consecutive rejects) raw=%u >> peak=%u, resetting peak",
                               raw_value, m_observed_peak_adc);
                    m_observed_peak_adc = raw_value;
                    m_calib.threshold_water = raw_value;
                    update_dynamic_threshold();
                    m_calib.crc = crc16_compute((const uint8_t *)&m_calib,
                                                 sizeof(m_calib) - sizeof(m_calib.crc), nullptr);
                    save_calibration_to_flash();
                    peak_updated = true;
                    m_consecutive_spike_rejects = 0;
                } else {
                    DEBUG_WARN("SWSAnalog: Peak spike rejected raw=%u >> peak=%u", raw_value, m_observed_peak_adc);
                }
            }
        } else if (m_observed_peak_adc > 0) {
            // Slow decay: EMA 0.999 allows gradual recovery from stale/corrupted peaks
            uint16_t decayed = (uint16_t)(m_observed_peak_adc * 0.999f + raw_value * 0.001f);
            if (decayed != m_observed_peak_adc) {
                m_observed_peak_adc = decayed;
                peak_updated = true;
            }
        }

        // Coherence guard: peak should not exceed water baseline × 3
        if (m_observed_peak_adc > 0 && m_calib.threshold_water > 0 &&
            m_observed_peak_adc > (uint32_t)m_calib.threshold_water * 3) {
            DEBUG_WARN("SWSAnalog: Peak incoherent peak=%u > water×3=%u, resetting",
                       m_observed_peak_adc, m_calib.threshold_water * 3);
            m_observed_peak_adc = m_calib.threshold_water;
            peak_updated = true;
        }

        if (peak_updated) {
            m_observed_peak_crc = crc16_compute((const uint8_t *)&m_observed_peak_adc,
                                                 sizeof(m_observed_peak_adc), nullptr);
        }
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

        // EC-2: Always enforce a lockout after L-override to prevent oscillation.
        // Use configured min surface time if set, otherwise fall back to default lockout.
        {
            uint32_t lockout = (m_min_surface_time_sec > 0) ?
                m_min_surface_time_sec : SURFACE_LOCKOUT_DURATION_SEC;
            m_surface_lockout_remaining = lockout;
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

        // Persist calibration to flash on state transitions — debounced to reduce flash wear
        save_calibration_to_flash_debounced();

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
    if (m_calib_phase != CalibPhase::IDLE && m_calib_phase != CalibPhase::DONE) {
        m_calib_timeout_ticks++;
        if (m_calib_timeout_ticks >= GUIDED_CALIB_TIMEOUT_TICKS) {
            DEBUG_WARN("SWSAnalog: Guided calibration TIMEOUT after %u ticks — cancelling",
                       m_calib_timeout_ticks);
            cancel_guided_calibration();
            if (m_calib_notify) {
                CalibResult r = {0, 0, 0};  // status 0 = timeout
                m_calib_notify(r);
            }
        }
    }
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
                // EC-4: Non-blocking pause — transition via state machine tick
                m_calib_phase = CalibPhase::AIR_DONE_PAUSE;
                m_calib_stable_count = 0;
                m_calib_prev_value = 0;
            }
            break;

        case CalibPhase::AIR_DONE_PAUSE:
            // EC-4: 1 tick pause (1s via service_next_schedule_in_ms) replaces PMU::delay_ms(1000)
            m_calib_phase = CalibPhase::WATER_WAITING;
            if (status_led) status_led->flash(RGBLedColor::BLUE, 500);
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

                m_calib_phase = CalibPhase::COMPLETION_PAUSE;
                m_calib_count = 0;  // Reuse as tick counter for pause
                if (m_calib_notify) {
                    CalibResult r = {(uint8_t)(success ? 1 : 2),
                                     m_calib_air_result, m_calib_water_result};
                    m_calib_notify(r);
                }
            }
            break;

        case CalibPhase::COMPLETION_PAUSE:
            // EC-4: Non-blocking 2s pause for LED feedback (2 ticks × 1s each)
            m_calib_count++;
            if (m_calib_count >= 2) {
                m_calib_phase = CalibPhase::DONE;
                m_test_mode = false;
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

