/**
 * @file argos_tx_service.cpp
 * @brief Argos TX service — scheduling, burst preparation, TX event handling.
 */

#include <climits>
#include <algorithm>

#include "argos_tx_service.hpp"
#include "gps.hpp"
#include "messages.hpp"
#include "timeutils.hpp"
#include "binascii.hpp"
#include "debug.hpp"
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern GPSDevice *gps_device;
#if defined(BOARD_RSPB) && ENABLE_MORTALITY_SENSOR
#include "mortality_service.hpp"
extern MortalityService *mortality_service;
#endif


/// @brief Construct Argos TX service with a KineisDevice backend (SMD/KIM2/LoRa).
ArgosTxService::ArgosTxService(KineisDevice& device) : Service(ServiceIdentifier::ARGOS_TX, "ARGOSTX"),
	m_kineis(device)
{
}

/// @brief Init: subscribe to KineisDevice events, load config, set TCXO/LPM.
void ArgosTxService::service_init() {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);

	//@TODO => Get ID & ADDR ? m_artic.set_device_identifier(argos_config.argos_id);

	m_kineis.subscribe(*this);
	m_kineis.set_tcxo_warmup_time(argos_config.argos_tcxo_warmup_time);

	// Set SMD LPM mode from configuration (written to SMD at every boot via SPI)
	uint8_t lpm = static_cast<uint8_t>(configuration_store->read_param<unsigned int>(ParamID::SMD_LPM_MODE));
	m_kineis.set_lpm_mode(lpm);

	// Warn if SURFACING_BURST mode is configured without underwater detection
	if (argos_config.mode == BaseArgosMode::SURFACING_BURST && !argos_config.underwater_en) {
		DEBUG_WARN("ArgosTxService: SURFACING_BURST mode requires UNDERWATER_EN=1 — burst will not trigger without SWS");
	}

	DEBUG_TRACE("ArgosTxService::service_init DEBUG ARGOS ID %d", argos_config.argos_id);
	m_sched.reset(argos_config.argos_id); // TODO verify if already set at this moment
	m_depth_pile_manager.clear();
	m_is_first_tx = true;
	m_is_tx_pending = false;
	m_tcxo_skip_on_next_tx = false;
	m_session_tx_count = 0;
	m_is_surfacing_burst = false;
	m_doppler_burst_count = 0;
	m_has_gnss_fix_since_surfacing = false;
	m_first_gnss_tx_sent = false;
	m_last_tx_had_gps = false;
	m_cooldown_armed = false;
	m_last_preconfig_mod = KineisModulation::LDA2;
	m_modulation_preconfig.reset();
	m_consecutive_device_errors = 0;

	// Set the idle timeout depending on the configuration settings
	// i) In certification mode, keep powered on for 10 seconds in idle
	// ii) In normal operation, keep powered on for 1 second in idle
// 	if (argos_config.cert_tx_enable)
// 		m_artic.set_idle_timeout(10000);
// 	else
// 		m_artic.set_idle_timeout(1000);
}

/// @brief Terminate: power off device immediately.
void ArgosTxService::service_term() {
	m_kineis.unsubscribe(*this);
}

/// @brief Enabled if Argos mode is not OFF (respects cert TX override).
bool ArgosTxService::service_is_enabled() {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	return (argos_config.mode != BaseArgosMode::OFF || argos_config.cert_tx_enable);
}

/// @brief Compute next TX schedule based on mode (cert/legacy/duty/prepass/surfacing).
/// @return Delay in ms until next TX, or SCHEDULE_DISABLED if TX is off.
unsigned int ArgosTxService::service_next_schedule_in_ms() {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	std::time_t now = service_current_time();

	DEBUG_TRACE("ArgosTxService::service_next_schedule_in_ms");

	// Critical battery check: immediate powerdown, no transmission
	if (argos_config.is_lb) {
		service_update_battery();
		unsigned int critical_level = configuration_store->read_param<unsigned int>(ParamID::LB_CRITICAL_THRESH);
		unsigned int current_soc = service_get_level();
		if (current_soc < critical_level) {
			DEBUG_INFO("ArgosTxService: CRITICAL battery SOC %u%% < %u%% - shutdown",
			           current_soc, critical_level);
			configuration_store->save_params();
			PMU::powerdown();
			return Service::SCHEDULE_DISABLED;
		}
	}

	// if (argos_config.cert_tx_enable) {
	// 	m_scheduled_task = [this]() { process_certification_burst(); };
	// 	unsigned int delta = m_is_first_tx ? 0 : argos_config.cert_tx_repetition * 1000;
	// 	m_sched.schedule_at(now + delta);
	// 	return delta;
	// } else {
		if (argos_config.mode == BaseArgosMode::OFF) {
			return Service::SCHEDULE_DISABLED;
		} else if (argos_config.mode == BaseArgosMode::DOPPLER) {
			m_scheduled_task = [this]() { process_doppler_burst(); };
			m_scheduled_mode = argos_config.adaptive_modulation ? KineisModulation::VLDA4 : KineisModulation::LDA2;
			return m_sched.schedule_legacy(argos_config, now);
		} else if (argos_config.mode == BaseArgosMode::SURFACING_BURST) {
			m_scheduled_mode = KineisModulation::LDA2;

			// Phase 1: Doppler burst with progressive intervals until GNSS fix
			if (m_is_surfacing_burst && !m_has_gnss_fix_since_surfacing) {
				// Check max Doppler message limit (0 = unlimited)
				unsigned int max_msg = configuration_store->read_param<unsigned int>(ParamID::SURFACING_BURST_MAX_MSG);
				if (max_msg > 0 && m_doppler_burst_count >= max_msg) {
					DEBUG_INFO("ArgosTxService::SURFACING_BURST: Doppler limit reached (%u/%u), stopping burst", m_doppler_burst_count, max_msg);
					// Arm cooldown if trigger mode is END_OF_DOPPLER (max messages reached without fix)
					unsigned int trigger = configuration_store->read_param<unsigned int>(ParamID::COOLDOWN_TRIGGER_MODE);
					if (trigger == (unsigned int)BaseCooldownTrigger::END_OF_DOPPLER && !m_cooldown_armed) {
						m_cooldown_armed = true;
						DEBUG_INFO("ArgosTxService: cooldown armed (END_OF_DOPPLER, max msg)");
					}
					m_is_surfacing_burst = false;
					m_awaiting_surfacing = true;
					m_first_gnss_tx_sent = false;
					return Service::SCHEDULE_DISABLED;
				}

				m_scheduled_task = [this]() { process_doppler_burst(); };

				// First message is immediate (0 delay)
				if (m_doppler_burst_count == 0) {
					DEBUG_INFO("ArgosTxService::SURFACING_BURST: Doppler #%u (immediate)", m_doppler_burst_count + 1);
					m_sched.schedule_at(now);
					return 0;
				}

				// Progressive interval: init + (count-1) * step, capped at max
				unsigned int interval_s = argos_config.surfacing_burst_init_s +
					(m_doppler_burst_count - 1) * argos_config.surfacing_burst_step_s;
				if (interval_s > argos_config.surfacing_burst_max_s)
					interval_s = argos_config.surfacing_burst_max_s;

				DEBUG_INFO("ArgosTxService::SURFACING_BURST: Doppler #%u in %u s", m_doppler_burst_count + 1, interval_s);
				m_sched.schedule_at(now + interval_s);
				return interval_s * 1000;
			}

			// Phase 2: GNSS fix available — switch to normal GNSS TX with tx_interval_s
			if (m_has_gnss_fix_since_surfacing) {
				if (!service_is_time_known()) {
					DEBUG_TRACE("ArgosTxService::SURFACING_BURST: GNSS phase but RTC not set");
					return Service::SCHEDULE_DISABLED;
				}
				if (m_depth_pile_manager.eligible() == 0) {
					DEBUG_TRACE("ArgosTxService::SURFACING_BURST: GNSS phase but no eligible entries");
					m_is_surfacing_burst = false;
					m_awaiting_surfacing = true;
					m_has_gnss_fix_since_surfacing = false;
					m_first_gnss_tx_sent = false;
					return Service::SCHEDULE_DISABLED;
				}

				if (argos_config.sensor_tx_enable) {
					m_scheduled_task = [this]() { process_sensor_burst(); };
				} else {
					m_scheduled_task = [this]() { process_gnss_burst(); };
				}

				// First GNSS TX is immediate after fix, then use tx_interval_s
				// Note: m_first_gnss_tx_sent is set in service_initiate(), not here,
				// because scheduling can be called while a TX is still in progress.
				if (!m_first_gnss_tx_sent) {
					DEBUG_INFO("ArgosTxService::SURFACING_BURST: GNSS TX #1 (immediate after fix)");
					m_sched.schedule_at(now);
					return 0;
				}

				DEBUG_INFO("ArgosTxService::SURFACING_BURST: GNSS TX in %u s", argos_config.tx_interval_s);
				return m_sched.schedule_legacy(argos_config, now);
			}

			// Burst ended — wait for next surfacing event
			if (m_awaiting_surfacing) {
				return Service::SCHEDULE_DISABLED;
			}

			// Not yet surfaced (boot): send Doppler at legacy rate
			m_scheduled_task = [this]() { process_doppler_burst(); };
			return m_sched.schedule_legacy(argos_config, now);
		} else {
			if (!argos_config.gnss_en) {
				m_scheduled_task = [this]() { process_doppler_burst(); };
				if (argos_config.mode == BaseArgosMode::DUTY_CYCLE) {
					m_scheduled_mode = argos_config.adaptive_modulation ? KineisModulation::VLDA4 : KineisModulation::LDA2;
					return m_sched.schedule_duty_cycle(argos_config, now);
				}
				if (argos_config.mode == BaseArgosMode::LEGACY) {
					m_scheduled_mode = argos_config.adaptive_modulation ? KineisModulation::VLDA4 : KineisModulation::LDA2;
					return m_sched.schedule_legacy(argos_config, now);
				}
				return Service::SCHEDULE_DISABLED;
			} else if (!service_is_time_known()) {
				DEBUG_TRACE("ArgosTxService::service_next_schedule_in_ms: can't schedule as GNSS_EN and RTC not set");
				return Service::SCHEDULE_DISABLED;
			}
			if (m_is_first_tx && argos_config.time_sync_burst_en) {
				m_scheduled_mode = KineisModulation::LDA2;
				m_scheduled_task = [this]() { process_time_sync_burst(); };
				m_sched.schedule_at(now);
				return 0;
			}
			if (m_depth_pile_manager.eligible() == 0) {
				DEBUG_TRACE("ArgosTxService::service_next_schedule_in_ms: depth pile has no eligible entries");
				return Service::SCHEDULE_DISABLED;
			}
			if (argos_config.mode == BaseArgosMode::DUTY_CYCLE) {
				m_scheduled_mode = KineisModulation::LDA2;
#ifdef BOARD_RSPB
				if (argos_config.adaptive_modulation && argos_config.sensor_tx_enable) {
					unsigned int pkt_fmt = configuration_store->read_param<unsigned int>(ParamID::RSPB_PACKET_FORMAT);
					if (pkt_fmt == 1) m_scheduled_mode = KineisModulation::LDK;
				}
#endif
				if (argos_config.sensor_tx_enable) {
					m_scheduled_task = [this]() { process_sensor_burst(); };
				} else {
					m_scheduled_task = [this]() { process_gnss_burst(); };
				}
				return m_sched.schedule_duty_cycle(argos_config, now);
			}
			if (argos_config.mode == BaseArgosMode::LEGACY) {
				m_scheduled_mode = KineisModulation::LDA2;
#ifdef BOARD_RSPB
				if (argos_config.adaptive_modulation && argos_config.sensor_tx_enable) {
					unsigned int pkt_fmt = configuration_store->read_param<unsigned int>(ParamID::RSPB_PACKET_FORMAT);
					if (pkt_fmt == 1) m_scheduled_mode = KineisModulation::LDK;
				}
#endif
				if (argos_config.sensor_tx_enable) {
					m_scheduled_task = [this]() { process_sensor_burst(); };
				} else {
					m_scheduled_task = [this]() { process_gnss_burst(); };
				}
				return m_sched.schedule_legacy(argos_config, now);
			}
			if (argos_config.mode == BaseArgosMode::PASS_PREDICTION) {
				m_scheduled_mode = KineisModulation::LDA2;
				if (argos_config.sensor_tx_enable) {
					m_scheduled_task = [this]() { process_sensor_burst(); };
				} else {
					m_scheduled_task = [this]() { process_gnss_burst(); };
				}
				BasePassPredict& pass_predict = configuration_store->read_pass_predict();
				unsigned int schedule = m_sched.schedule_prepass(argos_config, pass_predict, m_scheduled_mode, now);
				if (schedule == ArgosTxScheduler::INVALID_SCHEDULE) {
					// No pass found — fall back to duty cycle to keep TX alive
					DEBUG_WARN("ArgosTxService: PASS_PREDICTION returned no pass, falling back to DUTY_CYCLE");
					return m_sched.schedule_duty_cycle(argos_config, now);
				}
				return schedule;
			}
		}
	// }

	return Service::SCHEDULE_DISABLED;
}

/// @brief Execute scheduled TX — run the prepared burst task (cert/gnss/sensor/doppler).
/// Called by ServiceManager when the scheduled time arrives.
void ArgosTxService::service_initiate() {
	DEBUG_TRACE("ArgosTxService::service_initiate");

	// Skip TX if device has failed too many consecutive times this session.
	// This prevents battery drain from persistent hardware failures (e.g. SPI breakdown).
	if (m_consecutive_device_errors >= DEVICE_ERROR_MAX_CONSECUTIVE) {
		DEBUG_WARN("ArgosTxService::service_initiate: skipping TX — %u consecutive device errors, suspending until next session",
		           m_consecutive_device_errors);
		service_complete(nullptr, nullptr, false);  // complete without rescheduling
		return;
	}

	m_is_first_tx = false;
	m_is_tx_pending = true;

	// Skip TCXO warmup on first TX after surfacing from underwater
	if (m_tcxo_skip_on_next_tx) {
		DEBUG_TRACE("ArgosTxService::service_initiate: TCXO warmup skipped (first TX after submerge)");
		m_kineis.set_tcxo_warmup_time(0);
	}

	// Apply deferred modulation switch (cached while SMD was powered off)
	if (m_modulation_preconfig.has_value()) {
		DEBUG_INFO("ArgosTxService::service_initiate: applying deferred modulation switch to %d", (int)m_modulation_preconfig.value());
		ensure_modulation(m_modulation_preconfig.value());
		m_modulation_preconfig.reset();
	}

	// Adaptive modulation pre-switch for LEGACY/DUTY_CYCLE/DOPPLER modes.
	// In these modes we switch RCONF + reload KMAC at init time (no timing
	// constraint). For SURFACING_BURST, the switch is done at TX complete
	// or during process to avoid KMAC reload at boot.
	if (!m_modulation_preconfig.has_value()) {
		ArgosConfig argos_config;
		configuration_store->get_argos_configuration(argos_config);
		if (argos_config.adaptive_modulation &&
			argos_config.mode != BaseArgosMode::SURFACING_BURST) {
			if (m_kineis.get_current_modulation() != m_scheduled_mode) {
				DEBUG_INFO("ArgosTxService::service_initiate: adaptive pre-switch to %s",
				           argos_modulation_to_string((BaseArgosModulation)m_scheduled_mode));
				ensure_modulation(m_scheduled_mode);
			}
		}
	}

	// Track Doppler burst count for scheduling interval calculation. Incremented
	// HERE — before process_doppler_burst() runs — so that after this point
	// m_doppler_burst_count is the index of the *current* TX (1-based).
	// Convention used by logs:
	//   - service_schedule() reads the counter BEFORE this increment, so it logs
	//     `count + 1` to refer to the upcoming TX (e.g. "Doppler #1 (immediate)").
	//   - process_doppler_burst() reads the counter AFTER this increment, so it
	//     logs `count` directly to refer to the current TX.
	// Note: only incremented in SURFACING_BURST mode — in legacy DOPPLER mode the
	// counter stays 0, which gates out CloudLocate/Fastloc payload substitution.
	if (m_is_surfacing_burst && !m_has_gnss_fix_since_surfacing) {
		m_doppler_burst_count++;
	}

	// Mark first GNSS TX as sent only when actually executing (not during scheduling).
	// Check m_has_gnss_fix (not m_is_surfacing_burst) because the GPS fix can arrive
	// after the burst ended (m_awaiting_surfacing path).
	if (m_has_gnss_fix_since_surfacing && !m_first_gnss_tx_sent) {
		m_first_gnss_tx_sent = true;
	}

	m_scheduled_task();
}

/// @brief Returns true — TX is active during initiate (async completion via events).
/// @return Always true (TX completes asynchronously via KineisDevice events).
bool ArgosTxService::service_is_active_on_initiate() {
	return false;
}

/// @brief Cancel pending TX — power off device and stop send.
/// @return true if TX was pending and cancelled, false otherwise.
bool ArgosTxService::service_cancel() {
	DEBUG_TRACE("ArgosTxService::service_cancel: pending=%u", m_is_tx_pending);
	bool is_pending = m_is_tx_pending;
	m_is_tx_pending = false;
	m_kineis.stop_send();
	return is_pending;
}

/// @brief TX timeout — TCXO warmup + 60s margin for satellite module response.
/// @return Timeout in ms after which TX is considered failed.
unsigned int ArgosTxService::service_next_timeout() {
	// Safety timeout: if KineisEventTxComplete/DeviceError never arrives,
	// the service framework will cancel and reschedule.
	// Budget: power-on(2s) + KMAC(1s) + TX setup(1s) + TCXO warmup(5s) + TX(3s) + margin(18s) = 30s
	return 30000;
}

/// @brief Trigger reschedule on surfacing (for surfacing burst mode).
/// @param[out] immediate  Set to true if TX should fire immediately on surfacing.
/// @return true if this service should be rescheduled when device surfaces.
bool ArgosTxService::service_is_triggered_on_surfaced(bool &immediate) {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	// In SURFACING_BURST mode, reschedule immediately (Doppler burst starts at T=0)
	immediate = (argos_config.mode == BaseArgosMode::SURFACING_BURST);
	return true;  // Re-schedule us on a surfaced event
}

/// @brief Handle peer service events (GPS fix, sensor data, underwater state, surfacing).
/// @param e  Event from another service (GPS, SWS, sensors, etc.).
void ArgosTxService::notify_peer_event(ServiceEvent& e) {
	//DEBUG_TRACE("ArgosTxService::notify_peer_event: (%u|%u)", e.event_source, e.event_type);

	// During SURFACING_BURST Doppler phase, CloudLocate/Fastloc/NO_FIX entries are already
	// sent directly in process_doppler_burst() — skip depth pile to avoid double transmission.
	// Only real GPS fixes should enter the depth pile for the GNSS phase.
	bool skip_depth_pile = false;
	if (m_is_surfacing_burst && !m_has_gnss_fix_since_surfacing &&
	    e.event_source == ServiceIdentifier::GNSS_SENSOR &&
	    e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		GPSLogEntry& gps = std::get<GPSLogEntry>(e.event_data);
		if (gps.info.event_type == GPSEventType::CLOUDLOCATE ||
		    gps.info.event_type == GPSEventType::FASTLOC ||
		    gps.info.event_type == GPSEventType::NO_FIX) {
			skip_depth_pile = true;
		}
	}

	if (!skip_depth_pile) {
		m_depth_pile_manager.notify_peer_event(e);
	}

	if (e.event_source == ServiceIdentifier::GNSS_SENSOR &&
		e.event_type == ServiceEventType::SERVICE_LOG_UPDATED)
	{
		GPSLogEntry& entry = std::get<GPSLogEntry>(e.event_data);

		// Update last known location (real fix only — fastloc is too inaccurate for scheduling)
		if (entry.info.valid && entry.info.event_type != GPSEventType::FASTLOC) {
			DEBUG_TRACE("ArgosTxService::notify_peer_event: updated GPS location");
			m_sched.set_last_location(entry.info.lon, entry.info.lat);

			// Real GPS fix supersedes any CloudLocate/Fastloc/NO_FIX entries in the depth pile
			unsigned int purged = m_depth_pile_manager.purge_non_fix_entries();
			if (purged) {
				DEBUG_INFO("ArgosTxService::notify_peer_event: purged %u non-fix entries from depth pile", purged);
			}

			// SURFACING_BURST: GNSS fix received — switch to GNSS phase.
			// Works during active burst OR after burst ended (awaiting surfacing):
			// a real GPS fix always deserves to be transmitted.
			if ((m_is_surfacing_burst || m_awaiting_surfacing) && !m_has_gnss_fix_since_surfacing) {
				DEBUG_INFO("ArgosTxService::SURFACING_BURST: GNSS fix acquired after %u Doppler messages - switching to GNSS phase",
				           m_doppler_burst_count);
				m_has_gnss_fix_since_surfacing = true;
				m_awaiting_surfacing = false;

				// Arm cooldown if trigger mode is END_OF_DOPPLER (Doppler phase
				// ends on GNSS fix). Guard against a delayed fix arriving during
				// an already-active cooldown (rare race: GPS in flight when
				// cooldown started + surface bounce sets m_is_surfacing_burst).
				unsigned int trigger = configuration_store->read_param<unsigned int>(ParamID::COOLDOWN_TRIGGER_MODE);
				if (trigger == (unsigned int)BaseCooldownTrigger::END_OF_DOPPLER && !m_cooldown_armed &&
				    !ServiceManager::is_in_cooldown(service_current_time())) {
					m_cooldown_armed = true;
					DEBUG_INFO("ArgosTxService: cooldown armed (END_OF_DOPPLER, GNSS fix)");
				}
				service_reschedule();
				Service::notify_peer_event(e);
				return;
			}
		}

		// Reschedule the service
		if (!service_is_scheduled()) {
			DEBUG_TRACE("ArgosTxService::notify_peer_event: rescheduling as no existing schedule");
			service_reschedule();
		}

	} else if (e.event_source == ServiceIdentifier::UW_SENSOR && e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		if (std::get<bool>(e.event_data) == true) {
			// Device went underwater:
			// 1. Cache TCXO=0 for next surfacing (RAM only, sent via SPI at next boot)
			m_tcxo_skip_on_next_tx = true;
			m_kineis.set_tcxo_warmup_time(0);
			// 2. Kill SMD and restore default idle timeout
			m_kineis.set_idle_timeout(1000);
			m_kineis.power_off_immediate();

			// Activate cooldown on dive if armed during this surfacing session
			if (m_cooldown_armed) {
				ServiceManager::set_cycle_complete(service_current_time());
				m_cooldown_armed = false;
			}

			// Reset surfacing burst state on dive
			m_is_surfacing_burst = false;
			m_awaiting_surfacing = false;
			m_doppler_burst_count = 0;
			m_has_gnss_fix_since_surfacing = false;
			m_first_gnss_tx_sent = false;

			// Adaptive modulation: the modulation switch to VLDA4 should have been
			// done at TX complete time while the SMD was still on. If for some reason
			// the SMD is still in the wrong modulation (e.g. error recovery, first boot),
			// cache VLDA4 as fallback so it gets applied at next power-on.
			{
				ArgosConfig ac;
				configuration_store->get_argos_configuration(ac);
				if (ac.adaptive_modulation && ac.mode == BaseArgosMode::SURFACING_BURST) {
					if (m_kineis.get_current_modulation() != KineisModulation::VLDA4) {
						m_modulation_preconfig = KineisModulation::VLDA4;
						DEBUG_INFO("ArgosTxService::UW: VLDA4 fallback cached (modulation was not pre-switched)");
					} else {
						m_modulation_preconfig.reset();
						DEBUG_TRACE("ArgosTxService::UW: VLDA4 already active, no deferred switch needed");
					}
				}
			}
		} else {
			// Device surfaced
			ArgosConfig argos_config;
			configuration_store->get_argos_configuration(argos_config);
			std::time_t earliest_schedule = service_current_time() + argos_config.dry_time_before_tx;
			m_sched.set_earliest_schedule(earliest_schedule);

			// Arm cooldown immediately if trigger mode is AT_SURFACE.
			// Skip arming if a cooldown is already active — otherwise a passive
			// surface bounce during cooldown would re-arm m_cooldown_armed, and
			// the next dive would call set_cycle_complete(now) which resets the
			// cooldown timer, extending it indefinitely under repeated bounces.
			unsigned int trigger = configuration_store->read_param<unsigned int>(ParamID::COOLDOWN_TRIGGER_MODE);
			if (trigger == (unsigned int)BaseCooldownTrigger::AT_SURFACE &&
			    !ServiceManager::is_in_cooldown(service_current_time())) {
				m_cooldown_armed = true;
				DEBUG_INFO("ArgosTxService: cooldown armed (AT_SURFACE)");
			}

			// Activate surfacing burst mode — only when cooldown is not active.
			// During an active cooldown the base class will skip reschedule
			// anyway (no TX will fire), so setting burst state + logging
			// "starting Doppler burst sequence" would be misleading and waste
			// no-op state churn on every passive bounce.
			if (argos_config.mode == BaseArgosMode::SURFACING_BURST &&
			    !ServiceManager::is_in_cooldown(service_current_time())) {
				m_is_surfacing_burst = true;
				m_awaiting_surfacing = false;
				m_doppler_burst_count = 0;
				m_has_gnss_fix_since_surfacing = false;
				m_first_gnss_tx_sent = false;
				// Keep SMD alive between burst pings — default 1s idle timeout
				// is too short for 5-30s Doppler intervals, causing shutdown+reboot
				// failures on the 3rd TX
				m_kineis.set_idle_timeout((argos_config.surfacing_burst_max_s + 10) * 1000);
				m_scheduled_task = [this]() { process_doppler_burst(); };
				m_scheduled_mode = argos_config.adaptive_modulation ? KineisModulation::VLDA4 : KineisModulation::LDA2;
				DEBUG_INFO("ArgosTxService::SURFACING_BURST: surface detected - starting Doppler burst sequence");
			}
		}
	}

	// CloudLocate-ready notification from GPS: mirror of the LoRa path.
	// Triggers an early Doppler-burst tick so the next TX uses CloudLocate
	// (via the count>0 + has_raw_measurement check in process_doppler_burst)
	// instead of waiting for the normal surfacing_burst timer. Edge case
	// "raw arrives during in-flight TX" is NOT handled here (no pending
	// flag — Argos keeps it simple per user request); on that path the
	// CloudLocate just fires at the next normal timer tick, with a small
	// timing penalty vs the LoRa "dans la foulée" guarantee.
	if (e.event_source == ServiceIdentifier::GNSS_SENSOR &&
	    e.event_type == ServiceEventType::GNSS_CLOUDLOCATE_READY) {
		if (m_is_surfacing_burst && !m_has_gnss_fix_since_surfacing && !m_is_tx_pending) {
			DEBUG_INFO("ArgosTxService::notify_peer_event: GNSS_CLOUDLOCATE_READY — rescheduling early CloudLocate TX");
			m_scheduled_task = [this]() { process_doppler_burst(); };
			service_reschedule(true);
			return;
		}
		DEBUG_TRACE("ArgosTxService::notify_peer_event: GNSS_CLOUDLOCATE_READY but not in burst phase 1 or TX in flight");
	}

	Service::notify_peer_event(e);
}

// ============================================================================
// Adaptive modulation helpers
// ============================================================================

/// @brief Get RCONF hex string for a given modulation from config store.
/// @param mode  Target modulation (LDK, LDA2, VLDA4).
/// @return 32-char hex RCONF string, or empty if not configured.
std::string ArgosTxService::get_rconf_for_modulation(KineisModulation mode) {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	switch (mode) {
		case KineisModulation::LDK:  return argos_config.radioconf_ldk;
		case KineisModulation::VLDA4: return argos_config.radioconf_vlda4;
		case KineisModulation::LDA2:
		default:                      return argos_config.radioconf_lda2;
	}
}

/// @brief Switch RCONF on KineisDevice if current modulation doesn't match target.
/// @param target  Desired modulation for next TX.
/// @return true if modulation is ready, false if switch failed.
bool ArgosTxService::ensure_modulation(KineisModulation target) {
	if (m_kineis.get_current_modulation() == target) {
		return true;
	}
	std::string rconf = get_rconf_for_modulation(target);
	if (rconf.empty() || rconf.size() != 32) {
		DEBUG_ERROR("ArgosTxService::ensure_modulation: invalid RCONF for mode %d (len=%u)",
		            (int)target, (unsigned)rconf.size());
		return false;
	}
	DEBUG_INFO("ArgosTxService::ensure_modulation: switching to %d", (int)target);
	return m_kineis.switch_modulation(target, rconf);
}

/// @brief Build and send certification TX burst from config payload.
void ArgosTxService::process_certification_burst() {
	DEBUG_TRACE("ArgosTxService::process_certification_burst");
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	unsigned int size_bits;
	KineisPacket packet = ArgosPacketBuilder::build_certification_packet(argos_config.cert_tx_payload, size_bits);
	DEBUG_INFO("ArgosTxService::process_certification_burst: mode=%s data=%s sz=%u", argos_modulation_to_string(argos_config.cert_tx_modulation), Binascii::hexlify(packet).c_str(), size_bits);
	m_kineis.send((KineisModulation)argos_config.cert_tx_modulation, packet, size_bits);
}

/// @brief Send immediate time sync burst using most recent GPS fix.
void ArgosTxService::process_time_sync_burst() {
	DEBUG_TRACE("ArgosTxService::process_time_sync_burst");
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	unsigned int size_bits;
	std::vector<GPSLogEntry*> v = m_depth_pile_manager.retrieve_gps_latest();
	if (v.size()) {
		KineisPacket packet = ArgosPacketBuilder::build_gnss_packet(v, argos_config.is_out_of_zone, argos_config.is_lb,
				argos_config.delta_time_loc,
				size_bits);
		// Ensure modulation matches (time_sync always uses LDA2)
		if (argos_config.adaptive_modulation) {
			ensure_modulation(m_scheduled_mode);
		}
		DEBUG_INFO("ArgosTxService::process_time_sync_burst: mode=%s data=%s sz=%u", argos_modulation_to_string((BaseArgosModulation)m_scheduled_mode), Binascii::hexlify(packet).c_str(), size_bits);
		m_last_tx_had_gps = true;
		m_kineis.send(m_scheduled_mode, packet, size_bits);
	} else {
		// No eligible entries for transmission in the depth pile, so send a doppler burst instead
		DEBUG_WARN("ArgosTxService::process_time_sync_burst: no entries eligible in depth pile");
		service_complete();
	}
}

/// @brief Build and send sensor packet (GPS + optional ALS/PH/pressure/temp/AXL).
void ArgosTxService::process_sensor_burst() {
	DEBUG_TRACE("ArgosTxService::process_sensor_burst");
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	unsigned int size_bits;
	GPSLogEntry *gps = m_depth_pile_manager.retrieve_gps_single((unsigned int)argos_config.depth_pile);
	if (gps != nullptr) {
		// If GPS entry is a CloudLocate, send CloudLocate packet (extract blob from overlay)
		if (gps->info.event_type == GPSEventType::CLOUDLOCATE) {
			const uint8_t* overlay = reinterpret_cast<const uint8_t*>(&gps->info.lon);
			uint8_t format_id = overlay[0];
			const uint8_t* blob = &overlay[1];
			unsigned int blob_size = (format_id == (uint8_t)BaseCloudLocateFormat::MEASC12) ? 12 : 20;

			KineisPacket packet = ArgosPacketBuilder::build_cloudlocate_packet(
				blob, blob_size, format_id, gps->info.batt_voltage, argos_config.is_lb);
			size_bits = ArgosPacketBuilder::cloudlocate_packet_bits(format_id);

			m_scheduled_mode = (format_id == (uint8_t)BaseCloudLocateFormat::MEASC12) ?
				KineisModulation::LDK : KineisModulation::LDA2;
			if (argos_config.adaptive_modulation) {
				if (!ensure_modulation(m_scheduled_mode)) {
					DEBUG_WARN("ArgosTxService::process_sensor_burst: CloudLocate modulation switch failed");
					m_scheduled_mode = m_kineis.get_current_modulation();
				}
			}
			DEBUG_INFO("ArgosTxService::process_sensor_burst: CloudLocate fmt=%u mode=%s data=%s",
			           format_id, argos_modulation_to_string((BaseArgosModulation)m_scheduled_mode),
			           Binascii::hexlify(packet).c_str());
			m_last_tx_had_gps = true;
			m_kineis.send(m_scheduled_mode, packet, size_bits);
			return;
		}

		// If GPS entry is a fastloc (degraded fix), send fastloc packet instead of sensor packet
		if (gps->info.event_type == GPSEventType::FASTLOC) {
			KineisPacket packet = ArgosPacketBuilder::build_fastloc_packet(gps, argos_config.is_lb);
			size_bits = ArgosPacketBuilder::FASTLOC_PACKET_BITS;
			m_scheduled_mode = KineisModulation::LDA2;
			if (argos_config.adaptive_modulation) {
				if (!ensure_modulation(m_scheduled_mode)) {
					DEBUG_WARN("ArgosTxService::process_sensor_burst: fastloc modulation switch failed, using current");
					m_scheduled_mode = m_kineis.get_current_modulation();
				}
			}
			DEBUG_INFO("ArgosTxService::process_sensor_burst: fastloc mode=%s data=%s sz=%u",
			           argos_modulation_to_string((BaseArgosModulation)m_scheduled_mode), Binascii::hexlify(packet).c_str(), size_bits);
			m_last_tx_had_gps = true;
			m_kineis.send(m_scheduled_mode, packet, size_bits);
			return;
		}

		KineisPacket packet;
#ifdef BOARD_RSPB
		// RSPB uses dedicated packet format with compact AXL + mortality confidence
		unsigned int mort_conf = 0;
#if ENABLE_MORTALITY_SENSOR
		if (mortality_service) mort_conf = mortality_service->get_confidence();
#endif
		ServiceSensorData *pressure = m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile, ServiceIdentifier::PRESSURE_SENSOR);
		ServiceSensorData *thermistor = m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile, ServiceIdentifier::THERMISTOR_SENSOR);
		ServiceSensorData *axl = nullptr;
#if ENABLE_AXL_SENSOR
		axl = m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile, ServiceIdentifier::AXL_SENSOR);
#endif
		unsigned int pkt_fmt = configuration_store->read_param<unsigned int>(ParamID::RSPB_PACKET_FORMAT);
		if (pkt_fmt == 1) {
			m_scheduled_mode = KineisModulation::LDK;
			packet = ArgosPacketBuilder::build_rspb_short_packet(gps, pressure, thermistor, axl,
					argos_config.is_out_of_zone, argos_config.is_lb, mort_conf, size_bits);
		} else {
			m_scheduled_mode = KineisModulation::LDA2;
			packet = ArgosPacketBuilder::build_rspb_long_packet(gps, pressure, thermistor, axl,
					argos_config.is_out_of_zone, argos_config.is_lb, mort_conf, size_bits);
		}

		// Adaptive modulation: switch RCONF to match packet modulation
		if (argos_config.adaptive_modulation) {
			if (!ensure_modulation(m_scheduled_mode)) {
				DEBUG_WARN("ArgosTxService::process_sensor_burst: RSPB modulation switch failed, using current");
				m_scheduled_mode = m_kineis.get_current_modulation();
			}
		}
#else
		// Generic sensor packet for LinkIt V4 (all sensors, no RSPB-specific packing)
		DEBUG_INFO("TX_RAW: SENS lat=%.6f lon=%.6f hAcc=%u nSV=%u hDOP=%.1f batt=%umV",
		           gps->info.lat, gps->info.lon, gps->info.hAcc, gps->info.numSV, (double)gps->info.hDOP, (unsigned)gps->info.batt_voltage);
		m_scheduled_mode = KineisModulation::LDA2;
		packet = ArgosPacketBuilder::build_sensor_packet(gps,
				m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile, ServiceIdentifier::ALS_SENSOR),
				m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile, ServiceIdentifier::PH_SENSOR),
				m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile, ServiceIdentifier::PRESSURE_SENSOR),
				m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile, ServiceIdentifier::SEA_TEMP_SENSOR),
#if ENABLE_AXL_SENSOR
				m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile, ServiceIdentifier::AXL_SENSOR),
#else
				nullptr,
#endif
				argos_config.is_out_of_zone,
				argos_config.is_lb,
				size_bits);

		// Adaptive modulation for generic sensor packet
		if (argos_config.adaptive_modulation) {
			// Sensor packet fits in LDK (128 bits)?
			if (size_bits <= 128) {
				m_scheduled_mode = KineisModulation::LDK;
			}
			if (!ensure_modulation(m_scheduled_mode)) {
				DEBUG_WARN("ArgosTxService::process_sensor_burst: modulation switch failed, using current");
				m_scheduled_mode = m_kineis.get_current_modulation();
			}
		}
#endif
		DEBUG_INFO("ArgosTxService::process_sensor_burst: mode=%s data=%s sz=%u", argos_modulation_to_string((BaseArgosModulation)m_scheduled_mode), Binascii::hexlify(packet).c_str(), size_bits);
		m_last_tx_had_gps = true;
		m_kineis.send(m_scheduled_mode, packet, size_bits);
	} else {
		DEBUG_WARN("ArgosTxService::process_sensor_burst: no entries eligible in depth pile");
		if (m_is_surfacing_burst || m_has_gnss_fix_since_surfacing) {
			DEBUG_INFO("ArgosTxService::process_sensor_burst: ending surfacing burst (depth pile exhausted)");
			m_is_surfacing_burst = false;
			m_awaiting_surfacing = true;
			m_has_gnss_fix_since_surfacing = false;
			m_first_gnss_tx_sent = false;
		}
		service_complete();
	}
}

/// @brief Build and send GNSS packet (short or long based on depth pile entries).
void ArgosTxService::process_gnss_burst() {
	DEBUG_TRACE("ArgosTxService::process_gnss_burst");
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	unsigned int size_bits;
	std::vector<GPSLogEntry*> v = m_depth_pile_manager.retrieve_gps((unsigned int)argos_config.depth_pile);
	if (v.size()) {
		KineisPacket packet;

		// Check if the latest entry is a CloudLocate
		if (v.back()->info.event_type == GPSEventType::CLOUDLOCATE) {
			const uint8_t* overlay = reinterpret_cast<const uint8_t*>(&v.back()->info.lon);
			uint8_t format_id = overlay[0];
			const uint8_t* blob = &overlay[1];
			unsigned int blob_size = (format_id == (uint8_t)BaseCloudLocateFormat::MEASC12) ? 12 : 20;
			packet = ArgosPacketBuilder::build_cloudlocate_packet(blob, blob_size, format_id,
			                                                      v.back()->info.batt_voltage, argos_config.is_lb);
			size_bits = ArgosPacketBuilder::cloudlocate_packet_bits(format_id);
			m_scheduled_mode = (format_id == (uint8_t)BaseCloudLocateFormat::MEASC12) ?
				KineisModulation::LDK : KineisModulation::LDA2;
			if (argos_config.adaptive_modulation) {
				if (!ensure_modulation(m_scheduled_mode)) {
					DEBUG_WARN("ArgosTxService::process_gnss_burst: CloudLocate modulation switch failed");
					m_scheduled_mode = m_kineis.get_current_modulation();
				}
			}
			DEBUG_INFO("ArgosTxService::process_gnss_burst: CloudLocate fmt=%u mode=%s data=%s",
			           format_id, argos_modulation_to_string((BaseArgosModulation)m_scheduled_mode),
			           Binascii::hexlify(packet).c_str());
			m_last_tx_had_gps = true;
			m_kineis.send(m_scheduled_mode, packet, size_bits);
			return;
		}

		// Check if the latest entry is a fastloc (degraded fix) — always LDA2
		if (v.back()->info.event_type == GPSEventType::FASTLOC) {
			packet = ArgosPacketBuilder::build_fastloc_packet(v.back(), argos_config.is_lb);
			size_bits = ArgosPacketBuilder::FASTLOC_PACKET_BITS;
			m_scheduled_mode = KineisModulation::LDA2;
		} else {
			// Filter out any CloudLocate/fastloc entries that may be mixed in
			v.erase(std::remove_if(v.begin(), v.end(), [](const GPSLogEntry* e) {
				return e->info.event_type == GPSEventType::CLOUDLOCATE ||
				       e->info.event_type == GPSEventType::FASTLOC;
			}), v.end());
			if (v.empty()) {
				DEBUG_WARN("ArgosTxService::process_gnss_burst: all entries filtered (mixed types)");
				service_complete();
				return;
			}
			for (unsigned int i = 0; i < v.size(); i++) {
				DEBUG_INFO("TX_RAW: GNSS[%u] lat=%.6f lon=%.6f hAcc=%u nSV=%u hDOP=%.1f batt=%umV",
				           i, v[i]->info.lat, v[i]->info.lon, v[i]->info.hAcc, v[i]->info.numSV, (double)v[i]->info.hDOP, (unsigned)v[i]->info.batt_voltage);
			}
			packet = ArgosPacketBuilder::build_gnss_packet(v, argos_config.is_out_of_zone, argos_config.is_lb,
					argos_config.delta_time_loc,
					size_bits);
		}

		// Adaptive modulation: short/fastloc packet (96 bits) fits LDK, long needs LDA2
		if (argos_config.adaptive_modulation) {
			m_scheduled_mode = (size_bits <= 128) ? KineisModulation::LDK : KineisModulation::LDA2;
			if (!ensure_modulation(m_scheduled_mode)) {
				DEBUG_WARN("ArgosTxService::process_gnss_burst: modulation switch failed, using current");
				m_scheduled_mode = m_kineis.get_current_modulation();
			}
		}

		DEBUG_INFO("ArgosTxService::process_gnss_burst: mode=%s data=%s sz=%u", argos_modulation_to_string((BaseArgosModulation)m_scheduled_mode), Binascii::hexlify(packet).c_str(), size_bits);
		m_last_tx_had_gps = true;
		m_kineis.send(m_scheduled_mode, packet, size_bits);
	} else {
		DEBUG_WARN("ArgosTxService::process_gnss_burst: no entries eligible in depth pile");
		if (m_is_surfacing_burst || m_has_gnss_fix_since_surfacing) {
			DEBUG_INFO("ArgosTxService::process_gnss_burst: ending surfacing burst (depth pile exhausted)");
			m_is_surfacing_burst = false;
			m_awaiting_surfacing = true;
			m_has_gnss_fix_since_surfacing = false;
			m_first_gnss_tx_sent = false;
		}
		service_complete();
	}
}

/// @brief Build and send Doppler burst (24-bit, no GPS — or RSPB Doppler with mortality).
void ArgosTxService::process_doppler_burst() {
	DEBUG_TRACE("ArgosTxService::process_doppler_burst");
	unsigned int size_bits;
	service_update_battery();
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);

	// Progressive CloudLocate: when running a SURFACING_BURST and raw GNSS
	// measurements are available, replace the Doppler payload with a CloudLocate
	// packet. The `m_doppler_burst_count > 0` check is what gates this off in
	// legacy DOPPLER mode (where the counter is never incremented — see
	// service_initiate()) — it does NOT protect the first ping of a burst.
	unsigned int fastloc_mode = configuration_store->read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE);
	if (fastloc_mode == (unsigned int)BaseFastlocMode::CLOUDLOCATE &&
	    m_doppler_burst_count > 0 && gps_device && gps_device->has_raw_measurement()) {
		GNSSRawMeasurement raw = gps_device->get_raw_measurement();
		unsigned int cl_format = configuration_store->read_param<unsigned int>(ParamID::GNSS_CLOUDLOCATE_FORMAT);

		// Select best available blob: prefer configured format, fallback to MEAS20 then MEASC12
		const uint8_t* blob = nullptr;
		unsigned int blob_size = 0;
		uint8_t format_id = 0;
		if (cl_format == (unsigned int)BaseCloudLocateFormat::MEASC12 && raw.has_measc12) {
			blob = raw.measc12; blob_size = 12; format_id = (uint8_t)BaseCloudLocateFormat::MEASC12;
		} else if (raw.has_meas20) {
			blob = raw.meas20; blob_size = 20; format_id = (uint8_t)BaseCloudLocateFormat::MEAS20;
		} else if (raw.has_measc12) {
			blob = raw.measc12; blob_size = 12; format_id = (uint8_t)BaseCloudLocateFormat::MEASC12;
		}

		if (blob) {
			DEBUG_INFO("TX_RAW: CL fmt=%u sz=%u batt=%umV blob=%s",
			           format_id, blob_size, (unsigned)service_get_voltage(), Binascii::hexlify(std::string((const char*)blob, blob_size)).c_str());
			KineisPacket packet = ArgosPacketBuilder::build_cloudlocate_packet(blob, blob_size, format_id,
			                                                                   service_get_voltage(), argos_config.is_lb);
			size_bits = ArgosPacketBuilder::cloudlocate_packet_bits(format_id);

			KineisModulation tx_mode = (format_id == (uint8_t)BaseCloudLocateFormat::MEASC12) ?
				KineisModulation::LDK : KineisModulation::LDA2;
			if (argos_config.adaptive_modulation) {
				if (!ensure_modulation(tx_mode)) {
					DEBUG_WARN("ArgosTxService::process_doppler_burst: CloudLocate modulation switch failed");
					tx_mode = m_kineis.get_current_modulation();
				}
			}

			DEBUG_INFO("ArgosTxService::process_doppler_burst: CLOUDLOCATE #%u fmt=%u sz=%u mode=%s",
			           m_doppler_burst_count, format_id, blob_size,
			           argos_modulation_to_string((BaseArgosModulation)tx_mode));
			m_last_tx_had_gps = true;
			m_kineis.send(tx_mode, packet, size_bits);
			return;
		}
	}

	// Progressive fastloc: when running a SURFACING_BURST and the GPS has a
	// degraded PVT available, replace the Doppler payload with a fastloc packet.
	// The position improves over time as the GPS refines its fix. As above, the
	// `m_doppler_burst_count > 0` check gates legacy DOPPLER mode, not the first
	// ping of a burst.
	if (fastloc_mode >= (unsigned int)BaseFastlocMode::DEGRADED_PVT &&
	    m_doppler_burst_count > 0 && gps_device && gps_device->has_degraded_pvt()) {
		GNSSData degraded = gps_device->get_degraded_pvt();

		// Build a temporary GPSLogEntry from the degraded PVT
		GPSLogEntry fastloc_entry{};
		fastloc_entry.header.log_type = LOG_GPS;
		fastloc_entry.info.lat = degraded.lat;
		fastloc_entry.info.lon = degraded.lon;
		fastloc_entry.info.height = degraded.height;
		fastloc_entry.info.hMSL = degraded.hMSL;
		fastloc_entry.info.hAcc = degraded.hAcc;
		fastloc_entry.info.vAcc = degraded.vAcc;
		fastloc_entry.info.velN = degraded.velN;
		fastloc_entry.info.velE = degraded.velE;
		fastloc_entry.info.velD = degraded.velD;
		fastloc_entry.info.gSpeed = degraded.gSpeed;
		fastloc_entry.info.headMot = degraded.headMot;
		fastloc_entry.info.sAcc = degraded.sAcc;
		fastloc_entry.info.headAcc = degraded.headAcc;
		fastloc_entry.info.pDOP = degraded.pDOP;
		fastloc_entry.info.vDOP = degraded.vDOP;
		fastloc_entry.info.hDOP = degraded.hDOP;
		fastloc_entry.info.headVeh = degraded.headVeh;
		fastloc_entry.info.fixType = degraded.fixType;
		fastloc_entry.info.numSV = degraded.numSV;
		fastloc_entry.info.ttff = degraded.ttff;
		fastloc_entry.info.onTime = degraded.ttff;  // Best approximation of GPS on time
		fastloc_entry.info.batt_voltage = service_get_voltage();
		fastloc_entry.info.schedTime = service_current_time();
		fastloc_entry.info.valid = true;
		fastloc_entry.info.event_type = GPSEventType::FASTLOC;

		DEBUG_INFO("TX_RAW: FLOC lat=%.6f lon=%.6f hAcc=%u nSV=%u hDOP=%.1f batt=%umV",
		           degraded.lat, degraded.lon, degraded.hAcc, degraded.numSV, (double)degraded.hDOP, (unsigned)service_get_voltage());
		KineisPacket packet = ArgosPacketBuilder::build_fastloc_packet(&fastloc_entry, argos_config.is_lb);
		size_bits = ArgosPacketBuilder::FASTLOC_PACKET_BITS;

		KineisModulation tx_mode = KineisModulation::LDA2;
		if (argos_config.adaptive_modulation) {
			if (!ensure_modulation(tx_mode)) {
				DEBUG_WARN("ArgosTxService::process_doppler_burst: fastloc modulation switch failed, using current");
				tx_mode = m_kineis.get_current_modulation();
			}
		}

		DEBUG_INFO("ArgosTxService::process_doppler_burst: FASTLOC #%u hAcc=%um numSV=%u mode=%s data=%s",
		           m_doppler_burst_count, degraded.hAcc, degraded.numSV,
		           argos_modulation_to_string((BaseArgosModulation)tx_mode), Binascii::hexlify(packet).c_str());
		m_last_tx_had_gps = true;
		m_kineis.send(tx_mode, packet, size_bits);
		return;
	}

	// Standard Doppler packet (first ping, or fastloc not available)
	KineisPacket packet;

#if defined(BOARD_RSPB) && ENABLE_MORTALITY_SENSOR
	// RSPB Doppler: battery SOC + activity + mortality (Type 6)
	unsigned int mort_conf = 0;
	uint8_t activity = 0;
	if (mortality_service) {
		mort_conf = mortality_service->get_confidence();
		activity = mortality_service->get_last_activity();
	}
	DEBUG_INFO("TX_RAW: DOPP soc=%u activity=%u mortality=%u",
	           service_get_level(), activity, mort_conf);
	packet = ArgosPacketBuilder::build_rspb_doppler_packet(
		service_get_level(), activity, mort_conf, size_bits);
#else
	// Standard Doppler: battery voltage only
	DEBUG_INFO("TX_RAW: DOPP batt=%umV low_batt=%u",
	           (unsigned)service_get_voltage(), service_is_battery_level_low() ? 1U : 0U);
	packet = ArgosPacketBuilder::build_doppler_packet(
		service_get_voltage(), service_is_battery_level_low(), size_bits);
#endif

	// Adaptive modulation: Doppler = 24 bits = VLDA4
	KineisModulation tx_mode = KineisModulation::LDA2;
	if (argos_config.adaptive_modulation) {
		tx_mode = KineisModulation::VLDA4;
		if (!ensure_modulation(tx_mode)) {
			DEBUG_WARN("ArgosTxService::process_doppler_burst: modulation switch failed, using current");
			tx_mode = m_kineis.get_current_modulation();
		}
	}

	DEBUG_INFO("ArgosTxService::process_doppler_burst: mode=%s data=%s sz=%u",
	           argos_modulation_to_string((BaseArgosModulation)tx_mode), Binascii::hexlify(packet).c_str(), size_bits);
	m_last_tx_had_gps = false;
	m_kineis.send(tx_mode, packet, size_bits);
}

/// @brief TX started event — notify service manager that TX is in progress.
void ArgosTxService::react(KineisEventTxStarted const&) {
	DEBUG_TRACE("ArgosTxService::react: KineisEventTxStarted");
	service_active();
}

/// @brief TX complete event — update counters, manage surfacing burst, complete service.
void ArgosTxService::react(KineisEventTxComplete const&) {
	DEBUG_TRACE("ArgosTxService::react: KineisEventTxComplete");
	m_is_tx_pending = false;
	m_consecutive_device_errors = 0;

	// Restore TCXO warmup after first TX post-submerge
	if (m_tcxo_skip_on_next_tx) {
		ArgosConfig argos_config;
		configuration_store->get_argos_configuration(argos_config);
		DEBUG_TRACE("ArgosTxService::react: restoring TCXO warmup to %u s", argos_config.argos_tcxo_warmup_time);
		m_kineis.set_tcxo_warmup_time(argos_config.argos_tcxo_warmup_time);
		m_tcxo_skip_on_next_tx = false;
	}

	// Increment TX counter
	configuration_store->increment_tx_counter();
	m_session_tx_count++;

	// Update last TX date time
	std::time_t t = service_current_time();
	configuration_store->write_param(ParamID::LAST_TX, t);

	// Counters updated in RAM — flash persistence deferred to periodic flush / powerdown

	// Check session TX limit (SHUTDOWN_NTIME_SAT / LB_SHUTDOWN_NTIME_SAT)
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	if (argos_config.shutdown_ntime_sat > 0 && m_session_tx_count >= argos_config.shutdown_ntime_sat) {
		DEBUG_INFO("ArgosTxService: Session TX limit reached (%u/%u) | shutdown",
		           m_session_tx_count, argos_config.shutdown_ntime_sat);
		configuration_store->save_params();  // Flush before shutdown
		PMU::powerdown();
		return;
	}

	// Post-TX adaptive modulation: pre-switch to VLDA4 while SMD is still
	// powered on so the *next surfacing's* Doppler #1 boots with RCONF already
	// in VLDA4 → no KMAC reload, no deferred RCONF apply in state_load_kmac.
	// The first ping at surface must be as fast as possible (user requirement).
	//
	// Note: SmdSat::state_transmitting transitions directly to `stopped` after
	// every TX (smd_sat.cpp:639), so SMD is fully powered off during inter-TX
	// gaps. The flash write done here persists in STM32WL flash and survives
	// the power-off; the next power-on auto-inits MAC from flash (no SPI
	// KMAC reload needed when RCONF hasn't changed — see state_load_kmac).
	//
	// Guards (skip the pre-switch):
	//   - next_likely_lda2: degraded PVT waiting for better fix → next TX is
	//     fastloc in LDA2, don't churn to VLDA4 in between.
	//   - fix_just_arrived: GNSS fix landed during this (Doppler/CL/FLOC) TX
	//     and the first GNSS TX is about to fire immediately. Pre-switching
	//     would be undone 0 ms later when process_gnss_burst() ensures LDK.
	//     Narrow window (<1 s between TX-complete and next TX-start) — low
	//     risk of dive interruption here.
	//
	// Every other post-TX path pre-switches unconditionally when current is
	// not VLDA4 — including every GNSS-phase TX. Yes, this costs one flash
	// write per GNSS TX (VLDA4 written here, LDK re-written at the next
	// ensure_modulation()), but it GUARANTEES that any dive mid-burst leaves
	// the STM32WL flash in VLDA4, so the next surfacing's first ping skips
	// the deferred-RCONF path entirely.
	if (argos_config.adaptive_modulation && argos_config.mode == BaseArgosMode::SURFACING_BURST) {
		bool next_likely_lda2 = (gps_device && gps_device->has_degraded_pvt() &&
		                         !m_has_gnss_fix_since_surfacing && m_doppler_burst_count > 0);
		bool fix_just_arrived = (m_has_gnss_fix_since_surfacing && !m_first_gnss_tx_sent);

		if (!next_likely_lda2 && !fix_just_arrived &&
		    m_kineis.get_current_modulation() != KineisModulation::VLDA4) {
			DEBUG_INFO("ArgosTxService::react: pre-switch to VLDA4 for next Doppler");
			ensure_modulation(KineisModulation::VLDA4);
		}
	}

	// Cooldown arming based on trigger mode.
	// The cooldown timer actually starts on the next UW event (dive), not here.
	{
		unsigned int trigger = configuration_store->read_param<unsigned int>(ParamID::COOLDOWN_TRIGGER_MODE);
		if (trigger == (unsigned int)BaseCooldownTrigger::AFTER_LAST_TX) {
			// Mode 3: arm on every TX complete (timer restarts each time)
			if (m_last_tx_had_gps || m_is_surfacing_burst) {
				m_cooldown_armed = true;
			}
		} else if (trigger == (unsigned int)BaseCooldownTrigger::AFTER_FIRST_GNSS) {
			// Mode 2: arm after first GNSS TX only
			if (m_last_tx_had_gps && !m_cooldown_armed) {
				m_cooldown_armed = true;
				DEBUG_INFO("ArgosTxService: cooldown armed (AFTER_FIRST_GNSS)");
			}
		}
		// Modes 0 (AT_SURFACE) and 1 (END_OF_DOPPLER) are handled elsewhere
	}

	m_sched.notify_tx_complete();
	service_complete();
}

/// @brief Device error event — increment backoff counter, complete service with error.
void ArgosTxService::react(KineisEventDeviceError const&) {
	m_consecutive_device_errors++;
	DEBUG_WARN("ArgosTxService::react: KineisEventDeviceError (consecutive=%u/%u)",
	           m_consecutive_device_errors, DEVICE_ERROR_MAX_CONSECUTIVE);

	// Restore TCXO warmup if it was skipped
	if (m_tcxo_skip_on_next_tx) {
		ArgosConfig argos_config;
		configuration_store->get_argos_configuration(argos_config);
		m_kineis.set_tcxo_warmup_time(argos_config.argos_tcxo_warmup_time);
		m_tcxo_skip_on_next_tx = false;
	}

	// On error, force RCONF reload on next power-on as safety measure.
	// The SMD may have been in an inconsistent state.
	{
		ArgosConfig ac;
		configuration_store->get_argos_configuration(ac);
		if (ac.adaptive_modulation && ac.mode == BaseArgosMode::SURFACING_BURST) {
			KineisModulation target = m_has_gnss_fix_since_surfacing ?
				KineisModulation::LDK : KineisModulation::VLDA4;
			m_modulation_preconfig = target;
			DEBUG_INFO("ArgosTxService::react: error recovery — caching RCONF for modulation %d", (int)target);
		}
	}

	if (service_cancel()) {
		if (m_consecutive_device_errors >= DEVICE_ERROR_MAX_CONSECUTIVE) {
			// Max errors reached — stop rescheduling to save battery.
			// TX will resume at next boot/session (service_init resets counter).
			DEBUG_ERROR("ArgosTxService: %u consecutive device errors — suspending TX for this session",
			            m_consecutive_device_errors);
			service_complete(nullptr, nullptr, false);  // no reschedule
		} else {
			// Apply exponential backoff: 1min, 2min, 4min... capped at 10min
			unsigned int backoff_ms = DEVICE_ERROR_BACKOFF_BASE_MS << (m_consecutive_device_errors - 1);
			if (backoff_ms > DEVICE_ERROR_BACKOFF_MAX_MS) backoff_ms = DEVICE_ERROR_BACKOFF_MAX_MS;
			DEBUG_WARN("ArgosTxService: backoff %u ms before next TX attempt", backoff_ms);
			m_sched.set_earliest_schedule(service_current_time() + backoff_ms / 1000);
			service_complete();
		}
	}
}
