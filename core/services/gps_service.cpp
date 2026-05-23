/**
 * @file gps_service.cpp
 * @brief GNSS acquisition service — scheduling, fix processing, fastloc, CloudLocate.
 */

#include <cstdint>
#include <climits>
#include "gps_service.hpp"
#include "config_store.hpp"
#include "scheduler.hpp"
#include "rtc.hpp"
#include "rate_limiter.hpp"

extern RTC *rtc;
#if ENABLE_AXL_SENSOR
#include "axl_sensor_service.hpp"
#endif

extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;

static constexpr unsigned int MS_PER_SEC              = 1000;
static constexpr unsigned int FIRST_AQPERIOD_SEC      = 30;  ///< Accelerated first fix schedule
static constexpr unsigned int SERVICE_SAFETY_MARGIN_S = 30;  ///< Extra margin on top of acquisition_timeout for Service watchdog

/// @brief Copy GNSSData fields into GPSLogEntry (avoids 30-line duplication).
static void copy_gnss_to_log(const GNSSData& src, GPSLogEntry& dst) {
	dst.info.iTOW     = src.iTOW;
	dst.info.year     = src.year;
	dst.info.month    = src.month;
	dst.info.day      = src.day;
	dst.info.hour     = src.hour;
	dst.info.min      = src.min;
	dst.info.sec      = src.sec;
	dst.info.valid    = src.valid;
	dst.info.fixType  = src.fixType;
	dst.info.flags    = src.flags;
	dst.info.flags2   = src.flags2;
	dst.info.flags3   = src.flags3;
	dst.info.numSV    = src.numSV;
	dst.info.lon      = src.lon;
	dst.info.lat      = src.lat;
	dst.info.height   = src.height;
	dst.info.hMSL     = src.hMSL;
	dst.info.hAcc     = src.hAcc;
	dst.info.vAcc     = src.vAcc;
	dst.info.velN     = src.velN;
	dst.info.velE     = src.velE;
	dst.info.velD     = src.velD;
	dst.info.gSpeed   = src.gSpeed;
	dst.info.headMot  = src.headMot;
	dst.info.sAcc     = src.sAcc;
	dst.info.headAcc  = src.headAcc;
	dst.info.pDOP     = src.pDOP;
	dst.info.vDOP     = src.vDOP;
	dst.info.hDOP     = src.hDOP;
	dst.info.headVeh  = src.headVeh;
	dst.info.ttff     = src.ttff;
}


/// @brief Init: reset fix state, warn about fastloc mode.
void GPSService::service_init() {
	m_is_active = false;
    m_is_first_fix_found = false;
    m_is_first_schedule = true;
    m_num_gps_fixes = 0;
    m_backup_active = false;
    m_pending_backup_duration_s = 0;
    m_underwater = false;
    m_backup_exit_task = {};
    m_backup_retry_task = {};
    m_backup_periodic_task = {};
    backup_charge_schedule_next();

    // Warn user about fastloc mode impact on sensor messages
    unsigned int fastloc_mode = configuration_store->read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE);
    if (fastloc_mode == (unsigned int)BaseFastlocMode::DEGRADED_PVT) {
        DEBUG_WARN("GPSService: FASTLOC_MODE=DEGRADED_PVT — sensor messages disabled during GPS fallback");
    } else if (fastloc_mode == (unsigned int)BaseFastlocMode::CLOUDLOCATE) {
        unsigned int cl_format = configuration_store->read_param<unsigned int>(ParamID::GNSS_CLOUDLOCATE_FORMAT);
        if (cl_format == (unsigned int)BaseCloudLocateFormat::MEASC12) {
            DEBUG_INFO("GPSService: FASTLOC_MODE=CLOUDLOCATE format=MEASC12 — sensor messages included in fallback");
        } else {
            DEBUG_WARN("GPSService: FASTLOC_MODE=CLOUDLOCATE format=%u — sensor messages disabled during GPS fallback",
                       cl_format);
        }
    }
}

/// @brief Terminate GNSS service (no-op — device powered off by service_cancel).
void GPSService::service_term() {
    system_scheduler->cancel_task(m_backup_periodic_task);
    system_scheduler->cancel_task(m_backup_exit_task);
    system_scheduler->cancel_task(m_backup_retry_task);
    m_pending_backup_duration_s = 0;
    if (m_backup_active) {
        m_device.exit_backup_charge_mode();
        m_backup_active = false;
    }
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
    m_defer_gnss_until_argos_first_tx = false;
#endif
}

/// @brief Enabled if GNSS_EN is set in config (respects LB/zone overrides).
/// @return true if GNSS acquisition is enabled.
bool GPSService::service_is_enabled() {
	GNSSConfig gnss_config;
	configuration_store->get_gnss_configuration(gnss_config);
	return gnss_config.enable;
}

/// @brief Compute next GNSS schedule aligned to UTC 00:00 boundary.
/// @return Delay in ms until next acquisition, or SCHEDULE_DISABLED.
unsigned int GPSService::service_next_schedule_in_ms() {
	GNSSConfig gnss_config;
	configuration_store->get_gnss_configuration(gnss_config);

	// Cooldown gate (2026-05): refuse to schedule a GPS acquisition while
	// MIN_SURFACE_CYCLE_INTERVAL_S is still running. Without this guard, the
	// boot path (Service::start → reschedule → here) computes a UTC-aligned
	// next schedule that fires inside the cooldown window — symptom: GPS
	// launching right after a watchdog/POR reset that lands mid-cooldown.
	// Same trap on the AXL-wakeup / Argos-RX / first-fix retry reschedule
	// paths, all of which route through here without otherwise touching
	// is_in_cooldown(). When cooldown expires, SWS re-emits its state and
	// notify_underwater_state(false) rewakes us via the normal path.
	if (rtc && rtc->is_set() && ServiceManager::is_in_cooldown(service_current_time())) {
		DEBUG_TRACE("GPSService::service_next_schedule_in_ms: cooldown active — SCHEDULE_DISABLED");
		return Service::SCHEDULE_DISABLED;
	}

	// Rate-limit gate (2026-05-23): if Argos TX is rate-limited, acquiring
	// a GPS fix now is pure waste — no TX can fire to actually use the fix
	// until the window expires. Field log on 2026-05-23 showed GPS running
	// a full 70 s cold-acq session during a 45 s rate-limit reschedule —
	// 100 % wasted current. Symptom from sealed deploy POV: shorter
	// effective battery. Mirror the same RateLimiter::is_blocked check
	// ArgosTxService uses (argos_tx_service.cpp:139); return reschedule_in_s
	// matching the rate-limit window so GPS re-checks when it could be useful.
	{
		unsigned int rl_reschedule_s = 0;
		if (rtc && rtc->is_set() && RateLimiter::is_blocked(service_current_time(), rl_reschedule_s)) {
			DEBUG_INFO("GPSService::service_next_schedule_in_ms: rate-limited, reschedule in %u s",
			           rl_reschedule_s);
			return rl_reschedule_s * 1000;
		}
	}

#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	// Gate: while waiting for the first Argos TX to complete, no GPS
	// scheduling at all. The gate-arming code in notify_peer_event UW=0
	// calls service_reschedule(false), which routes through here — and
	// returning SCHEDULE_DISABLED here is what actually cancels the
	// UTC-aligned boot-time schedule. The ArgosTx SERVICE_INACTIVE
	// handler clears the gate then calls service_reschedule(immediate)
	// which routes back here and now returns a real delay.
	if (m_defer_gnss_until_argos_first_tx) {
		DEBUG_TRACE("GPSService::service_next_schedule_in_ms: gate armed — SCHEDULE_DISABLED");
		return Service::SCHEDULE_DISABLED;
	}
#endif

    // Single fix mode: don't reschedule GNSS after first successful fix
    if (m_is_first_fix_found && configuration_store->read_param<bool>(ParamID::GNSS_SESSION_SINGLE_FIX)) {
        DEBUG_INFO("GPSService: GNSS_SESSION_SINGLE_FIX enabled | not rescheduling after first fix");
        return Service::SCHEDULE_DISABLED;
    }

    // GNSS_NTRY: max consecutive failed acquisitions per cycle (0=unlimited).
    // Once the counter reaches the limit, back off to dloc_arg_nom
    // (GNSS_DELTATIME_ACQ). Do NOT reset here — the counter is only reset at
    // (a) end of the GNSS_DELTATIME_ACQ period, when service_initiate fires
    // the back-off attempt, and (b) a new surfacing event.
    unsigned int gnss_ntry = configuration_store->read_param<unsigned int>(ParamID::GNSS_NTRY);
    bool ntry_exhausted = (gnss_ntry > 0 && m_cold_start_ntry >= gnss_ntry);
    if (ntry_exhausted) {
        DEBUG_INFO("GPSService::retry_counter: NTRY limit reached (%u/%u) | back-off to dloc_arg_nom",
                   m_cold_start_ntry, gnss_ntry);
    }

    std::time_t now = service_current_time();
    std::time_t aq_period;
    if (m_is_first_schedule) {
        aq_period = FIRST_AQPERIOD_SEC;
    } else if (ntry_exhausted) {
        // NTRY exhausted: back off to dloc_arg_nom regardless of first_fix state
        aq_period = gnss_config.dloc_arg_nom;
    } else {
        aq_period = m_is_first_fix_found ? gnss_config.dloc_arg_nom : gnss_config.cold_start_retry_period;
    }

    DEBUG_INFO("GPSService::retry_counter: ntry=%u limit=%u exhausted=%u aq_period=%us (%s)",
               m_cold_start_ntry, gnss_ntry, (unsigned)ntry_exhausted,
               (unsigned)aq_period,
               m_is_first_schedule ? "first_schedule"
                 : ntry_exhausted ? "NTRY_BACKOFF"
                 : m_is_first_fix_found ? "dloc_arg_nom" : "cold_start_retry_period");

    if (aq_period == 0) {
    	return Service::SCHEDULE_DISABLED;
    }

    // Find the next schedule time aligned to UTC 00:00
    std::time_t next_schedule = now - (now % aq_period) + aq_period;

    DEBUG_TRACE("GPSService::reschedule: is_first=%u first_fix=%u cold=%u aqperiod=%u now=%u next=%u",
    		(unsigned int)m_is_first_schedule, (unsigned int)m_is_first_fix_found, (unsigned int)gnss_config.cold_start_retry_period, (unsigned int)aq_period,
			(unsigned int)now, (unsigned int)next_schedule);

    // Find the time in milliseconds until this schedule (cast to uint64_t to prevent
    // overflow when aq_period > 4294 seconds, since the result is truncated to unsigned int)
    uint64_t delay_ms = static_cast<uint64_t>(next_schedule - now) * MS_PER_SEC;
    return (delay_ms > UINT32_MAX) ? UINT32_MAX : static_cast<unsigned int>(delay_ms);
}

/// @brief Start GNSS acquisition — configure nav settings, power on M10Q.
void GPSService::service_initiate() {
	// If a backup-cell charge session is in flight (active or pending retry),
	// abort it synchronously so the device returns to idle before we request a
	// normal acquisition. Cancel both the auto-exit timer (active case) and the
	// retry scheduler (pending case) — only one will be armed but cancelling a
	// fresh handle is a no-op.
	if (m_backup_active || m_pending_backup_duration_s > 0) {
		DEBUG_INFO("GPSService::service_initiate: aborting backup-charge to start GNSS acquisition");
		system_scheduler->cancel_task(m_backup_exit_task);
		system_scheduler->cancel_task(m_backup_retry_task);
		m_pending_backup_duration_s = 0;
		if (m_backup_active) {
			m_device.exit_backup_charge_mode();
			m_backup_active = false;
		}
	}

	GNSSConfig gnss_config;
	configuration_store->get_gnss_configuration(gnss_config);

	// End of GNSS_DELTATIME_ACQ back-off: if we arrive here with the counter
	// still at/above NTRY, the previous schedule was a dloc_arg_nom back-off
	// that has now expired. Reset the counter so this new attempt starts
	// fresh. (The other reset trigger is a surfacing event — see
	// service_is_triggered_on_surfaced.)
	unsigned int gnss_ntry = configuration_store->read_param<unsigned int>(ParamID::GNSS_NTRY);
	if (gnss_ntry > 0 && m_cold_start_ntry >= gnss_ntry) {
		DEBUG_INFO("GPSService::retry_counter: reset ntry=%u->0 (end of GNSS_DELTATIME_ACQ back-off)",
		           m_cold_start_ntry);
		m_cold_start_ntry = 0;
	}

	GPSNavSettings nav_settings = {
		gnss_config.fix_mode,
		gnss_config.dyn_model,
		gnss_config.assistnow_enable,
		gnss_config.assistnow_offline_enable,
		gnss_config.hdop_filter_enable,
		gnss_config.hdop_filter_threshold,
		gnss_config.hacc_filter_enable,
		gnss_config.hacc_filter_threshold,
	};

	nav_settings.num_consecutive_fixes = gnss_config.min_num_fixes;
	nav_settings.sat_tracking = true;

	// Adaptive timeout: use shorter timeout when assistance data is available
	if (m_is_first_fix_found) {
		nav_settings.max_nav_samples = gnss_config.acquisition_timeout;
	} else {
		// Check if ANO data is available and fresh — if so, use warm timeout
		unsigned int stale_threshold_s = gnss_config.ano_stale_days ? gnss_config.ano_stale_days * 24 * 3600 : 0;
		GNSSAlmanacStatus ano_status = m_device.get_almanac_status(stale_threshold_s);
		if (ano_status.file_present && !ano_status.stale && ano_status.valid_records > 0) {
			DEBUG_TRACE("GPSService: ANO data fresh (%u records) | using warm timeout", ano_status.valid_records);
			nav_settings.max_nav_samples = gnss_config.acquisition_timeout;
		} else {
			nav_settings.max_nav_samples = gnss_config.acquisition_timeout_cold_start;
		}
	}
	// Cold start: enable all constellations (GPS+GAL+GLO+BDS) to maximize visible SVs
	nav_settings.constellation_mask = m_is_first_fix_found ? gnss_config.constellation_mask : (gnss_config.constellation_mask | 0x0F);
	nav_settings.orbmaxerr = gnss_config.orbmaxerr;
	nav_settings.min_cno = gnss_config.min_cno;
	nav_settings.min_elev = gnss_config.min_elev;
	nav_settings.ano_stale_threshold_s = gnss_config.ano_stale_days ? gnss_config.ano_stale_days * 24 * 3600 : 0;

	// CloudLocate: enable MEAS message capture when ALL of:
	//   - fastloc mode is CLOUDLOCATE
	//   - ARGOS_MODE is SURFACING_BURST (LEGACY mode raw-measurement-via-TR_NOM
	//     is not yet wired up — TODO: add a dedicated GUI behavior for LEGACY
	//     CloudLocate timing before re-enabling)
	//   - AND one of:
	//     - we don't already have a real fix in this surface session (default
	//       battery-friendly behavior — once a normal fix is obtained,
	//       subsequent acquisitions should not waste a full cold_acq_timeout
	//       on raw measurement capture)
	//     - OR GNSS_CLOUDLOCATE_ALWAYS=true (override: capture raw-meas at
	//       every surface, including warm. Use case: short-surface tortue
	//       where warm fix often misses the 30 s timeout and we want a
	//       CloudLocate fallback for every surface. Costs the full
	//       cold_acq_timeout each surface even when a real fix arrives.)
	unsigned int fastloc_mode = configuration_store->read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE);
	BaseArgosMode argos_mode = configuration_store->read_param<BaseArgosMode>(ParamID::ARGOS_MODE);
	bool cloudlocate_always = configuration_store->read_param<bool>(ParamID::GNSS_CLOUDLOCATE_ALWAYS);
	nav_settings.cloudlocate_enable = (fastloc_mode == (unsigned int)BaseFastlocMode::CLOUDLOCATE)
	                                  && (argos_mode == BaseArgosMode::SURFACING_BURST)
	                                  && (!m_is_first_fix_found || cloudlocate_always);

	m_next_schedule = service_current_time();
	m_is_first_schedule = false;
	m_wakeup_time = service_current_timer();
	m_is_active = true;
	m_device.power_on(nav_settings);
}

/// @brief Cancel active acquisition — power off device, log invalid entry.
/// @return true if acquisition was active and cancelled.
bool GPSService::service_cancel() {
	// Cleanly terminate
	DEBUG_TRACE("GPSService::service_cancel");

	if (m_is_active) {
		m_is_active = false;
		m_device.power_off();
		GPSLogEntry log_entry = invalid_log_entry();
		ServiceEventData event_data = log_entry;
		service_complete(&event_data, &log_entry);
		return true;
	}

	return false;
}

/// @brief Safety timeout — acquisition timeout + 30s margin.
/// @return Timeout in ms after which acquisition is force-cancelled.
unsigned int GPSService::service_next_timeout() {
	// Safety net in case the device never responds (e.g., hardware failure)
	// Must exceed the GNSS acquisition timeout to avoid premature termination
	GNSSConfig gnss_config;
	configuration_store->get_gnss_configuration(gnss_config);
	unsigned int timeout_s = m_is_first_fix_found ? gnss_config.acquisition_timeout : gnss_config.acquisition_timeout_cold_start;
	return (timeout_s + SERVICE_SAFETY_MARGIN_S) * MS_PER_SEC;
}

/// @brief Trigger immediate GNSS on surfacing if configured.
/// @param[out] immediate  Set to true if GNSS should fire immediately on surfacing.
/// @return true (always reschedule on surface).
bool GPSService::service_is_triggered_on_surfaced(bool& immediate) {
	GNSSConfig gnss_config;
	configuration_store->get_gnss_configuration(gnss_config);
	immediate = gnss_config.trigger_on_surfaced;
	if (m_cold_start_ntry != 0) {
		// Demoted to TRACE: fires on every surface cycle after a NO_FIX (which is
		// frequent). Adds ~50-300 ms LFS commit on the surfacing critical path.
		// Counter value is visible in the "retry_counter: ntry=N limit=N..." log
		// emitted by service_next_schedule_in_ms when the new GNSS attempt is scheduled.
		DEBUG_TRACE("GPSService::retry_counter: reset ntry=%u->0 (surfacing)", m_cold_start_ntry);
	}
	m_cold_start_ntry = 0;  // Reset retry counter on each surfacing
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	if (m_defer_gnss_until_argos_first_tx) {
		// Demoted to TRACE: redundant with "GPSService: surfaced — GNSS deferred..."
		// which already marks the gate arming.
		DEBUG_TRACE("GPSService: surfacing trigger DEFERRED — waiting for first Argos TX");
		return false;
	}
#endif
	return true;
}

/// @brief GNSS is not usable underwater (RF blocked by water).
/// @return Always false.
bool GPSService::service_is_usable_underwater() {
	return false;
}

/// @brief Build a GPS log entry with NO_FIX status (timeout or error).
/// @return GPSLogEntry with valid=false and current battery/time.
GPSLogEntry GPSService::invalid_log_entry()
{
    DEBUG_INFO("GPSService::invalid_log_entry");
    m_cold_start_ntry++;
    unsigned int ntry_limit = configuration_store->read_param<unsigned int>(ParamID::GNSS_NTRY);
    DEBUG_INFO("GPSService::retry_counter: NO_FIX | ntry=%u/%s", m_cold_start_ntry,
               ntry_limit == 0 ? "unlimited" : std::to_string(ntry_limit).c_str());

    GPSLogEntry gps_entry{};

    gps_entry.header.log_type = LOG_GPS;

    service_set_log_header_time(gps_entry.header, service_current_time());

	service_update_battery();
    gps_entry.info.batt_voltage = service_get_voltage();
    gps_entry.info.event_type = GPSEventType::NO_FIX;
    gps_entry.info.valid = false;
    gps_entry.info.onTime = service_current_timer() - m_wakeup_time;
    gps_entry.info.schedTime = m_next_schedule;

    return gps_entry;
}

/// @brief Process valid GNSS fix — log entry, notify config store, complete service.
void GPSService::task_process_gnss_data()
{
    DEBUG_TRACE("GPSService::task_process_gnss_data");
    if (m_cold_start_ntry != 0) {
        DEBUG_INFO("GPSService::retry_counter: reset ntry=%u->0 (PVT fix)", m_cold_start_ntry);
    }
    m_cold_start_ntry = 0;

    GPSLogEntry gps_entry{};
    gps_entry.header.log_type = LOG_GPS;
    service_set_log_header_time(gps_entry.header, service_current_time());

	service_update_battery();
    gps_entry.info.batt_voltage = service_get_voltage();
    copy_gnss_to_log(m_gnss_data.data, gps_entry);
    gps_entry.info.onTime = service_current_timer() - m_wakeup_time;

    if (m_num_gps_fixes == 1) {
    	// For the very first fix, we need to compute the scheduled GPS time since the RTC
    	// wasn't set beforehand
    	std::time_t fix_time = convert_epochtime(gps_entry.info.year, gps_entry.info.month, gps_entry.info.day, gps_entry.info.hour, gps_entry.info.min, gps_entry.info.sec);
    	gps_entry.info.schedTime     = fix_time - (gps_entry.info.onTime / MS_PER_SEC);
    } else {
    	gps_entry.info.schedTime     = m_next_schedule;
    }

    gps_entry.info.event_type = GPSEventType::FIX;
    gps_entry.info.valid = true;

    DEBUG_INFO("GPSService::task_process_gnss_data: lat=%lf lon=%lf hDOP=%lf hAcc=%lf numSV=%u batt=%lfV ", gps_entry.info.lat, gps_entry.info.lon,
    		static_cast<double>(gps_entry.info.hDOP),
			static_cast<double>(gps_entry.info.hAcc),
			gps_entry.info.numSV,
			(double)gps_entry.info.batt_voltage / 1000);

    // Notify configuration store that we have a new valid GPS fix
    configuration_store->notify_gps_location(gps_entry);

    // Anchor LED HRS_24 RTC cutoff on the first valid GNSS fix.
    // The cutoff (RTC epoch at which the HRS_24 window expires) is persisted
    // in flash so it survives TPL5111 hard shutdowns on EXTERNAL_WAKEUP boards.
    // 0 = unset sentinel; we write it once and ledsm.cpp reads it back to
    // gate LEDs in HRS_24 mode. RTC is set by the M10Q driver from this same
    // fix's date/time fields, so service_current_time() is the GNSS time.
    //
    // Only relevant on EXTERNAL_WAKEUP boards (RSPB) — ledsm.cpp gates the
    // cutoff read with the same #ifdef, so on regular boards (LINKIT) the
    // value would never be read. Writing it anyway burns a flash erase/write
    // cycle and a save_params() round-trip on every fresh deployment for
    // nothing. Mirror the #ifdef here to keep the symmetry tight.
#ifdef EXTERNAL_WAKEUP
    if (service_is_time_known()) {
        std::time_t led_cutoff = configuration_store->read_param<std::time_t>(ParamID::LED_HRS24_RTC_CUTOFF);
        if (led_cutoff == 0) {
            led_cutoff = service_current_time() + 24 * 3600;
            configuration_store->write_param(ParamID::LED_HRS24_RTC_CUTOFF, led_cutoff);
            configuration_store->save_params();
            DEBUG_INFO("GPSService: LED HRS_24 cutoff anchored at RTC=%u", (unsigned int)led_cutoff);
        }
    }
#endif

    ServiceEventData event_data = gps_entry;
    service_complete(&event_data, &gps_entry);
}

/// @brief Process degraded GNSS fix (fastloc) — log entry with FASTLOC event type.
void GPSService::task_process_degraded_gnss_data()
{
    DEBUG_INFO("GPSService::task_process_degraded_gnss_data");
    DEBUG_INFO("GPSService::retry_counter: reset ntry=%u->0 (DEGRADED_PVT success)", m_cold_start_ntry);
    m_cold_start_ntry = 0;

    GPSLogEntry gps_entry{};
    gps_entry.header.log_type = LOG_GPS;
    service_set_log_header_time(gps_entry.header, service_current_time());

    service_update_battery();
    gps_entry.info.batt_voltage = service_get_voltage();
    copy_gnss_to_log(m_gnss_data.data, gps_entry);
    gps_entry.info.onTime    = service_current_timer() - m_wakeup_time;
    gps_entry.info.schedTime = m_next_schedule;

    // Mark as fastloc (degraded position — valid but low quality)
    gps_entry.info.event_type = GPSEventType::FASTLOC;
    gps_entry.info.valid = true;

    DEBUG_INFO("GPSService::task_process_degraded_gnss_data: lat=%lf lon=%lf hAcc=%u numSV=%u fixType=%u",
               gps_entry.info.lat, gps_entry.info.lon,
               gps_entry.info.hAcc, gps_entry.info.numSV, gps_entry.info.fixType);

    ServiceEventData event_data = gps_entry;
    service_complete(&event_data, &gps_entry);
}

/// @brief Process CloudLocate raw measurement — overlay blob into GPSLogEntry position fields.
void GPSService::task_process_cloudlocate_data()
{
    DEBUG_INFO("GPSService::task_process_cloudlocate_data");
    // Note (QA review R6): CloudLocate emits raw measurements without an on-device
    // position. Resetting m_cold_start_ntry here treats the raw capture as a
    // "successful" cold-start outcome — chains of CloudLocate captures will
    // therefore never trigger the NTRY back-off to dloc_arg_nom. This is
    // intentional: CloudLocate is the deployed fallback strategy, so we should
    // keep trying every cold_start_retry_period rather than giving up.
    DEBUG_INFO("GPSService::retry_counter: reset ntry=%u->0 (CLOUDLOCATE success)", m_cold_start_ntry);
    m_cold_start_ntry = 0;

    GPSLogEntry gps_entry{};
    gps_entry.header.log_type = LOG_GPS;
    service_set_log_header_time(gps_entry.header, service_current_time());

    service_update_battery();
    gps_entry.info.batt_voltage = service_get_voltage();
    gps_entry.info.onTime       = service_current_timer() - m_wakeup_time;
    gps_entry.info.schedTime    = m_next_schedule;
    gps_entry.info.event_type   = GPSEventType::CLOUDLOCATE;
    gps_entry.info.valid        = false;  // No on-device position

    // Overlay raw measurement blob into unused position fields (lat, lon, height, hMSL)
    // Layout: byte 0 = format ID, bytes 1-N = blob data
    // lat(8) + lon(8) + height(4) + hMSL(4) = 24 bytes available
    static_assert(offsetof(GPSInfo, hAcc) - offsetof(GPSInfo, lon) >= 21,
                  "GPSInfo overlay space too small for MEAS20 + format byte");
    uint8_t* overlay = reinterpret_cast<uint8_t*>(&gps_entry.info.lon);
    std::memset(overlay, 0, 24);

    unsigned int cl_format = configuration_store->read_param<unsigned int>(ParamID::GNSS_CLOUDLOCATE_FORMAT);
    if (cl_format == (unsigned int)BaseCloudLocateFormat::MEASC12 && m_raw_measurement.has_measc12) {
        overlay[0] = (uint8_t)BaseCloudLocateFormat::MEASC12;
        std::memcpy(&overlay[1], m_raw_measurement.measc12, 12);
        DEBUG_INFO("GPSService::task_process_cloudlocate_data: stored MEASC12 (12 bytes)");
    } else if (m_raw_measurement.has_meas20) {
        overlay[0] = (uint8_t)BaseCloudLocateFormat::MEAS20;
        std::memcpy(&overlay[1], m_raw_measurement.meas20, 20);
        DEBUG_INFO("GPSService::task_process_cloudlocate_data: stored MEAS20 (20 bytes)");
    } else if (m_raw_measurement.has_measc12) {
        // Fallback to MEASC12 if MEAS20 not available
        overlay[0] = (uint8_t)BaseCloudLocateFormat::MEASC12;
        std::memcpy(&overlay[1], m_raw_measurement.measc12, 12);
        DEBUG_INFO("GPSService::task_process_cloudlocate_data: fallback to MEASC12");
    } else {
        DEBUG_WARN("GPSService::task_process_cloudlocate_data: no raw measurement available");
    }

    ServiceEventData event_data = gps_entry;
    service_complete(&event_data, &gps_entry);
}

/// @brief Acquisition timeout (max nav samples reached) — power off, log invalid.
void GPSService::react(const GPSEventMaxNavSamples&) {
	if (!m_is_active)
		return;
    DEBUG_TRACE("GPSService::react(GPSEventMaxNavSamples)");
    m_is_active = false;
    m_device.power_off();
    GPSLogEntry log_entry = invalid_log_entry();
    ServiceEventData event_data = log_entry;
    service_complete(&event_data, &log_entry);
}

/// @brief No signal acquired (max sat samples) — power off, log invalid.
void GPSService::react(const GPSEventMaxSatSamples&) {
	if (!m_is_active)
		return;
    DEBUG_TRACE("GPSService::react(GPSEventMaxSatSamples) — no signal acquired, aborting");
    m_is_active = false;
    m_device.power_off();
    GPSLogEntry log_entry = invalid_log_entry();
    ServiceEventData event_data = log_entry;
    service_complete(&event_data, &log_entry);
}

/// @brief GPS hardware error — power off, log invalid.
void GPSService::react(const GPSEventError&) {
	if (!m_is_active)
		return;
    DEBUG_TRACE("GPSService::react(GPSEventError)");
    m_is_active = false;
    m_device.power_off();
    GPSLogEntry log_entry = invalid_log_entry();
    ServiceEventData event_data = log_entry;
    service_complete(&event_data, &log_entry);
}

/// @brief Device powered off — clear backup-charge flag if it was set
/// (the M10 emits this on exit_backup_charge_mode too, and on async failure of
/// state_enterbackup which bails to poweroff).
void GPSService::react(const GPSEventPowerOff&) {
    if (m_backup_active) {
        m_backup_active = false;
        system_scheduler->cancel_task(m_backup_exit_task);
        // Fire stop callback so the FSM can recover (async hardware failure path).
        // Move semantics make this idempotent vs. backup_charge_stop_internal.
        auto cb = std::move(m_on_backup_charge_stop);
        m_on_backup_charge_start = nullptr;
        m_on_backup_charge_stop = nullptr;
        if (cb) cb();
    }
}

/// @brief Valid PVT fix received — power off, process and log.
void GPSService::react(const GPSEventPVT& e) {
	if (!m_is_active)
		return;
	m_is_active = false;
    m_device.power_off();
    gnss_data_callback(e.data);
}

/// @brief Degraded PVT received (fastloc fallback) — emit if DEGRADED_PVT mode enabled.
void GPSService::react(const GPSEventPVTDegraded& e) {
	if (!m_is_active)
		return;
	m_is_active = false;
	m_device.power_off();

	// Only emit fastloc if mode is DEGRADED_PVT (1) or higher
	unsigned int fastloc_mode = configuration_store->read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE);
	if (fastloc_mode >= (unsigned int)BaseFastlocMode::DEGRADED_PVT) {
		DEBUG_INFO("GPSService::react(GPSEventPVTDegraded): fastloc hAcc=%u fixType=%u numSV=%u",
		           e.data.hAcc, e.data.fixType, e.data.numSV);
		gnss_degraded_callback(e.data);
	} else {
		DEBUG_INFO("GPSService::react(GPSEventPVTDegraded): fastloc disabled, treating as no fix");
		GPSLogEntry log_entry = invalid_log_entry();
		ServiceEventData event_data = log_entry;
		service_complete(&event_data, &log_entry);
	}
}

/// @brief First raw CloudLocate measurement available mid-acquisition — broadcast
/// to peer services so they can fire an early TX without waiting for the full
/// PVT timeout. GPS keeps running (NOT terminated here, unlike
/// react(GPSEventRawMeasurement)). One-shot per acquisition: the M10Q driver
/// has its own guard so this fires once per power_on().
void GPSService::react(const GPSEventCloudLocateReady& e) {
	(void)e;  // payload available via gps_device->get_raw_measurement() if peer wants it
	if (!m_is_active) return;
	unsigned int fastloc_mode = configuration_store->read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE);
	if (fastloc_mode != (unsigned int)BaseFastlocMode::CLOUDLOCATE) {
		DEBUG_TRACE("GPSService::react(GPSEventCloudLocateReady): CLOUDLOCATE mode disabled — ignoring");
		return;
	}
	DEBUG_INFO("GPSService::react(GPSEventCloudLocateReady): broadcasting GNSS_CLOUDLOCATE_READY");
	notify_service_event(ServiceEventType::GNSS_CLOUDLOCATE_READY);
}

/// @brief Raw GNSS measurement received (CloudLocate fallback) — emit if CLOUDLOCATE mode.
void GPSService::react(const GPSEventRawMeasurement& e) {
	if (!m_is_active)
		return;
	m_is_active = false;
	m_device.power_off();

	unsigned int fastloc_mode = configuration_store->read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE);
	if (fastloc_mode == (unsigned int)BaseFastlocMode::CLOUDLOCATE) {
		DEBUG_INFO("GPSService::react(GPSEventRawMeasurement): CloudLocate measc12=%u meas20=%u meas50=%u",
		           e.data.has_measc12, e.data.has_meas20, e.data.has_meas50);
		gnss_cloudlocate_callback(e.data);
	} else {
		DEBUG_INFO("GPSService::react(GPSEventRawMeasurement): CloudLocate disabled, treating as no fix");
		GPSLogEntry log_entry = invalid_log_entry();
		ServiceEventData event_data = log_entry;
		service_complete(&event_data, &log_entry);
	}
}

/// @brief Valid fix callback — store data, mark first fix, process.
/// @param data  GNSS PVT data from M10Q driver.
void GPSService::gnss_data_callback(GNSSData data) {
    // Mark first fix flag
    m_gnss_data.data = data;
    m_is_first_fix_found = true;
    m_num_gps_fixes++;
    task_process_gnss_data();
}

/// @brief Degraded fix callback — store data (does NOT set first_fix_found).
/// @param data  Degraded PVT data (valid but low quality).
void GPSService::gnss_degraded_callback(GNSSData data) {
    m_gnss_data.data = data;
    // Don't set m_is_first_fix_found — degraded fix should not change cold start behavior
    task_process_degraded_gnss_data();
}

/// @brief CloudLocate callback — store raw measurement blob for TX.
/// @param data  Raw GNSS measurement snapshot (MEASC12/MEAS20/MEAS50).
void GPSService::gnss_cloudlocate_callback(GNSSRawMeasurement data) {
    m_raw_measurement = data;
    // Don't set m_is_first_fix_found — CloudLocate does not provide on-device position
    task_process_cloudlocate_data();
}

/// @brief Check if an AXL wakeup event should trigger an immediate GNSS acquisition.
/// @param event       Incoming peer event.
/// @param[out] immediate  Set to true if GNSS should fire immediately.
/// @return true if this event should trigger GNSS rescheduling.
bool GPSService::service_is_triggered_on_event(ServiceEvent& event, bool& immediate) {
#if ENABLE_AXL_SENSOR
	if (event.event_source == ServiceIdentifier::AXL_SENSOR &&
			event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		// Check if AXL wakeup was triggered by reading the ServiceSensorData
		auto* sensor_data = std::get_if<ServiceSensorData>(&event.event_data);
		if (sensor_data && sensor_data->port[AXLSensorPort::WAKEUP_TRIGGERED]) {
			bool trigger_on_axl = service_read_param<bool>(ParamID::GNSS_TRIGGER_ON_AXL_WAKEUP);
			immediate = trigger_on_axl;
			return trigger_on_axl;
		}
	}
#else
	(void)event;
	(void)immediate;
#endif

	return false;
}

/// @brief Handle peer events — trigger cold start on surfacing if configured.
/// @param e  Peer service event (UW_SENSOR surfacing, etc.).
void GPSService::notify_peer_event(ServiceEvent& e) {
	//DEBUG_TRACE("GPSService::notify_peer_event: (%u|%u)", e.event_source, e.event_type);
	if (e.event_source == ServiceIdentifier::UW_SENSOR && e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		bool now_underwater = std::get<bool>(e.event_data);
		m_underwater = now_underwater;
		// Check for surfacing condition
		if (!now_underwater) {
			// Check if we need to trigger a cold start
			if (service_read_param<bool>(ParamID::GNSS_TRIGGER_COLD_START_ON_SURFACED)) {
				DEBUG_TRACE("GPSService: cold start required on surfaced");
				m_is_first_fix_found = false;
			}
			// Backup charge: if UW_ONLY is enabled and we just surfaced,
			// abort the charge so the normal surface flow can take over.
			if (m_backup_active && service_read_param<bool>(ParamID::GNSS_BCKP_CHARGE_UW_ONLY)) {
				DEBUG_INFO("GPSService: surfaced — aborting backup-charge (UW_ONLY)");
				backup_charge_stop_internal();
			}
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
			// Arm the gate: GNSS power-on will wait until ArgosTxService emits
			// SERVICE_INACTIVE (first satellite TX done) to free CPU for the burst.
			// Safety: only arm if Argos will actually TX — otherwise SERVICE_INACTIVE
			// is never emitted and the gate would block GNSS forever. Cooldown is
			// the second silent path: during cooldown ArgosTxService skips setting
			// m_is_surfacing_burst so no SERVICE_INACTIVE ever fires; the gate
			// would stay armed for the whole surface cycle and starve GPS.
			{
				ArgosConfig argos_config;
				configuration_store->get_argos_configuration(argos_config);
				bool argos_will_tx = ((argos_config.mode != BaseArgosMode::OFF) ||
				                      argos_config.cert_tx_enable) &&
				                     !ServiceManager::is_in_cooldown(service_current_time());
				if (argos_will_tx) {
					m_defer_gnss_until_argos_first_tx = true;
					// Cancel any pending GPS schedule so the gate actually defers
					// the next power_on. WITHOUT this, a UTC-boundary schedule
					// that was queued at boot (gps_service.cpp:157 aligns the
					// next_schedule on the next aq_period UTC boundary, which
					// can be 0-30 s away) fires BEFORE ArgosTx completes the
					// first surfacing-burst TX — exactly the contention the
					// gate is meant to avoid. service_is_triggered_on_surfaced
					// returns false (gate armed) so the base class doesn't
					// re-schedule, but it also doesn't cancel the existing
					// task. Force a reschedule here; service_next_schedule_in_ms
					// returns SCHEDULE_DISABLED while the gate is armed, so the
					// reschedule cancels the pending task without arming a new
					// one. The ArgosTx SERVICE_INACTIVE handler below
					// reschedules properly once TX is done (or has errored out).
					service_reschedule(false);
					DEBUG_TRACE("GPSService: surfaced — GNSS deferred until first Argos TX completes");
				}
			}
#endif
		} else {
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
			// Reset gate on submerge so a future surfacing re-arms cleanly.
			m_defer_gnss_until_argos_first_tx = false;
#endif
		}
	}

#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	// Release the gate when Argos TX completes (any SERVICE_INACTIVE from ARGOS_TX),
	// and manually trigger the deferred GNSS reschedule.
	if (e.event_source == ServiceIdentifier::ARGOS_TX &&
	    e.event_type == ServiceEventType::SERVICE_INACTIVE &&
	    m_defer_gnss_until_argos_first_tx && !m_underwater) {
		m_defer_gnss_until_argos_first_tx = false;
		DEBUG_INFO("GPSService: first Argos TX done — releasing GNSS gate, rescheduling now");
		Service::notify_peer_event(e);  // let base process the event normally
		bool immediate;
		if (service_is_triggered_on_surfaced(immediate)) {
			service_reschedule(immediate);
		}
		return;
	}
#endif

	Service::notify_peer_event(e);
}

/// @brief Public entry-point: start/stop a V_BCKP coin-cell charge session.
bool GPSService::request_backup_charge(unsigned int duration_s) {
	if (duration_s == 0) {
		DEBUG_INFO("GPSService::request_backup_charge: stop requested");
		backup_charge_stop_internal();
		return true;
	}

	// Refuse if a GNSS acquisition is currently in progress — backup charge
	// and active acquisition are mutually exclusive (M10 device state machine).
	if (m_is_active) {
		DEBUG_WARN("GPSService::request_backup_charge: GNSS acquisition active — refused");
		return false;
	}

	// Already active: just refresh the auto-exit timer (extend / shorten).
	if (m_backup_active) {
		system_scheduler->cancel_task(m_backup_exit_task);
		m_backup_exit_task = system_scheduler->post_task_prio([this]() {
			DEBUG_INFO("GPSService: backup-charge auto-exit (duration elapsed)");
			backup_charge_stop_internal();
		}, "GPSBackupChargeExit", Scheduler::DEFAULT_PRIORITY, duration_s * MS_PER_SEC);
		return true;
	}

	if (m_device.enter_backup_charge_mode()) {
		// Hardware happy path: M10 was in idle, switched to enterbackup straight away.
		m_backup_active = true;
		DEBUG_INFO("GPSService::request_backup_charge: starting (duration=%us)", duration_s);
		if (m_on_backup_charge_start) m_on_backup_charge_start();
		m_backup_exit_task = system_scheduler->post_task_prio([this]() {
			DEBUG_INFO("GPSService: backup-charge auto-exit (duration elapsed)");
			backup_charge_stop_internal();
		}, "GPSBackupChargeExit", Scheduler::DEFAULT_PRIORITY, duration_s * MS_PER_SEC);
		return true;
	}

	// Hardware refused. Two different recovery strategies depending on the caller:
	//
	// 1. DTE path (user sent GNSSBCKP via BLE — m_on_backup_charge_start is set by
	//    ConfigurationState): the user expects the device to enter silent charging mode
	//    NOW. We fire the start callback immediately (BLE will cut after 200 ms) and
	//    poll enter_backup_charge_mode every 250 ms for up to 5 s. If still failing
	//    after that, abort cleanly via the stop callback (transit OperationalState).
	//
	// 2. Periodic algorithm path (backup_charge_periodic_fire — no callbacks set):
	//    just fail. The next periodic tick will try again. We MUST NOT call power_off
	//    here because the GNSS service may still be cycling, and we'd disrupt the
	//    acquisition schedule.
	if (!m_on_backup_charge_start) {
		DEBUG_INFO("GPSService::request_backup_charge: periodic path, hardware busy — skipping cycle");
		return false;
	}

	DEBUG_INFO("GPSService::request_backup_charge: DTE path, hardware busy — start charge in 'pending' mode (duration=%us)", duration_s);
	m_pending_backup_duration_s = duration_s;
	m_on_backup_charge_start();  // BLE cuts now even though charge hasn't physically started
	schedule_backup_charge_retry(0);
	return true;
}

/// @brief Poll enter_backup_charge_mode every 250 ms until success or attempts exhausted.
/// Called only from the DTE path of request_backup_charge. Aborts cleanly via the stop
/// callback after MAX_ATTEMPTS (~5 s), ensuring the device never freezes — the FSM
/// transits back to OperationalState which restarts services and BLE.
void GPSService::schedule_backup_charge_retry(unsigned int attempt) {
	static constexpr unsigned int MAX_ATTEMPTS    = 20;   // 20 * 250 ms = 5 s
	static constexpr unsigned int RETRY_DELAY_MS  = 250;

	if (attempt >= MAX_ATTEMPTS) {
		DEBUG_ERROR("GPSService::backup_charge_retry: exhausted %u attempts — abandoning", attempt);
		m_pending_backup_duration_s = 0;
		auto cb = std::move(m_on_backup_charge_stop);
		m_on_backup_charge_start = nullptr;
		m_on_backup_charge_stop = nullptr;
		if (cb) cb();
		return;
	}

	m_backup_retry_task = system_scheduler->post_task_prio([this, attempt]() {
		if (m_pending_backup_duration_s == 0)
			return;  // cancelled (manual stop / reed switch / etc.)
		unsigned int d = m_pending_backup_duration_s;
		if (m_device.enter_backup_charge_mode()) {
			m_pending_backup_duration_s = 0;
			m_backup_active = true;
			DEBUG_INFO("GPSService::backup_charge_retry: started after %u attempts (duration=%us)", attempt, d);
			m_backup_exit_task = system_scheduler->post_task_prio([this]() {
				DEBUG_INFO("GPSService: backup-charge auto-exit (duration elapsed)");
				backup_charge_stop_internal();
			}, "GPSBackupChargeExit", Scheduler::DEFAULT_PRIORITY, d * MS_PER_SEC);
		} else {
			schedule_backup_charge_retry(attempt + 1);
		}
	}, "BackupChargeRetry", Scheduler::DEFAULT_PRIORITY, RETRY_DELAY_MS);
}

/// @brief Internal: stop the charge session if active. Idempotent.
/// Handles three states: active charge (exit_backup_charge_mode), pending retry
/// (cancel safety timer), or neither (no-op apart from clearing callbacks).
void GPSService::backup_charge_stop_internal() {
	system_scheduler->cancel_task(m_backup_exit_task);
	system_scheduler->cancel_task(m_backup_retry_task);
	m_pending_backup_duration_s = 0;
	if (m_backup_active) {
		m_device.exit_backup_charge_mode();
		m_backup_active = false;
	}
	// Fire and clear stop callback (cleared before calling to be re-entrant safe).
	auto cb = std::move(m_on_backup_charge_stop);
	m_on_backup_charge_start = nullptr;
	m_on_backup_charge_stop = nullptr;
	if (cb) cb();
}

void GPSService::set_backup_charge_callbacks(std::function<void()> on_start, std::function<void()> on_stop) {
	m_on_backup_charge_start = std::move(on_start);
	m_on_backup_charge_stop  = std::move(on_stop);
}

/// @brief Schedule the next periodic backup-charge tick (or arm immediately if
/// the interval was just enabled).  Re-reads the param each call so DTE changes
/// take effect without restart.
void GPSService::backup_charge_schedule_next() {
	system_scheduler->cancel_task(m_backup_periodic_task);
	unsigned int interval_s = configuration_store->read_param<unsigned int>(ParamID::GNSS_BCKP_CHARGE_INT);
	if (interval_s == 0) {
		DEBUG_TRACE("GPSService: backup-charge periodic disabled (GNSS_BCKP_CHARGE_INT=0)");
		return;
	}
	m_backup_periodic_task = system_scheduler->post_task_prio([this]() {
		backup_charge_periodic_fire();
	}, "GPSBackupChargePeriodic", Scheduler::DEFAULT_PRIORITY, interval_s * MS_PER_SEC);
}

/// @brief Periodic tick: try to start a backup-charge session if conditions allow.
/// Always reschedules itself (caller can disable via INT=0).
void GPSService::backup_charge_periodic_fire() {
	unsigned int duration_s = configuration_store->read_param<unsigned int>(ParamID::GNSS_BCKP_CHARGE_DUR);
	bool uw_only = configuration_store->read_param<bool>(ParamID::GNSS_BCKP_CHARGE_UW_ONLY);

	if (m_is_active) {
		DEBUG_INFO("GPSService: backup-charge tick skipped — GNSS acquisition in progress");
	} else if (uw_only && !m_underwater) {
		DEBUG_INFO("GPSService: backup-charge tick skipped — UW_ONLY and surfaced");
	} else if (m_backup_active) {
		DEBUG_INFO("GPSService: backup-charge tick skipped — already charging");
	} else {
		request_backup_charge(duration_s);
	}

	backup_charge_schedule_next();
}
