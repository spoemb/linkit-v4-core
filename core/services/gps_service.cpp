/**
 * @file gps_service.cpp
 * @brief GNSS acquisition service — scheduling, fix processing, fastloc, CloudLocate.
 */

#include <cstdint>
#include <climits>
#include "gps_service.hpp"
#include "config_store.hpp"
#include "scheduler.hpp"
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

	// CloudLocate: enable MEAS message capture only when ALL of:
	//   - fastloc mode is CLOUDLOCATE
	//   - we don't already have a real fix in this surface session (once a normal
	//     fix is obtained, subsequent acquisitions should not waste a full
	//     cold_acq_timeout on raw measurement capture)
	//   - ARGOS_MODE is SURFACING_BURST (LEGACY mode raw-measurement-via-TR_NOM
	//     is not yet wired up — TODO: add a dedicated GUI behavior for LEGACY
	//     CloudLocate timing before re-enabling)
	unsigned int fastloc_mode = configuration_store->read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE);
	BaseArgosMode argos_mode = configuration_store->read_param<BaseArgosMode>(ParamID::ARGOS_MODE);
	nav_settings.cloudlocate_enable = (fastloc_mode == (unsigned int)BaseFastlocMode::CLOUDLOCATE)
	                                  && !m_is_first_fix_found
	                                  && (argos_mode == BaseArgosMode::SURFACING_BURST);

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
		DEBUG_INFO("GPSService::retry_counter: reset ntry=%u->0 (surfacing)", m_cold_start_ntry);
	}
	m_cold_start_ntry = 0;  // Reset retry counter on each surfacing
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
    if (service_is_time_known()) {
        std::time_t led_cutoff = configuration_store->read_param<std::time_t>(ParamID::LED_HRS24_RTC_CUTOFF);
        if (led_cutoff == 0) {
            led_cutoff = service_current_time() + 24 * 3600;
            configuration_store->write_param(ParamID::LED_HRS24_RTC_CUTOFF, led_cutoff);
            configuration_store->save_params();
            DEBUG_INFO("GPSService: LED HRS_24 cutoff anchored at RTC=%u", (unsigned int)led_cutoff);
        }
    }

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
		// Check for surfacing condition
		if (std::get<bool>(e.event_data) == false) {
			// Check if we need to trigger a cold start
			if (service_read_param<bool>(ParamID::GNSS_TRIGGER_COLD_START_ON_SURFACED)) {
				DEBUG_TRACE("GPSService: cold start required on surfaced");
				m_is_first_fix_found = false;
			}
		}
	}

	Service::notify_peer_event(e);
}
