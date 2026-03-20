#include "mortality_service.hpp"
#include "sensor.hpp"

extern Scheduler *system_scheduler;

// Sensor port indices (must match axl_sensor_service.hpp / sensor.hpp)
static constexpr unsigned int AXL_PORT_ACTIVITY = 4;
static constexpr unsigned int AXL_PORT_WAKEUP   = 5;

MortalityService::MortalityService(Logger *logger)
	: Service(ServiceIdentifier::MORTALITY, "MORTALITY", logger)
{
	memset(&m_state, 0, sizeof(m_state));
	m_state.status = MortalityStatus::ALIVE;
	reset_session_data();
}

void MortalityService::reset_session_data()
{
	m_has_activity = false;
	m_has_temperature = false;
	m_has_gps = false;
	m_session_activity = 0;
	m_session_body_temp = 0.0;
	m_session_lat = 0.0;
	m_session_lon = 0.0;
	m_session_gps_speed = 0;
}

void MortalityService::service_init()
{
	reset_session_data();

	// Restore persisted state from log
	if (get_logger() && get_logger()->num_entries() > 0) {
		MortalityLogEntry last_entry;
		get_logger()->read(&last_entry, get_logger()->num_entries() - 1);
		m_state = last_entry.info;
		DEBUG_INFO("MortalityService: Restored state: confidence=%u%% days=%u status=%u",
				m_state.confidence, m_state.consecutive_days, (unsigned int)m_state.status);
	} else {
		memset(&m_state, 0, sizeof(m_state));
		m_state.status = MortalityStatus::ALIVE;
		DEBUG_INFO("MortalityService: No prior state, starting fresh");
	}
}

void MortalityService::service_term()
{
	reset_session_data();
}

bool MortalityService::service_is_enabled()
{
#if ENABLE_MORTALITY_SENSOR
	return service_read_param<bool>(ParamID::MORTALITY_ENABLE);
#else
	return false;
#endif
}

unsigned int MortalityService::service_next_schedule_in_ms()
{
	// Event-driven only — no periodic scheduling
	return SCHEDULE_DISABLED;
}

void MortalityService::service_initiate()
{
	// Should not be called (schedule disabled), but handle gracefully
	service_complete();
}

bool MortalityService::service_cancel()
{
	return false;
}

void MortalityService::notify_peer_event(ServiceEvent& event)
{
#if ENABLE_MORTALITY_SENSOR
	if (!service_is_enabled())
		return;

	// Collect AXL activity data
	if (event.event_source == ServiceIdentifier::AXL_SENSOR &&
		event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		auto *sensor_data = std::get_if<ServiceSensorData>(&event.event_data);
		if (sensor_data) {
			m_session_activity = static_cast<uint8_t>(sensor_data->port[AXL_PORT_ACTIVITY]);
			m_has_activity = true;
			DEBUG_TRACE("MortalityService: AXL activity=%u", m_session_activity);
		}
	}

	// Collect thermistor temperature data
	if (event.event_source == ServiceIdentifier::THERMISTOR_SENSOR &&
		event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		auto *sensor_data = std::get_if<ServiceSensorData>(&event.event_data);
		if (sensor_data) {
			m_session_body_temp = sensor_data->port[0];
			m_has_temperature = true;
			DEBUG_TRACE("MortalityService: body_temp=%.1f", m_session_body_temp);
		}
	}

	// Collect GPS position data
	if (event.event_source == ServiceIdentifier::GNSS_SENSOR &&
		event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		auto *gps_log = std::get_if<GPSLogEntry>(&event.event_data);
		if (gps_log && gps_log->info.valid) {
			m_session_lat = gps_log->info.lat;
			m_session_lon = gps_log->info.lon;
			m_session_gps_speed = gps_log->info.gSpeed;
			m_has_gps = true;
			DEBUG_TRACE("MortalityService: GPS lat=%.4f lon=%.4f speed=%d mm/s",
					m_session_lat, m_session_lon, m_session_gps_speed);
		}
	}

	// When we have at least GPS + one other sensor, evaluate
	if (m_has_gps && (m_has_activity || m_has_temperature)) {
		evaluate_mortality();
	}
#else
	(void)event;
#endif
}

bool MortalityService::all_inputs_collected() const
{
	return m_has_activity && m_has_temperature && m_has_gps;
}

unsigned int MortalityService::day_of_year(std::time_t epoch) const
{
	if (epoch == 0) return 0;
	struct tm *t = gmtime(&epoch);
	return t ? t->tm_yday : 0;
}

void MortalityService::evaluate_mortality()
{
#if ENABLE_MORTALITY_SENSOR
	unsigned int activity_thresh = service_read_param<unsigned int>(ParamID::MORTALITY_ACTIVITY_THRESH);
	double temp_thresh = service_read_param<double>(ParamID::MORTALITY_TEMP_THRESH);
	unsigned int gps_distance_thresh = service_read_param<unsigned int>(ParamID::MORTALITY_GPS_DISTANCE_THRESH);
	unsigned int confirm_days = service_read_param<unsigned int>(ParamID::MORTALITY_CONFIRM_DAYS);

	// --- Score calculation ---
	unsigned int activity_score = 0;
	if (m_has_activity && m_session_activity < activity_thresh) {
		activity_score = 40;
	}

	unsigned int temp_score = 0;
	if (m_has_temperature && m_session_body_temp < temp_thresh) {
		temp_score = 30;
	}

	unsigned int gps_score = 0;
	if (m_has_gps) {
		// Check stationarity: distance from last known position + low speed
		bool has_prior_position = (m_state.last_lat != 0.0 || m_state.last_lon != 0.0);
		if (has_prior_position) {
			double distance_km = haversine_distance(m_state.last_lon, m_state.last_lat,
					m_session_lon, m_session_lat);
			double distance_m = distance_km * 1000.0;
			bool is_stationary = (distance_m < (double)gps_distance_thresh) &&
					(m_session_gps_speed < 100); // < 100 mm/s = 0.36 km/h
			if (is_stationary) {
				gps_score = 30;
			}
		}
		// Update last known position
		m_state.last_lat = m_session_lat;
		m_state.last_lon = m_session_lon;
	}

	unsigned int session_score = activity_score + temp_score + gps_score;

	// --- Exponential moving average ---
	unsigned int old_confidence = m_state.confidence;
	m_state.confidence = static_cast<uint8_t>((old_confidence * 7 + session_score * 3) / 10);
	if (m_state.confidence > 100) m_state.confidence = 100;

	DEBUG_INFO("MortalityService: score=%u (act=%u temp=%u gps=%u) confidence=%u%% (was %u%%)",
			session_score, activity_score, temp_score, gps_score,
			m_state.confidence, old_confidence);

	// --- Day boundary check ---
	std::time_t now = service_current_time();
	unsigned int current_day = day_of_year(now);
	unsigned int last_day = day_of_year(static_cast<std::time_t>(m_state.last_eval_epoch));

	if (now > 0 && current_day != last_day) {
		m_state.last_eval_epoch = static_cast<uint32_t>(now);
		if (m_state.confidence >= 80) {
			if (m_state.consecutive_days < 255)
				m_state.consecutive_days++;
			DEBUG_INFO("MortalityService: consecutive_days++ = %u", m_state.consecutive_days);
		} else {
			if (m_state.consecutive_days > 0)
				m_state.consecutive_days--;
			DEBUG_INFO("MortalityService: consecutive_days-- = %u", m_state.consecutive_days);
		}
	}

	// --- Status determination ---
	[[maybe_unused]] MortalityStatus old_status = m_state.status;
	if (m_state.consecutive_days >= confirm_days) {
		m_state.status = MortalityStatus::CONFIRMED;
	} else if (m_state.confidence >= 50) {
		m_state.status = MortalityStatus::SUSPECTED;
	} else {
		m_state.status = MortalityStatus::ALIVE;
	}

	// --- Duty cycle adaptation (opt-in, requires EXTERNAL_WAKEUP for BOOT_COUNTER_MODULO) ---
#ifdef EXTERNAL_WAKEUP
	unsigned int duty_modulo = service_read_param<unsigned int>(ParamID::MORTALITY_DUTY_CYCLE_MODULO);

	if (duty_modulo > 0) {
		if (m_state.status == MortalityStatus::CONFIRMED && old_status != MortalityStatus::CONFIRMED) {
			// First time confirmed — save original modulo and adapt
			unsigned int original = service_read_param<unsigned int>(ParamID::MORTALITY_ORIGINAL_MODULO);
			if (original == 0) {
				unsigned int current_modulo = service_read_param<unsigned int>(ParamID::BOOT_COUNTER_MODULO);
				service_write_param(ParamID::MORTALITY_ORIGINAL_MODULO, current_modulo);
			}
			service_write_param(ParamID::BOOT_COUNTER_MODULO, duty_modulo);
			DEBUG_INFO("MortalityService: CONFIRMED — duty cycle adapted to modulo=%u", duty_modulo);
		} else if (m_state.status == MortalityStatus::ALIVE && old_status == MortalityStatus::CONFIRMED) {
			// Recovery — restore original modulo
			unsigned int original = service_read_param<unsigned int>(ParamID::MORTALITY_ORIGINAL_MODULO);
			if (original > 0) {
				service_write_param(ParamID::BOOT_COUNTER_MODULO, original);
				unsigned int zero = 0U;
				service_write_param(ParamID::MORTALITY_ORIGINAL_MODULO, zero);
				DEBUG_INFO("MortalityService: ALIVE again — restored duty cycle modulo=%u", original);
			}
		}
	}
#endif

	// --- Update session data for log ---
	if (m_has_activity)
		m_state.last_activity = m_session_activity;
	if (m_has_temperature)
		m_state.last_body_temp = static_cast<uint16_t>(m_session_body_temp);

	// --- Persist to flash ---
	persist_state();

	// Reset session data for next collection
	reset_session_data();

	DEBUG_INFO("MortalityService: status=%u confidence=%u%% days=%u",
			(unsigned int)m_state.status, m_state.confidence, m_state.consecutive_days);
#endif
}

void MortalityService::persist_state()
{
	MortalityLogEntry entry;
	memset(&entry, 0, sizeof(entry));
	entry.header.log_type = LOG_MORTALITY;
	service_set_log_header_time(entry.header, service_current_time());
	entry.info = m_state;
	service_log(nullptr, &entry);
}
