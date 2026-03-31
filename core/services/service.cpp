#include "service.hpp"
#include "scheduler.hpp"
#include "rtc.hpp"
#include "timeutils.hpp"
#include "timer.hpp"
#include "config_store.hpp"
#include "battery.hpp"
#include <cstddef>

extern Timer *system_timer;
extern Scheduler *system_scheduler;
extern RTC *rtc;
extern BatteryMonitor *battery_monitor;

unsigned int ServiceManager::add(Service& s) {
	m_map.insert({m_unique_identifier, s});
	DEBUG_TRACE("ServiceManager::add: service=%s added id=%u", s.get_name(), m_unique_identifier);
	return m_unique_identifier++;
}

void ServiceManager::remove(Service& s) {
	DEBUG_TRACE("ServiceManager::remove: service=%s added", s.get_name());
	m_map.erase(s.get_unique_id());
}


void ServiceManager::startall(std::function<void(ServiceEvent&)> data_notification_callback) {
	m_data_notification_callback = data_notification_callback;
	restore_cooldown_state();
	for (auto const& p : m_map) {
		DEBUG_TRACE("ServiceManager::startall: starting %s id=%u", p.second.get_name(), p.first);
		p.second.start(data_notification_callback);
	}
}

void ServiceManager::stopall() {
	for (auto const& p : m_map)
		p.second.stop();
}

void ServiceManager::notify_peer_event(ServiceEvent& event) {
	for (auto const& p : m_map) {
		if (p.first != event.event_originator_unique_id)
			p.second.notify_peer_event(event);
	}
}

unsigned int ServiceManager::get_unique_id(const char *name) {
	for (auto const& p : m_map) {
		if (std::string(p.second.get_name()) == std::string(name))
			return p.second.get_unique_id();
	}

	throw ErrorCode::RESOURCE_NOT_AVAILABLE;
}

Logger *ServiceManager::get_logger(ServiceIdentifier service_id) {
	for (auto const& p : m_map) {
		if (p.second.get_service_id() == service_id && p.second.get_logger())
			return p.second.get_logger();
	}

	return nullptr;
}

void ServiceManager::inject_event(ServiceEvent& event) {
	if (m_data_notification_callback)
		m_data_notification_callback(event);
}

// Noinit RAM structure for cooldown persistence across System OFF (PSEUDO_POWER_OFF)
struct CooldownNoinit {
	std::time_t last_cycle_time;
	uint16_t    passive_count;
	uint16_t    crc;
};
#ifndef CPPUTEST
static CooldownNoinit s_cooldown_noinit __attribute__((section(".noinit")));
#else
static CooldownNoinit s_cooldown_noinit;
#endif

static uint16_t cooldown_noinit_crc() {
	uint16_t crc = 0;
	const uint8_t *p = reinterpret_cast<const uint8_t *>(&s_cooldown_noinit);
	for (size_t i = 0; i < offsetof(decltype(s_cooldown_noinit), crc); i++)
		crc = (crc << 1) ^ p[i];
	return crc;
}

void ServiceManager::save_cooldown_state() {
	s_cooldown_noinit.last_cycle_time = m_last_successful_cycle_time;
	s_cooldown_noinit.passive_count = static_cast<uint16_t>(m_passive_surfacing_count);
	s_cooldown_noinit.crc = cooldown_noinit_crc();
}

void ServiceManager::restore_cooldown_state() {
	if (s_cooldown_noinit.crc == cooldown_noinit_crc() && s_cooldown_noinit.last_cycle_time > 0) {
		m_last_successful_cycle_time = s_cooldown_noinit.last_cycle_time;
		m_passive_surfacing_count = s_cooldown_noinit.passive_count;
		DEBUG_INFO("ServiceManager: cooldown restored from noinit (last_cycle=%u, passive=%u)",
		           (unsigned int)m_last_successful_cycle_time, m_passive_surfacing_count);
	} else {
		m_last_successful_cycle_time = 0;
		m_passive_surfacing_count = 0;
		DEBUG_TRACE("ServiceManager: cooldown noinit invalid, starting fresh");
	}
}

void ServiceManager::set_cycle_complete(std::time_t t) {
	m_last_successful_cycle_time = t;
	save_cooldown_state();
	DEBUG_INFO("ServiceManager: cycle complete at %u, cooldown started", (unsigned int)t);
}

bool ServiceManager::is_in_cooldown(std::time_t now) {
	unsigned int interval = configuration_store->read_param<unsigned int>(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S);
	if (interval == 0)
		return false;
	if (m_last_successful_cycle_time == 0)
		return false;
	return (now - m_last_successful_cycle_time) < (std::time_t)interval;
}

void ServiceManager::notify_passive_surfacing() {
	m_passive_surfacing_count++;
	save_cooldown_state();
	DEBUG_INFO("ServiceManager: passive surfacing #%u (cooldown active, no GPS/TX)", m_passive_surfacing_count);
}

unsigned int ServiceManager::get_passive_surfacing_count() {
	return m_passive_surfacing_count;
}

void ServiceManager::enter_cooldown_sleep() {
	DEBUG_INFO("ServiceManager: entering cooldown sleep — stopping SWS");

	// Stop SWS (UW_SENSOR) to save power during cooldown
	for (auto& p : m_map) {
		if (p.second.get_service_id() == ServiceIdentifier::UW_SENSOR) {
			p.second.pause_for_cooldown();
		}
	}

	// Program wake timer for remaining cooldown duration
	if (rtc && rtc->is_set() && m_last_successful_cycle_time > 0) {
		unsigned int interval = configuration_store->read_param<unsigned int>(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S);
		std::time_t elapsed = rtc->gettime() - m_last_successful_cycle_time;
		if (elapsed < (std::time_t)interval) {
			unsigned int remaining_ms = (interval - (unsigned int)elapsed) * 1000;
			system_scheduler->cancel_task(m_cooldown_wake_task);
			m_cooldown_wake_task = system_scheduler->post_task_prio([]() {
				exit_cooldown_sleep();
			}, "CooldownWake", Scheduler::DEFAULT_PRIORITY, remaining_ms);
			DEBUG_INFO("ServiceManager: cooldown wake timer set for %u s", (interval - (unsigned int)elapsed));
		} else {
			// Cooldown already expired — restart immediately
			exit_cooldown_sleep();
		}
	}
}

void ServiceManager::exit_cooldown_sleep() {
	DEBUG_INFO("ServiceManager: exiting cooldown sleep — restarting SWS");

	// Restart SWS (UW_SENSOR) — it will detect current state and resume sampling
	for (auto& p : m_map) {
		if (p.second.get_service_id() == ServiceIdentifier::UW_SENSOR) {
			p.second.resume_from_cooldown();
		}
	}
}

void Service::pause_for_cooldown() {
	DEBUG_INFO("Service::pause_for_cooldown: %s", m_name);
	deschedule();
}

void Service::resume_from_cooldown() {
	DEBUG_INFO("Service::resume_from_cooldown: %s", m_name);
	if (m_is_started)
		reschedule();
}

Service::Service(ServiceIdentifier service_id, const char *name, Logger *logger) {
	m_is_started = false;
	m_name = name;
	m_is_underwater = false;
	m_service_id = service_id;
	m_logger = logger;
	m_last_schedule = Service::SCHEDULE_DISABLED;
	m_unique_id = ServiceManager::add(*this);
}

Service::~Service() {
	ServiceManager::remove(*this);
}

unsigned int Service::get_unique_id() { return m_unique_id; }
const char *Service::get_name() { return m_name; }
ServiceIdentifier Service::get_service_id() { return m_service_id; }
Logger *Service::get_logger() { return m_logger; }
void Service::set_logger(Logger *logger) { m_logger = logger; }

void Service::start(std::function<void(ServiceEvent&)> data_notification_callback) {
	DEBUG_TRACE("Service::start: service %s started", m_name);
	m_is_started = true;
	m_is_initiated = false;
	m_data_notification_callback = data_notification_callback;
	m_last_schedule = Service::SCHEDULE_DISABLED;
	service_init();
	reschedule();
}

void Service::stop() {
	DEBUG_TRACE("Service::stop: service %s stopped (is_started=%u)", m_name, (unsigned int)m_is_started);
	if (m_is_started) {
		m_is_started = false;
		deschedule();
		service_cancel();
		if (m_is_initiated)
			notify_service_inactive();
		m_is_initiated = false;
		service_term();
	}
}

unsigned int Service::get_last_schedule() {
	return m_last_schedule;
}

bool Service::is_underwater_deferred() {
	return m_is_underwater;
}

void Service::notify_underwater_state(bool state) {
	if (service_is_usable_underwater())
		return; // Don't care since the sensor can be used underwater
	//DEBUG_TRACE("Service::notify_underwater_state: service %s notify UW %u", m_name, state);
	m_is_underwater = state;
	if (m_is_underwater) {
		deschedule();
		service_cancel();
		if (m_is_initiated)
			notify_service_inactive();
		m_is_initiated = false;
	} else {
		// Check cooldown: skip reschedule if a successful cycle completed recently
		if (rtc && rtc->is_set() && ServiceManager::is_in_cooldown(rtc->gettime())) {
			// Log passive surfacing and enter cooldown sleep once per surfacing event
			if (m_service_id == ServiceIdentifier::GNSS_SENSOR) {
				ServiceManager::notify_passive_surfacing();
				ServiceManager::enter_cooldown_sleep();  // Stop SWS + program wake timer
			}
			DEBUG_INFO("Service::notify_underwater_state: service %s skipped (cooldown active)", m_name);
			return;
		}
		bool immediate;
		if (service_is_triggered_on_surfaced(immediate))
			reschedule(immediate);
	}
}

// May also be overridden in child class to receive peer service events
void Service::notify_peer_event(ServiceEvent& event) {
	//DEBUG_TRACE("Service::notify_peer_event: src=%u type=%u", (unsigned int)event.event_source, (unsigned int)event.event_type);
	bool immediate = true;
	if (event.event_source == ServiceIdentifier::UW_SENSOR && event.event_type == ServiceEventType::SERVICE_LOG_UPDATED)
		notify_underwater_state(std::get<bool>(event.event_data));
	else if (service_is_triggered_on_event(event, immediate)) {
		reschedule(immediate);
	}
};

bool Service::is_started() {
	return m_is_started;
}

bool Service::is_initiated() {
	return m_is_initiated;
}

void Service::service_reschedule(bool immediate) {
	reschedule(immediate);
}

bool Service::service_is_scheduled() {
	return m_last_schedule != Service::SCHEDULE_DISABLED;
}

void Service::service_log(ServiceEventData *event_data, void *entry) {
	if (m_logger && entry != nullptr)
		m_logger->write(entry);
	if (event_data)
		notify_log_updated(*event_data);
}

void Service::service_complete(ServiceEventData *event_data, void *entry, bool shall_reschedule) {
	DEBUG_TRACE("Service::service_complete: service %s", m_name);
	if (!m_is_initiated) {
		if (!m_is_started) {
			// Service was stopped (e.g., state transition during async sensor read) — expected, ignore silently
			DEBUG_TRACE("Service::service_complete: service %s async completion after stop (ignored)", m_name);
		} else {
			DEBUG_WARN("Service::service_complete: service %s completed without being initiated", m_name);
		}
		return;
	}
	m_is_initiated = false;
	notify_service_inactive();
	service_log(event_data, entry);
	if (shall_reschedule)
		reschedule();
}

void Service::service_set_log_header_time(LogHeader& header, std::time_t time)
{
    uint16_t year;
    uint8_t month, day, hour, min, sec;

    convert_datetime_to_epoch(time, year, month, day, hour, min, sec);

    header.year = year;
    header.month = month;
    header.day = day;
    header.hours = hour;
    header.minutes = min;
    header.seconds = sec;
}

void Service::service_active() {
	notify_service_active();
}

std::time_t Service::service_current_time() {
	return rtc->gettime();
}

bool Service::service_is_time_known() {
	return rtc->is_set();
}

uint64_t Service::service_current_timer() {
	return system_timer->get_counter();
}

void Service::service_set_time(std::time_t t) {
	rtc->settime(t);
}

void Service::service_update_battery() {
	return battery_monitor->update();
}

uint16_t Service::service_get_voltage() {
	return battery_monitor->get_voltage();
}

uint8_t Service::service_get_level() {
	return battery_monitor->get_level();
}

bool Service::service_is_battery_level_low() {
	return battery_monitor->is_battery_low();
}

void Service::reschedule(bool immediate) {
	DEBUG_TRACE("Service::reschedule: service %s", m_name);
	deschedule();
	if (is_started()) {
		if (service_is_enabled()) {
			unsigned int next_schedule = immediate ? 0 : service_next_schedule_in_ms();
			if (!m_is_initiated) {
				if (next_schedule != SCHEDULE_DISABLED) {
					DEBUG_TRACE("Service::reschedule: service %s scheduled in %u msecs", m_name, next_schedule);
					m_last_schedule = next_schedule;
					m_task_period = system_scheduler->post_task_prio(
						[this]() {
						unsigned int timeout_ms = service_next_timeout();
						DEBUG_TRACE("Service::reschedule: service %s time out in %u msecs", m_name, timeout_ms);
						if (timeout_ms) {
							m_task_timeout = system_scheduler->post_task_prio(
								[this]() {
								DEBUG_TRACE("Service::reschedule: service %s timed out", m_name);
								service_cancel();
								if (m_is_initiated)
									notify_service_inactive();
								m_is_initiated = false;
								reschedule();
							}, "ServiceTimeoutPeriod", Scheduler::DEFAULT_PRIORITY, timeout_ms);
						}

						if (!m_is_underwater) {
							DEBUG_TRACE("Service::reschedule: service %s active", m_name);
							m_is_initiated = true;
							if (service_is_active_on_initiate())
								notify_service_active();
							service_initiate();
						} else {
							DEBUG_TRACE("Service::reschedule: service %s can't run underwater", m_name);
						}
					}, "ServicePeriod", Scheduler::DEFAULT_PRIORITY, next_schedule);
				} else {
					DEBUG_TRACE("Service::reschedule: service %s schedule currently disabled", m_name);
				}
			} else {
				DEBUG_TRACE("Service::reschedule: service %s already initiated", m_name);
			}
		} else {
			DEBUG_TRACE("Service::reschedule: service %s is not enabled", m_name);
		}
	} else {
		DEBUG_TRACE("Service::reschedule: service %s is stopped", m_name);
	}
}

void Service::deschedule() {
	system_scheduler->cancel_task(m_task_timeout);
	system_scheduler->cancel_task(m_task_period);
	m_last_schedule = Service::SCHEDULE_DISABLED;
}

void Service::notify_log_updated(ServiceEventData& data) {
	if (m_data_notification_callback) {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_source = m_service_id;
		e.event_data = data;
		e.event_originator_unique_id = m_unique_id;
		m_data_notification_callback(e);
	}
}

void Service::notify_service_active() {
	if (m_data_notification_callback) {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_ACTIVE;
		e.event_source = m_service_id;
		e.event_originator_unique_id = m_unique_id;
		m_data_notification_callback(e);
	}
}

void Service::notify_service_inactive() {
	if (m_data_notification_callback) {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_INACTIVE;
		e.event_source = m_service_id;
		e.event_originator_unique_id = m_unique_id;
		m_data_notification_callback(e);
	}
}
