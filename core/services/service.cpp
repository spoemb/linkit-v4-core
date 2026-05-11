/**
 * @file service.cpp
 * @brief Service framework — Service lifecycle + ServiceManager orchestration + cooldown.
 */

#include "service.hpp"
#include "scheduler.hpp"
#include "rtc.hpp"
#include "timeutils.hpp"
#include "timer.hpp"
#include "config_store.hpp"
#include "battery.hpp"
#include "interrupt_lock.hpp"
#include "../sm/error.hpp"
#include <cstddef>
#include <stdexcept>
#include <variant>

extern Timer *system_timer;
extern Scheduler *system_scheduler;
extern RTC *rtc;
extern BatteryMonitor *battery_monitor;

/// @brief Register a service and assign a unique ID.
/// @param s  Service to register.
/// @return Unique ID for this service instance.
unsigned int ServiceManager::add(Service& s) {
	m_map.insert({m_unique_identifier, s});
	DEBUG_TRACE("ServiceManager::add: service=%s added id=%u", s.get_name(), m_unique_identifier);
	return m_unique_identifier++;
}

/// @brief Unregister a service.
void ServiceManager::remove(Service& s) {
	DEBUG_TRACE("ServiceManager::remove: service=%s added", s.get_name());
	m_map.erase(s.get_unique_id());
}


/// @brief Start all registered services (called on FSM transition to Operational).
/// @param data_notification_callback  Global event callback for FSM.
void ServiceManager::startall(std::function<void(ServiceEvent&)> data_notification_callback) {
	m_data_notification_callback = data_notification_callback;
	restore_cooldown_state();
	for (auto const& p : m_map) {
		DEBUG_TRACE("ServiceManager::startall: starting %s id=%u", p.second.get_name(), p.first);
		p.second.start(data_notification_callback);
	}
}

/// @brief Stop all registered services (called on FSM transition out of Operational).
void ServiceManager::stopall() {
	for (auto const& p : m_map)
		p.second.stop();
}

/// @brief Broadcast a peer event to all services except the originator.
/// @param event  Service event to broadcast.
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

/// @brief Inject an event directly to the FSM callback (bypasses peer broadcast).
/// @param event  Event to inject.
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

/// @brief Persist cooldown state to .noinit RAM (survives System OFF / pseudo power-off).
void ServiceManager::save_cooldown_state() {
	InterruptLock lock;
	s_cooldown_noinit.last_cycle_time = m_last_successful_cycle_time;
	s_cooldown_noinit.passive_count = static_cast<uint16_t>(m_passive_surfacing_count);
	s_cooldown_noinit.crc = cooldown_noinit_crc();
}

/// @brief Restore cooldown state from .noinit RAM on boot (CRC-validated).
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

/// @brief Mark a successful surface cycle — starts cooldown timer.
/// @param t  RTC time of cycle completion.
void ServiceManager::set_cycle_complete(std::time_t t) {
	m_last_successful_cycle_time = t;
	save_cooldown_state();
	DEBUG_INFO("ServiceManager: cycle complete at %u, cooldown started", (unsigned int)t);
}

/// @brief Check if surface cycle cooldown is active.
/// @param now  Current RTC time.
/// @return true if less than MIN_SURFACE_CYCLE_INTERVAL_S has elapsed since last cycle.
bool ServiceManager::is_in_cooldown(std::time_t now) {
	return get_cooldown_remaining_s(now) > 0;
}

/// @brief Seconds remaining until cooldown expires.
/// @param now  Current RTC time.
/// @return  Remaining cooldown seconds, or 0 if no active cooldown.
unsigned int ServiceManager::get_cooldown_remaining_s(std::time_t now) {
	unsigned int interval = configuration_store->read_param<unsigned int>(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S);
	if (interval == 0)
		return 0;
	if (m_last_successful_cycle_time == 0)
		return 0;
	if (now < m_last_successful_cycle_time)
		return 0;  // RTC went backward — treat as cooldown expired
	// `now == m_last_successful_cycle_time` falls through with elapsed=0,
	// which correctly reports the full interval as remaining. This matters
	// for callers that do set_cycle_complete(now) then is_in_cooldown(now)
	// in the same tick (e.g. LoRaTxService dive handler).
	std::time_t elapsed = now - m_last_successful_cycle_time;
	if (elapsed >= (std::time_t)interval)
		return 0;
	return (unsigned int)((std::time_t)interval - elapsed);
}

void ServiceManager::notify_passive_surfacing() {
	m_passive_surfacing_count++;
	save_cooldown_state();
	DEBUG_INFO("ServiceManager: passive surfacing #%u (cooldown active, no GPS/TX)", m_passive_surfacing_count);
}

unsigned int ServiceManager::get_passive_surfacing_count() {
	return m_passive_surfacing_count;
}

/// @brief Enter cooldown sleep — stop SWS, program wake timer for remaining cooldown.
void ServiceManager::enter_cooldown_sleep() {
	unsigned int remaining_s = 0;
	if (rtc && rtc->is_set() && m_last_successful_cycle_time > 0) {
		std::time_t now = rtc->gettime();
		if (now > m_last_successful_cycle_time) {
			unsigned int interval = configuration_store->read_param<unsigned int>(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S);
			std::time_t elapsed = now - m_last_successful_cycle_time;
			remaining_s = (elapsed < (std::time_t)interval) ? (interval - (unsigned int)elapsed) : 0;
		}
	}
	DEBUG_INFO("ServiceManager: entering cooldown sleep (remaining %u s) — stopping SWS", remaining_s);

	// Stop SWS (UW_SENSOR) to save power during cooldown
	for (auto& p : m_map) {
		if (p.second.get_service_id() == ServiceIdentifier::UW_SENSOR) {
			p.second.pause_for_cooldown();
		}
	}

	// Program wake timer for remaining cooldown duration
	if (rtc && rtc->is_set() && m_last_successful_cycle_time > 0) {
		std::time_t now = rtc->gettime();
		if (now <= m_last_successful_cycle_time) {
			// RTC went backward — cooldown expired, restart immediately
			exit_cooldown_sleep();
			return;
		}
		unsigned int interval = configuration_store->read_param<unsigned int>(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S);
		std::time_t elapsed = now - m_last_successful_cycle_time;
		if (elapsed < (std::time_t)interval) {
			unsigned int remaining_ms = (interval - (unsigned int)elapsed) * 1000;
			system_scheduler->cancel_task(m_cooldown_wake_task);
			m_cooldown_wake_task = system_scheduler->post_task_prio([]() {
				exit_cooldown_sleep();
			}, "CooldownWake", Scheduler::DEFAULT_PRIORITY, remaining_ms);
			DEBUG_TRACE("ServiceManager: cooldown wake timer set for %u s", (interval - (unsigned int)elapsed));
		} else {
			// Cooldown already expired — restart immediately
			exit_cooldown_sleep();
		}
	}
}

/// @brief Exit cooldown sleep — restart SWS with forced first-time detection.
/// The SWS will re-emit its current state on the first sample, which triggers
/// surface/UW events to wake all services. This avoids incorrectly broadcasting
/// a surface event when the device might actually be underwater.
void ServiceManager::exit_cooldown_sleep() {
	DEBUG_INFO("ServiceManager: exiting cooldown sleep (RTC=%u) — restarting SWS",
	           (rtc && rtc->is_set()) ? (unsigned int)rtc->gettime() : 0);

	// Restart SWS with first-time flag — it will re-emit its current state
	// on the next sample, triggering surface/UW notification to all peers.
	for (auto& p : m_map) {
		if (p.second.get_service_id() == ServiceIdentifier::UW_SENSOR) {
			p.second.reset_state_for_cooldown_exit();
			p.second.resume_from_cooldown();
		}
	}
}

/// @brief Pause service for cooldown — deschedule without stopping.
void Service::pause_for_cooldown() {
	DEBUG_INFO("Service::pause_for_cooldown: %s", m_name);
	deschedule();
}

/// @brief Resume service after cooldown — reschedule if still started.
void Service::resume_from_cooldown() {
	DEBUG_INFO("Service::resume_from_cooldown: %s", m_name);
	if (m_is_started)
		reschedule();
}

/// @brief Constructor — register with ServiceManager, init state.
/// @param service_id  Unique service identifier.
/// @param name        Debug name (string literal).
/// @param logger      Optional persistent logger.
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

/// @brief Start the service — init, register callback, schedule first execution.
/// @param data_notification_callback  Global event callback.
void Service::start(std::function<void(ServiceEvent&)> data_notification_callback) {
	DEBUG_TRACE("Service::start: service %s started", m_name);
	m_is_started = true;
	m_is_initiated = false;
	m_data_notification_callback = data_notification_callback;
	m_last_schedule = Service::SCHEDULE_DISABLED;
	service_init();
	reschedule();
}

/// @brief Stop the service — cancel active task, notify inactive, terminate.
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

/// @brief Handle underwater state change — deschedule when submerged, reschedule on surfacing.
/// @param state  true = submerged, false = surfaced.
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

/// @brief Handle peer events — routes UW state changes and triggered events.
/// @param event  Peer service event. May be overridden by subclass for custom handling.
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

/// @brief Mark service as complete — log, notify inactive, optionally reschedule.
/// @param event_data       Optional event payload for peer notification.
/// @param entry            Optional raw log entry for persistent logger.
/// @param shall_reschedule true to reschedule after completion.
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

/// @brief Internal: compute next schedule, post task to scheduler, arm timeout.
/// @param immediate  true to schedule with 0 delay.
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
						// Barrier against unhandled exceptions in service code.
						// `service_initiate()` is virtual and runs user-defined logic that
						// may throw (e.g. ConfigStore::CONFIG_STORE_CORRUPTED, ErrorCode
						// enums, std::out_of_range from at(), bad_variant_access, etc.).
						// Without this catch, the exception escapes the scheduler task
						// runner and reaches std::terminate → __verbose_terminate_handler
						// → abort(), which on this platform hangs in fputc until WDT.
						// Catching here logs the failure and lets the service be
						// rescheduled on the next event/timer instead of bricking the FW.
						try {
							unsigned int timeout_ms = service_next_timeout();
							DEBUG_TRACE("Service::reschedule: service %s time out in %u msecs", m_name, timeout_ms);
							if (timeout_ms) {
								m_task_timeout = system_scheduler->post_task_prio(
									[this]() {
									try {
										DEBUG_TRACE("Service::reschedule: service %s timed out", m_name);
										service_cancel();
										if (m_is_initiated)
											notify_service_inactive();
										m_is_initiated = false;
										reschedule();
									} catch (ErrorCode e) {
										DEBUG_ERROR("Service::reschedule: timeout ErrorCode=%d in service %s — recovering", (int)e, m_name);
										m_is_initiated = false;
									} catch (const std::exception& e) {
										DEBUG_ERROR("Service::reschedule: timeout std::exception in service %s: %s — recovering", m_name, e.what());
										m_is_initiated = false;
									} catch (...) {
										DEBUG_ERROR("Service::reschedule: timeout unknown exception in service %s — recovering", m_name);
										m_is_initiated = false;
									}
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
						} catch (ErrorCode e) {
							// Firmware-thrown enum (CONFIG_STORE_CORRUPTED, RESOURCE_NOT_AVAILABLE, etc.)
							DEBUG_ERROR("Service::reschedule: ErrorCode=%d in service %s task — recovering", (int)e, m_name);
							// Cancel the safety-net timeout we just armed: otherwise the service
							// is stuck waiting for `timeout_ms` before any retry. (See QA review B2.)
							system_scheduler->cancel_task(m_task_timeout);
							m_is_initiated = false;
						} catch (const std::bad_variant_access& e) {
							// Type mismatch when reading config_store params (variant<>).
							DEBUG_ERROR("Service::reschedule: bad_variant_access in service %s task (config type mismatch?) — recovering", m_name);
							system_scheduler->cancel_task(m_task_timeout);
							m_is_initiated = false;
						} catch (const std::out_of_range& e) {
							// Index out of range — typically from std::array::at() or vector::at().
							DEBUG_ERROR("Service::reschedule: out_of_range in service %s task (%s) — recovering", m_name, e.what());
							system_scheduler->cancel_task(m_task_timeout);
							m_is_initiated = false;
						} catch (const std::exception& e) {
							// Any other std exception (bad_alloc, runtime_error, …)
							DEBUG_ERROR("Service::reschedule: std::exception in service %s task: %s — recovering", m_name, e.what());
							system_scheduler->cancel_task(m_task_timeout);
							m_is_initiated = false;
						} catch (...) {
							// Last-resort barrier. Without this, the exception escapes the
							// scheduler runner and propagates up to terminate() →
							// __verbose_terminate_handler → abort() → fputc-hang → WDT
							// (15 min on this board). Log and recover instead.
							DEBUG_ERROR("Service::reschedule: unknown exception in service %s task — recovering", m_name);
							system_scheduler->cancel_task(m_task_timeout);
							m_is_initiated = false;
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

/// @brief Cancel all pending tasks (period + timeout).
void Service::deschedule() {
	system_scheduler->cancel_task(m_task_timeout);
	system_scheduler->cancel_task(m_task_period);
	m_last_schedule = Service::SCHEDULE_DISABLED;
}

/// @brief Broadcast SERVICE_LOG_UPDATED event to all peers via callback.
/// @param data  Event payload (GPS fix, sensor data, etc.).
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

/// @brief Broadcast SERVICE_ACTIVE event to all peers.
void Service::notify_service_active() {
	if (m_data_notification_callback) {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_ACTIVE;
		e.event_source = m_service_id;
		e.event_originator_unique_id = m_unique_id;
		m_data_notification_callback(e);
	}
}

/// @brief Broadcast SERVICE_INACTIVE event to all peers.
void Service::notify_service_inactive() {
	if (m_data_notification_callback) {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_INACTIVE;
		e.event_source = m_service_id;
		e.event_originator_unique_id = m_unique_id;
		m_data_notification_callback(e);
	}
}
