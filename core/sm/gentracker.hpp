/**
 * @file gentracker.hpp
 * @brief GenTracker FSM — main state machine: Off → PreOp → Config → Operational → Error.
 *
 * Uses TinyFSM. Reed switch confirmation gesture drives state transitions.
 * Not board-specific despite the name — used by all tracker variants.
 */

#pragma once

#include "tinyfsm.hpp"
#include "scheduler.hpp"
#include "error.hpp"
#include "ble_service.hpp"
#include "reed.hpp"
#include "service.hpp"
#include "battery.hpp"

/// @brief Reed switch gesture event for FSM transitions.
struct ReedSwitchEvent : tinyfsm::Event { ReedSwitchGesture state; };
/// @brief Error event — triggers transition to ErrorState.
struct ErrorEvent : tinyfsm::Event { ErrorCode error_code; };

/// @brief Main FSM base class — handles reed switch, watchdog, config flush.
class GenTracker : public tinyfsm::Fsm<GenTracker>
{
public:
	void react(tinyfsm::Event const &);
	void react(ReedSwitchEvent const &event);
	void react(ErrorEvent const &event);
	virtual void entry(void);
	virtual void exit(void);
	void set_ble_device_name();

	static void kick_watchdog();
	static void periodic_config_flush();
	static inline bool m_config_flush_active = false;
	static void notify_bad_filesystem_error();

protected:
	enum class ConfirmationPending { NONE, ENTER_CONFIG, EXIT_CONFIG, POWEROFF };
	static inline ConfirmationPending m_confirmation_pending = ConfirmationPending::NONE;
	static inline bool m_awaiting_re_engage = false;
	static inline Scheduler::TaskHandle m_confirmation_timeout_task;
	static constexpr unsigned int CONFIRMATION_TIMEOUT_MS = 2000;
	void cancel_confirmation();
};


/// @brief Boot state — init hardware, check reset cause, transition to Off or PreOp.
class BootState : public GenTracker
{
public:
	void entry() override;
	void exit() override;
};

/// @brief Off state — device sleeping, periodic LED blink, reed switch wakes to PreOp.
class OffState : public GenTracker
{
private:
	static constexpr unsigned int OFF_LED_PERIOD_MS = 5000;
	Scheduler::TaskHandle m_off_state_task;

public:
	void entry() override;
	void exit() override;
};

/// @brief PreOperational — init config, mount filesystem, transition to Operational.
class PreOperationalState : public GenTracker
{
private:
#ifdef EXTERNAL_WAKEUP
	static constexpr unsigned int TRANSIT_PERIOD_MS = 100;
#else
	static constexpr unsigned int TRANSIT_PERIOD_MS = 5000;
#endif
	Scheduler::TaskHandle m_preop_state_task;

public:
	void entry() override;
	void exit() override;
};

/// @brief Operational — all services running, GPS/TX/sensors active, battery monitoring.
class OperationalState : public GenTracker, public BatteryMonitorEventListener
{
private:
	void service_event_handler(ServiceEvent &e);

public:
	void entry() override;
	void exit() override;
	void react(BatteryMonitorEventVoltageCritical const &) override;
};

/// @brief Configuration — BLE/USB DTE interface active, accepts commands, OTA updates.
class ConfigurationState : public GenTracker
{
private:
	static constexpr unsigned int BLE_INACTIVITY_TIMEOUT_MS = 20 * 60 * 1000;  ///< 20 min for OTA
	static constexpr unsigned int USB_POLL_INTERVAL_MS = 50;
	Scheduler::TaskHandle m_ble_inactivity_timeout_task;
	Scheduler::TaskHandle m_usb_poll_task;
	int on_ble_event(BLEServiceEvent&);
	void on_ble_inactivity_timeout();
	void restart_inactivity_timeout();
	void process_received_data();
	void process_usb_data();
	void schedule_usb_poll();

public:
	void entry() override;
	void exit() override;
};

/// @brief Battery critical — services stopped, 2-min timeout before powerdown.
class BatteryCriticalState : public GenTracker
{
private:
	static constexpr unsigned int BATTERY_CRITICAL_TIMEOUT_MS = 2 * 60 * 1000;
	Scheduler::TaskHandle m_transit_task;
public:
	void entry() override;
	void exit() override;
};

/// @brief Error state — RED LED, 30s delay before powerdown.
class ErrorState : public GenTracker
{
private:
	Scheduler::TaskHandle m_shutdown_task;
public:
	void entry();
	void exit();
};
