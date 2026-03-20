#pragma once

#include "tinyfsm.hpp"
#include "scheduler.hpp"
#include "error.hpp"
#include "ble_service.hpp"
#include "reed.hpp"
#include "service.hpp"
#include "battery.hpp"

struct ReedSwitchEvent : tinyfsm::Event { ReedSwitchGesture state; };
struct ErrorEvent : tinyfsm::Event { ErrorCode error_code; };


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
	static void notify_bad_filesystem_error();

protected:
	enum class ConfirmationPending { NONE, ENTER_CONFIG, EXIT_CONFIG, POWEROFF };
	static inline ConfirmationPending m_confirmation_pending = ConfirmationPending::NONE;
	static inline bool m_awaiting_re_engage = false;
	static inline Scheduler::TaskHandle m_confirmation_timeout_task;
	static inline const unsigned int CONFIRMATION_TIMEOUT_MS = 2000;
	void cancel_confirmation();
};


class BootState : public GenTracker
{
public:
	void entry() override;
	void exit() override;
};

class OffState : public GenTracker
{
private:
	static inline const unsigned int OFF_LED_PERIOD_MS = 5000;
	Scheduler::TaskHandle m_off_state_task;

public:
	void entry() override;
	void exit() override;
};

class PreOperationalState : public GenTracker
{
private:
#ifdef EXTERNAL_WAKEUP
	static inline const unsigned int TRANSIT_PERIOD_MS = 100;
#else
	static inline const unsigned int TRANSIT_PERIOD_MS = 5000;
#endif
	Scheduler::TaskHandle m_preop_state_task;

public:
	void entry() override;
	void exit() override;
};

class OperationalState : public GenTracker, public BatteryMonitorEventListener
{
private:
	void service_event_handler(ServiceEvent &e);

public:
	void entry() override;
	void exit() override;
	void react(BatteryMonitorEventVoltageCritical const &) override;
};

class ConfigurationState : public GenTracker
{
private:
	static inline const unsigned int BLE_INACTIVITY_TIMEOUT_MS = 20 * 60 * 1000;  // Increased to 20 minutes for OTA transfers
	static inline const unsigned int USB_POLL_INTERVAL_MS = 50;  // USB polling interval
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

class BatteryCriticalState : public GenTracker
{
private:
	static inline const unsigned int BATTERY_CRITICAL_TIMEOUT_MS = 2 * 60 * 1000;
	Scheduler::TaskHandle m_transit_task;
public:
	void entry() override;
	void exit() override;
};

class ErrorState : public GenTracker
{
private:
	Scheduler::TaskHandle m_shutdown_task;
public:
	void entry();
	void exit();
};
