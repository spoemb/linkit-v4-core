#include "bsp.hpp"
#include "pmu.hpp"
#include "ota_file_updater.hpp"
#include "logger.hpp"
#include "config_store.hpp"
#include "service_scheduler.hpp"
#include "scheduler.hpp"
#include "service.hpp"
#include "dte_handler.hpp"
#include "filesystem.hpp"
#include "config_store.hpp"
#include "error.hpp"
#include "timer.hpp"
#include "debug.hpp"
#include "switch.hpp"
#include "reed.hpp"
#include "ledsm.hpp"
#include "buzzm.hpp"
#include "battery.hpp"
#include "rgb_led.hpp"
#include "led.hpp"
#include "gps.hpp"
#include "ble_service.hpp"
#include "gentracker.hpp"

// USB DTE interface (platform-specific)
#ifdef USB_DTE_ENABLED
#include "usb_interface.hpp"
#endif

// LoRa device instance (for bridge mode in process_usb_data)
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
#include "lora_rak3172.hpp"
extern LoRaDevice *lora_device_instance;
#endif

// LED hardware access for forced LED off before powerdown
extern RGBLed *status_led;
extern Led *ext_status_led;

// These contexts must be created before the FSM is initialised
extern FileSystem *main_filesystem;
extern Scheduler *system_scheduler;
extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern BLEService *ble_service;
extern OTAFileUpdater *ota_updater;
extern DTEHandler *dte_handler;
extern ReedSwitch *reed_switch;
extern BatteryMonitor *battery_monitor;
extern BaseDebugMode g_debug_mode;

// FSM initial state -> LEDOff
FSM_INITIAL_STATE(LEDState, LEDOff);
FSM_INITIAL_STATE(BuzzState, BuzzOff);

using led_handle = LEDState;
using buzz_handle = BuzzState;


void GenTracker::react(tinyfsm::Event const &) { }

void GenTracker::cancel_confirmation()
{
	system_scheduler->cancel_task(m_confirmation_timeout_task);
	m_confirmation_pending = ConfirmationPending::NONE;
	m_awaiting_re_engage = false;
}

void GenTracker::react(ReedSwitchEvent const &event)
{
	DEBUG_INFO("react: ReedSwitchEvent: %u", (int)event.state);

	// Reed switch event handling with confirmation gesture:
	// ENGAGE -- engaged LED state (or confirm pending action if re-engaged within 2s)
	// RELEASE -- disengage LED state (or start 2s confirmation window)
	// SHORT_HOLD (3s) -- rapid blink, awaiting release+re-engage to confirm
	// LONG_HOLD (6s) -- rapid blink, awaiting release+re-engage to confirm power off
	if (event.state == ReedSwitchGesture::ENGAGE) {
		if (m_awaiting_re_engage && m_confirmation_pending != ConfirmationPending::NONE) {
			// User re-engaged within 2s confirmation window — action confirmed
			system_scheduler->cancel_task(m_confirmation_timeout_task);
			auto pending = m_confirmation_pending;
			m_confirmation_pending = ConfirmationPending::NONE;
			m_awaiting_re_engage = false;

			DEBUG_INFO("Reed confirmation accepted: %u", (int)pending);

			if (pending == ConfirmationPending::ENTER_CONFIG) {
				transit<ConfigurationState>();
			} else if (pending == ConfirmationPending::EXIT_CONFIG) {
				transit<PreOperationalState>();
			} else if (pending == ConfirmationPending::POWEROFF) {
				buzz_handle::dispatch<SetBuzzPowerDown>({});
				transit<OffState>();
			}
			return;
		}
		led_handle::dispatch<SetLEDMagnetEngaged>({});
		buzz_handle::dispatch<SetBuzzMagnetEngaged>({});
	} else if (event.state == ReedSwitchGesture::RELEASE) {
		if (m_confirmation_pending != ConfirmationPending::NONE) {
			// Magnet released during confirmation — start 2s re-engage window
			m_awaiting_re_engage = true;
			led_handle::dispatch<SetLEDMagnetDisengaged>({});
			m_confirmation_timeout_task = system_scheduler->post_task_prio([this](){
				DEBUG_INFO("Reed confirmation timeout");
				m_confirmation_pending = ConfirmationPending::NONE;
				m_awaiting_re_engage = false;
				buzz_handle::dispatch<SetBuzzMagnetDisengaged>({});
				// Restore LED to current GenTracker state
				if (is_in_state<ConfigurationState>()) {
					led_handle::dispatch<SetLEDConfigNotConnected>({});
				} else if (is_in_state<PreOperationalState>()) {
					transit<OperationalState>();
				} else {
					led_handle::dispatch<SetLEDOff>({});
				}
			}, "ReedConfirmTimeout", Scheduler::DEFAULT_PRIORITY, CONFIRMATION_TIMEOUT_MS);
			return;
		}
		led_handle::dispatch<SetLEDMagnetDisengaged>({});
		buzz_handle::dispatch<SetBuzzMagnetDisengaged>({});
	} else if (event.state == ReedSwitchGesture::SHORT_HOLD) {
		if (is_in_state<ConfigurationState>()) {
			m_confirmation_pending = ConfirmationPending::EXIT_CONFIG;
			led_handle::dispatch<SetLEDConfirmExitConfig>({});
			buzz_handle::dispatch<SetBuzzPreOperationalPending>({});
		} else {
			m_confirmation_pending = ConfirmationPending::ENTER_CONFIG;
			led_handle::dispatch<SetLEDConfirmConfig>({});
			buzz_handle::dispatch<SetBuzzConfigPending>({});
		}
		m_awaiting_re_engage = false;
	} else if (event.state == ReedSwitchGesture::LONG_HOLD) {
		m_confirmation_pending = ConfirmationPending::POWEROFF;
		m_awaiting_re_engage = false;
		led_handle::dispatch<SetLEDConfirmPowerOff>({});
	}
}

void GenTracker::react(ErrorEvent const &event) {
	DEBUG_ERROR("GenTracker::react: ErrorEvent: error_code=%u", event.error_code);
	cancel_confirmation();
	transit<ErrorState>();
}

void GenTracker::entry(void) { };
void GenTracker::exit(void)  { };

void GenTracker::notify_bad_filesystem_error() {
	ErrorEvent event;
	event.error_code = BAD_FILESYSTEM;
	dispatch(event);
}

void GenTracker::kick_watchdog() {
	DEBUG_TRACE("GenTracker::kick_watchdog: calling PMU::kick_watchdog");
	PMU::kick_watchdog();
	system_scheduler->post_task_prio([](){
		kick_watchdog();
	}, "KickWatchdog", Scheduler::HIGHEST_PRIORITY, BSP::WDT_Inits[BSP::WDT].config.reload_value * 0.90);
}

void BootState::entry() {

	DEBUG_INFO("entry: BootState");

	// Ensure the system timer is started to allow scheduling to work
	system_timer->start();

	// Turn status LED white to indicate boot up
	led_handle::start();
	led_handle::dispatch<SetLEDBoot>({});
	buzz_handle::start();

	// If we can't mount the filesystem then try to format it first and retry
	if (!main_filesystem->is_mounted() && main_filesystem->mount() < 0)
	{
		DEBUG_TRACE("format filesystem");
		if (main_filesystem->format() < 0 || main_filesystem->mount() < 0)
		{
			// We can't mount a formatted filesystem, something bad has happened
			system_scheduler->post_task_prio(notify_bad_filesystem_error, "GenTrackerFileSystemError");
			return;
		}
	}

	// Start reed switch monitoring and dispatch events to state machine
	reed_switch->start([](ReedSwitchGesture s) { ReedSwitchEvent e; e.state = s; dispatch(e); });

	// If magnet is already present at boot, NrfSwitch::resume() reads the state
	// but does NOT fire a callback (only state CHANGES trigger callbacks).
	// Manually dispatch an ENGAGE event so the FSM knows the magnet is present.
	if (reed_switch->is_engaged()) {
		DEBUG_INFO("BootState: Magnet already present at boot | dispatching ENGAGE");
		ReedSwitchEvent e;
		e.state = ReedSwitchGesture::ENGAGE;
		dispatch(e);
	}

	try {
		// The underlying classes will create the files on the filesystem if they do not
		// already yet exist
		LoggerManager::create();
		configuration_store->init();
	    DEBUG_INFO("Firmware Version: %s", FW_APP_VERSION_STR_C);
		LoggerManager::show_info();
		DEBUG_INFO("configuration_store: is_valid=%u", configuration_store->is_valid());
		DEBUG_INFO("reset cause: %s", PMU::reset_cause().c_str());
		PMU::print_stack();

		// Check if we just applied a firmware update (green blink for 3s)
		if (PMU::was_firmware_updated()) {
			DEBUG_INFO("Firmware update was applied successfully!");
			led_handle::dispatch<SetLEDFirmwareApplied>({});
			PMU::delay_ms(3000);
		}

		// Transition to PreOperational state after initialisation
		system_scheduler->post_task_prio([this](){
			kick_watchdog();
			transit<PreOperationalState>();
		},
		"GenTrackerBootStateTransitPreOperationalState",
		Scheduler::DEFAULT_PRIORITY,
		1000);
	} catch (...) {
		system_scheduler->post_task_prio(notify_bad_filesystem_error, "GenTrackerFilesystemError");
	}
}

void BootState::exit() {

	DEBUG_INFO("exit: BootState");

	// Turn status LED off to indicate exit from boot state
	led_handle::dispatch<SetLEDOff>({});
}

void OffState::entry() {
	DEBUG_INFO("entry: OffState");
	led_handle::dispatch<SetLEDPowerDown>({});
	m_off_state_task = system_scheduler->post_task_prio([](){
		// Force LED off at hardware level before powerdown
		status_led->off();
		if (ext_status_led) ext_status_led->off();
		buzz_handle::dispatch<SetBuzzOff>({});
		PMU::powerdown();
	},
	"GenTrackerOffStateTransitPowerDown",
	Scheduler::DEFAULT_PRIORITY, OFF_LED_PERIOD_MS);
}

void OffState::exit() {
	DEBUG_INFO("exit: OffState");
	system_scheduler->cancel_task(m_off_state_task);
	led_handle::dispatch<SetLEDOff>({});
}

void PreOperationalState::entry() {
	DEBUG_INFO("entry: PreOperationalState");
	if (configuration_store->is_valid()) {
		// Force battery monitor to update its levels
		try {
			battery_monitor->update();
		} catch (...) {
			DEBUG_WARN("PreOperationalState: battery update failed — continuing with defaults");
		}
		if (battery_monitor->is_battery_critical()) {
			transit<BatteryCriticalState>();
			return;
		}

		if (battery_monitor->is_battery_low())
			led_handle::dispatch<SetLEDPreOperationalBatteryLow>({});
		else
			led_handle::dispatch<SetLEDPreOperationalBatteryNominal>({});

		m_preop_state_task = system_scheduler->post_task_prio([this](){
			// If a confirmation gesture is in progress, don't transit yet —
			// the confirmation handler or timeout will handle the transition.
			if (m_confirmation_pending != ConfirmationPending::NONE)
				return;
			transit<OperationalState>();
		},
		"GenTrackerPreOperationalStateTransitOperationalState",
		Scheduler::DEFAULT_PRIORITY, TRANSIT_PERIOD_MS);
	} else {
		led_handle::dispatch<SetLEDPreOperationalError>({});
		m_preop_state_task = system_scheduler->post_task_prio([this](){
			transit<ErrorState>();
		},
		"GenTrackerPreOperationalStateTransitErrorState",
		Scheduler::DEFAULT_PRIORITY, TRANSIT_PERIOD_MS);
	}
}

void PreOperationalState::exit() {
	DEBUG_INFO("exit: PreOperationalState");
	system_scheduler->cancel_task(m_preop_state_task);
	led_handle::dispatch<SetLEDOff>({});
}

void OperationalState::entry() {
	DEBUG_INFO("entry: OperationalState");

	battery_monitor->subscribe(*this);
	led_handle::dispatch<SetLEDOff>({});
	buzz_handle::dispatch<SetBuzzOff>({});

	ServiceManager::startall([this](ServiceEvent& e) {
		service_event_handler(e);
	});

	BaseDebugMode debug_mode = configuration_store->read_param<BaseDebugMode>(ParamID::DEBUG_OUTPUT_MODE);
	if (debug_mode == BaseDebugMode::BLE_NUS) {
		set_ble_device_name();
		ble_service->start([](BLEServiceEvent&){ return 0; });
		g_debug_mode = debug_mode;
	}
}

void OperationalState::react(BatteryMonitorEventVoltageCritical const &) {
	DEBUG_INFO("OperationalState::react: BatteryMonitorEventVoltageCritical");
	system_scheduler->post_task_prio([this]() {
		transit<BatteryCriticalState>();
	}, "BatteryCriticalHandler", Scheduler::DEFAULT_PRIORITY, 1000);
}

void OperationalState::service_event_handler(ServiceEvent& e) {

	// Notify event to all peer services
	ServiceManager::notify_peer_event(e);

	if (e.event_source == ServiceIdentifier::GNSS_SENSOR) {
		if (e.event_type == ServiceEventType::SERVICE_ACTIVE) {
			led_handle::dispatch<SetLEDGNSSOn>({});
		} else if (e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
			auto *gps_log = std::get_if<GPSLogEntry>(&e.event_data);
			if (gps_log && gps_log->info.valid)
				led_handle::dispatch<SetLEDGNSSOffWithFix>({});
			else
				led_handle::dispatch<SetLEDGNSSOffWithoutFix>({});
		}
		return;
	}
	else if (e.event_source == ServiceIdentifier::UW_SENSOR) {
		if (e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
			bool is_underwater = std::get<bool>(e.event_data);
			if (is_underwater)
				led_handle::dispatch<SetLEDDiveDetected>({});
			else
				led_handle::dispatch<SetLEDSurfaceDetected>({});
		}
		return;
	}
	else if (e.event_source == ServiceIdentifier::ARGOS_TX) {
		if (e.event_type == ServiceEventType::SERVICE_ACTIVE) {
			led_handle::dispatch<SetLEDArgosTX>({});
		}
		else if (e.event_type == ServiceEventType::SERVICE_INACTIVE) {
			led_handle::dispatch<SetLEDArgosTXComplete>({});
		}
	}
}

void OperationalState::exit() {
	DEBUG_INFO("exit: OperationalState");
	led_handle::dispatch<SetLEDOff>({});
	ServiceManager::stopall();
	BaseDebugMode debug_mode = configuration_store->read_param<BaseDebugMode>(ParamID::DEBUG_OUTPUT_MODE);
	if (debug_mode == BaseDebugMode::BLE_NUS) {
		g_debug_mode = BaseDebugMode::USB_CDC;  // Reset to USB CDC when exiting BLE debug mode
		ble_service->stop();
		DEBUG_TRACE("exit: OperationalState: BLE service stopped");
		PMU::delay_ms(100);
	}
	battery_monitor->unsubscribe(*this);
}

void ConfigurationState::entry() {
	DEBUG_INFO("entry: ConfigurationState");

	// Flash the blue LED to indicate we have started BLE and we are
	// waiting for a connection
	led_handle::dispatch<SetLEDConfigNotConnected>({});
	buzz_handle::dispatch<SetBuzzConfiguration>({});

	set_ble_device_name();
	ble_service->start([this](BLEServiceEvent& event) -> int { return on_ble_event(event); } );
	restart_inactivity_timeout();

#ifdef USB_DTE_ENABLED
	// Start USB DTE polling (runs in parallel with BLE)
	DEBUG_TRACE("ConfigurationState: Starting DTE polling");
	schedule_usb_poll();
#endif
}

void ConfigurationState::exit() {
	DEBUG_INFO("exit: ConfigurationState");
	system_scheduler->cancel_task(m_ble_inactivity_timeout_task);
#ifdef USB_DTE_ENABLED
	system_scheduler->cancel_task(m_usb_poll_task);
#endif
	ble_service->stop();
	led_handle::dispatch<SetLEDOff>({});
}

void GenTracker::set_ble_device_name() {
	std::string device_model = configuration_store->read_param<std::string>(ParamID::DEVICE_MODEL);
	unsigned int identifier = configuration_store->read_param<unsigned int>(ParamID::ARGOS_DECID);
	std::string device_name = device_model + " " + std::to_string(identifier);
	DEBUG_TRACE("GenTracker::set_ble_device_name: %s", device_name.c_str());
	ble_service->set_device_name(device_name);
}

int ConfigurationState::on_ble_event(BLEServiceEvent& event) {
	int rc = 0;

	switch (event.event_type) {
	case BLEServiceEventType::CONNECTED:
		DEBUG_TRACE("ConfigurationState::on_ble_event: CONNECTED");
		// Indicate DTE connection is made
		dte_handler->reset_state();
		dte_handler->set_async_write([](const std::string& msg) {
			if (ble_service) ble_service->write(msg);
		});
		led_handle::dispatch<SetLEDConfigConnected>({});
		restart_inactivity_timeout();
		break;
	case BLEServiceEventType::DISCONNECTED:
		DEBUG_TRACE("ConfigurationState::on_ble_event: DISCONNECTED");
		ota_updater->abort_file_transfer();
		led_handle::dispatch<SetLEDConfigNotConnected>({});
		break;
	case BLEServiceEventType::DTE_DATA_RECEIVED:
		DEBUG_TRACE("ConfigurationState::on_ble_event: DTE_DATA_RECEIVED");
		restart_inactivity_timeout();
		system_scheduler->post_task_prio(std::bind(&ConfigurationState::process_received_data, this),
				"BLEProcessReceivedData");
		break;
	case BLEServiceEventType::OTA_START:
		DEBUG_INFO("ConfigurationState::on_ble_event: OTA_START");
		restart_inactivity_timeout();
		ota_updater->start_file_transfer((OTAFileIdentifier)event.file_id, event.file_size, event.crc32);
		led_handle::dispatch<SetLEDDFUUpdate>({});
		break;
	case BLEServiceEventType::OTA_END:
		DEBUG_INFO("ConfigurationState::on_ble_event: OTA_END");
		restart_inactivity_timeout();
		ota_updater->complete_file_transfer();
		led_handle::dispatch<SetLEDConfigConnected>({});
		system_scheduler->post_task_prio(std::bind(&OTAFileUpdater::apply_file_update, ota_updater),
				"BLEApplyOTAFileUpdate"
				);
		break;
	case BLEServiceEventType::OTA_ABORT:
		DEBUG_INFO("ConfigurationState::on_ble_event: OTA_ABORT");
		restart_inactivity_timeout();
		ota_updater->abort_file_transfer();
		led_handle::dispatch<SetLEDConfigConnected>({});
		break;
	case BLEServiceEventType::OTA_FILE_DATA:
		//DEBUG_TRACE("ConfigurationState::on_ble_event: OTA_FILE_DATA");
		restart_inactivity_timeout();
		ota_updater->write_file_data(event.data, event.length);
		break;
	default:
		break;
	}

	return rc;
}


void ConfigurationState::on_ble_inactivity_timeout() {
	DEBUG_INFO("BLE Inactivity Timeout");
	transit<OffState>();
}

void ConfigurationState::restart_inactivity_timeout() {
	//DEBUG_TRACE("Restart BLE inactivity timeout: %lu", system_timer->get_counter());
	system_scheduler->cancel_task(m_ble_inactivity_timeout_task);
	m_ble_inactivity_timeout_task = system_scheduler->post_task_prio(std::bind(&ConfigurationState::on_ble_inactivity_timeout, this),
			"BLEInactivityTimeout",
			Scheduler::DEFAULT_PRIORITY, BLE_INACTIVITY_TIMEOUT_MS);
}

void ConfigurationState::process_received_data() {
	auto req = ble_service->read_line();

	if (req.size())
	{
		DEBUG_TRACE("received %u bytes:", req.size());
#if defined(DEBUG_ENABLE) && DEBUG_LEVEL >= 4
		printf("%s\n", req.c_str());
#endif

		std::string resp;
		DTEAction action;

		do
		{
			action = dte_handler->handle_dte_message(req, resp);
			if (resp.size())
			{
				DEBUG_TRACE("responding: %s", resp.c_str());
				if (!ble_service->write(resp)) {
					dte_handler->reset_state();
					break;
				}

				// Reset inactivity timeout whenever we send a response
				// This is important during a command sequences that can take
				// a long time to complete (eg DUMPD)
				restart_inactivity_timeout();

				// We must also kick the watchdog here since the main scheduler
				// is being deferred
				PMU::kick_watchdog();
			}

			if (action == DTEAction::FACTR)
			{
				DEBUG_INFO("Perform factory reset of configuration store");
				configuration_store->factory_reset();

				// After formatting the filesystem we must do a system reset so that
				// the boot up procedure can repopulate files
				PMU::reset(false);
			}
			else if (action == DTEAction::RESET)
			{
				DEBUG_INFO("Perform device reset");

				// Execute this after 3 seconds to allow time for the BLE response to be sent
				system_scheduler->post_task_prio([](){
					PMU::reset(false);
				},
				"DTEActionPMUReset",
				Scheduler::DEFAULT_PRIORITY, 3000);
			}
			else if (action == DTEAction::SECUR)
			{
				// TODO: add secure procedure
				DEBUG_INFO("Perform secure procedure");
			}
			else if (action == DTEAction::CONFIG_UPDATED)
			{
				// TODO: reserved for future use
			}

		} while (action == DTEAction::AGAIN);
	}
}

#ifdef USB_DTE_ENABLED
void ConfigurationState::schedule_usb_poll() {
	m_usb_poll_task = system_scheduler->post_task_prio(
		std::bind(&ConfigurationState::process_usb_data, this),
		"USBDTEPoll",
		Scheduler::DEFAULT_PRIORITY,
		USB_POLL_INTERVAL_MS
	);
}

void ConfigurationState::process_usb_data() {
	auto& usb = UsbInterface::get_instance();

	// GNSS UART bridge mode: forward USB ↔ GNSS UART directly (binary UBX)
	if (gps_device && gps_device->is_bridge_active()) {
		// Process UART RX → USB (via passthrough callback)
		gps_device->bridge_process_rx();

		// Process USB → UART (raw bytes, no line parsing — UBX is binary)
		if (usb.has_data()) {
			char buf[256];
			int n = NrfUSB::read(buf, sizeof(buf));
			if (n > 0) {
				restart_inactivity_timeout();
				PMU::kick_watchdog();

				// Check for exit sequence "+++" (text typed in terminal)
				if (n == 3 && buf[0] == '+' && buf[1] == '+' && buf[2] == '+') {
					gps_device->stop_bridge();
					usb.write("\r\n[BRIDGE OFF]\r\n");
					schedule_usb_poll();
					return;
				}
				// Also check with line ending
				if (n >= 3 && buf[0] == '+' && buf[1] == '+' && buf[2] == '+' &&
				    (n == 3 || buf[3] == '\r' || buf[3] == '\n')) {
					gps_device->stop_bridge();
					usb.write("\r\n[BRIDGE OFF]\r\n");
					schedule_usb_poll();
					return;
				}

				gps_device->bridge_send(
					reinterpret_cast<const uint8_t*>(buf), n);
			}
		}

		schedule_usb_poll();
		return;
	}

#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	// LoRa UART bridge mode: forward USB ↔ RAK3172 UART directly
	if (lora_device_instance && lora_device_instance->is_bridge_active()) {
		// Process UART RX → USB (via passthrough callback)
		lora_device_instance->bridge_process_rx();

		// Process USB → UART
		if (usb.has_data()) {
			auto line = usb.read_line();
			if (line.size()) {
				restart_inactivity_timeout();
				PMU::kick_watchdog();

				// Check for exit sequence "+++"
				if (line == "+++") {
					lora_device_instance->stop_bridge();
					usb.write("\r\n[BRIDGE OFF]\r\n");
					schedule_usb_poll();
					return;
				}

				// Forward to RAK3172 UART with \r\n termination
				std::string data = line + "\r\n";
				lora_device_instance->bridge_send(
					reinterpret_cast<const uint8_t*>(data.c_str()), data.size());
			}
		}

		schedule_usb_poll();
		return;
	}
#endif

	// Check if USB has data
	if (usb.has_data()) {
		auto req = usb.read_line();

		if (req.size()) {
			// DTE protocol expects trailing \r which read_line() strips
			req += '\r';
			// Suppress console debug logs during DTE exchange to avoid
			// polluting the USB DTE response stream
			auto *saved_log = DebugLogger::console_log;
			DebugLogger::console_log = nullptr;

			std::string resp;
			DTEAction action;

			do {
				action = dte_handler->handle_dte_message(req, resp);
				if (resp.size()) {
					usb.write(resp);

					// Reset inactivity timeout on USB activity too
					restart_inactivity_timeout();

					// Kick the watchdog
					PMU::kick_watchdog();
				}

				if (action == DTEAction::FACTR) {
					DEBUG_INFO("Perform factory reset of configuration store (USB)");
					configuration_store->factory_reset();
					PMU::reset(false);
				}
				else if (action == DTEAction::RESET) {
					DEBUG_INFO("Perform device reset (USB)");
					system_scheduler->post_task_prio([](){
						PMU::reset(false);
					},
					"DTEActionPMUReset",
					Scheduler::DEFAULT_PRIORITY, 3000);
				}
				else if (action == DTEAction::SECUR) {
					DEBUG_INFO("Perform secure procedure (USB)");
				}

			} while (action == DTEAction::AGAIN);

			// Restore console logging
			DebugLogger::console_log = saved_log;
		}
	}

	// Schedule next poll
	schedule_usb_poll();
}
#else
// Empty implementations when USB DTE is disabled
void ConfigurationState::schedule_usb_poll() {}
void ConfigurationState::process_usb_data() {}
#endif

void BatteryCriticalState::entry() {
	DEBUG_INFO("entry: BatteryCriticalState");
#ifdef EXTERNAL_WAKEUP
	// If magnet is present, user wants to configure via BLE - don't powerdown
	if (reed_switch->is_engaged()) {
		DEBUG_INFO("BatteryCriticalState: Magnet detected, entering configuration mode");
		transit<ConfigurationState>();
		return;
	}
	DEBUG_INFO("EXTERNAL_WAKEUP: Critical battery | immediate powerdown");
	status_led->off();
	if (ext_status_led) ext_status_led->off();
	PMU::powerdown();
#else
	led_handle::dispatch<SetLEDBatteryCritical>({});
	buzz_handle::dispatch<SetBuzzOff>({});
	m_transit_task = system_scheduler->post_task_prio([this](){
		transit<OffState>();
	},
	"GenTrackerBatteryCriticalTransitOffState",
	Scheduler::DEFAULT_PRIORITY, BATTERY_CRITICAL_TIMEOUT_MS);
#endif
}

void BatteryCriticalState::exit() {
	DEBUG_INFO("exit: BatteryCriticalState");
	system_scheduler->cancel_task(m_transit_task);
	led_handle::dispatch<SetLEDOff>({});
}

void ErrorState::entry() {
	DEBUG_INFO("entry: ErrorState");
	led_handle::dispatch<SetLEDError>({});
	buzz_handle::dispatch<SetBuzzOff>({});
	m_shutdown_task = system_scheduler->post_task_prio([this](){
		transit<OffState>();
	},
	"GenTrackerErrorStateTransitOffState",
	Scheduler::DEFAULT_PRIORITY, 5000);
}

void ErrorState::exit() {
	DEBUG_INFO("exit: ErrorState");
	system_scheduler->cancel_task(m_shutdown_task);
	led_handle::dispatch<SetLEDOff>({});
}
