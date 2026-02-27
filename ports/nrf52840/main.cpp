#include "sws_analog_service.hpp"
#include "gnss_detector_service.hpp"
#if ENABLE_PRESSURE_SENSOR
#include "pressure_detector_service.hpp"
#include "pressure_sensor_service.hpp"
#endif
#if ENABLE_ALS_SENSOR
#include "als_sensor_service.hpp"
#endif
#if ENABLE_PH_SENSOR
#include "ph_sensor_service.hpp"
#endif
#if ENABLE_SEA_TEMP_SENSOR
#include "sea_temp_sensor_service.hpp"
#endif
#if ENABLE_CDT_SENSOR
#include "cdt_sensor_service.hpp"
#endif
#include "gps_service.hpp"
#if ENABLE_AXL_SENSOR
#include "axl_sensor_service.hpp"
#endif
#include "cam_service.hpp"
#include "argos_tx_service.hpp"
#include "argos_rx_service.hpp"
#include "sys_log.hpp"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log_redirect.h"
#include "nrfx_spim.h"
#include "ble_interface.hpp"
#include "ota_flash_file_updater.hpp"
#include "dte_handler.hpp"
#include "nrf_memory_access.hpp"
#include "config_store_fs.hpp"
#include "debug.hpp"
#include "console_log.hpp"
#include "bsp.hpp"
#include "gentracker.hpp"
#include "nrf_timer.hpp"
#include "nrf_switch.hpp"
#include "reed.hpp"
#include "nrf_rtc.hpp"
#include "gpio.hpp"
#include "is25_flash.hpp"
#include "nrf_rgb_led.hpp"

// Battery monitor includes based on BATTERY_MONITOR_TYPE
#if defined(BATTERY_MONITOR_ANALOG)
#include "nrf_battery_mon.hpp"
#elif defined(BATTERY_MONITOR_FAKE)
#include "fake_battery_mon.hpp"
#elif defined(BATTERY_MONITOR_STC3117)
#include "stc3117_gasgauge.hpp"
#endif

#if ENABLE_ALS_SENSOR
#include "ltr_303.hpp"
#endif
#if ENABLE_PH_SENSOR
#include "oem_ph.hpp"
#endif
#if ENABLE_SEA_TEMP_SENSOR
#include "oem_rtd.hpp"
#include "ezo_rtd.hpp"
#include "TSYS01.hpp"
#endif
#if ENABLE_CDT_SENSOR || ENABLE_PRESSURE_SENSOR
#include "ms58xx.hpp"
#include "bar100.hpp"
#include "lps28dfw.hpp"
#endif
#if ENABLE_CDT_SENSOR
#include "cdt.hpp"
#include "ad5933.hpp"
#endif
#if ENABLE_AXL_SENSOR
#include "bma400.hpp"
#endif
#include "fs_log.hpp"
#include "nrf_i2c.hpp"
#include "nrf_usb.hpp"
#include "gpio_led.hpp"
#include "heap.h"
#include "etl/error_handler.h"
#include "memory_monitor_service.hpp"
#include "dive_mode_service.hpp"
#include "gpio_buzzer.hpp"
#ifdef CAM_PWR_EN
#include "runcam.hpp"
#endif

// Always use M10Q GPS module
#include "m10qasync.hpp"
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
#include "smd_sat.hpp"
#else
#include "kim2.hpp"
#endif
#if ENABLE_THERMISTOR_SENSOR
#include "thermistor_sensor_service.hpp"
#include "thermistor.hpp"
#endif
#ifdef DEBUG_UART_TX_PIN
#include "nrf_debug_uart.hpp"
#endif

FileSystem *main_filesystem;

ConfigurationStore *configuration_store;
BLEService *ble_service;
OTAFileUpdater *ota_updater;
MemoryAccess *memory_access;
Timer *system_timer;
Scheduler *system_scheduler;
RGBLed *status_led;
Led *ext_status_led;
ReedSwitch *reed_switch;
DTEHandler *dte_handler;
RTC *rtc;
BatteryMonitor *battery_monitor;
GPSDevice *gps_device;
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
SmdSat *smd_sat_instance = nullptr;  // For SMD DFU OTA support
#endif
#if defined(BOARD_RSPB)
BaseDebugMode g_debug_mode = BaseDebugMode::UART;     // Default debug output to UART on SWO pin for RSPB
#else
BaseDebugMode g_debug_mode = BaseDebugMode::USB_CDC;  // Default debug output to USB CDC for LINKIT
#endif
Buzzer *buzzer_ctl;

static bool m_is_debug_init = false;

// FSM initial state -> BootState
FSM_INITIAL_STATE(GenTracker, BootState)

// Reserve the last 1MB of IS25 flash memory for firmware updates
#define OTA_UPDATE_RESERVED_BLOCKS ((1024 * 1024) / IS25_BLOCK_SIZE)

// Reed switch debouncing time (ms) - increased from 25ms to filter noise from VSENSORS switching
#define REED_SWITCH_DEBOUNCE_TIME_MS    250


extern "C" void HardFault_Handler() {
	for (;;)
	{
#ifdef NDEBUG
		PMU::save_stack(PMULogType::HARDFAULT);
		PMU::reset(false);
#else
		// Hardfault occurred
#ifdef GPIO_LED_REG
		GPIOPins::set(GPIO_LED_REG);
#endif
		GPIOPins::clear(BSP::GPIO::GPIO_LED_RED);
		GPIOPins::set(BSP::GPIO::GPIO_LED_GREEN);
		GPIOPins::set(BSP::GPIO::GPIO_LED_BLUE);
		nrf_delay_ms(50);
		GPIOPins::set(BSP::GPIO::GPIO_LED_RED);
		GPIOPins::set(BSP::GPIO::GPIO_LED_GREEN);
		GPIOPins::set(BSP::GPIO::GPIO_LED_BLUE);
		nrf_delay_ms(50);
		PMU::kick_watchdog();
#endif
	}
}

extern "C" void MemoryManagement_Handler(void)
{
	for (;;)
	{
#ifdef NDEBUG
		PMU::save_stack(PMULogType::MMAN);
		PMU::reset(false);
#else
		// Stack overflow detected
#ifdef GPIO_LED_REG
		GPIOPins::set(GPIO_LED_REG);
#endif
		GPIOPins::set(BSP::GPIO::GPIO_LED_GREEN);
		GPIOPins::set(BSP::GPIO::GPIO_LED_RED);
		GPIOPins::set(BSP::GPIO::GPIO_LED_BLUE);
		nrf_delay_ms(50);
		GPIOPins::clear(BSP::GPIO::GPIO_LED_GREEN);
		GPIOPins::clear(BSP::GPIO::GPIO_LED_RED);
		GPIOPins::set(BSP::GPIO::GPIO_LED_BLUE);
		nrf_delay_ms(50);
		PMU::kick_watchdog();
#endif
	}
}

extern "C" {
	void *__stack_check_guard = (void*)0xDEADBEEF;
	void __wrap___stack_chk_fail(void) {
#ifdef NDEBUG
		PMU::save_stack(PMULogType::STACK);
		PMU::reset(false);
#else
		for (;;)
		{
			// Stack corruption detected
#ifdef GPIO_LED_REG
			GPIOPins::set(GPIO_LED_REG);
#endif
			GPIOPins::set(BSP::GPIO::GPIO_LED_RED);
			GPIOPins::set(BSP::GPIO::GPIO_LED_GREEN);
			GPIOPins::set(BSP::GPIO::GPIO_LED_BLUE);
			nrf_delay_ms(50);
			GPIOPins::clear(BSP::GPIO::GPIO_LED_RED);
			GPIOPins::set(BSP::GPIO::GPIO_LED_GREEN);
			GPIOPins::clear(BSP::GPIO::GPIO_LED_BLUE);
			nrf_delay_ms(50);
			PMU::kick_watchdog();
		}
#endif
	}
}

extern "C" void vApplicationMallocFailedHook() {
	for (;;)
	{
#ifdef NDEBUG
		PMU::save_stack(PMULogType::MALLOC);
		PMU::reset(false);
#else
		// Out of heap memory occurred
#ifdef GPIO_LED_REG
		GPIOPins::set(GPIO_LED_REG);
#endif
		GPIOPins::set(BSP::GPIO::GPIO_LED_RED);
		GPIOPins::clear(BSP::GPIO::GPIO_LED_GREEN);
		GPIOPins::clear(BSP::GPIO::GPIO_LED_BLUE);
		nrf_delay_ms(50);
		GPIOPins::set(BSP::GPIO::GPIO_LED_RED);
		GPIOPins::set(BSP::GPIO::GPIO_LED_GREEN);
		GPIOPins::set(BSP::GPIO::GPIO_LED_BLUE);
		nrf_delay_ms(50);
		PMU::kick_watchdog();
#endif
	}
}

void etl_error_handler(const etl::exception& e)
{
	DEBUG_TRACE("ETL error: %s in %s : %u", e.what(), e.file_name(), e.line_number());

	for (;;)
	{
#ifdef NDEBUG
		PMU::save_stack(PMULogType::ETL);
		PMU::reset(false);
#else
		// ETL error occurred
#ifdef GPIO_LED_REG
		GPIOPins::set(GPIO_LED_REG);
#endif
		GPIOPins::clear(BSP::GPIO::GPIO_LED_RED);
		GPIOPins::set(BSP::GPIO::GPIO_LED_GREEN);
		GPIOPins::set(BSP::GPIO::GPIO_LED_BLUE);
		nrf_delay_ms(200);
		GPIOPins::set(BSP::GPIO::GPIO_LED_RED);
		GPIOPins::set(BSP::GPIO::GPIO_LED_GREEN);
		GPIOPins::set(BSP::GPIO::GPIO_LED_BLUE);
		nrf_delay_ms(200);
		PMU::kick_watchdog();
#endif
	}
}

// Redirect std::cout and printf output to UART, USB CDC, or BLE NUS
// We have to define this as extern "C" as we are overriding a weak C function
extern "C" int _write(int file, char *ptr, int len)
{
#ifdef DEBUG_UART_TX_PIN
	if (g_debug_mode == BaseDebugMode::UART && NrfDebugUart::is_init())
		NrfDebugUart::write(ptr, len);
	else
#endif
	if (g_debug_mode == BaseDebugMode::USB_CDC && m_is_debug_init)
		NrfUSB::write(ptr, len);
	else if (ble_service && !__get_IPSR() && g_debug_mode == BaseDebugMode::BLE_NUS) {
		ble_service->write(std::string(ptr, len));
	}
	return len;
}


int main()
{
	PMU::initialise();
#ifndef DEBUG_NO_WATCHDOG
	PMU::start_watchdog();
	PMU::kick_watchdog();
#endif
	GPIOPins::initialise();

	etl::error_handler::set_callback<etl_error_handler>();

// NOT NECESSARY : On linkit v4 i2c pullups are on V_ADC, not VSYS
// #ifdef GPIO_AG_PWR_PIN
// 	// Current backfeeds from 3V3 -> i2c pullups -> BMA400 -> GPIO_AG_PWR
// 	// Because of this we need to float our GPIO_AG_PWR pin to avoid sinking that current and thus increasing our sleep current
// 	nrf_gpio_cfg_default(BSP::GPIO_Inits[GPIO_AG_PWR_PIN].pin_number);
// #endif

	rtc = &NrfRTC::get_instance();
	NrfRTC::get_instance().init();

    DEBUG_TRACE("Timer..."); // Logs over USB not activated yet, needs system timer
	system_timer = &NrfTimer::get_instance();
	NrfTimer::get_instance().init();

	// Init debug output (UART, USB CDC, or BLE)
	m_is_debug_init = true;
	ConsoleLog console_log;
	DebugLogger::console_log = &console_log;
#ifdef DEBUG_UART_TX_PIN
	if (g_debug_mode == BaseDebugMode::UART) {
		NrfDebugUart::init(DEBUG_UART_TX_PIN);
	} else
#endif
	{
		NrfUSB::init();
	}
    setvbuf(stdout, NULL, _IONBF, 0);
    nrf_log_redirect_init();

	DEBUG_TRACE("RGB LED...");
	NrfRGBLed nrf_status_led("STATUS", BSP::GPIO::GPIO_LED_RED, BSP::GPIO::GPIO_LED_GREEN, BSP::GPIO::GPIO_LED_BLUE, RGBLedColor::WHITE);
	status_led = &nrf_status_led;

#ifdef EXT_LED_PIN
	DEBUG_TRACE("External LED...");
	GPIOLed gpio_led(EXT_LED_PIN);
	ext_status_led = &gpio_led;
#else
	ext_status_led = nullptr;
#endif

	// Ext LED off
	if (ext_status_led)
		ext_status_led->off();

    DEBUG_TRACE("Reed switch...");
    NrfSwitch nrf_reed_switch(BSP::GPIO::GPIO_REED_SW, REED_SWITCH_DEBOUNCE_TIME_MS, REED_SWITCH_ACTIVE_STATE);

#ifdef BUZZER_EN_PIN
	DEBUG_TRACE("Buzzer...");
	Buzzer buzzer(BUZZER_EN_PIN);
	buzzer_ctl = &buzzer;
#else
	buzzer_ctl = nullptr;
#endif

	DEBUG_TRACE("BLE...");
    BleInterface::get_instance().init();

	DEBUG_TRACE("IS25 flash...");
	Is25Flash is25_flash;
	is25_flash.init();

	// Check the reed switch is engaged for 3 seconds if this is a power on event
    DEBUG_TRACE("PMU Reset Cause = %s", PMU::reset_cause().c_str());

	// Note: VSENSORS power is now managed by each sensor via SensorsPowerGuard (reference counting)

#ifdef POWER_ON_RESET_REQUIRES_REED_SWITCH
#ifdef PSEUDO_POWER_OFF

	{
		SensorsPowerGuard power_guard;  // Acquire VSENSORS for I2C bus stability during init
		NrfI2C::init();
		PMU::hardware_version(); // Side-effect: I2C probes for hardware detection
#if ENABLE_SEA_TEMP_SENSOR
		{
			try {
				EZO_RTD_Sensor rtd; // Puts the device into standby mode
			} catch (...) {}
		}
#endif
		NrfI2C::uninit();
	}
	bool is_linkit_v3_v4 = (PMU::hardware_version() == "LinkIt V3") || (PMU::hardware_version() == "LinkIt V4");

	if ((is_linkit_v3_v4 && PMU::reset_cause() == "Pseudo Power On Reset") ||
		(!is_linkit_v3_v4 && (PMU::reset_cause() == "Power On Reset" ||
				PMU::reset_cause() == "Pseudo Power On Reset"))) {

		volatile bool power_on_ready = false;
		system_timer->start();
		Timer::TimerHandle timer_handle;

		// Check if switch starting state is active
		if (nrf_reed_switch.get_state()) {
			status_led->set(RGBLedColor::WHITE);
			BUZZER_ON(buzzer_ctl);
			timer_handle = system_timer->add_schedule([&power_on_ready]() {
				DEBUG_TRACE("Reed switch 3s period elapsed");
				power_on_ready = true;
			}, system_timer->get_counter() + 3000);
		} else {
		    // Turn status LED off
		    status_led->off();
			BUZZER_OFF(buzzer_ctl);
		}

		nrf_reed_switch.start([&timer_handle, &power_on_ready](bool state) {
			system_timer->cancel_schedule(timer_handle);
			if (state) {
				DEBUG_TRACE("Reed State: %u", state);
				GPIOPins::set(VSYS_SEL);
				status_led->set(RGBLedColor::WHITE);
				BUZZER_ON(buzzer_ctl);
				timer_handle = system_timer->add_schedule([&power_on_ready]() {
					DEBUG_TRACE("Reed switch 3s period elapsed");
					power_on_ready = true;
				}, system_timer->get_counter() + 3000);
			} else {
				status_led->off();
				GPIOPins::clear(VSYS_SEL);
				BUZZER_OFF(buzzer_ctl);
			}
		});

		Timer::TimerHandle wdog_handle;

		std::function<void()> kick_watchdog = [&wdog_handle,&kick_watchdog]() {
			wdog_handle = system_timer->add_schedule([&wdog_handle,&kick_watchdog]() {
				PMU::kick_watchdog();
				kick_watchdog();
			}, system_timer->get_counter() + (14 * 60 * 1000));
		};

		// Kick the watchdog periodically to avoid a WDT reset
		kick_watchdog();

		GPIOPins::clear(VSYS_SEL);
		while (!power_on_ready) {
			PMU::run();
		}

		GPIOPins::set(VSYS_SEL);
		InterruptLock lock;
		system_timer->cancel_schedule(wdog_handle);
		PMU::kick_watchdog();
		nrf_reed_switch.stop();
		BUZZER_BEEP_COUNT(buzzer_ctl,200,200,2);
	}
#else
	if (PMU::reset_cause() == "Power On Reset") {
		unsigned int countdown = 3000;
		DEBUG_TRACE("Enter Power On Reed Switch Check");
		while (countdown) {
			//DEBUG_TRACE("Reed Switch: %u", GPIOPins::value(BSP::GPIO_REED_SW));
			if (GPIOPins::value(BSP::GPIO_REED_SW) != REED_SWITCH_ACTIVE_STATE)
				break;
			PMU::delay_ms(1);
			countdown--;
		}

		if (countdown) {
			DEBUG_TRACE("Reed Switch Inactive -- Powering Down...");
			PMU::powerdown();
		}
		DEBUG_TRACE("Exiting Power On Reed Switch Check");
	}
#endif // POWER_CONTROL_PIN
#endif // POWER_ON_RESET_REQUIRES_REED_SWITCH


	DEBUG_TRACE("Scheduler...");
	Scheduler scheduler(system_timer);
	system_scheduler = &scheduler;

    // Initialise the I2C drivers
    // Note: VSENSORS must be on during I2C init to prevent bus appearing stuck
    {
        SensorsPowerGuard power_guard;
        NrfI2C::init();
    }

    DEBUG_TRACE("Reed gesture...");
	ReedSwitch reed_gesture_switch(nrf_reed_switch);
	reed_switch = &reed_gesture_switch;

	DEBUG_TRACE("LFS filesystem...");
	LFSFileSystem lfs_file_system(&is25_flash, IS25_BLOCK_COUNT - OTA_UPDATE_RESERVED_BLOCKS);
	main_filesystem = &lfs_file_system;

	// If we can't mount the filesystem then try to format it first and retry
	DEBUG_TRACE("Mount LFS filesystem...");

#ifdef FORCE_FORMAT_FILESYSTEM
	DEBUG_WARN("FORCE_FORMAT_FILESYSTEM enabled - formatting filesystem!");
	if (main_filesystem->format() < 0 || main_filesystem->mount() < 0)
	{
		DEBUG_ERROR("Failed to format LFS filesystem");
		PMU::powerdown();
	}
#else
	if (main_filesystem->mount() < 0)
	{
		DEBUG_TRACE("Format LFS filesystem...");
		if (main_filesystem->format() < 0 || main_filesystem->mount() < 0)
		{
			// We can't mount a formatted filesystem, something bad has happened
			DEBUG_ERROR("Failed to format LFS filesystem");
			PMU::powerdown();
		}
	}
#endif

	DEBUG_TRACE("Configuration store...");
	LFSConfigurationStore store(lfs_file_system);
	configuration_store = &store;
	configuration_store->init();

#ifdef EXTERNAL_WAKEUP
	{
		// Pseudo RTC: advance the last known RTC by WAKEUP_PERIOD at each boot.
		// This provides an approximate time immediately, before a GNSS fix corrects it.
		unsigned int last_rtc = configuration_store->read_param<unsigned int>(ParamID::LAST_KNOWN_RTC);
		unsigned int wakeup_period = configuration_store->read_param<unsigned int>(ParamID::WAKEUP_PERIOD);
		if (last_rtc > 0 && wakeup_period > 0) {
			unsigned int pseudo_rtc = last_rtc + wakeup_period;
			configuration_store->write_param(ParamID::LAST_KNOWN_RTC, pseudo_rtc);
			rtc->settime(static_cast<std::time_t>(pseudo_rtc));
			DEBUG_INFO("EXTERNAL_WAKEUP: Pseudo RTC set to %u (last=%u + period=%u)", pseudo_rtc, last_rtc, wakeup_period);
		} else {
			DEBUG_INFO("EXTERNAL_WAKEUP: No pseudo RTC available (last_rtc=%u, wakeup_period=%u)", last_rtc, wakeup_period);
		}

		// TPL5111 boot counter management
		// boot_count_increment() calls save_params() which also persists the LAST_KNOWN_RTC update above
		unsigned int boot_counter = configuration_store->boot_count_increment();
		DEBUG_INFO("EXTERNAL_WAKEUP: Boot counter = %u", boot_counter);

		// Check if this is not our turn to run based on modulo
		if (boot_counter > 0 && !configuration_store->boot_count_check_modulo(boot_counter)) {
			if (nrf_reed_switch.get_state()) {
				DEBUG_INFO("EXTERNAL_WAKEUP: Magnet detected | skipping modulo powerdown");
			} else {
				DEBUG_INFO("EXTERNAL_WAKEUP: Not our turn to run (modulo check) | powering down");
				PMU::powerdown();
			}
		}
		DEBUG_INFO("EXTERNAL_WAKEUP: Our turn to run | continuing boot");
	}
#endif

	DEBUG_TRACE("Battery monitor...");
	unsigned int critical_batt_level = configuration_store->read_param<unsigned int>(ParamID::LB_CRITICAL_THRESH);
	unsigned int low_batt_level = configuration_store->read_param<unsigned int>(ParamID::LB_THRESHOLD);

#if defined(BATTERY_MONITOR_ANALOG)
    #ifdef BATTERY_ADC
    NrfBatteryMonitor nrf_battery_monitor(BATTERY_ADC, BATT_CHEM_NCR18650_3100_3400,
    		(uint8_t)critical_batt_level, low_batt_level);
    battery_monitor = &nrf_battery_monitor;
    #else
    static BatteryMonitor stub_battery_monitor(low_batt_level, (uint8_t)critical_batt_level);
    battery_monitor = &stub_battery_monitor;
    #endif
#elif defined(BATTERY_MONITOR_FAKE)
    DEBUG_INFO("Using FAKE battery monitor (always 4.1V)");
    static FakeBatteryMonitor fake_battery_monitor((uint8_t)critical_batt_level, low_batt_level);
    battery_monitor = &fake_battery_monitor;
#elif defined(BATTERY_MONITOR_STC3117)
    DEBUG_INFO("Using STC3117 fuel gauge battery monitor");
    static GaugeBatteryMonitor stc3117_battery_monitor((uint8_t)critical_batt_level, low_batt_level);
    battery_monitor = &stc3117_battery_monitor;
#else
    #error "No battery monitor type defined! Set BATTERY_MONITOR_TYPE in CMake"
#endif

	DEBUG_TRACE("LFS System Log...");
	SysLogFormatter sys_log_formatter;
	FsLog fs_system_log(&lfs_file_system, "system.log", 1024*1024);
	fs_system_log.set_log_formatter(&sys_log_formatter);
	DebugLogger::system_log = &fs_system_log;

	DEBUG_TRACE("LFS (GPS) Sensor Log...");
	GPSLogFormatter fs_sensor_log_formatter;
	FsLog fs_sensor_log(&lfs_file_system, "sensor.log", 1024*1024);
	fs_sensor_log.set_log_formatter(&fs_sensor_log_formatter);

#if ENABLE_ALS_SENSOR
	DEBUG_TRACE("ALS Sensor Log...");
	ALSLogFormatter als_sensor_log_formatter;
	FsLog als_sensor_log(&lfs_file_system, "ALS", 1024*1024);
	als_sensor_log.set_log_formatter(&als_sensor_log_formatter);
#endif

#if ENABLE_PH_SENSOR
	DEBUG_TRACE("PH Sensor Log...");
	PHLogFormatter ph_sensor_log_formatter;
	FsLog ph_sensor_log(&lfs_file_system, "PH", 1024*1024);
	ph_sensor_log.set_log_formatter(&ph_sensor_log_formatter);
#endif

#if ENABLE_SEA_TEMP_SENSOR
	DEBUG_TRACE("RTD Sensor Log...");
	SeaTempLogFormatter rtd_sensor_log_formatter;
	FsLog rtd_sensor_log(&lfs_file_system, "RTD", 1024*1024);
	rtd_sensor_log.set_log_formatter(&rtd_sensor_log_formatter);

	DEBUG_TRACE("TSYS01 Sensor Log...");
	SeaTempLogFormatter tsys01_sensor_log_formatter;
	FsLog tsys01_sensor_log(&lfs_file_system, "TSYS01", 1024*1024);
	tsys01_sensor_log.set_log_formatter(&tsys01_sensor_log_formatter);
#endif

#if ENABLE_CDT_SENSOR
	DEBUG_TRACE("CDT Sensor Log...");
	CDTLogFormatter cdt_sensor_log_formatter;
	FsLog cdt_sensor_log(&lfs_file_system, "CDT", 1024*1024);
	cdt_sensor_log.set_log_formatter(&cdt_sensor_log_formatter);
#endif

#if ENABLE_PRESSURE_SENSOR
	DEBUG_TRACE("PRESSURE Sensor Log...");
	PressureLogFormatter pressure_sensor_log_formatter;
	FsLog pressure_sensor_log(&lfs_file_system, "PRESSURE", 1024*1024);
	pressure_sensor_log.set_log_formatter(&pressure_sensor_log_formatter);
#endif

#if ENABLE_AXL_SENSOR
	DEBUG_TRACE("AXL Sensor Log...");
	AXLLogFormatter axl_sensor_log_formatter;
	FsLog axl_sensor_log(&lfs_file_system, "AXL", 1024*1024);
	axl_sensor_log.set_log_formatter(&axl_sensor_log_formatter);
#endif

#if ENABLE_THERMISTOR_SENSOR
	DEBUG_TRACE("Thermistor Sensor Log...");
	ThermistorLogFormatter thermistor_sensor_log_formatter;
	FsLog thermistor_sensor_log(&lfs_file_system, "THERMISTOR", 1024*1024);
	thermistor_sensor_log.set_log_formatter(&thermistor_sensor_log_formatter);
#endif

#ifdef CAM_PWR_EN
	DEBUG_TRACE("CAM Sensor Log...");
	CAMLogFormatter cam_sensor_log_formatter;
	FsLog cam_sensor_log(&lfs_file_system, "CAM", 1024*1024);
	cam_sensor_log.set_log_formatter(&cam_sensor_log_formatter);
#endif

	DEBUG_TRACE("RAM access...");
	NrfMemoryAccess nrf_memory_access;
	memory_access = &nrf_memory_access;

	DEBUG_TRACE("DTE handler...");
	DTEHandler dte_handler_local;
	dte_handler = &dte_handler_local;

	DEBUG_TRACE("OTA updater...");
	ble_service = &BleInterface::get_instance();
	OTAFlashFileUpdater ota_flash_file_updater(&lfs_file_system, &is25_flash, IS25_BLOCK_COUNT - OTA_UPDATE_RESERVED_BLOCKS, OTA_UPDATE_RESERVED_BLOCKS);
	ota_updater = &ota_flash_file_updater;

	DEBUG_TRACE("SWS Analog...");
	static SWSAnalogService sws_analog;


#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	DEBUG_TRACE("SMD Satellite...");
	try {
		static SmdSat argos_smd;
		smd_sat_instance = &argos_smd;  // Store pointer for SMD DFU OTA
		static ArgosTxService argos_tx_service(argos_smd);
	} catch (...) {
		DEBUG_TRACE("SMD not detected");
		smd_sat_instance = nullptr;
	}
#else
	DEBUG_TRACE("KIM2...");
	try {
		static KIM2Device kim2;
		static ArgosTxService argos_tx_service(kim2);
	} catch (...) {
		DEBUG_TRACE("KIM2 not detected");
	}
#endif

	// Always use M10Q GPS module
	DEBUG_TRACE("GPS M10Q ...");
	try {
		static M10QAsyncReceiver m10q_gnss;
		gps_device = &m10q_gnss;
		static GPSService gps_service(m10q_gnss, &fs_sensor_log);
		static GNSSDetectorService gps_detector(m10q_gnss);
	} catch (...) {
		DEBUG_TRACE("GPS M10Q not detected");
	}

#if ENABLE_PRESSURE_SENSOR || ENABLE_CDT_SENSOR
	DEBUG_TRACE("Pressure Sensor...");
	PressureSensorDevice *pressure_sensor_devices[BSP::I2C_TOTAL_NUMBER] = {nullptr};
#ifndef DUMMY_PRESSURE_SENSOR
	for (unsigned int i = 0; i < BSP::I2C_TOTAL_NUMBER; i++) {
		// Sensor detection order - LPS28DFW is default for RSPB board
#if defined(BOARD_RSPB)
		static unsigned int i2caddr[4] = { LPS28DFW_ADDRESS, MS5803_ADDRESS, MS5837_ADDRESS, BAR100_ADDRESS };
		static std::string variant[4] = { "LPS28DFW", MS5803_VARIANT, MS5837_VARIANT, "BAR100-R3-RP" };
#else
		static unsigned int i2caddr[4] = { MS5803_ADDRESS, MS5837_ADDRESS, BAR100_ADDRESS, LPS28DFW_ADDRESS };
		static std::string variant[4] = { MS5803_VARIANT, MS5837_VARIANT, "BAR100-R3-RP", "LPS28DFW" };
#endif
		for (unsigned int j = 0; j < 4; j++) {
			try {
				PressureSensorDevice *device;
#if defined(BOARD_RSPB)
				if (j == 0) {
					device = new LPS28DFW(i, i2caddr[j]);
				} else if (j == 3) {
					device = new Bar100(i, i2caddr[j]);
				} else {
					device = new MS58xxLL(i, i2caddr[j], variant[j]);
				}
#else
				if (j == 2) {
					device = new Bar100(i, i2caddr[j]);
				} else if (j == 3) {
					device = new LPS28DFW(i, i2caddr[j]);
				} else {
					device = new MS58xxLL(i, i2caddr[j], variant[j]);
				}
#endif
				pressure_sensor_devices[i] = device;
				DEBUG_TRACE("%s: found on i2cbus=%u i2caddr=0x%02x", variant[j].c_str(), i, i2caddr[j]);
				break;
			} catch (...) {
				DEBUG_TRACE("Nothing detected on i2cbus=%u i2caddr=0x%02x", i, i2caddr[j]);
				pressure_sensor_devices[i] = nullptr;
			}
		}
	}
#else
	pressure_sensor_devices[0] = new PressureSensorDummyDevice();
#endif

#if ENABLE_CDT_SENSOR
	DEBUG_TRACE("AD5933...");
	AD5933 *ad5933_devices[BSP::I2C_TOTAL_NUMBER];
	for (unsigned int i = 0; i < BSP::I2C_TOTAL_NUMBER; i++) {
		try {
			ad5933_devices[i] = new AD5933LL(i, AD5933_ADDRESS);
			DEBUG_TRACE("AD5933: found on i2cbus=%u i2caddr=0x%02x", i, AD5933_ADDRESS);
		} catch (...) {
			DEBUG_TRACE("AD5933: not detected on i2cbus=%u i2caddr=0x%02x", i, AD5933_ADDRESS);
			ad5933_devices[i] = nullptr;
		}
	}
#endif

	bool cdt_present = false;
	bool standalone_pressure = false;
	// Iterate twice to allows flags to be set
	for (unsigned int x = 0; x < 2; x++) {
		// Check available devices on each bus
		for (unsigned int i = 0; i < BSP::I2C_TOTAL_NUMBER; i++) {
#if ENABLE_CDT_SENSOR
			if (!cdt_present && ad5933_devices[i] && pressure_sensor_devices[i]) {
				DEBUG_TRACE("CDT on bus %u...", i);
				cdt_present = true;
				static CDT cdt(*pressure_sensor_devices[i], *ad5933_devices[i]);
				static CDTSensorService cdt_sensor_service(cdt, &cdt_sensor_log);
			} else
#endif
#if ENABLE_PRESSURE_SENSOR
			if (!standalone_pressure && pressure_sensor_devices[i]) {
				DEBUG_TRACE("Standalone Pressure Sensor on bus %u...", i);
				standalone_pressure = true;
				static PressureSensor pressure_sensor(*pressure_sensor_devices[i]);
				static PressureDetectorService pressure_detector(pressure_sensor);
				static PressureSensorService pressure_sensor_service(pressure_sensor, &pressure_sensor_log);
			}
#endif
			(void)i; // Suppress unused warning when both disabled
		}

		if (standalone_pressure && cdt_present)
			break;
	}
#endif // ENABLE_PRESSURE_SENSOR || ENABLE_CDT_SENSOR

#if ENABLE_ALS_SENSOR
	DEBUG_TRACE("LTR303...");
	try {
		static LTR303 ltr303;
		static ALSSensorService als_sensor_service(ltr303, &als_sensor_log);
	} catch (...) {
		DEBUG_TRACE("LTR303: not detected");
	}
#endif

#if ENABLE_PH_SENSOR
	DEBUG_TRACE("OEM PH...");
	try {
		static OEM_PH_Sensor ph;
		static PHSensorService ph_sensor_service(ph, &ph_sensor_log);
	} catch (...) {
		DEBUG_TRACE("OEM PH: not detected");
	}
#endif

#if ENABLE_SEA_TEMP_SENSOR
	DEBUG_TRACE("EZO RTD...");
	try {
		static EZO_RTD_Sensor rtd;
		static SeaTempSensorService rtd_sensor_service(rtd, &rtd_sensor_log);
	} catch (ErrorCode e) {
		DEBUG_TRACE("EZO RTD: not detected [%04X]", e);
	}

	DEBUG_TRACE("TSYS01...");
	try {
		static TSYS01 tsys01;
		static SeaTempSensorService tsys01_sensor_service(tsys01, &tsys01_sensor_log);
	} catch (ErrorCode e) {
		DEBUG_TRACE("TSYS01: not detected [%04X]", e);
	}
#endif

#if ENABLE_AXL_SENSOR
	DEBUG_TRACE("BMA400...");
	try {
		static BMA400 bma400;
		static AXLSensorService axl_sensor_service(bma400, &axl_sensor_log);
	} catch (...) {
		DEBUG_TRACE("BMA400: not detected");
	}
#endif

#if ENABLE_THERMISTOR_SENSOR
	DEBUG_TRACE("Thermistor NTC...");
	try {
		static Thermistor thermistor(THERMISTOR_ADC);
		static ThermistorSensorService thermistor_sensor_service(thermistor, &thermistor_sensor_log);
	} catch (...) {
		DEBUG_TRACE("Thermistor: not detected");
	}
#endif

#ifdef CAM_PWR_EN
	DEBUG_TRACE("RunCam...");
	try {
		static RunCam run_cam;
		static CAMService cam_service(run_cam, &cam_sensor_log);
	} catch (...) {
		DEBUG_TRACE("RunCam: not detected");
	}
#endif

	DEBUG_TRACE("Memory monitor...");
	MemoryMonitorService memory_monitor_service;

	DEBUG_TRACE("Dive mode monitor...");
	DiveModeService dive_mode_service(nrf_reed_switch);

#ifdef EXTERNAL_WAKEUP
	// TPL5111 shutdown timer - powerdown after SHUTDOWN_TIMER seconds
	{
		unsigned int shutdown_timer = configuration_store->read_param<unsigned int>(ParamID::SHUTDOWN_TIMER);
		if (shutdown_timer > 0) {
			DEBUG_INFO("EXTERNAL_WAKEUP: Shutdown timer scheduled for %u seconds", shutdown_timer);
			system_scheduler->post_task_prio([]() {
				DEBUG_INFO("EXTERNAL_WAKEUP: Shutdown timer expired | powering down");
				PMU::powerdown();
			}, "SHUTDOWN_TIMER", Scheduler::DEFAULT_PRIORITY, shutdown_timer * 1000);
		} else {
			DEBUG_INFO("EXTERNAL_WAKEUP: Shutdown timer disabled (SHUTDOWN_TIMER=0)");
		}
	}
#endif

	DEBUG_TRACE("Entering main SM...");

	// This will initialise the FSM
	GenTracker::start();

	// The scheduler should run forever.  Any run-time exceptions should be handled and passed to FSM.
	while (true)
	{
		try {
#ifdef DEBUG_UART_TX_PIN
			if (g_debug_mode != BaseDebugMode::UART)
#endif
				NrfUSB::process();
			system_scheduler->run();
			PMU::run();
		} catch (ErrorCode e) {
			ErrorEvent event;
			event.error_code = e;
			GenTracker::dispatch(event);
		}
	}
}
