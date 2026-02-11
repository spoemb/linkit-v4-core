#include <map>
#include <cmath>
#include <algorithm>

#include "bma400.h"
#include "bma400.hpp"
#include "bma400_defs.h"

#include "debug.hpp"
#include "nrf_delay.h"
#include "nrf_i2c.hpp"
#include "bsp.hpp"
#include "pmu.hpp"
#include "error.hpp"
#include "config_store.hpp"

extern ConfigurationStore *configuration_store;

// ============================================================================
// BMA400 Device Manager
// ============================================================================

class BMA400LLManager
{
private:
	static inline uint8_t m_next_id = 0;
	static inline std::map<uint8_t, BMA400LL&> m_devices;

public:
	static uint8_t register_device(BMA400LL& device) {
		m_devices.insert({m_next_id, device});
		return m_next_id++;
	}

	static void unregister_device(uint8_t id) {
		m_devices.erase(id);
	}

	static BMA400LL& lookup_device(uint8_t id) {
		return m_devices.at(id);
	}
};

// ============================================================================
// BMA400LL Implementation
// ============================================================================

BMA400LL::BMA400LL(unsigned int bus, unsigned char addr, int wakeup_pin)
	: m_bus(bus)
	, m_addr(addr)
	, m_irq(NrfIRQ(wakeup_pin))
	, m_unique_id(BMA400LLManager::register_device(*this))
	, m_irq_pending(false)
	, m_g_range(BMA400_RANGE_2G)  // Default to 2G (matches config default)
	, m_power_mode(0)
	, m_wakeup_threshold(0.1)
	, m_wakeup_duration(1)
	, m_cal_x(0)
	, m_cal_y(0)
	, m_cal_z(1.0)
{
	DEBUG_TRACE("BMA400LL::BMA400LL: constructor entered, bus=%u addr=0x%02X wakeup_pin=%d", bus, addr, wakeup_pin);
	try {
		init();
	} catch (...) {
		DEBUG_ERROR("BMA400LL::BMA400LL: init() threw exception, unregistering device");
		BMA400LLManager::unregister_device(m_unique_id);
		throw;
	}
	DEBUG_TRACE("BMA400LL::BMA400LL: constructor complete");
}

BMA400LL::~BMA400LL()
{
	BMA400LLManager::unregister_device(m_unique_id);
}

void BMA400LL::init()
{
	int8_t rslt;

	DEBUG_TRACE("BMA400LL::init: configuring device interface");

	// Configure device interface
	m_bma400_dev.intf = BMA400_I2C_INTF;
	m_bma400_dev.intf_ptr = &m_unique_id;
	m_bma400_dev.read = (bma400_read_fptr_t)i2c_read;
	m_bma400_dev.write = (bma400_write_fptr_t)i2c_write;
	m_bma400_dev.delay_us = (bma400_delay_us_fptr_t)delay_us;
	m_bma400_dev.read_write_len = BMA400_READ_WRITE_LENGTH;
	m_bma400_dev.resolution = 12;

	// Initialize device (reads chip ID via I2C)
	DEBUG_TRACE("BMA400LL::init: calling bma400_init (I2C read chip ID)");
	rslt = bma400_init(&m_bma400_dev);
	check_result("bma400_init", rslt);

	// Soft reset to known state
	DEBUG_TRACE("BMA400LL::init: calling bma400_soft_reset");
	rslt = bma400_soft_reset(&m_bma400_dev);
	check_result("bma400_soft_reset", rslt);

	// Default to sleep mode (lowest power consumption)
	// When sensor is activated (wakeup enabled), it will switch to configured power mode
	DEBUG_TRACE("BMA400LL::init: setting sleep mode");
	setup_sleep_mode();

	DEBUG_TRACE("BMA400LL::init complete");
}

// ============================================================================
// I2C Interface Callbacks
// ============================================================================

int8_t BMA400LL::i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	BMA400LL& device = BMA400LLManager::lookup_device(*(uint8_t *)intf_ptr);

	if (!length)
		return BMA400_OK;

	if (length > BMA400_MAX_LEN)
		return BMA400_E_INVALID_CONFIG;

	uint8_t buffer[BMA400_MAX_LEN + 1];
	buffer[0] = reg_addr;
	memcpy(&buffer[1], reg_data, length);

	DEBUG_TRACE("BMA400LL::i2c_write: bus=%u addr=0x%02X reg=0x%02X len=%u", device.m_bus, device.m_addr, reg_addr, length);
	NrfI2C::write(device.m_bus, device.m_addr, buffer, length + 1, false);
	DEBUG_TRACE("BMA400LL::i2c_write: done");

	return BMA400_OK;
}

int8_t BMA400LL::i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	BMA400LL& device = BMA400LLManager::lookup_device(*(uint8_t *)intf_ptr);

	DEBUG_TRACE("BMA400LL::i2c_read: bus=%u addr=0x%02X reg=0x%02X len=%u", device.m_bus, device.m_addr, reg_addr, length);
	NrfI2C::write(device.m_bus, device.m_addr, &reg_addr, 1, true);
	DEBUG_TRACE("BMA400LL::i2c_read: write done, reading...");
	NrfI2C::read(device.m_bus, device.m_addr, reg_data, length);
	DEBUG_TRACE("BMA400LL::i2c_read: done");

	return BMA400_OK;
}

void BMA400LL::delay_us(uint32_t period, void *intf_ptr)
{
	(void)intf_ptr;
	PMU::delay_us(period);
}

// ============================================================================
// Utility Functions
// ============================================================================

double BMA400LL::lsb_to_ms2(int16_t accel_data, uint8_t g_range, uint8_t bit_width)
{
	const double gravity = 9.80665;
	// BMA400: for range ±Xg, sensitivity is (2*X*g) / 2^bit_width per LSB
	// For ±2g with 12-bit: 1g = 2048/2 = 1024 LSB
	int16_t half_scale = 1 << (bit_width - 1);
	return (gravity * accel_data * g_range) / half_scale;
}

uint8_t BMA400LL::range_to_g(uint8_t range_reg)
{
	static const uint8_t g_table[] = { 2, 4, 8, 16 };
	return (range_reg < 4) ? g_table[range_reg] : 4;
}

uint8_t BMA400LL::calculate_threshold_reg(double threshold_g, uint8_t acc_range)
{
	double lsb = static_cast<double>(1 << (2 + acc_range)) / 4096.0;
	uint16_t threshold_raw = static_cast<uint16_t>(threshold_g / lsb);
	return static_cast<uint8_t>(std::min<uint16_t>(255, threshold_raw));
}

void BMA400LL::check_result(const char* api_name, int8_t rslt)
{
	if (rslt == BMA400_OK) {
		DEBUG_TRACE("BMA400 [%s] OK", api_name);
		return;
	}

	const char* error_msg;
	switch (rslt) {
		case BMA400_E_NULL_PTR:       error_msg = "Null pointer"; break;
		case BMA400_E_COM_FAIL:       error_msg = "Communication failure"; break;
		case BMA400_E_INVALID_CONFIG: error_msg = "Invalid configuration"; break;
		case BMA400_E_DEV_NOT_FOUND:  error_msg = "Device not found"; break;
		default:                      error_msg = "Unknown error"; break;
	}

	DEBUG_ERROR("BMA400 [%s] Error %d: %s", api_name, rslt, error_msg);
	throw ErrorCode::I2C_COMMS_ERROR;
}

// ============================================================================
// Power Mode Configuration
// ============================================================================

void BMA400LL::setup_sleep_mode()
{
	int8_t rslt = bma400_set_power_mode(BMA400_MODE_SLEEP, &m_bma400_dev);
	check_result("set_power_mode(SLEEP)", rslt);
}

void BMA400LL::setup_low_power_mode()
{
	int8_t rslt;

	// Use NORMAL mode for consistent readings with calibration
	rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, &m_bma400_dev);
	check_result("set_power_mode(NORMAL)", rslt);

	// Configure accelerometer - same settings as calibration for consistency
	m_sensor_conf[0].type = BMA400_ACCEL;
	m_sensor_conf[0].param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_2;
	m_sensor_conf[0].param.accel.odr = BMA400_ODR_100HZ;
	m_sensor_conf[0].param.accel.range = m_g_range;
	m_sensor_conf[0].param.accel.osr = BMA400_ACCEL_OSR_SETTING_0;

	rslt = bma400_set_sensor_conf(m_sensor_conf, 1, &m_bma400_dev);
	check_result("set_sensor_conf(NORMAL)", rslt);
}

void BMA400LL::setup_normal_mode()
{
	int8_t rslt;

	rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, &m_bma400_dev);
	check_result("set_power_mode(NORMAL)", rslt);

	// Get current configuration
	rslt = bma400_get_sensor_conf(m_sensor_conf, 1, &m_bma400_dev);
	check_result("get_sensor_conf", rslt);

	// Configure accelerometer for normal mode
	m_sensor_conf[0].type = BMA400_ACCEL;
	m_sensor_conf[0].param.accel.odr = BMA400_ODR_100HZ;
	m_sensor_conf[0].param.accel.range = m_g_range;
	m_sensor_conf[0].param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;

	rslt = bma400_set_sensor_conf(m_sensor_conf, 1, &m_bma400_dev);
	check_result("set_sensor_conf(NORMAL)", rslt);
}

// ============================================================================
// Sensor Reading
// ============================================================================

void BMA400LL::read_xyz(double& x, double& y, double& z, int16_t& temperature)
{
	int8_t rslt;
	struct bma400_sensor_data data;
	uint8_t g_force = range_to_g(m_g_range);

	DEBUG_TRACE("BMA400::read_xyz: m_g_range=%u, g_force=%u, m_power_mode=%u, cal(%.4f,%.4f,%.4f)",
	            m_g_range, g_force, m_power_mode, m_cal_x, m_cal_y, m_cal_z);

	// Use same mode as calibration for consistent readings
	if (m_power_mode == 0) {
		setup_low_power_mode();
	} else {
		setup_normal_mode();
	}

	// Enable data ready interrupt
	struct bma400_int_enable int_en;
	int_en.type = BMA400_DRDY_INT_EN;
	int_en.conf = BMA400_ENABLE;
	rslt = bma400_enable_interrupt(&int_en, 1, &m_bma400_dev);
	check_result("enable_interrupt(DRDY)", rslt);

	// Wait for sensor to stabilize after mode change
	PMU::delay_ms(20);

	// Poll for data ready
	uint16_t int_status = 0;
	bool data_ready = false;
	for (int i = 0; i < BMA400_POLL_MAX_ATTEMPTS; i++) {
		rslt = bma400_get_interrupt_status(&int_status, &m_bma400_dev);
		if (int_status & BMA400_ASSERTED_DRDY_INT) {
			rslt = bma400_get_accel_data(BMA400_DATA_ONLY, &data, &m_bma400_dev);
			data_ready = true;
			break;
		}
		PMU::delay_ms(1);
	}
	if (!data_ready) {
		DEBUG_WARN("BMA400::read_xyz: timeout waiting for DRDY");
		setup_sleep_mode();
		return;
	}

	// Convert to m/s²
	double x_ms2 = lsb_to_ms2(data.x, g_force, 12);
	double y_ms2 = lsb_to_ms2(data.y, g_force, 12);
	double z_ms2 = lsb_to_ms2(data.z, g_force, 12);

	// Convert to g-force
	constexpr double G_PER_MS2 = 9.80665;
	double x_raw = x_ms2 / G_PER_MS2;
	double y_raw = y_ms2 / G_PER_MS2;
	double z_raw = z_ms2 / G_PER_MS2;

	// Apply calibration offsets (target: X=0, Y=0, Z=1)
	// calibrated = raw - measured_at_rest + target
	// For X,Y: target=0, so calibrated = raw - cal_x/y
	// For Z: target=1, so calibrated = raw - cal_z + 1 = raw - (cal_z - 1)
	x = x_raw - m_cal_x;
	y = y_raw - m_cal_y;
	z = z_raw - (m_cal_z - 1.0);

	DEBUG_INFO("BMA400::read_xyz: raw(%d,%d,%d) raw_g(%.2f,%.2f,%.2f) cal_g(%.2f,%.2f,%.2f)",
	           data.x, data.y, data.z, x_raw, y_raw, z_raw, x, y, z);

	// Return to sleep mode
	setup_sleep_mode();
}

int16_t BMA400LL::read_temperature()
{
	// Wait for temperature reading
	PMU::delay_us(11200);

	int16_t temperature_data;
	bma400_get_temperature_data(&temperature_data, &m_bma400_dev);

	DEBUG_TRACE("BMA400::read_temperature = %d (x0.1 C)", temperature_data);
	return temperature_data;
}

// ============================================================================
// Configuration Setters
// ============================================================================

void BMA400LL::set_wakeup_threshold(double threshold)
{
	m_wakeup_threshold = threshold;
}

void BMA400LL::set_wakeup_duration(double duration)
{
	m_wakeup_duration = duration;
}

void BMA400LL::set_range(unsigned int g_force)
{
	m_g_range = static_cast<uint8_t>(g_force);
}

void BMA400LL::set_power_mode(unsigned int power_mode)
{
	DEBUG_INFO("BMA400LL::set_power_mode: setting wakeup mode to %u (%s)",
	           power_mode, power_mode == 0 ? "LOW_POWER" : "NORMAL");
	m_power_mode = static_cast<uint8_t>(power_mode);
}

void BMA400LL::set_x_calibration(double x) { m_cal_x = x; }
void BMA400LL::set_y_calibration(double y) { m_cal_y = y; }
void BMA400LL::set_z_calibration(double z) { m_cal_z = z; }

// ============================================================================
// Calibration
// ============================================================================

void BMA400LL::calibrate_offset(uint8_t g_range, double& offset_x, double& offset_y, double& offset_z)
{
	int8_t rslt;
	const uint8_t n_samples = 200;
	const double gravity = 9.80665;
	double accumulated_x = 0, accumulated_y = 0, accumulated_z = 0;
	uint8_t g_force = range_to_g(g_range);

	// Set NORMAL mode for calibration (100Hz ODR)
	rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, &m_bma400_dev);
	check_result("calibrate_offset: set_power_mode", rslt);

	// Configure sensor for calibration
	struct bma400_sensor_conf conf;
	conf.type = BMA400_ACCEL;
	rslt = bma400_get_sensor_conf(&conf, 1, &m_bma400_dev);
	conf.param.accel.odr = BMA400_ODR_100HZ;
	conf.param.accel.range = g_range;
	conf.param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_2;
	rslt = bma400_set_sensor_conf(&conf, 1, &m_bma400_dev);
	check_result("calibrate_offset: set_sensor_conf", rslt);

	// Enable data ready interrupt
	struct bma400_int_enable int_en;
	int_en.type = BMA400_DRDY_INT_EN;
	int_en.conf = BMA400_ENABLE;
	rslt = bma400_enable_interrupt(&int_en, 1, &m_bma400_dev);
	check_result("calibrate_offset: enable_interrupt", rslt);

	// Wait for sensor to stabilize
	PMU::delay_ms(100);

	// Collect samples using DRDY interrupt
	DEBUG_TRACE("BMA400::calibrate_offset: collecting %u samples...", n_samples);
	for (uint8_t i = 0; i < n_samples; ++i) {
		// Wait for data ready
		uint16_t int_status = 0;
		for (int j = 0; j < 100; j++) {
			rslt = bma400_get_interrupt_status(&int_status, &m_bma400_dev);
			if (int_status & BMA400_ASSERTED_DRDY_INT) {
				break;
			}
			PMU::delay_ms(1);
		}

		struct bma400_sensor_data data;
		rslt = bma400_get_accel_data(BMA400_DATA_ONLY, &data, &m_bma400_dev);

		// Convert to g (divide by gravity to convert from m/s² to g)
		accumulated_x += lsb_to_ms2(data.x, g_force, 12) / gravity;
		accumulated_y += lsb_to_ms2(data.y, g_force, 12) / gravity;
		accumulated_z += lsb_to_ms2(data.z, g_force, 12) / gravity;
	}
	DEBUG_TRACE("BMA400::calibrate_offset: samples collected");

	// Return to sleep mode after calibration
	setup_sleep_mode();

	// Return calibration coefficients (measured averages)
	// These will be used to correct readings to X=0, Y=0, Z=1g
	offset_x = accumulated_x / n_samples;
	offset_y = accumulated_y / n_samples;
	offset_z = accumulated_z / n_samples;

	DEBUG_TRACE("BMA400::calibrate_offset: x=%.4f g, y=%.4f g, z=%.4f g", offset_x, offset_y, offset_z);
}

// ============================================================================
// Wakeup/Interrupt Management
// ============================================================================

void BMA400LL::enable_wakeup(std::function<void()> func)
{
	int8_t rslt;

	DEBUG_INFO("BMA400::enable_wakeup: configured power_mode=%u (%s wakeup)",
	           m_power_mode, m_power_mode == 0 ? "LOW_POWER" : "NORMAL");

	// Re-initialize device
	rslt = bma400_init(&m_bma400_dev);
	check_result("enable_wakeup: init", rslt);

	rslt = bma400_soft_reset(&m_bma400_dev);
	check_result("enable_wakeup: soft_reset", rslt);

	// Configure based on power mode
	if (m_power_mode == 0) {
		DEBUG_INFO("BMA400::enable_wakeup: using LOW_POWER wakeup mode");
		setup_low_power_mode();
		enable_wakeup_low_power(func);
	} else {
		DEBUG_INFO("BMA400::enable_wakeup: using NORMAL wakeup mode");
		setup_normal_mode();
		enable_wakeup_normal(func);
	}
}

void BMA400LL::enable_wakeup_low_power(std::function<void()> func)
{
	int8_t rslt;

	struct bma400_device_conf dev_conf;
	dev_conf.type = BMA400_AUTOWAKEUP_INT;

	rslt = bma400_get_device_conf(&dev_conf, 1, &m_bma400_dev);
	if (rslt == BMA400_OK) {
		dev_conf.param.wakeup.wakeup_axes_en = BMA400_AXIS_XYZ_EN;
		dev_conf.param.wakeup.wakeup_ref_update = BMA400_UPDATE_EVERY_TIME;
		dev_conf.param.wakeup.sample_count = BMA400_SAMPLE_COUNT_4;
		dev_conf.param.wakeup.int_wkup_threshold = calculate_threshold_reg(m_wakeup_threshold, m_g_range);
		dev_conf.param.wakeup.int_chan = BMA400_MAP_BOTH_INT_PINS;

		rslt = bma400_set_device_conf(&dev_conf, 1, &m_bma400_dev);
		if (rslt == BMA400_OK) {
			m_int_enable.type = BMA400_GEN1_INT_EN;
			m_int_enable.conf = BMA400_ENABLE;
			rslt = bma400_enable_interrupt(&m_int_enable, 1, &m_bma400_dev);
		}
	}

	m_irq.enable([this, func]() {
		if (!m_irq_pending) {
			m_irq_pending = true;
			func();
		}
	});
}

void BMA400LL::enable_wakeup_normal(std::function<void()> func)
{
	int8_t rslt;

	// Configure GEN1 interrupt for motion detection
	m_sensor_conf[1].type = BMA400_GEN1_INT;
	m_sensor_conf[1].param.gen_int.gen_int_thres = calculate_threshold_reg(m_wakeup_threshold, m_g_range);
	m_sensor_conf[1].param.gen_int.gen_int_dur = static_cast<uint8_t>(m_wakeup_duration - 1);
	m_sensor_conf[1].param.gen_int.axes_sel = BMA400_AXIS_XYZ_EN;
	m_sensor_conf[1].param.gen_int.data_src = BMA400_DATA_SRC_ACC_FILT2;
	m_sensor_conf[1].param.gen_int.criterion_sel = BMA400_ACTIVITY_INT;
	m_sensor_conf[1].param.gen_int.evaluate_axes = BMA400_ANY_AXES_INT;
	m_sensor_conf[1].param.gen_int.ref_update = BMA400_UPDATE_EVERY_TIME;
	m_sensor_conf[1].param.gen_int.hysteresis = BMA400_HYST_96_MG;
	m_sensor_conf[1].param.gen_int.int_thres_ref_x = 0;
	m_sensor_conf[1].param.gen_int.int_thres_ref_y = 0;
	m_sensor_conf[1].param.gen_int.int_thres_ref_z = 32;
	m_sensor_conf[1].param.gen_int.int_chan = BMA400_INT_CHANNEL_1;

	rslt = bma400_set_sensor_conf(m_sensor_conf, 2, &m_bma400_dev);
	check_result("enable_wakeup_normal: set_sensor_conf", rslt);

	m_int_enable.type = BMA400_GEN1_INT_EN;
	m_int_enable.conf = BMA400_ENABLE;
	rslt = bma400_enable_interrupt(&m_int_enable, 1, &m_bma400_dev);
	check_result("enable_wakeup_normal: enable_interrupt", rslt);

	m_irq.enable([this, func]() {
		if (!m_irq_pending) {
			m_irq_pending = true;
			func();
		}
	});
}

void BMA400LL::disable_wakeup()
{
	int8_t rslt;

	m_irq.disable();

	rslt = bma400_set_power_mode(BMA400_MODE_SLEEP, &m_bma400_dev);
	check_result("disable_wakeup: set_power_mode", rslt);

	m_int_enable.type = BMA400_GEN1_INT_EN;
	m_int_enable.conf = BMA400_DISABLE;
	rslt = bma400_enable_interrupt(&m_int_enable, 1, &m_bma400_dev);
	check_result("disable_wakeup: disable_interrupt", rslt);

	DEBUG_INFO("BMA400::disable_wakeup complete");
}

bool BMA400LL::check_and_clear_wakeup()
{
	InterruptLock lock;
	bool value = m_irq_pending;
	m_irq_pending = false;
	return value;
}

// ============================================================================
// BMA400 High-Level Sensor Class
// ============================================================================

BMA400::BMA400()
	: Sensor("AXL")
	, m_bma400(BMA400LL(BMA400_DEVICE, BMA400_ADDRESS, BMA400_WAKEUP_PIN))
	, m_cal(Calibration("AXL"))
	, m_last_x(0)
	, m_last_y(0)
	, m_last_z(0)
	, m_last_temperature(0)
	, m_last_activity(0)
{
	// Initialize g-range and power mode from configuration (needed for SCALW calibration via pylinkit)
	// The service may override these values later via calibration_write() if needed
	if (configuration_store) {
		unsigned int g_range = configuration_store->read_param<unsigned int>(ParamID::AXL_SENSOR_MEASUREMENT_RANGE);
		unsigned int power_mode = configuration_store->read_param<unsigned int>(ParamID::AXL_SENSOR_POWER_MODE);
		m_bma400.set_range(g_range);
		m_bma400.set_power_mode(power_mode);
		DEBUG_INFO("BMA400::BMA400: initialized from config g_range=%u, power_mode=%u", g_range, power_mode);
	}

	// Load calibration from file and apply to sensor
	load_calibration();
	DEBUG_TRACE("BMA400::BMA400 initialized");
}

void BMA400::load_calibration()
{
	try {
		double x = m_cal.read((unsigned int)CalibrationPoint::X);
		double y = m_cal.read((unsigned int)CalibrationPoint::Y);
		double z = m_cal.read((unsigned int)CalibrationPoint::Z);
		m_bma400.set_x_calibration(x);
		m_bma400.set_y_calibration(y);
		m_bma400.set_z_calibration(z);
		DEBUG_INFO("BMA400::load_calibration: loaded X=%.4f g, Y=%.4f g, Z=%.4f g", x, y, z);
	} catch (...) {
		// No calibration file or missing values, use defaults
		m_bma400.set_x_calibration(0.0);
		m_bma400.set_y_calibration(0.0);
		m_bma400.set_z_calibration(1.0);
		DEBUG_INFO("BMA400::load_calibration: using defaults X=0, Y=0, Z=1");
	}
}

double BMA400::read(unsigned int offset)
{
	switch (offset) {
		case 0: // Temperature
			return static_cast<double>(m_bma400.read_temperature()) / 10.0;

		case 1: // X (triggers new reading)
			m_bma400.read_xyz(m_last_x, m_last_y, m_last_z, m_last_temperature);
			return m_last_x;

		case 2: // Y (cached)
			return m_last_y;

		case 3: // Z (cached)
			return m_last_z;

		case 4: // Activity
			return compute_activity();

		case 5: // IRQ pending
			return static_cast<double>(m_bma400.check_and_clear_wakeup());

		default:
			return 0.0;
	}
}

double BMA400::compute_activity()
{
	// Compute magnitude of acceleration vector
	double g_magnitude = std::sqrt(m_last_x * m_last_x + m_last_y * m_last_y + m_last_z * m_last_z);

	// Activity = deviation from 1g (gravity at rest)
	// At rest: magnitude ≈ 1g → activity = 0
	// In motion: magnitude differs from 1g → activity > 0
	double activity_g = std::abs(g_magnitude - 1.0);

	// Get configured range for scaling
	uint8_t range_reg = m_bma400.get_range();
	static const uint8_t g_table[] = { 2, 4, 8, 16 };
	uint8_t g_range = (range_reg < 4) ? g_table[range_reg] : 4;

	// Max possible deviation from 1g is (range - 1) for high-g motion
	// or 1g for free-fall. Use (range - 1) as max for scaling.
	double max_deviation = static_cast<double>(g_range) - 1.0;
	if (max_deviation < 1.0) max_deviation = 1.0;

	// Clamp and scale to 0-255
	if (activity_g > max_deviation) activity_g = max_deviation;

	m_last_activity = static_cast<uint8_t>((activity_g / max_deviation) * 255.0);

	DEBUG_TRACE("BMA400::compute_activity: mag=%.2f, dev=%.2f, activity=%u",
	            g_magnitude, activity_g, m_last_activity);
	return static_cast<double>(m_last_activity);
}

void BMA400::calibration_write(const double value, const unsigned int offset)
{
	DEBUG_TRACE("BMA400::calibration_write: value=%.2f, offset=%u", value, offset);

	switch (offset) {
		case 0: // X calibration
			m_bma400.set_x_calibration(value);
			m_cal.write((unsigned int)CalibrationPoint::X, value);
			break;
		case 1: // Y calibration
			m_bma400.set_y_calibration(value);
			m_cal.write((unsigned int)CalibrationPoint::Y, value);
			break;
		case 2: // Z calibration
			m_bma400.set_z_calibration(value);
			m_cal.write((unsigned int)CalibrationPoint::Z, value);
			break;
		case 3: // Auto-calibrate all axes (X=0, Y=0, Z=1)
			{
				double offset_x, offset_y, offset_z;
				m_bma400.calibrate_offset(m_bma400.get_range(), offset_x, offset_y, offset_z);
				// Set calibration values (measured averages in g)
				// When applied: X_cal = X_raw - offset_x, Y_cal = Y_raw - offset_y
				// Z_cal = Z_raw - (offset_z - 1) so at rest Z reads 1g
				m_bma400.set_x_calibration(offset_x);
				m_bma400.set_y_calibration(offset_y);
				m_bma400.set_z_calibration(offset_z);
				// Store in calibration file
				m_cal.write((unsigned int)CalibrationPoint::X, offset_x);
				m_cal.write((unsigned int)CalibrationPoint::Y, offset_y);
				m_cal.write((unsigned int)CalibrationPoint::Z, offset_z);
				DEBUG_INFO("BMA400::calibration_write: auto-calibrated X=%.4f g, Y=%.4f g, Z=%.4f g",
				           offset_x, offset_y, offset_z);
			}
			break;
		case 4: // Read and display calibrated X, Y, Z values
			{
				m_bma400.read_xyz(m_last_x, m_last_y, m_last_z, m_last_temperature);
				DEBUG_INFO("BMA400::calibration_write: calibrated values X=%.4f g, Y=%.4f g, Z=%.4f g",
				           m_last_x, m_last_y, m_last_z);
			}
			break;
		case 5: // Read and display current calibration coefficients
			{
				DEBUG_INFO("BMA400::calibration_write: calibration coefficients X=%.4f g, Y=%.4f g, Z=%.4f g",
				           m_bma400.get_x_calibration(), m_bma400.get_y_calibration(), m_bma400.get_z_calibration());
			}
			break;
		case 6: // Save calibration to file
			{
				m_cal.save();
				DEBUG_INFO("BMA400::calibration_write: calibration saved to file");
			}
			break;
		case 7: // Threshold (used internally by AXL service)
			m_bma400.set_wakeup_threshold(value);
			break;
		case 8: // Duration (used internally by AXL service)
			m_bma400.set_wakeup_duration(value);
			break;
		case 9: // G-force range (used internally by AXL service)
			m_bma400.set_range(static_cast<unsigned int>(value));
			break;
		case 10: // Power mode (used internally by AXL service)
			m_bma400.set_power_mode(static_cast<unsigned int>(value));
			break;
		default:
			DEBUG_WARN("BMA400::calibration_write: invalid offset %u", offset);
			break;
	}
}

void BMA400::calibration_read(double& value, const unsigned int offset)
{
	double offset_x = 0.0, offset_y = 0.0, offset_z = 0.0;

	// Read calibrated sensor values (offset 1-3)
	if (offset >= 1 && offset <= 3) {
		m_bma400.read_xyz(m_last_x, m_last_y, m_last_z, m_last_temperature);
	}

	// Perform calibration measurement if reading raw offsets (offset 4-6)
	if (offset >= 4 && offset <= 6) {
		m_bma400.calibrate_offset(m_bma400.get_range(), offset_x, offset_y, offset_z);
	}

	switch (offset) {
		case 1: // Read calibrated X value
			value = m_last_x;
			break;
		case 2: // Read calibrated Y value
			value = m_last_y;
			break;
		case 3: // Read calibrated Z value
			value = m_last_z;
			break;
		case 4: // X calibration offset (raw measurement)
			value = offset_x;
			break;
		case 5: // Y calibration offset (raw measurement)
			value = offset_y;
			break;
		case 6: // Z calibration offset (raw measurement)
			value = offset_z;
			break;
		default:
			DEBUG_WARN("BMA400::calibration_read: invalid offset %u", offset);
			value = 0.0;
			break;
	}

	DEBUG_INFO("BMA400::calibration_read: offset=%u, value=%.4f", offset, value);
}

void BMA400::install_event_handler(unsigned int, std::function<void()> handler)
{
	m_bma400.enable_wakeup(handler);
}

void BMA400::remove_event_handler(unsigned int)
{
	m_bma400.disable_wakeup();
}

void BMA400::calibration_save(bool force)
{
	m_cal.save(force);
}
