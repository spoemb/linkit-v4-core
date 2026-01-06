#pragma once

#include <functional>
#include <cstdint>
#include "sensor.hpp"
#include "nrf_irq.hpp"

extern "C" {
#include "bma400_defs.h"
}

// Read/write buffer length for I2C transfers
#define BMA400_READ_WRITE_LENGTH  UINT8_C(64)

// Polling configuration for data ready
#define BMA400_POLL_MAX_ATTEMPTS  100

/**
 * @brief Low-level driver for BMA400 accelerometer
 */
class BMA400LL
{
public:
	BMA400LL(unsigned int bus, unsigned char addr, int wakeup_pin);
	~BMA400LL();

	// Sensor reading
	void read_xyz(double& x, double& y, double& z, int16_t& temperature);
	int16_t read_temperature();

	// Configuration setters
	void set_wakeup_threshold(double threshold);
	void set_wakeup_duration(double duration);
	void set_range(unsigned int g_force);
	void set_power_mode(unsigned int power_mode);

	// Calibration setters
	void set_x_calibration(double x);
	void set_y_calibration(double y);
	void set_z_calibration(double z);

	// Calibration getters
	double get_x_calibration() const { return m_cal_x; }
	double get_y_calibration() const { return m_cal_y; }
	double get_z_calibration() const { return m_cal_z; }

	// Configuration getters
	uint8_t get_power_mode() const { return m_power_mode; }
	uint8_t get_range() const { return m_g_range; }

	// Calibration offset calculation
	void calibrate_offset(uint8_t g_range, double& offset_x, double& offset_y, double& offset_z);

	// Wakeup/interrupt management
	void enable_wakeup(std::function<void()> func);
	void disable_wakeup();
	bool check_and_clear_wakeup();

private:
	// Hardware configuration
	unsigned int m_bus;
	unsigned char m_addr;
	NrfIRQ m_irq;
	uint8_t m_unique_id;

	// Device state
	struct bma400_dev m_bma400_dev;
	struct bma400_sensor_conf m_sensor_conf[2];
	struct bma400_int_enable m_int_enable;
	bool m_irq_pending;

	// Configuration parameters
	uint8_t m_g_range;
	uint8_t m_power_mode;
	double m_wakeup_threshold;
	double m_wakeup_duration;

	// Calibration offsets
	double m_cal_x;
	double m_cal_y;
	double m_cal_z;

	// Initialization
	void init();
	void setup_sleep_mode();
	void setup_low_power_mode();
	void setup_normal_mode();

	// Wakeup mode configuration
	void enable_wakeup_low_power(std::function<void()> func);
	void enable_wakeup_normal(std::function<void()> func);

	// I2C interface (static callbacks for BOSCH driver)
	static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
	static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
	static void delay_us(uint32_t period, void *intf_ptr);

	// Utility functions
	double lsb_to_ms2(int16_t accel_data, uint8_t g_range, uint8_t bit_width);
	void check_result(const char* api_name, int8_t rslt);
	uint8_t calculate_threshold_reg(double threshold_g, uint8_t acc_range);
	uint8_t range_to_g(uint8_t range_reg);
};


/**
 * @brief High-level BMA400 accelerometer sensor class
 *
 * Read offsets:
 *   0: Temperature (degrees C / 10)
 *   1: X axis (g-force, triggers read_xyz)
 *   2: Y axis (g-force, cached)
 *   3: Z axis (g-force, cached)
 *   4: Activity (0-255 computed from magnitude)
 *   5: Wakeup IRQ pending flag
 *
 * Calibration write offsets:
 *   0: Wakeup threshold (g)
 *   1: Wakeup duration (samples)
 *   2: G-force range (0=2G, 1=4G, 2=8G, 3=16G)
 *   3: Power mode (0=low power, 1=normal)
 *   4: X calibration offset
 *   5: Y calibration offset
 *   6: Z calibration offset
 *
 * Calibration read offsets:
 *   4: X calibration offset
 *   5: Y calibration offset
 *   6: Z calibration offset
 */
class BMA400 : public Sensor
{
public:
	BMA400();
	~BMA400() = default;

	// Sensor interface
	double read(unsigned int offset) override;
	void calibration_write(const double value, const unsigned int offset) override;
	void calibration_read(double& value, const unsigned int offset) override;
	void install_event_handler(unsigned int, std::function<void()>) override;
	void remove_event_handler(unsigned int) override;

private:
	BMA400LL m_bma400;

	// Cached readings
	double m_last_x;
	double m_last_y;
	double m_last_z;
	int16_t m_last_temperature;
	uint8_t m_last_activity;

	// Compute activity from last xyz readings
	double compute_activity();
};
