#pragma once

#include <functional>
#include <cstdint>
#include "sensor.hpp"
#include "nrf_irq.hpp"
#include <array>

extern "C" {
#include "bma400_defs.h"
}

/*! Read write length varies based on user requirement */
#define BMA400_READ_WRITE_LENGTH  UINT8_C(64)
#define GET_API_NAME(func) (#func)
#define GET_VAR_NAME(var) (#var)
#define GRAVITY_EARTH     (9.80665f)

class BMA400LL
{
public:
	friend class BMA400;
	unsigned int m_bus;
	unsigned char m_addr;
	BMA400LL(unsigned int bus, unsigned char addr, int wakeup_pin);
	~BMA400LL();
	double read_temperature();
	// void read_xyz(double& x, double& y, double& z);
	void read_xyz(double& x, double& y, double& z);
	void read_xyz_128_at_20hz(double& x, double& y, double& z);
	void set_wakeup_threshold(double threshold);
	void set_wakeup_duration(double duration);
	void set_wakeup_gforce(unsigned int g_force);
	void set_power_mode(unsigned int power_mode);

	void set_x_calibration(double x);
	void set_y_calibration(double y);
	void set_z_calibration(double z);

	double get_x_calibration(void);
	double get_y_calibration(void);
	double get_z_calibration(void);
	uint8_t get_power_mode(void);
	uint8_t get_gforce(void);

	void enable_wakeup(std::function<void()> func);
	void disable_wakeup();
	bool check_and_clear_wakeup();

	void setup_sleep_mode(void);
	void setup_lp_conf(void);
	void setup_normal_conf(void);
	void setup_autowakeup_autolowpower_conf(void);

	void calibrate_offset(const uint8_t g_range, double& offset_x, double& offset_y, double& offset_z);

	double m_x;
	double m_y;
	double m_z;

protected:
	void init();
	uint8_t init(std::function<void()> setup_mode);

private:
	NrfIRQ m_irq;
	uint8_t m_unique_id;
	uint8_t m_accel_sleep_mode;
	struct bma400_dev m_bma400_dev;
	double m_wakeup_threshold;
	double m_wakeup_slope;
	double m_wakeup_duration;
	bool m_irq_pending;
	// struct bma400_sensor_conf m_bma400_sensor_conf;
	bma400_sensor_conf m_bma400_sensor_conf[2];
	struct bma400_device_conf m_bma400_device_conf;
    struct bma400_int_enable m_bma400_int_en; // interrupt to be declared
	uint8_t m_g_force;
	uint8_t m_power_mode;


	static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
	static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
	static void delay_us(uint32_t period, void *intf_ptr);
	double convert_g_force(unsigned int g_scale, int16_t axis_value);
	double lsb_to_ms2(int16_t accel_data, uint8_t g_range, uint8_t bit_width);
    void bma400_check_rslt(const char api_name[], int8_t rslt);

	void enable_wakeup_lp_mode(std::function<void()> func);
	void enable_wakeup_normal_mode(std::function<void()> func);
	void enable_wakeup_auto_mode(std::function<void()> func);
};


class BMA400 : public Sensor{
public:
	BMA400();
	~BMA400() = default;
	void calibration_write(const double, const unsigned int) override;
	void calibration_read(double &, const unsigned int) override;

	// void calibration_save(bool) override {}

	double read(unsigned int offset) override;
	void install_event_handler(unsigned int, std::function<void()>) override;
	void remove_event_handler(unsigned int) override;


private:
	// Calibration m_cal;

	enum class CalibrationAxis {
		X   = 0,
		Y   = 1,
		Z   = 2,
		XYZ = 3
	};

	enum class CalibrationWriteParameter {
		THRESHOLD   = 0,
		DURATION   	= 1,
		GFORCE   	= 2,
		POWER_MODE 	= 3,
		X			= 4,
		Y			= 5,
		Z			= 6
	};

	enum class CalibrationPowerMode {
		LOW_POWER 	= 0,
		NORMAL 		= 1,
		AUTO_MODE 	= 2
	};

	std::unordered_map<CalibrationAxis, std::function<double()>> calibration_read_map;
	std::unordered_map<CalibrationPowerMode, std::function<void()>> calibration_power_mode_map = {
        {CalibrationPowerMode::LOW_POWER,   [this]() { m_bma400.init([&]() { m_bma400.setup_lp_conf(); }); }},
        {CalibrationPowerMode::NORMAL,      [this]() { m_bma400.init([&]() { m_bma400.setup_normal_conf(); }); }},
        {CalibrationPowerMode::AUTO_MODE,   [this]() { m_bma400.init([&]() { m_bma400.setup_autowakeup_autolowpower_conf(); }); }},
    };

	std::unordered_map<CalibrationWriteParameter, std::function<void()>> calibration_write_map;
	BMA400LL m_bma400;
	double m_last_x;
	double m_last_y;
	double m_last_z;
};
