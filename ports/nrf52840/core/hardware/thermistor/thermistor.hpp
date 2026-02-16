#pragma once

#include <cstdint>
#include "sensor.hpp"


class Thermistor : public Sensor {
	Calibration m_cal;
	uint8_t m_adc_channel;
	bool m_is_init = false;
	double m_last_temperature = 0.0;
	double offset_temp = 0;

	void adc_calibration();
	float sample_adc();
	double convert_temp(float adc);
	double find_calibration_point(double target_value);

	enum class CalibrationPoint : unsigned int {
		TEMP_THRESHOLD
	};
public:
	Thermistor(uint8_t adc_channel);
	double read(unsigned int offset) override;
	void calibration_write(const double, const unsigned int) override;
	void calibration_save(bool force) override;
	void calibration_read(double &value, const unsigned int) override;
};
