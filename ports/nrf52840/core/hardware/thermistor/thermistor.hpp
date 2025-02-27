#pragma once

#include <cstdint>
#include "sensor.hpp"


class Thermistor : public Sensor {
	uint8_t m_adc_channel;
	bool m_is_init;
	double m_last_temperature;
	const double offset_temp = 9;

	void adc_calibration();
	float sample_adc();
	double convert_temp(float adc);
public:
	Thermistor(uint8_t adc_channel);
	//~Thermistor();
	double read(unsigned int offset) override;
};
