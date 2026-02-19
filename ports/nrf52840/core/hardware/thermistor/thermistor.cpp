#include <stdint.h>

#include "thermistor.hpp"
#include "nrfx_saadc.h"
#include "bsp.hpp"
#include "error.hpp"
#include "debug.hpp"
#include "nrf_delay.h"
#include <cmath>

// ADC constants
#define ADC_MAX_VALUE (16384)      // 2^14
#define ADC_REFERENCE (0.6f)       // 0.6v internal reference

static void nrfx_saadc_event_handler(nrfx_saadc_evt_t const *p_event)
{
	(void)p_event;
}

Thermistor::Thermistor(uint8_t adc_channel) : Sensor("THERMISTOR"), m_cal(Calibration("THERMISTOR"))
{
	DEBUG_INFO("THERMISTOR::Init: ADC Channel %u", adc_channel);
	adc_calibration();
	try {
		offset_temp = (double)m_cal.read((unsigned int)CalibrationPoint::TEMP_THRESHOLD);
		DEBUG_TRACE("THERMISTOR::Init: Reading calibration offset %lf", offset_temp);
	} catch (...) {
		DEBUG_WARN("THERMISTOR::Init: Failed to read calibration offset");
		offset_temp = 0.0;
		m_cal.write((unsigned int)CalibrationPoint::TEMP_THRESHOLD, 0.0);
		m_cal.save();
	}
	m_adc_channel = adc_channel;
}

void Thermistor::adc_calibration()
{
	DEBUG_TRACE("THERMISTOR::%s: Calibrating ADC...", __func__);
	nrfx_saadc_init(&BSP::ADC_Inits.config, nrfx_saadc_event_handler);
	nrfx_saadc_calibrate_offset();

	const uint32_t timeout_ms = 200;
	uint32_t elapsed = 0;
	while (nrfx_saadc_is_busy()) {
		nrf_delay_ms(1);
		if (++elapsed >= timeout_ms) {
			DEBUG_WARN("THERMISTOR::%s: ADC calibration timeout!", __func__);
			break;
		}
	}

	nrfx_saadc_uninit();
	m_is_init = true;
	DEBUG_INFO("THERMISTOR::%s: Calibration done!", __func__);
}

float Thermistor::sample_adc()
{
	if (!m_is_init) {
		adc_calibration();
	}
	nrf_saadc_value_t raw = 0;

	nrfx_saadc_init(&BSP::ADC_Inits.config, nrfx_saadc_event_handler);
	nrfx_saadc_channel_init(m_adc_channel, &BSP::ADC_Inits.channel_config[m_adc_channel]);
	nrfx_saadc_sample_convert(m_adc_channel, &raw);
	nrfx_saadc_uninit();

	float adc_voltage = (static_cast<float>(raw) / static_cast<float>(ADC_MAX_VALUE)) *
		static_cast<float>(ADC_REFERENCE) * (1.0f / static_cast<float>(ADC_GAIN));

	return adc_voltage * 1000.0f;  // Return millivolts
}

double Thermistor::convert_temp(float adc)
{
	// Convert ADC value (mV) to Voltage (V)
	double v = static_cast<double>(adc) / 1000.0;

	// Guard against invalid ADC readings
	if (v < 0.001) {
		DEBUG_ERROR("Thermistor: ADC voltage near 0");
		return NAN;
	}
	if (v > 3.3) v = 3.3;

	// Voltage divider: NTC thermistor on top, 10k fixed resistor on bottom
	constexpr double R_BOTTOM = 10000.0;  // 10k fixed resistor
	double r_therm = R_BOTTOM * ((3.3 / v) - 1.0);

	// NTC thermistor properties
	constexpr double R0 = 10000.0;   // Resistance at 25C
	constexpr double T0 = 298.15;    // 25C in Kelvin

	// Dynamically select B-value based on resistance range
	double B = 3434.0;
	if (r_therm > 12000.0) {
		B = 3380.0;       // Cold (<25C)
	} else if (r_therm > 8000.0) {
		B = 3434.0;       // Warm (25-50C)
	} else {
		B = 3455.0;       // Hot (>50C)
	}

	// Beta equation: 1/T = 1/T0 + (1/B) * ln(R_therm / R0)
	double tempK = 1.0 / ((1.0 / T0) + (1.0 / B) * log(r_therm / R0));
	double tempC = tempK - 273.15;

	tempC = tempC + offset_temp;
	return tempC;
}

double Thermistor::read(unsigned int offset)
{
	(void)offset;
	float adc = sample_adc();
	double temperature = convert_temp(adc);
	DEBUG_INFO("THERMISTOR::%s: TempC = %.5lf", __func__, temperature);
	m_last_temperature = temperature;
	return temperature;
}

double Thermistor::find_calibration_point(double target_value) {
	constexpr unsigned int num_samples = 10;
	double total_temperature = 0.0;

	for (unsigned int i = 0; i < num_samples; ++i) {
		total_temperature += read(0);
		nrf_delay_ms(100);
	}

	double average_temperature = total_temperature / num_samples;
	double difference = std::fabs(average_temperature - target_value);
	if (average_temperature > target_value) {
		difference = -difference;
	}
	DEBUG_TRACE("THERMISTOR::%s: Average Temperature = %.5lf | Difference = %.5lf",
		__func__, average_temperature, difference);

	return difference;
}

void Thermistor::calibration_write(const double value, const unsigned int offset) {
	if (offset == 0) {
		m_cal.reset();
	} else if (offset == 1) {
		double calibration_value = find_calibration_point(value);
		m_cal.write((unsigned int)CalibrationPoint::TEMP_THRESHOLD, calibration_value);
	} else if (offset == 2) {
		m_cal.save();
	}
}

void Thermistor::calibration_save(bool force) {
	m_cal.save(force);
}

void Thermistor::calibration_read(double& value, const unsigned int offset)
{
	if (0 == offset) {
		value = m_cal.read((unsigned int)CalibrationPoint::TEMP_THRESHOLD);
	}
}
