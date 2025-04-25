#include <stdint.h>

#include "thermistor.hpp"
#include "nrfx_saadc.h"
#include "bsp.hpp"
#include "gpio.hpp"
#include "error.hpp"
#include "debug.hpp"
#include "nrf_delay.h"
#include <math.h> 
#include "nrf_delay.h"
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
    //m_cal.write((unsigned int)CalibrationPoint::TEMP_THRESHOLD, 0.0);
    //m_cal.save();
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

    // Wait for calibration to complete
    while (nrfx_saadc_is_busy()) {}

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

    // Initialize SAADC to reduce sleep current
    nrfx_saadc_init(&BSP::ADC_Inits.config, nrfx_saadc_event_handler);
    nrfx_saadc_channel_init(m_adc_channel, &BSP::ADC_Inits.channel_config[m_adc_channel]);
    nrfx_saadc_sample_convert(m_adc_channel, &raw);

    nrfx_saadc_uninit();

    // Compute ADC voltage
     float adc_voltage = (static_cast<float>(raw) / static_cast<float>(ADC_MAX_VALUE)) * 
         static_cast<float>(ADC_REFERENCE) * (1.0f / static_cast<float>(ADC_GAIN));

    // Ensure floating-point values are cast to double for DEBUG_TRACE
    // DEBUG_TRACE("THERMISTOR::%s: ADC Config - Reference: %.3lfV, Gain: %.3lf", 
    //     __func__, static_cast<double>(ADC_REFERENCE), static_cast<double>(ADC_GAIN));

    // DEBUG_TRACE("THERMISTOR::%s: ADC Raw = %d, Voltage = %.5lf mV", 
    //     __func__, raw, static_cast<double>(adc_voltage * 1000.0f));

    return adc_voltage * 1000.0f;  // Return millivolts as a float
}

double Thermistor::convert_temp(float adc)
{
    DEBUG_TRACE("THERMISTOR::%s:Enter...", __func__);

    // Convert ADC value (mV) to Voltage (V)
    double v = static_cast<double>(adc) / 1000.0;

    // Voltage divider equation:
    constexpr double R_BOTTOM = 10000.0;  // 10kΩ fixed resistor
    double r_therm = R_BOTTOM * ((3.3 / v) - 1.0);

    // Thermistor properties
    constexpr double R0 = 10000.0;   // Resistance at 25°C
    constexpr double T0 = 298.15;    // 25°C in Kelvin

    // Dynamically select B-value based on resistance
    double B = 3434.0; // Default (for 25-85°C)
    if (r_therm > 12000.0) {  // Cold (<50°C)
        B = 3380.0;
    } else if (r_therm > 8000.0) {  // Warm (50-85°C)
        B = 3434.0;
    } else {  // Hot (>85°C)
        B = 3455.0;
    }

    // Apply Beta Equation: 1/T = 1/T0 + (1/B) * ln(R_therm / R0)
    double tempK = 1.0 / ((1.0 / T0) + (1.0 / B) * log(r_therm / R0));
    double tempC = tempK - 273.15;  // Convert Kelvin to Celsius

    tempC = tempC+offset_temp;  // Adjust temperature based on calibration
    return (tempC);  // Return temperature in Celsius
}

double Thermistor::read(unsigned int offset)
{
    DEBUG_TRACE("THERMISTOR::%s:Enter...", __func__);

    uint32_t pwr_pin_state = GPIOPins::value(SENSORS_PWR_PIN);
    if (pwr_pin_state == 0) 
    {
       GPIOPins::set(SENSORS_PWR_PIN);
    }
    pwr_pin_state = GPIOPins::value(SENSORS_PWR_PIN);

    float adc = sample_adc();  // Now ADC reads as a float
    double temperature = convert_temp(adc);  // Convert to double
    DEBUG_INFO("THERMISTOR::%s: TempC = %.5lf", __func__, temperature);

    if (pwr_pin_state == 0) 
    {
        GPIOPins::clear(SENSORS_PWR_PIN);
    }

    return temperature;
}
#include <limits>

double Thermistor::find_calibration_point(double target_value) {
    constexpr unsigned int num_samples = 10;
    double total_temperature = 0.0;

    for (unsigned int i = 0; i < num_samples; ++i) {
        total_temperature += read(0); // Read temperature 10 times
        nrf_delay_ms(100); // Add a delay of 100ms between readings
    }

    double average_temperature = total_temperature / num_samples;
    double difference = abs(average_temperature - target_value);
    if (average_temperature > target_value) {
        difference = -difference; // Negative difference if average is greater than target
    }
    DEBUG_TRACE("THERMISTOR::%s: Average Temperature = %.5lf, Difference = %.5lf", __func__, average_temperature, difference);

    return difference;
}

void Thermistor::calibration_write(const double value, const unsigned int offset) {
    DEBUG_TRACE("THERMISTOR::%s:Enter...", __func__);
	if (offset == 0) { // 0=>reset
		m_cal.reset();
	} else if (offset == 1) { // Calibrate to Value
        double calibration_value = find_calibration_point(value);
        m_cal.write((unsigned int)CalibrationPoint::TEMP_THRESHOLD, calibration_value);
	} else if (offset == 2) { // Save calibration
		m_cal.save();
	}
}

void Thermistor::calibration_save(bool force) {
    DEBUG_TRACE("THERMISTOR::%s:Enter...", __func__);
	m_cal.save(force);
}

void Thermistor::calibration_read(double& value, const unsigned int offset)
{
    DEBUG_TRACE("THERMISTOR::%s:Enter...", __func__);
	if (0 == offset) {
		DEBUG_TRACE("Thermistor::calibrate: read Threshold value");
		value = m_cal.read((unsigned int)CalibrationPoint::TEMP_THRESHOLD);
	} 
}
