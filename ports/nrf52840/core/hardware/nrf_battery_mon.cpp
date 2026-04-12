/**
 * @file nrf_battery_mon.cpp
 * @brief nRF52840 SAADC-based battery voltage monitor implementation.
 */

#include <cstdint>

#include "nrf_battery_mon.hpp"
#include "nrfx_saadc.h"
#include "nrf_delay.h"
#include "bsp.hpp"
#include "error.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include "pmu.hpp"

#ifndef CPPUTEST
#include "crc16.h"
#else
#define crc16_compute(x, y, z)  0xFFFF
#endif

// Default voltage divider gain if not defined in BSP
#ifndef V_DIV_GAIN
#define V_DIV_GAIN 1.0f
#endif

/// @name SOC hysteresis thresholds (percentage points)
/// @{
static constexpr uint8_t CRITICAL_SOC_HYSTERESIS = 3;
static constexpr uint8_t LOW_BATT_THRESHOLD      = 5;
/// @}

/// @name ADC conversion constants
/// @{
static constexpr int     ADC_MAX_VALUE = 16384;    ///< 2^14 (14-bit resolution)
static constexpr float   ADC_REFERENCE = 0.6f;     ///< Internal reference voltage (V)
/// @}

/// @name Battery discharge lookup tables (SOC % at 0.1 V steps from 4.2 V → 3.2 V)
/// @{
static constexpr unsigned int BATT_LUT_ENTRIES = 11;
static constexpr uint16_t    BATT_LUT_MIN_V   = 3200;
static constexpr uint16_t    BATT_LUT_MAX_V   = 4200;

static constexpr uint8_t battery_voltage_lut[][BATT_LUT_ENTRIES] = {
	{ 100, 91, 79, 62, 42, 12,  2, 0, 0, 0, 0 },  // S18650_2600
	{ 100, 93, 84, 75, 64, 52, 22, 9, 0, 0, 0 },  // CGR18650_2250
	{ 100, 94, 83, 59, 50, 33, 15, 6, 0, 0, 0 }   // NCR18650_3100_3400
};
/// @}

static void nrfx_saadc_event_handler(nrfx_saadc_evt_t const *p_event)
{
	(void)p_event;
}

/// @brief Filtered SOC values persisted in .noinit RAM to survive soft resets.
///        Index 0 = voltage (mV), index 1 = SOC level (%).
static __attribute__((section(".noinit"))) volatile uint16_t m_filtered_values[2];

/// @brief CRC16 of m_filtered_values — detects uninitialised .noinit after power-on.
static __attribute__((section(".noinit"))) volatile uint16_t m_crc;

/// @brief Maximum attempts for SAADC calibration busy-wait (each ~10 us).
static constexpr uint32_t ADC_CAL_TIMEOUT = 10000;


NrfBatteryMonitor::NrfBatteryMonitor(uint8_t adc_channel,
		BatteryChemistry chem,
		uint8_t critical_level,
		uint8_t low_level)
		: BatteryMonitor(low_level, critical_level)
{
	// One-time SAADC calibration (retained until power reset, survives init/uninit)
	nrfx_saadc_init(&BSP::ADC_Inits.config, nrfx_saadc_event_handler);
	nrfx_saadc_calibrate_offset();

	DEBUG_TRACE("Enter ADC calibration...");
	uint32_t cal_attempts = ADC_CAL_TIMEOUT;
	while (nrfx_saadc_is_busy() && --cal_attempts) {
		nrf_delay_us(10);
	}

	if (cal_attempts == 0) {
		DEBUG_ERROR("ADC calibration timeout");
	} else {
		DEBUG_TRACE("ADC calibration complete");
	}

	nrfx_saadc_uninit();

	m_adc_channel = adc_channel;
	m_is_init = false;
	m_chem = chem;
}

/**
 * @brief Sample battery voltage via SAADC.
 *
 * Enables the battery read circuit (if present), waits for the RC settling
 * time defined by BAT_ADC_SETTLE_MS in the BSP, samples one conversion,
 * then powers down the SAADC to minimise sleep current.
 *
 * @return Raw voltage in millivolts (before divider gain correction).
 */
float NrfBatteryMonitor::sample_adc()
{
	nrf_saadc_value_t raw = 0;

#ifdef BAT_READ_ENABLE
	GPIOPins::set(BAT_READ_ENABLE);
	PMU::delay_ms(BAT_ADC_SETTLE_MS);
#endif

	// Init/uninit SAADC per sample to reduce sleep current
	nrfx_saadc_init(&BSP::ADC_Inits.config, nrfx_saadc_event_handler);
	nrfx_saadc_channel_init(m_adc_channel, &BSP::ADC_Inits.channel_config[m_adc_channel]);

	nrfx_saadc_sample_convert(m_adc_channel, &raw);

	nrfx_saadc_uninit();
#ifdef BAT_READ_ENABLE
	GPIOPins::clear(BAT_READ_ENABLE);
#endif

	return (static_cast<float>(raw)) / ((ADC_GAIN / ADC_REFERENCE) * ADC_MAX_VALUE) * 1000.0f;
}

/**
 * @brief Periodic update — sample ADC, apply SOC hysteresis, persist to .noinit RAM.
 *
 * SOC hysteresis prevents rapid toggling of critical/low flags near the threshold:
 *  - Once SOC drops below critical_level, it must recover to (critical_level + 3%)
 *  - Once SOC drops below low_level, it must recover to (low_level + 5%)
 */
void NrfBatteryMonitor::internal_update()
{
	uint16_t mv    = convert_voltage(sample_adc());
	uint8_t  level = convert_level(mv);

	// Check CRC of previously stored filtered values (.noinit RAM)
	uint16_t crc = crc16_compute(reinterpret_cast<const uint8_t *>(const_cast<const uint16_t *>(m_filtered_values)), sizeof(m_filtered_values), nullptr);
	if (crc == m_crc) {
		// Previous values valid — apply hysteresis
		m_filtered_values[0] = mv;
		if (m_filtered_values[1] < m_critical_level) {
			if (level >= (m_critical_level + CRITICAL_SOC_HYSTERESIS))
				m_filtered_values[1] = level;
		} else if (m_filtered_values[1] < m_low_level) {
			if (level >= (m_low_level + LOW_BATT_THRESHOLD))
				m_filtered_values[1] = level;
		} else {
			m_filtered_values[1] = level;
		}
	} else {
		// CRC mismatch (first boot or power-on) — seed with fresh values
		m_filtered_values[0] = mv;
		m_filtered_values[1] = level;
	}

	// Update CRC
	m_crc = crc16_compute(reinterpret_cast<const uint8_t *>(const_cast<const uint16_t *>(m_filtered_values)), sizeof(m_filtered_values), nullptr);

	// Apply to base class members
	m_last_voltage_mv = mv;
	m_last_level = level;

	// Set flags (both based on filtered SOC, not raw)
	m_is_critical_voltage = m_filtered_values[1] < m_critical_level;
	m_is_low_level = m_filtered_values[1] < m_low_level;
}

/**
 * @brief Convert millivolts to SOC percentage via LUT with linear interpolation.
 * @param mv Battery voltage in millivolts.
 * @return SOC percentage (0-100).
 */
uint8_t NrfBatteryMonitor::convert_level(uint16_t mv)
{
	int lut_index = (BATT_LUT_ENTRIES - 1) - ((mv / 100) - (BATT_LUT_MIN_V / 100));

	// Bounds check on chemistry enum
	unsigned int chem = static_cast<unsigned int>(m_chem);
	if (chem >= sizeof(battery_voltage_lut) / sizeof(battery_voltage_lut[0]))
		chem = 0;

	if (lut_index <= 0) {
		return battery_voltage_lut[chem][0];
	} else if (lut_index > static_cast<int>(BATT_LUT_ENTRIES - 1)) {
		return battery_voltage_lut[chem][BATT_LUT_ENTRIES - 1];
	} else {
		// Linear interpolation between adjacent LUT entries
		uint8_t upper = battery_voltage_lut[chem][lut_index - 1];
		uint8_t lower = battery_voltage_lut[chem][lut_index];
		uint16_t upper_mv = BATT_LUT_MAX_V - ((lut_index - 1) * 100);
		float t = static_cast<float>(upper_mv - mv) / 100.0f;
		float result = static_cast<float>(upper) + (t * (static_cast<float>(lower) - static_cast<float>(upper)));
		return static_cast<uint8_t>(result);
	}
}

/**
 * @brief Apply voltage divider gain to convert raw ADC millivolts to actual battery voltage.
 * @param adc_mv Raw ADC reading in millivolts.
 * @return Battery voltage in millivolts.
 */
uint16_t NrfBatteryMonitor::convert_voltage(float adc_mv)
{
#ifdef BATTERY_NOT_FITTED
	return BATT_LUT_MAX_V;
#else
	return static_cast<uint16_t>(adc_mv * V_DIV_GAIN);
#endif
}
