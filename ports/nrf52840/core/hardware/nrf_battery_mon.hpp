#pragma once

/**
 * @file nrf_battery_mon.hpp
 * @brief nRF52840 SAADC-based battery voltage monitor.
 *
 * Reads battery voltage through an analog voltage divider via the nRF SAADC,
 * converts to millivolts using a BSP-defined gain, and maps to a 0-100% SOC
 * via a per-chemistry lookup table.  SOC hysteresis prevents rapid state
 * toggling near the critical/low thresholds.
 *
 * Filtered voltage and SOC are persisted in .noinit RAM (CRC-protected) so
 * they survive a soft reset — this avoids false "battery OK" readings on
 * EXTERNAL_WAKEUP boards that reboot frequently.
 */

#include "battery.hpp"

/// @brief Supported Li-ion cell chemistries (each has its own discharge LUT).
enum BatteryChemistry {
	BATT_CHEM_S18650_2600,            ///< Sony/Murata US18650VTC5
	BATT_CHEM_CGR18650_2250,          ///< Panasonic CGR18650
	BATT_CHEM_NCR18650_3100_3400      ///< Panasonic NCR18650B (default)
};

class NrfBatteryMonitor : public BatteryMonitor {
private:
	BatteryChemistry m_chem;
	uint8_t m_adc_channel;
	bool m_is_init;

	/// @brief Sample the SAADC and return raw voltage in mV (before divider correction).
	float sample_adc();

	/// @brief Apply voltage divider gain to raw ADC millivolts.
	uint16_t convert_voltage(float adc_mv);

	/// @brief Map millivolts to 0-100% SOC via the chemistry-specific LUT.
	uint8_t convert_level(uint16_t mv);

	void internal_update() override;

public:
	NrfBatteryMonitor(uint8_t adc_channel,
			BatteryChemistry chem = BATT_CHEM_NCR18650_3100_3400,
			uint8_t critical_level = 5,
			uint8_t low_level = 10);
};
