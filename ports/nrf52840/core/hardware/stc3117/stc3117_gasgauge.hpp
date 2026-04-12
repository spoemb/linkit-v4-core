#pragma once

/**
 * @file stc3117_gasgauge.hpp
 * @brief STC3117 I2C fuel gauge battery monitor (ST Micro).
 *
 * Uses the ST STC3117 gas gauge for accurate SOC (State of Charge) and
 * voltage measurement.  Communicates via I2C through the Bosch C driver
 * pattern (static callbacks).  Used on RSPB board.
 */

#include "battery.hpp"
extern "C" {
	#include "stc311x_BatteryInfo.h"
	#include "stc311x_gasgauge.h"
}

class GaugeBatteryMonitor : public BatteryMonitor {
private:
	bool m_is_init = false;
	GasGauge_DataTypeDef STC3117_GG_struct = {};  ///< ST driver state (zero-init)

	bool pwr_pin_state = false;                   ///< Tracked VSENSORS state for power management
	bool previous_sensors_pwr_state = false;       ///< Previous VSENSORS state for edge detection

	/// @brief Check STC3117 responds on I2C.
	/// @return 0 on success, -1 on error.
	int check_i2c_device();

	/// @brief Init the STC3117 gas gauge driver.
	/// @return 0 on success, negative on error.
	int init();

	void internal_update() override;

public:
	/// @param critical_level  SOC % below which battery is critical (default 5%).
	/// @param low_level       SOC % below which battery is low (default 10%).
	GaugeBatteryMonitor(uint8_t critical_level = 5, uint8_t low_level = 10);

	/// @return true if STC3117 was successfully initialised.
	bool is_init() const { return m_is_init; }

	/// @brief Shut down the STC3117 to minimise current draw.
	/// @return 0 on success.
	int shutdown() override;

	/// @name I2C callbacks for the ST C driver (static, called from stc311x_gasgauge.c)
	/// @{
	static int i2c_write(int I2cSlaveAddr, int RegAddress, unsigned char *TxBuffer, int NumberOfBytes);
	static int i2c_read(int I2cSlaveAddr, int RegAddress, unsigned char *RxBuffer, int NumberOfBytes);
	/// @}
};