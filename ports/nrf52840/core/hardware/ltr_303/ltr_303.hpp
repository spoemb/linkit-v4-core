#pragma once

/**
 * @file ltr_303.hpp
 * @brief LTR-303ALS ambient light sensor driver (I2C, Lite-On).
 *
 * Measures visible + infrared light via two channels (CH0, CH1) and computes
 * lux using a ratio-based formula.  Supports configurable gain (1x to 96x).
 */

#include <cstdint>
#include "sensor.hpp"

class LTR303 : public Sensor {
public:
	/// @throws ErrorCode::I2C_COMMS_ERROR if device not detected.
	LTR303();

	/// @brief Read ambient light level in lux.
	/// @param gain  ALS gain (1, 2, 4, 8, 48, or 96).
	/// @return Lux value (0-65535).
	/// @throws ErrorCode::I2C_COMMS_ERROR on timeout, ErrorCode::BAD_GAIN_SETTING on invalid gain.
	double read(unsigned int gain = 1) override;

private:
	/// @brief Read one or more registers via I2C.
	/// @param reg   Register address.
	/// @param data  Output buffer.
	/// @param len   Number of bytes to read.
	void readReg(uint8_t reg, uint8_t *data, size_t len);

	/// @brief Write a single register via I2C.
	/// @param address  Register address.
	/// @param value    Value to write.
	void writeReg(uint8_t address, uint8_t value);

	static constexpr uint32_t STANDBY_TO_ACTIVE_MS = 10;  ///< Wakeup delay after mode change
};
