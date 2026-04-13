/**
 * @file ota_file_updater.hpp
 * @brief Abstract OTA file update interface — start, write, complete, apply firmware updates.
 */

#pragma once

#include <cstdint>
#include "filesystem.hpp"

/// @brief OTA file type identifiers (sent by pylinkit during BLE OTA).
enum class OTAFileIdentifier {
	MCU_FIRMWARE      = 0,  ///< nRF52840 firmware update
	ARTIC_FIRMWARE    = 1,  ///< ARTIC-R2 firmware update //Deprecated - no longer supported, but keep identifier for backward compatibility with older pylinkit versions
	GPS_CONFIG        = 2,  ///< GPS configuration file
	SMD_FIRMWARE_UART = 3,  ///< SMD firmware update via UART (AT commands)
	SMD_FIRMWARE_SPI  = 4   ///< SMD firmware update via SPI (Protocol A+)
};

/// @brief Abstract OTA file updater — manages file transfer lifecycle.
class OTAFileUpdater {
public:
	virtual ~OTAFileUpdater() {}

	/// @brief Start a new file transfer (erase target, validate size).
	virtual void start_file_transfer(OTAFileIdentifier file_id, const lfs_size_t length, const uint32_t crc32) = 0;

	/// @brief Write a chunk of file data (streaming, CRC updated incrementally).
	virtual void write_file_data(void * const data, lfs_size_t length) = 0;

	/// @brief Abort an in-progress transfer and clean up.
	virtual void abort_file_transfer() = 0;

	/// @brief Complete transfer — verify CRC, finalize file.
	virtual void complete_file_transfer() = 0;

	/// @brief Apply the update (reboot for MCU, DFU for SMD, etc.).
	virtual void apply_file_update() = 0;
};
