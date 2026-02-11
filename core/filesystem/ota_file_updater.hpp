#pragma once

#include <stdint.h>
#include "filesystem.hpp"

enum class OTAFileIdentifier {
	MCU_FIRMWARE      = 0,  // nRF52840 firmware update
	ARTIC_FIRMWARE    = 1,  // ARTIC-R2 firmware update
	GPS_CONFIG        = 2,  // GPS configuration file
	SMD_FIRMWARE_UART = 3,  // SMD firmware update via UART (AT commands)
	SMD_FIRMWARE_SPI  = 4   // SMD firmware update via SPI (Protocol A+)
};

class OTAFileUpdater {
public:
	virtual ~OTAFileUpdater() {}
	virtual void start_file_transfer(OTAFileIdentifier file_id, const lfs_size_t length, const uint32_t crc32) = 0;
	virtual void write_file_data(void * const data, lfs_size_t length) = 0;
	virtual void abort_file_transfer() = 0;
	virtual void complete_file_transfer() = 0;
	virtual void apply_file_update() = 0;
};
