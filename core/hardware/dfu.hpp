#pragma once

/**
 * @file dfu.hpp
 * @brief Device Firmware Update (DFU) settings interface.
 *
 * Wraps the Nordic DFU settings page to stage external-flash firmware
 * updates for the bootloader.  The bootloader reads bank_1 on next
 * reset and applies the update if BANK_VALID_EXT_APP is set.
 */

#include <cstdint>

class DFU {
public:
	/// @brief Read and validate the DFU settings page from flash.
	static void initialise();

	/**
	 * @brief Stage a firmware image for the bootloader to apply on next reset.
	 * @param src_addr   Start address of the image in external flash.
	 * @param image_size Size of the firmware image in bytes.
	 * @param crc        CRC32 of the firmware image.
	 */
	static void write_ext_flash_dfu_settings(uint32_t src_addr, uint32_t image_size, uint32_t crc);

	/// @brief Returns true if a validated firmware update is pending in bank_1.
	static bool update_pending();
};
