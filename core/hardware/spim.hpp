#pragma once

/**
 * @file spim.hpp
 * @brief Abstract SPI master interface.
 */

#include <cstdint>

class SPIM {
public:
	virtual ~SPIM() {}

	/// @brief Full-duplex SPI transfer (CS asserted, then deasserted).
	virtual int transfer(const uint8_t *tx_data, uint8_t *rx_data, uint16_t size) = 0;

	/// @brief Full-duplex SPI transfer with CS held low (for multi-part transfers).
	virtual int transfer_continuous(const uint8_t *tx_data, uint8_t *rx_data, uint16_t size) = 0;

	/// @brief Deassert CS after a transfer_continuous sequence.
	virtual int finish_transfer() = 0;
};
