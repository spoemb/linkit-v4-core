#pragma once

/**
 * @file nrf_spim.hpp
 * @brief nRF52840 SPI master driver (blocking mode, manual CS control).
 *
 * Uses nrfx_spim in blocking mode (no event handler).  Chip select is
 * managed manually to support both active-low and active-high CS pins,
 * and multi-part transfers (transfer_continuous + finish_transfer).
 */

#include <cstdint>
#include "spim.hpp"

class NrfSPIM final : public SPIM {
private:
	unsigned int m_instance;

public:
	/// @param instance  BSP SPI bus index (BSP::SPI enum).
	/// @throws ErrorCode on nrfx_spim_init failure.
	NrfSPIM(unsigned int instance);
	~NrfSPIM();

	int transfer(const uint8_t *tx_data, uint8_t *rx_data, uint16_t size) override;
	int transfer_continuous(const uint8_t *tx_data, uint8_t *rx_data, uint16_t size) override;
	int finish_transfer() override;
};
