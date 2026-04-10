/**
 * @file nrf_spim.cpp
 * @brief nRF52840 SPI master driver — blocking transfers with manual CS.
 */

#include <cstdint>
#include <cstring>

#include "nrf_spim.hpp"
#include "bsp.hpp"
#include "nrf_gpio.h"
#include "nrfx_spim.h"
#include "error.hpp"
#include "debug.hpp"

/// @brief Cached CS pin per SPI instance.  Default 0xFF = NRFX_SPIM_PIN_NOT_USED.
///        Set by NrfSPIM constructor.  activate_cs/deactivate_cs skip if 0xFF.
static_assert(NRFX_SPIM_PIN_NOT_USED == 0xFF, "CS init relies on memset(0xFF)");
static uint32_t s_cs_pin[BSP::SPI::SPI_TOTAL_NUMBER];

// .bss zero-inits to 0x00 which is P0.00 (a real pin) — unsafe.
// Force 0xFF at static init time so uninitialised instances are inert.
static const bool s_cs_pin_inited = [] {
	memset(s_cs_pin, 0xFF, sizeof(s_cs_pin));
	return true;
}();

static void activate_cs(unsigned int instance) {
	if (s_cs_pin[instance] == NRFX_SPIM_PIN_NOT_USED) return;
	if (BSP::SPI_Inits[instance].config.ss_active_high)
		nrf_gpio_pin_set(s_cs_pin[instance]);
	else
		nrf_gpio_pin_clear(s_cs_pin[instance]);
}

static void deactivate_cs(unsigned int instance) {
	if (s_cs_pin[instance] == NRFX_SPIM_PIN_NOT_USED) return;
	if (BSP::SPI_Inits[instance].config.ss_active_high)
		nrf_gpio_pin_clear(s_cs_pin[instance]);
	else
		nrf_gpio_pin_set(s_cs_pin[instance]);
}


NrfSPIM::NrfSPIM(unsigned int instance) : m_instance(instance)
{
	if (instance >= BSP::SPI::SPI_TOTAL_NUMBER) {
		DEBUG_ERROR("NrfSPIM: invalid instance %u", instance);
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}

	s_cs_pin[instance] = BSP::SPI_Inits[instance].config.ss_pin;

	if (s_cs_pin[instance] != NRFX_SPIM_PIN_NOT_USED) {
		deactivate_cs(instance);
		nrf_gpio_cfg_output(s_cs_pin[instance]);
	}

	// Copy config and remove SS pin — we manage CS manually
	BSP::SPI_InitTypeDefAndInst_t spi_init = BSP::SPI_Inits[instance];
	spi_init.config.ss_pin = NRFX_SPIM_PIN_NOT_USED;

	// Blocking mode (no event handler)
	nrfx_err_t err = nrfx_spim_init(&spi_init.spim, &spi_init.config, nullptr, nullptr);
	if (err != NRFX_SUCCESS) {
		DEBUG_ERROR("NrfSPIM: init failed instance %u (0x%08X)", instance, err);
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;
	}
}

NrfSPIM::~NrfSPIM()
{
	nrfx_spim_uninit(&BSP::SPI_Inits[m_instance].spim);
}

int NrfSPIM::transfer(const uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
	nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(tx_data, size, rx_data, size);

	activate_cs(m_instance);
	nrfx_err_t ret = nrfx_spim_xfer(&BSP::SPI_Inits[m_instance].spim, &xfer_desc, 0);
	deactivate_cs(m_instance);

	if (ret != NRFX_SUCCESS) {
		DEBUG_ERROR("SPI transfer failed instance %u (0x%08X)", m_instance, ret);
		return -1;
	}
	return 0;
}

int NrfSPIM::transfer_continuous(const uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
	nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(tx_data, size, rx_data, size);

	activate_cs(m_instance);
	nrfx_err_t ret = nrfx_spim_xfer(&BSP::SPI_Inits[m_instance].spim, &xfer_desc, 0);

	if (ret != NRFX_SUCCESS) {
		deactivate_cs(m_instance);
		DEBUG_ERROR("SPI transfer_continuous failed instance %u (0x%08X)", m_instance, ret);
		return -1;
	}
	return 0;
}

int NrfSPIM::finish_transfer()
{
	deactivate_cs(m_instance);
	return 0;
}
