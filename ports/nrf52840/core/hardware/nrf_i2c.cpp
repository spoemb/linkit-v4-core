#include "nrf_i2c.hpp"
#include "nrfx_twim.h"
#include "nrfx_twi_twim.h"
#include "error.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "pmu.hpp"

// Event handler callback adapters for each I2C bus
static void twim0_event_handler(nrfx_twim_evt_t const * p_event, void * p_context) {
	NrfI2C::event_handler(0, p_event->type == NRFX_TWIM_EVT_ADDRESS_NACK ||
	                         p_event->type == NRFX_TWIM_EVT_DATA_NACK);
}

static void twim1_event_handler(nrfx_twim_evt_t const * p_event, void * p_context) {
	NrfI2C::event_handler(1, p_event->type == NRFX_TWIM_EVT_ADDRESS_NACK ||
	                         p_event->type == NRFX_TWIM_EVT_DATA_NACK);
}

static nrfx_twim_evt_handler_t get_event_handler(uint8_t bus) {
	switch (bus) {
		case 0: return twim0_event_handler;
		case 1: return twim1_event_handler;
		default: return nullptr;
	}
}

void NrfI2C::event_handler(uint8_t bus, bool error) {
	if (bus >= BSP::I2C_TOTAL_NUMBER)
		return;

	m_transfer_done[bus] = true;
	m_transfer_error[bus] = error;
}

bool NrfI2C::wait_for_transfer(uint8_t bus, uint32_t timeout_ms) {
	uint32_t elapsed_ms = 0;

	// Use a simple delay-based timeout (no timer dependency for critical I2C layer)
	while (!m_transfer_done[bus] && elapsed_ms < timeout_ms) {
		PMU::delay_us(100); // 100us polling interval
		elapsed_ms++;

		// Rough conversion: 10 iterations = ~1ms
		if (elapsed_ms % 10 == 0) {
			PMU::kick_watchdog(); // Keep watchdog happy during I2C wait
		}
	}

	if (!m_transfer_done[bus]) {
		DEBUG_ERROR("NrfI2C::wait_for_transfer: timeout on bus %u after %u ms", bus, timeout_ms);
		m_stats[bus].timeouts++;
		return false;
	}

	if (m_transfer_error[bus]) {
		DEBUG_ERROR("NrfI2C::wait_for_transfer: error on bus %u", bus);
		m_stats[bus].errors++;
		return false;
	}

	return true;
}

bool NrfI2C::recover_bus(uint8_t bus) {
	if (bus >= BSP::I2C_TOTAL_NUMBER)
		return false;

	DEBUG_WARN("NrfI2C::recover_bus: attempting recovery on bus %u", bus);
	m_stats[bus].bus_recoveries++;

	// Disable I2C peripheral
	nrfx_twim_disable(&BSP::I2C_Inits[bus].twim);
	nrfx_twim_uninit(&BSP::I2C_Inits[bus].twim);

	// Get pin numbers from config
	uint32_t scl_pin = BSP::I2C_Inits[bus].twim_config.scl;
	uint32_t sda_pin = BSP::I2C_Inits[bus].twim_config.sda;

	// Configure pins as GPIO outputs
	nrf_gpio_cfg_output(scl_pin);
	nrf_gpio_cfg_input(sda_pin, NRF_GPIO_PIN_PULLUP);

	// Generate 9 clock pulses to clear any stuck slave
	for (int i = 0; i < I2C_BUS_RECOVERY_CYCLES; i++) {
		nrf_gpio_pin_clear(scl_pin);
		PMU::delay_us(5);
		nrf_gpio_pin_set(scl_pin);
		PMU::delay_us(5);
	}

	// Generate STOP condition: SDA low->high while SCL high
	nrf_gpio_cfg_output(sda_pin);
	nrf_gpio_pin_clear(sda_pin);
	PMU::delay_us(5);
	nrf_gpio_pin_set(scl_pin);
	PMU::delay_us(5);
	nrf_gpio_pin_set(sda_pin);
	PMU::delay_us(5);

	// Reinitialize I2C peripheral
	nrfx_err_t err = nrfx_twim_init(&BSP::I2C_Inits[bus].twim,
	                                 &BSP::I2C_Inits[bus].twim_config,
	                                 get_event_handler(bus),
	                                 nullptr);

	if (err != NRFX_SUCCESS) {
		DEBUG_ERROR("NrfI2C::recover_bus: failed to reinitialize bus %u", bus);
		m_is_enabled[bus] = false;
		return false;
	}

	nrfx_twim_enable(&BSP::I2C_Inits[bus].twim);
	m_is_enabled[bus] = true;

	DEBUG_INFO("NrfI2C::recover_bus: bus %u recovery complete", bus);
	return true;
}

void NrfI2C::init(void) {
	for (unsigned int i = 0; i < BSP::I2C_TOTAL_NUMBER; i++)
	{
		// Initialize with event handler for async operation
		if (nrfx_twim_init(&BSP::I2C_Inits[i].twim,
		                   &BSP::I2C_Inits[i].twim_config,
		                   get_event_handler(i),
		                   nullptr) != NRFX_SUCCESS)
			throw ErrorCode::RESOURCE_NOT_AVAILABLE;

		nrfx_twim_enable(&BSP::I2C_Inits[i].twim);
		m_is_enabled[i] = true;
		m_transfer_done[i] = false;
		m_transfer_error[i] = false;

		// Reset stats
		m_stats[i] = {};
	}
}

void NrfI2C::uninit(void) {
	for (unsigned int i = 0; i < BSP::I2C_TOTAL_NUMBER; i++)
	{
		nrfx_twim_disable(&BSP::I2C_Inits[i].twim);
		nrfx_twim_uninit(&BSP::I2C_Inits[i].twim);
		m_is_enabled[i] = false;
	}
}

void NrfI2C::disable(uint8_t bus) {
	nrfx_twim_disable(&BSP::I2C_Inits[bus].twim);
	nrfx_twim_uninit(&BSP::I2C_Inits[bus].twim);
	m_is_enabled[bus] = false;
}

void NrfI2C::read(uint8_t bus, uint8_t address, uint8_t *buffer, unsigned int length) {
	if (!m_is_enabled[bus])
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;

	m_stats[bus].total_operations++;

	for (uint8_t retry = 0; retry < I2C_MAX_RETRIES; retry++) {
		// Reset transfer flags
		m_transfer_done[bus] = false;
		m_transfer_error[bus] = false;

		// Start async transfer
		nrfx_err_t error = nrfx_twim_rx(&BSP::I2C_Inits[bus].twim, address, buffer, length);

		if (error != NRFX_SUCCESS) {
			DEBUG_ERROR("NrfI2C::read(%u,%02x,%u) start failed=%08x",
			           (unsigned int)bus, (unsigned int)address, (unsigned int)length, (unsigned int)error);
			m_stats[bus].errors++;

			// Attempt bus recovery on communication error
			if (retry < I2C_MAX_RETRIES - 1) {
				if (!recover_bus(bus)) {
					throw ErrorCode::I2C_COMMS_ERROR;
				}
				PMU::delay_ms(10); // Brief delay after recovery
				continue;
			}
			throw ErrorCode::I2C_COMMS_ERROR;
		}

		// Wait for transfer completion with timeout
		if (wait_for_transfer(bus, I2C_OPERATION_TIMEOUT_MS)) {
			// Success!
			return;
		}

		// Transfer failed (timeout or error)
		// Attempt bus recovery before retry
		if (retry < I2C_MAX_RETRIES - 1) {
			DEBUG_WARN("NrfI2C::read retry %u/%u on bus %u", retry + 1, I2C_MAX_RETRIES, bus);
			if (!recover_bus(bus)) {
				throw ErrorCode::I2C_COMMS_ERROR;
			}
			PMU::delay_ms(10); // Brief delay before retry
		}
	}

	// All retries exhausted
	DEBUG_ERROR("NrfI2C::read(%u,%02x,%u) failed after %u retries",
	           (unsigned int)bus, (unsigned int)address, (unsigned int)length, I2C_MAX_RETRIES);
	throw ErrorCode::I2C_COMMS_ERROR;
}

void NrfI2C::write(uint8_t bus, uint8_t address, const uint8_t *buffer, unsigned int length, bool no_stop) {
	if (!m_is_enabled[bus])
		throw ErrorCode::RESOURCE_NOT_AVAILABLE;

	m_stats[bus].total_operations++;

	for (uint8_t retry = 0; retry < I2C_MAX_RETRIES; retry++) {
		// Reset transfer flags
		m_transfer_done[bus] = false;
		m_transfer_error[bus] = false;

		// Start async transfer
		nrfx_err_t error = nrfx_twim_tx(&BSP::I2C_Inits[bus].twim, address, buffer, length, no_stop);

		if (error != NRFX_SUCCESS) {
			DEBUG_ERROR("NrfI2C::write(%u,%02x,%u) start failed=%08x",
			           (unsigned int)bus, (unsigned int)address, (unsigned int)length, (unsigned int)error);
			m_stats[bus].errors++;

			// Attempt bus recovery on communication error
			if (retry < I2C_MAX_RETRIES - 1) {
				if (!recover_bus(bus)) {
					throw ErrorCode::I2C_COMMS_ERROR;
				}
				PMU::delay_ms(10); // Brief delay after recovery
				continue;
			}
			throw ErrorCode::I2C_COMMS_ERROR;
		}

		// Wait for transfer completion with timeout
		if (wait_for_transfer(bus, I2C_OPERATION_TIMEOUT_MS)) {
			// Success!
			return;
		}

		// Transfer failed (timeout or error)
		// Attempt bus recovery before retry
		if (retry < I2C_MAX_RETRIES - 1) {
			DEBUG_WARN("NrfI2C::write retry %u/%u on bus %u", retry + 1, I2C_MAX_RETRIES, bus);
			if (!recover_bus(bus)) {
				throw ErrorCode::I2C_COMMS_ERROR;
			}
			PMU::delay_ms(10); // Brief delay before retry
		}
	}

	// All retries exhausted
	DEBUG_ERROR("NrfI2C::write(%u,%02x,%u) failed after %u retries",
	           (unsigned int)bus, (unsigned int)address, (unsigned int)length, I2C_MAX_RETRIES);
	throw ErrorCode::I2C_COMMS_ERROR;
}

uint8_t NrfI2C::num_buses(void) {
	return BSP::I2C::I2C_TOTAL_NUMBER;
}

const I2CStats& NrfI2C::get_stats(uint8_t bus) {
	if (bus >= BSP::I2C_TOTAL_NUMBER) {
		static I2CStats dummy = {};
		return dummy;
	}
	return m_stats[bus];
}

void NrfI2C::reset_stats(uint8_t bus) {
	if (bus < BSP::I2C_TOTAL_NUMBER) {
		m_stats[bus] = {};
	}
}
