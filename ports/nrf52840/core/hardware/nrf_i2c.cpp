#include "nrf_i2c.hpp"
#include "nrfx_twim.h"
#include "nrfx_twi_twim.h"
#include "nrf_twim.h"
#include "error.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "pmu.hpp"
#include "timer.hpp"

extern Timer *system_timer;

// ============================================================================
// Event Handlers for each I2C bus (C-style callbacks for nrfx driver)
// ============================================================================

#if NRFX_TWIM0_ENABLED
static void twim0_event_handler(nrfx_twim_evt_t const *p_event, void *p_context) {
	(void)p_context;
	NrfI2C::event_handler(0, p_event->type == NRFX_TWIM_EVT_ADDRESS_NACK ||
	                         p_event->type == NRFX_TWIM_EVT_DATA_NACK ||
	                         p_event->type == NRFX_TWIM_EVT_OVERRUN ||
	                         p_event->type == NRFX_TWIM_EVT_BUS_ERROR);
}
#endif

#if NRFX_TWIM1_ENABLED
static void twim1_event_handler(nrfx_twim_evt_t const *p_event, void *p_context) {
	(void)p_context;
	NrfI2C::event_handler(1, p_event->type == NRFX_TWIM_EVT_ADDRESS_NACK ||
	                         p_event->type == NRFX_TWIM_EVT_DATA_NACK ||
	                         p_event->type == NRFX_TWIM_EVT_OVERRUN ||
	                         p_event->type == NRFX_TWIM_EVT_BUS_ERROR);
}
#endif

static nrfx_twim_evt_handler_t get_event_handler(uint8_t bus) {
	switch (bus) {
#if NRFX_TWIM0_ENABLED
		case 0: return twim0_event_handler;
#endif
#if NRFX_TWIM1_ENABLED
		case 1: return twim1_event_handler;
#endif
		default: return nullptr;
	}
}

// ============================================================================
// Bus Recovery Helper Functions
// ============================================================================

bool NrfI2C::is_bus_stuck(uint8_t bus) {
	if (bus >= BSP::I2C_TOTAL_NUMBER)
		return false;

	uint32_t scl_pin = BSP::I2C_Inits[bus].twim_config.scl;
	uint32_t sda_pin = BSP::I2C_Inits[bus].twim_config.sda;

	// Configure as inputs with pull-ups to check bus state
	nrf_gpio_cfg_input(scl_pin, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(sda_pin, NRF_GPIO_PIN_PULLUP);
	PMU::delay_us(50);

	bool scl_low = (nrf_gpio_pin_read(scl_pin) == 0);
	bool sda_low = (nrf_gpio_pin_read(sda_pin) == 0);

	if (scl_low || sda_low) {
		DEBUG_WARN("NrfI2C::is_bus_stuck - bus %u: SCL=%s | SDA=%s",
			bus, scl_low ? "LOW" : "HIGH", sda_low ? "LOW" : "HIGH");
		return true;
	}
	return false;
}

bool NrfI2C::clock_stretch_recovery(uint8_t bus) {
	if (bus >= BSP::I2C_TOTAL_NUMBER)
		return false;

	uint32_t scl_pin = BSP::I2C_Inits[bus].twim_config.scl;
	uint32_t sda_pin = BSP::I2C_Inits[bus].twim_config.sda;

	// Configure SCL as output, SDA as input with pull-up
	nrf_gpio_cfg_output(scl_pin);
	nrf_gpio_cfg_input(sda_pin, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_pin_set(scl_pin);
	PMU::delay_us(10);

	// Generate clock pulses to release SDA
	for (int cycle = 0; cycle < I2C_BUS_RECOVERY_CYCLES * 2; cycle++) {
		if (nrf_gpio_pin_read(sda_pin) == 1) break;
		nrf_gpio_pin_clear(scl_pin);
		PMU::delay_us(5);
		nrf_gpio_pin_set(scl_pin);
		PMU::delay_us(5);
	}

	// Generate STOP condition
	nrf_gpio_cfg_output(sda_pin);
	nrf_gpio_pin_clear(sda_pin);
	PMU::delay_us(5);
	nrf_gpio_pin_set(scl_pin);
	PMU::delay_us(5);
	nrf_gpio_pin_set(sda_pin);
	PMU::delay_us(10);

	// Check if SDA is now high
	nrf_gpio_cfg_input(sda_pin, NRF_GPIO_PIN_PULLUP);
	PMU::delay_us(10);
	return (nrf_gpio_pin_read(sda_pin) == 1);
}

bool NrfI2C::full_bus_reset(uint8_t bus) {
	if (bus >= BSP::I2C_TOTAL_NUMBER)
		return false;

	DEBUG_WARN("NrfI2C::full_bus_reset - bus %u", bus);

	uint32_t scl_pin = BSP::I2C_Inits[bus].twim_config.scl;
	uint32_t sda_pin = BSP::I2C_Inits[bus].twim_config.sda;

	// Disable and uninit the TWIM peripheral
	nrfx_twim_disable(&BSP::I2C_Inits[bus].twim);
	nrfx_twim_uninit(&BSP::I2C_Inits[bus].twim);

	// Manual bus recovery
	nrf_gpio_cfg_output(scl_pin);
	nrf_gpio_cfg_output(sda_pin);
	nrf_gpio_pin_set(scl_pin);
	nrf_gpio_pin_set(sda_pin);
	PMU::delay_ms(1);

	// Generate START condition
	nrf_gpio_pin_clear(sda_pin);
	PMU::delay_us(10);

	// Clock out any stuck data
	nrf_gpio_cfg_input(sda_pin, NRF_GPIO_PIN_PULLUP);
	for (int i = 0; i < I2C_BUS_RECOVERY_CYCLES + 3; i++) {
		nrf_gpio_pin_clear(scl_pin);
		PMU::delay_us(5);
		nrf_gpio_pin_set(scl_pin);
		PMU::delay_us(5);
	}

	// Generate STOP condition
	nrf_gpio_cfg_output(sda_pin);
	nrf_gpio_pin_clear(sda_pin);
	PMU::delay_us(5);
	nrf_gpio_pin_set(scl_pin);
	PMU::delay_us(5);
	nrf_gpio_pin_set(sda_pin);
	PMU::delay_ms(1);

	// Check bus state
	nrf_gpio_cfg_input(scl_pin, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(sda_pin, NRF_GPIO_PIN_PULLUP);
	PMU::delay_us(50);

	if ((nrf_gpio_pin_read(scl_pin) == 0) || (nrf_gpio_pin_read(sda_pin) == 0)) {
		DEBUG_ERROR("NrfI2C::full_bus_reset - bus %u still stuck", bus);
		m_is_enabled[bus] = false;
		return false;
	}

	// Reinit with async handler
	nrfx_twim_evt_handler_t handler = get_event_handler(bus);
	nrfx_err_t err = nrfx_twim_init(&BSP::I2C_Inits[bus].twim,
	                                 &BSP::I2C_Inits[bus].twim_config,
	                                 handler, nullptr);
	if (err != NRFX_SUCCESS) {
		DEBUG_ERROR("NrfI2C::full_bus_reset - reinit failed (0x%08X)", err);
		m_is_enabled[bus] = false;
		return false;
	}

	nrfx_twim_enable(&BSP::I2C_Inits[bus].twim);
	m_is_enabled[bus] = true;
	DEBUG_INFO("NrfI2C::full_bus_reset - bus %u recovered", bus);
	return true;
}

bool NrfI2C::recover_bus(uint8_t bus) {
	if (bus >= BSP::I2C_TOTAL_NUMBER)
		return false;

	DEBUG_WARN("NrfI2C::recover_bus - bus %u", bus);
	m_stats[bus].bus_recoveries++;

	nrfx_twim_disable(&BSP::I2C_Inits[bus].twim);
	nrfx_twim_uninit(&BSP::I2C_Inits[bus].twim);

	for (int attempt = 0; attempt < I2C_RECOVERY_MAX_ATTEMPTS; attempt++) {
		if (clock_stretch_recovery(bus)) {
			nrfx_twim_evt_handler_t handler = get_event_handler(bus);
			nrfx_err_t err = nrfx_twim_init(&BSP::I2C_Inits[bus].twim,
			                                 &BSP::I2C_Inits[bus].twim_config,
			                                 handler, nullptr);
			if (err == NRFX_SUCCESS) {
				nrfx_twim_enable(&BSP::I2C_Inits[bus].twim);
				m_is_enabled[bus] = true;
				DEBUG_INFO("NrfI2C::recover_bus - bus %u recovered (attempt %d)", bus, attempt + 1);
				return true;
			}
		}
		PMU::delay_ms(5);
	}

	return full_bus_reset(bus);
}

// ============================================================================
// Initialization
// ============================================================================

void NrfI2C::init(void) {
	//DEBUG_TRACE("NrfI2C::init - initializing %u I2C buses", BSP::I2C_TOTAL_NUMBER);

	for (unsigned int i = 0; i < BSP::I2C_TOTAL_NUMBER; i++) {
		// Skip buses that are already initialized
		if (m_is_enabled[i]) {
			//DEBUG_TRACE("NrfI2C::init - bus %u already enabled | skipping", i);
			continue;
		}

		// Pre-configure I2C pins with pull-ups before bus-stuck check
		nrf_gpio_cfg(BSP::I2C_Inits[i].twim_config.scl, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT,
		             NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
		nrf_gpio_cfg(BSP::I2C_Inits[i].twim_config.sda, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT,
		             NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
		PMU::delay_us(100);

		// Check if bus is stuck before init
		if (is_bus_stuck(i)) {
			DEBUG_WARN("NrfI2C::init - bus %u stuck | attempting recovery", i);
			if (!clock_stretch_recovery(i)) {
				DEBUG_ERROR("NrfI2C::init - bus %u recovery failed | skipping", i);
				m_is_enabled[i] = false;
				continue;
			}
		}

		// Reset transfer state
		m_transfer_done[i] = false;
		m_transfer_error[i] = false;

		// Initialize with async event handler for proper timeout support
		nrfx_twim_evt_handler_t handler = get_event_handler(i);
		nrfx_err_t err = nrfx_twim_init(&BSP::I2C_Inits[i].twim,
		                   &BSP::I2C_Inits[i].twim_config,
		                   handler, nullptr);
		if (err != NRFX_SUCCESS) {
			DEBUG_ERROR("NrfI2C::init - bus %u failed (0x%08X)", i, err);
			m_is_enabled[i] = false;
			continue;
		}

		nrfx_twim_enable(&BSP::I2C_Inits[i].twim);
		m_is_enabled[i] = true;
		m_stats[i] = {};
		DEBUG_TRACE("NrfI2C::init - bus %u OK (async mode with timeout)", i);
	}
}

void NrfI2C::uninit(void) {
	for (unsigned int i = 0; i < BSP::I2C_TOTAL_NUMBER; i++) {
		if (m_is_enabled[i]) {
			nrfx_twim_disable(&BSP::I2C_Inits[i].twim);
			nrfx_twim_uninit(&BSP::I2C_Inits[i].twim);
			m_is_enabled[i] = false;
		}
	}
}

void NrfI2C::disable(uint8_t bus) {
	if (bus < BSP::I2C_TOTAL_NUMBER && m_is_enabled[bus]) {
		nrfx_twim_disable(&BSP::I2C_Inits[bus].twim);
		nrfx_twim_uninit(&BSP::I2C_Inits[bus].twim);
		m_is_enabled[bus] = false;
	}
}

// ============================================================================
// Event Handler and Wait Functions
// ============================================================================

void NrfI2C::event_handler(uint8_t bus, bool error) {
	if (bus >= BSP::I2C_TOTAL_NUMBER) return;
	m_transfer_error[bus] = error;
	m_transfer_done[bus] = true;
}

bool NrfI2C::wait_for_transfer(uint8_t bus, uint32_t timeout_ms) {
	if (bus >= BSP::I2C_TOTAL_NUMBER)
		return false;

	uint64_t start_time = system_timer->get_counter();

	uint64_t next_wdt_kick = 500;

	while (!m_transfer_done[bus]) {
		uint64_t elapsed = system_timer->get_counter() - start_time;
		if (elapsed >= timeout_ms) {
			DEBUG_WARN("NrfI2C::wait_for_transfer - bus %u timeout after %ums", bus, timeout_ms);
			m_stats[bus].timeouts++;

			// Force stop the transfer
			NRF_TWIM_Type *p_twim = BSP::I2C_Inits[bus].twim.p_twim;
			nrf_twim_task_trigger(p_twim, NRF_TWIM_TASK_STOP);
			nrf_delay_us(100);

			// Clear events
			nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_STOPPED);
			nrf_twim_event_clear(p_twim, NRF_TWIM_EVENT_ERROR);

			return false;
		}

		// Kick watchdog during long waits to prevent WDT reset
		if (elapsed >= next_wdt_kick) {
			PMU::kick_watchdog();
			next_wdt_kick = elapsed + 500;
		}

		// Small delay to avoid busy-waiting
		nrf_delay_us(50);
	}

	return !m_transfer_error[bus];
}

// ============================================================================
// Read/Write - Async mode with timeout
// ============================================================================

bool NrfI2C::read_safe(uint8_t bus, uint8_t address, uint8_t *buffer, unsigned int length) {
	if (bus >= BSP::I2C_TOTAL_NUMBER || !m_is_enabled[bus]) {
		return false;
	}

	m_stats[bus].total_operations++;

	for (uint8_t retry = 0; retry < I2C_MAX_RETRIES; retry++) {
		// Reset transfer state
		m_transfer_done[bus] = false;
		m_transfer_error[bus] = false;

		// Start async transfer
		nrfx_err_t err = nrfx_twim_rx(&BSP::I2C_Inits[bus].twim, address, buffer, length);

		if (err == NRFX_SUCCESS) {
			// Wait for completion with timeout
			if (wait_for_transfer(bus, I2C_OPERATION_TIMEOUT_MS)) {
				m_stats[bus].successful_operations++;
				return true;
			}
			// Timeout occurred - will retry
		} else {
			DEBUG_WARN("NrfI2C::read_safe(0x%02X) start err=0x%08X retry %u/%u",
			           address, err, retry+1, I2C_MAX_RETRIES);
		}

		m_stats[bus].errors++;

		if (retry < I2C_MAX_RETRIES - 1) {
			recover_bus(bus);
			PMU::delay_ms(5);
		}
	}

	DEBUG_ERROR("NrfI2C::read_safe(0x%02X) failed after %u retries", address, I2C_MAX_RETRIES);
	return false;
}

bool NrfI2C::write_safe(uint8_t bus, uint8_t address, const uint8_t *buffer, unsigned int length, bool no_stop) {
	if (bus >= BSP::I2C_TOTAL_NUMBER || !m_is_enabled[bus]) {
		return false;
	}

	m_stats[bus].total_operations++;

	for (uint8_t retry = 0; retry < I2C_MAX_RETRIES; retry++) {
		// Reset transfer state
		m_transfer_done[bus] = false;
		m_transfer_error[bus] = false;

		// Start async transfer
		nrfx_err_t err = nrfx_twim_tx(&BSP::I2C_Inits[bus].twim, address, buffer, length, no_stop);

		if (err == NRFX_SUCCESS) {
			// Wait for completion with timeout
			if (wait_for_transfer(bus, I2C_OPERATION_TIMEOUT_MS)) {
				m_stats[bus].successful_operations++;
				return true;
			}
			// Timeout occurred - will retry
		} else {
			DEBUG_WARN("NrfI2C::write_safe(0x%02X) start err=0x%08X retry %u/%u",
			           address, err, retry+1, I2C_MAX_RETRIES);
		}

		m_stats[bus].errors++;

		if (retry < I2C_MAX_RETRIES - 1) {
			recover_bus(bus);
			PMU::delay_ms(5);
		}
	}

	DEBUG_ERROR("NrfI2C::write_safe(0x%02X) failed after %u retries", address, I2C_MAX_RETRIES);
	return false;
}

void NrfI2C::read(uint8_t bus, uint8_t address, uint8_t *buffer, unsigned int length) {
	if (!read_safe(bus, address, buffer, length)) {
		throw ErrorCode::I2C_COMMS_ERROR;
	}
}

void NrfI2C::write(uint8_t bus, uint8_t address, const uint8_t *buffer, unsigned int length, bool no_stop) {
	if (!write_safe(bus, address, buffer, length, no_stop)) {
		throw ErrorCode::I2C_COMMS_ERROR;
	}
}

// ============================================================================
// Utility Functions
// ============================================================================

uint8_t NrfI2C::num_buses(void) {
	return BSP::I2C::I2C_TOTAL_NUMBER;
}

bool NrfI2C::is_enabled(uint8_t bus) {
	return (bus < BSP::I2C_TOTAL_NUMBER) && m_is_enabled[bus];
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
