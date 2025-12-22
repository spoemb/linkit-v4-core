#pragma once

#include <cstdint>

#include "bsp.hpp"

// I2C operation timeout in milliseconds
#define I2C_OPERATION_TIMEOUT_MS  100
#define I2C_MAX_RETRIES           3
#define I2C_BUS_RECOVERY_CYCLES   9

struct I2CStats {
	uint32_t timeouts;
	uint32_t errors;
	uint32_t bus_recoveries;
	uint32_t total_operations;
};

class NrfI2C {
private:
	static inline bool m_is_enabled[BSP::I2C_TOTAL_NUMBER] = { false };
	static inline volatile bool m_transfer_done[BSP::I2C_TOTAL_NUMBER] = { false };
	static inline volatile bool m_transfer_error[BSP::I2C_TOTAL_NUMBER] = { false };
	static inline I2CStats m_stats[BSP::I2C_TOTAL_NUMBER] = {};

	static bool wait_for_transfer(uint8_t bus, uint32_t timeout_ms);
	static bool recover_bus(uint8_t bus);

public:
	// Public so that C-style event handler callbacks can access it
	static void event_handler(uint8_t bus, bool error);
	static void init(void);
	static void uninit(void);
	static void disable(uint8_t bus);
	static void read(uint8_t bus, uint8_t address, uint8_t *buffer, unsigned int length);
	static void write(uint8_t bus, uint8_t address, const uint8_t *buffer, unsigned int length, bool no_stop);
	static uint8_t num_buses(void);
	static const I2CStats& get_stats(uint8_t bus);
	static void reset_stats(uint8_t bus);
};
