/**
 * @file uart.hpp
 * @brief Abstract synchronous UART interface — send/receive.
 */

#pragma once

#include <cstdint>

/// @brief Abstract synchronous UART — used by debug UART on RSPB.
class UART {
public:
	virtual ~UART() = default;
	virtual int send(const uint8_t *data, uint32_t size) = 0;
	virtual int receive(uint8_t *data, uint32_t size) = 0;
};
