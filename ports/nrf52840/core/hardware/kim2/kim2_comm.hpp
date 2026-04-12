/**
 * @file kim2_comm.hpp
 * @brief KIM2 satellite module UART AT command communication layer.
 *
 * Handles UART TX/RX with the CLS KIM2 module via AT commands.
 * Response parsing supports +OK, +ID=, +ADDR=, +TX=, +ERROR= formats.
 * Events are emitted to listeners (KIM2Device) via the EventEmitter pattern.
 *
 * UART lifecycle and deferred RX are handled by NrfUartAsync base class.
 */

#pragma once

#include "nrf_uart_async.hpp"
#include "events.hpp"
#include <cstdint>
#include <string>
#include <optional>

namespace KIM2 {

/// @brief AT command types supported by the KIM2 module.
enum ATCmd {
	AT_PING = 0,
	AT_GET_ID,
	AT_GET_ADDR,
	AT_SET_RCONF,
	AT_SAVE_RCONF,
	AT_SET_KMAC_BASIC,
	AT_SET_LPM,
	AT_TX,
	AT_UNKNOWN
};

/// @brief Response types from the KIM2 module.
enum RespType {
	RESP_OK = 0,
	RESP_ERROR,
	RESP_CONFIG,
	RESP_TX_STATUS,
	RESP_UNKNOWN
};

static constexpr uint8_t SYNC_CHAR  = '+';
static constexpr uint8_t END_CHAR_1 = '\r';
static constexpr uint8_t END_CHAR_2 = '\n';

/// @name Response prefix strings
/// @{
static constexpr const char *OK_RESPONSE   = "+OK";
static constexpr const char *ID_RESPONSE   = "+ID=";
static constexpr const char *ADDR_RESPONSE = "+ADDR=";
static constexpr const char *TX_RESPONSE   = "+TX=";
static constexpr const char *ERR_RESPONSE  = "+ERROR=";
/// @}

static constexpr uint8_t ID_SIZE   = 6;   ///< Decimal ID string length
static constexpr uint8_t ADDR_SIZE = 8;   ///< Hex address string length

} // namespace KIM2

/// @name KIM2 communication events
/// @{
struct KIM2CommEventTxDone {};
struct KIM2CommEventRespOk {};
struct KIM2CommEventRespError {};
struct KIM2CommEventUartError {
	unsigned int error_type;
	KIM2CommEventUartError(unsigned int a) : error_type(a) {}
};
/// @}

class KIM2CommEventListener {
public:
	virtual ~KIM2CommEventListener() = default;
	virtual void react(const KIM2CommEventTxDone&) {}
	virtual void react(const KIM2CommEventRespOk&) {}
	virtual void react(const KIM2CommEventRespError&) {}
	virtual void react(const KIM2CommEventUartError&) {}
};

/**
 * @brief UART AT command layer for the KIM2 satellite module.
 *
 * Inherits NrfUartAsync for UART lifecycle + deferred RX.
 * Adds: AT command table, KIM2 response parsing.
 */
class KIM2Comm : public NrfUartAsync, public EventEmitter<KIM2CommEventListener> {
public:
	unsigned int m_kineis_id = 0;      ///< Device ID from +ID= response
	unsigned int m_hex_addr = 0;       ///< Device address from +ADDR= response
	uint16_t m_tx_status = 0xFFFF;     ///< Last TX status from +TX= response

	/// @param libuarte_async_instance  BSP UART instance index (default 1).
	KIM2Comm(unsigned int libuarte_async_instance = 1);

	/// @brief Init UART and start RX.
	void init();

	/// @brief Deinit UART.
	void deinit();

	/// @brief Send an AT command (non-blocking).
	bool send(KIM2::ATCmd cmd, const std::optional<std::string>& params = std::nullopt);

	/// @brief Process ISR-buffered RX data. Call periodically from main context.
	void process_rx();

protected:
	/// @brief Parse a complete RX line and emit events (KIM2 protocol).
	void on_rx_line(std::string& line) override;

	/// @brief Handle UART error — emit event.
	void on_rx_error(unsigned int error_type) override;

private:
	bool send_at_cmd(KIM2::ATCmd cmd, const std::optional<std::string>& params = std::nullopt);
	KIM2::RespType parse_rx_line_protocol(const std::string& line);
};
