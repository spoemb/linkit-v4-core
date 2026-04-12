/**
 * @file kim2_comm.cpp
 * @brief KIM2 UART AT command layer — response parsing.
 *
 * UART lifecycle and deferred RX are handled by NrfUartAsync base class.
 * This file implements only: AT command table and KIM2 response parsing.
 */

#include "kim2_comm.hpp"
#include "debug.hpp"
#include <cstring>

using namespace KIM2;

// ============================================================================
// Constructor / Init / Deinit — delegate to NrfUartAsync
// ============================================================================

KIM2Comm::KIM2Comm(unsigned int instance)
	: NrfUartAsync(instance)
{
}

void KIM2Comm::init()
{
	NrfUartAsync::init();  // Uses BSP default baudrate
}

void KIM2Comm::deinit()
{
	NrfUartAsync::deinit();
}

// ============================================================================
// AT command send
// ============================================================================

bool KIM2Comm::send(ATCmd cmd, const std::optional<std::string>& params)
{
	if (m_is_send_busy) {
		DEBUG_ERROR("KIM2Comm: already busy");
		return false;
	}
	m_is_send_busy = true;
	return send_at_cmd(cmd, params);
}

/// @brief AT command table (static — no heap allocation).
struct ATCmdEntry {
	ATCmd type;
	const char *prefix;
	bool expects_params;
};

static constexpr ATCmdEntry cmd_table[] = {
	{ AT_PING,           "AT+PING=?\r\n",    false },
	{ AT_GET_ID,         "AT+ID=?\r\n",       false },
	{ AT_GET_ADDR,       "AT+ADDR=?\r\n",     false },
	{ AT_SET_RCONF,      "AT+RCONF=",          true },
	{ AT_SAVE_RCONF,     "AT+SAVE_RCONF\r\n", false },
	{ AT_SET_KMAC_BASIC, "AT+KMAC=1\r\n",     false },
	{ AT_SET_LPM,        "AT+LPM=",            true },
	{ AT_TX,             "AT+TX=",              true },
};

bool KIM2Comm::send_at_cmd(ATCmd cmd, const std::optional<std::string>& params)
{
	if (cmd >= AT_UNKNOWN || cmd >= static_cast<ATCmd>(std::size(cmd_table))) {
		m_is_send_busy = false;
		return false;
	}
	const auto& entry = cmd_table[cmd];

	m_tx_buffer = entry.prefix;
	if (entry.expects_params && params.has_value()) {
		m_tx_buffer.append(params.value());
		m_tx_buffer += "\r\n";
	}

	return send_string(m_tx_buffer);
}

// ============================================================================
// process_rx — delegate to base class
// ============================================================================

void KIM2Comm::process_rx()
{
	NrfUartAsync::process_rx();
}

// ============================================================================
// Protocol parsing — NrfUartAsync callbacks
// ============================================================================

void KIM2Comm::on_rx_line(std::string& line)
{
	RespType msg = parse_rx_line_protocol(line);

	if (msg == RESP_OK) {
		notify<KIM2CommEventRespOk>({});
	} else if (msg == RESP_TX_STATUS) {
		notify<KIM2CommEventTxDone>({});
	} else if (msg == RESP_ERROR) {
		DEBUG_INFO("KIM2: error response: %s", line.c_str());
		notify<KIM2CommEventRespError>({});
	}
	// RESP_CONFIG and RESP_UNKNOWN: no event, values stored in members
}

void KIM2Comm::on_rx_error(unsigned int error_type)
{
	notify(KIM2CommEventUartError(error_type));
}

// ============================================================================
// KIM2 response line parser
// ============================================================================

/// @brief Parse a single line and extract response type + data.
/// @note Line is already stripped of CR/LF by NrfUartAsync.
KIM2::RespType KIM2Comm::parse_rx_line_protocol(const std::string& line)
{
	if (line.empty() || line[0] != SYNC_CHAR)
		return RESP_UNKNOWN;

	size_t ok_len   = strlen(OK_RESPONSE);
	size_t id_len   = strlen(ID_RESPONSE);
	size_t addr_len = strlen(ADDR_RESPONSE);
	size_t tx_len   = strlen(TX_RESPONSE);
	size_t err_len  = strlen(ERR_RESPONSE);

	// +OK
	if (line.compare(0, ok_len, OK_RESPONSE) == 0 && line.size() == ok_len)
		return RESP_OK;

	// +ID=<6 decimal digits>
	if (line.compare(0, id_len, ID_RESPONSE) == 0 && line.size() >= id_len + ID_SIZE) {
		char ascii_id[ID_SIZE + 1] = {};
		std::memcpy(ascii_id, line.c_str() + id_len, ID_SIZE);
		char *end = nullptr;
		long id_val = strtol(ascii_id, &end, 10);
		m_kineis_id = (end != ascii_id && *end == '\0') ? static_cast<unsigned int>(id_val) : 0;
		return RESP_CONFIG;
	}

	// +ADDR=<8 hex digits>
	if (line.compare(0, addr_len, ADDR_RESPONSE) == 0 && line.size() >= addr_len + ADDR_SIZE) {
		char ascii_addr[ADDR_SIZE + 1] = {};
		std::memcpy(ascii_addr, line.c_str() + addr_len, ADDR_SIZE);
		char *end = nullptr;
		unsigned long addr_val = strtoul(ascii_addr, &end, 16);
		m_hex_addr = (end != ascii_addr) ? static_cast<unsigned int>(addr_val) : 0;
		return RESP_CONFIG;
	}

	// +ERROR=...
	if (line.compare(0, err_len, ERR_RESPONSE) == 0)
		return RESP_ERROR;

	// +TX=<status>,<data>
	if (line.compare(0, tx_len, TX_RESPONSE) == 0) {
		std::string resp = line.substr(tx_len);
		size_t comma = resp.find(',');
		if (comma != std::string::npos) {
			try {
				m_tx_status = static_cast<uint16_t>(std::stoi(resp.substr(0, comma)));
			} catch (...) {
				m_tx_status = 0xFFFF;
			}
		}
		return RESP_TX_STATUS;
	}

	return RESP_UNKNOWN;
}
