#include <stdint.h>
#include <cstring>
#include <cmath>

#include "bsp.hpp"
#include "error.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include "scheduler.hpp"
#include "timer.hpp"
#include "binascii.hpp"
#include "smd_sat.hpp"
#include "pmu.hpp"

#include "nrf_delay.h"
#include "nrf_gpio.h"

extern Scheduler *system_scheduler;
extern Timer *system_timer;

static constexpr const char *const spistatus_string[] =
{
    "SPI_UNKNOWN",
    "SPI_INIT",
    "SPI_IDLE",
    "SPI_PROCESS_CMD",
    "SPI_WAITING_RX",
    "SPI_WAITING_TX",
    "SPI_WAITING_MAC_EVT",
    "SPI_ERROR"
};
static constexpr const char *const kmacstatus_string[] =
{
    "MAC_UNKNOWN",
    "MAC_OK",
    "MAC_TX_DONE",
    "MAC_TX_SIZE_ERROR",
    "MAC_TXACK_DONE",
    "MAC_TX_TIMEOUT",
    "MAC_TXACK_TIMEOUT",
    "MAC_RX_ERROR",
    "MAC_RX_TIMEOUT",
    "MAC_ERROR"
};

void SmdSat::read_byte(uint8_t *byte_read) {
    nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
    uint8_t read_cmd = SMDSAT_CMD_NONE;
    int ret;

    ret = m_nrf_spim->transfer(&read_cmd, byte_read, sizeof(read_cmd));
    if (ret) {
       throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSat::send_command(uint8_t command) {
    uint8_t buffer_read;
    int ret;
	DEBUG_TRACE("%s::Send %u",__func__, command);

    ret = m_nrf_spim->transfer(&command, &buffer_read, sizeof(command));
    if (ret) {
       throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSat::send_command(const uint8_t *tx_data, uint8_t *rx_data, uint16_t size) {
    int ret = m_nrf_spim->transfer(tx_data, rx_data, size);
    if (ret) {
       throw ErrorCode::SPI_COMMS_ERROR;
	}
}

// ============================================================================
// Protocol A+ Implementation (Based on Zephyr argos-smd-driver)
// ============================================================================
// Key features:
// - Fixed 64-byte frames with 0xFF padding
// - CRC-8 over MAGIC + SEQ + CMD + LEN + DATA (includes MAGIC!)
// - Pipelined protocol: response to command N arrives in transaction N+2
// - BUSY pattern (0xBB) handling for flash operations
// ============================================================================

uint16_t SmdSat::build_aplus_frame(uint8_t *frame, uint8_t cmd, const uint8_t *data, uint16_t data_len) {
    // Validate data length
    if (data_len > SPI_PROTOCOL_APLUS_MAX_DATA_LEN) {
        DEBUG_ERROR("SmdSat::%s: Data too long (%u > %u)", __func__, data_len, SPI_PROTOCOL_APLUS_MAX_DATA_LEN);
        return 0;
    }

    // Initialize frame with padding
    memset(frame, SPI_PROTOCOL_APLUS_PAD_BYTE, SPI_PROTOCOL_APLUS_FRAME_SIZE);

    uint16_t idx = 0;

    // Build frame: [MAGIC][SEQ][CMD][LEN][DATA][CRC8][PADDING...]
    frame[idx++] = SPI_PROTOCOL_APLUS_MAGIC_REQUEST;
    frame[idx++] = m_sequence_number;
    frame[idx++] = cmd;
    frame[idx++] = static_cast<uint8_t>(data_len);

    // Copy data if any
    if (data != nullptr && data_len > 0) {
        memcpy(&frame[idx], data, data_len);
        idx += data_len;
    }

    // Calculate CRC-8 over MAGIC + SEQ + CMD + LEN + DATA (includes MAGIC!)
    uint8_t crc = spi_crc8_ccitt(frame, idx);
    frame[idx++] = crc;

    DEBUG_TRACE("SmdSat::%s: Frame built - SEQ=%u, CMD=0x%02X, LEN=%u, CRC=0x%02X",
                __func__, m_sequence_number, cmd, data_len, crc);

    // Return fixed frame size for SPI transfer
    return SPI_PROTOCOL_APLUS_FRAME_SIZE;
}

bool SmdSat::parse_aplus_response(const uint8_t *rx_buffer, uint16_t rx_len, SpiAplusResponse *response) {
    if (response == nullptr || rx_buffer == nullptr) {
        return false;
    }

    response->valid = false;
    response->data_len = 0;

    // Scan for response magic byte, skipping IDLE (0xAA) and padding (0xFF)
    // (matches Zephyr parse_response_frame behavior)
    uint16_t offset = 0;
    while (offset < rx_len) {
        if (rx_buffer[offset] == SPI_PROTOCOL_APLUS_MAGIC_RESPONSE) {
            break;
        }
        if (rx_buffer[offset] == SPI_PROTOCOL_APLUS_BUSY_PATTERN) {
            DEBUG_TRACE("SmdSat::%s: BUSY pattern at offset %u", __func__, offset);
            response->status = SPI_APLUS_STATUS_BUSY;
            return false;
        }
        if (rx_buffer[offset] != SPI_PROTOCOL_APLUS_IDLE_PATTERN &&
            rx_buffer[offset] != SPI_PROTOCOL_APLUS_PAD_BYTE) {
            break;
        }
        offset++;
    }

    if (offset >= rx_len || rx_buffer[offset] != SPI_PROTOCOL_APLUS_MAGIC_RESPONSE) {
        if (rx_buffer[0] == SPI_PROTOCOL_APLUS_IDLE_PATTERN) {
            DEBUG_TRACE("SmdSat::%s: IDLE pattern - no response ready", __func__);
            response->status = SPI_APLUS_STATUS_NOT_READY;
        } else {
            DEBUG_WARN("SmdSat::%s: No response magic found (first byte: 0x%02X)", __func__, rx_buffer[0]);
            response->status = SPI_APLUS_STATUS_FRAME_ERROR;
        }
        return false;
    }

    // Minimum frame: MAGIC + SEQ + STATUS + LEN + CRC = 5 bytes
    if (rx_len - offset < SPI_PROTOCOL_APLUS_HEADER_LEN + SPI_PROTOCOL_APLUS_CRC_LEN) {
        DEBUG_WARN("SmdSat::%s: Response too short after offset %u", __func__, offset);
        return false;
    }

    // Parse header at offset
    response->seq = rx_buffer[offset + 1];
    response->status = static_cast<SpiAplusStatus>(rx_buffer[offset + 2]);
    uint8_t data_len = rx_buffer[offset + 3];

    if (data_len > SPI_PROTOCOL_APLUS_MAX_DATA_LEN) {
        DEBUG_WARN("SmdSat::%s: Invalid data length %u", __func__, data_len);
        response->status = SPI_APLUS_STATUS_SIZE_ERROR;
        return false;
    }

    // Bounds check: header + data + CRC must fit in remaining buffer
    uint16_t expected_len = SPI_PROTOCOL_APLUS_HEADER_LEN + data_len + SPI_PROTOCOL_APLUS_CRC_LEN;
    if (rx_len - offset < expected_len) {
        DEBUG_WARN("SmdSat::%s: Response incomplete: got %u, expected %u",
                   __func__, rx_len - offset, expected_len);
        response->status = SPI_APLUS_STATUS_FRAME_ERROR;
        return false;
    }

    // Copy data
    response->data_len = data_len;
    if (data_len > 0) {
        memcpy(response->data, &rx_buffer[offset + 4], data_len);
    }

    // Validate CRC-8 over MAGIC + SEQ + STATUS + LEN + DATA
    uint16_t crc_data_len = SPI_PROTOCOL_APLUS_HEADER_LEN + data_len;
    uint8_t received_crc = rx_buffer[offset + crc_data_len];
    uint8_t calculated_crc = spi_crc8_ccitt(&rx_buffer[offset], crc_data_len);

    if (received_crc != calculated_crc) {
        DEBUG_WARN("SmdSat::%s: CRC mismatch (recv=0x%02X, calc=0x%02X)",
                   __func__, received_crc, calculated_crc);
        response->status = SPI_APLUS_STATUS_FRAME_CRC_ERROR;
        return false;
    }

    response->valid = true;
    DEBUG_TRACE("SmdSat::%s: Response parsed - SEQ=%u, STATUS=%u, LEN=%u",
                __func__, response->seq, static_cast<int>(response->status), response->data_len);

    return true;
}

// Command-specific delay (matches Zephyr timing constants)
static uint32_t get_command_delay(uint8_t cmd) {
    switch (cmd) {
        case SMDSAT_CMD_DFU_ERASE:
            return SMDSAT_TIMING_ERASE_MS;
        case SMDSAT_CMD_DFU_WRITE_DATA:
        case SMDSAT_CMD_DFU_SET_HEADER:
        case SMDSAT_CMD_WRITE_RCONF:
        case SMDSAT_CMD_WRITE_KMAC:
        case SMDSAT_CMD_WRITE_SECKEY:
        case SMDSAT_CMD_SAVE_RCONF:
            return SMDSAT_TIMING_WRITE_MS;
        case SMDSAT_CMD_DFU_RESET:
        case SMDSAT_CMD_DFU_JUMP:
            return SMDSAT_TIMING_RESET_MS;
        default:
            return SMDSAT_TIMING_STANDARD_MS;
    }
}

// Check if RX buffer is all IDLE or all BUSY (slave not ready)
static bool is_slave_not_ready(const uint8_t *buf, uint8_t check_len) {
    bool all_idle = true, all_busy = true;
    for (uint8_t i = 0; i < check_len; i++) {
        if (buf[i] != SPI_PROTOCOL_APLUS_IDLE_PATTERN) all_idle = false;
        if (buf[i] != SPI_PROTOCOL_APLUS_BUSY_PATTERN) all_busy = false;
    }
    return all_idle || all_busy;
}

bool SmdSat::send_command_aplus(uint8_t command, const uint8_t *tx_data, uint16_t tx_len,
                                SpiAplusResponse *response) {
    // Inter-transaction delay: wait for STM32 DMA re-arm from previous transaction
    nrf_delay_ms(SMDSAT_SPI_INTER_TX_DELAY_MS);

    // Build the A+ CMD frame (fixed 64 bytes)
    uint8_t tx_frame[SPI_PROTOCOL_APLUS_FRAME_SIZE];
    uint8_t rx_frame[SPI_PROTOCOL_APLUS_FRAME_SIZE];

    uint16_t frame_len = build_aplus_frame(tx_frame, command, tx_data, tx_len);
    if (frame_len == 0) {
        DEBUG_ERROR("SmdSat::%s: Failed to build frame", __func__);
        return false;
    }

    memset(rx_frame, 0, sizeof(rx_frame));

    // Transaction 1: Send command (response will be in transaction 2)
    int ret = m_nrf_spim->transfer(tx_frame, rx_frame, SPI_PROTOCOL_APLUS_FRAME_SIZE);
    if (ret) {
        DEBUG_ERROR("SmdSat::%s: SPI transfer failed", __func__);
        throw ErrorCode::SPI_COMMS_ERROR;
    }

    // Wait for STM32 to process command (command-specific delay)
    uint32_t cmd_delay = get_command_delay(command);
    nrf_delay_ms(cmd_delay);

    // Transaction 2+: Send NOP to retrieve response (pipelined protocol)
    m_sequence_number++;

    uint8_t nop_frame[SPI_PROTOCOL_APLUS_FRAME_SIZE];
    uint8_t response_frame[SPI_PROTOCOL_APLUS_FRAME_SIZE];

    bool is_flash_op = (cmd_delay >= SMDSAT_TIMING_WRITE_MS);
    uint8_t max_retries = is_flash_op ? SMDSAT_SPI_BUSY_MAX_RETRIES : 3;
    uint8_t retry_delay_ms = is_flash_op ? 50 : SMDSAT_SPI_BUSY_RETRY_DELAY_MS;

    for (uint8_t retry = 0; retry <= max_retries; retry++) {
        build_aplus_frame(nop_frame, SMDSAT_CMD_NONE, nullptr, 0);
        memset(response_frame, 0, sizeof(response_frame));

        ret = m_nrf_spim->transfer(nop_frame, response_frame, SPI_PROTOCOL_APLUS_FRAME_SIZE);
        if (ret) {
            DEBUG_ERROR("SmdSat::%s: Failed to read response", __func__);
            throw ErrorCode::SPI_COMMS_ERROR;
        }

        // Check for BUSY or IDLE or all-zero pattern (slave not ready) — check 8 bytes like Zephyr
        if (is_slave_not_ready(response_frame, 8) && retry < max_retries) {
            DEBUG_TRACE("SmdSat::%s: Slave not ready (0x%02X), retry %u/%u",
                        __func__, response_frame[0], retry + 1, max_retries);
            nrf_delay_ms(retry_delay_ms);
            m_sequence_number++;
            continue;
        }
        break;
    }

    // Increment sequence number for next transaction (matches Zephyr seq_num++ at end)
    m_sequence_number++;

    // Parse response if caller wants it
    if (response != nullptr) {
        bool parsed = parse_aplus_response(response_frame, SPI_PROTOCOL_APLUS_FRAME_SIZE, response);
        if (!parsed && !SPI_APLUS_IS_RECOVERABLE(response->status)) {
            return false;
        }
    }

    return true;
}

bool SmdSat::send_command_auto(uint8_t command, const uint8_t *tx_data, uint16_t tx_len,
                               uint8_t *rx_data, uint16_t *rx_len) {
    // Protocol A+ only (no legacy fallback)
    SpiAplusResponse response;

    for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
        if (send_command_aplus(command, tx_data, tx_len, &response)) {
            // Copy response data if requested
            if (rx_data != nullptr && rx_len != nullptr && *rx_len > 0 && response.data_len > 0) {
                uint16_t copy_len = (response.data_len < *rx_len) ? response.data_len : *rx_len;
                memcpy(rx_data, response.data, copy_len);
                *rx_len = copy_len;
            } else if (rx_len != nullptr) {
                *rx_len = response.data_len;
            }

            if (response.status == SPI_APLUS_STATUS_OK) {
                return true;
            }

            if (!SPI_APLUS_IS_RECOVERABLE(response.status)) {
                DEBUG_WARN("SmdSat::%s: Non-recoverable error %u for cmd 0x%02X",
                           __func__, static_cast<int>(response.status), command);
                return false;
            }
        }

        DEBUG_TRACE("SmdSat::%s: Retry %u/%u for cmd 0x%02X",
                    __func__, retry + 1, SMDSAT_SPI_MAX_RETRIES, command);
        nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
    }

    return false;
}

// 2-phase write helper (matches Zephyr argos_spi_write_2phase)
// Phase 1: Send REQ command to prepare STM32 for data reception
// Phase 2: Send WRITE command with actual data
bool SmdSat::send_command_2phase(uint8_t req_cmd, uint8_t write_cmd,
                                  const uint8_t *data, uint16_t len) {
    DEBUG_TRACE("SmdSat::%s: REQ=0x%02X, WRITE=0x%02X, len=%u", __func__, req_cmd, write_cmd, len);

    // Phase 1: Send REQ command
    if (!send_command_auto(req_cmd)) {
        DEBUG_ERROR("SmdSat::%s: REQ 0x%02X failed", __func__, req_cmd);
        return false;
    }

    // Small delay for STM32 to prepare RX buffer
    nrf_delay_ms(1);

    // Phase 2: Send WRITE command with data
    if (!send_command_auto(write_cmd, data, len)) {
        DEBUG_ERROR("SmdSat::%s: WRITE 0x%02X failed", __func__, write_cmd);
        return false;
    }

    return true;
}

void SmdSat::load_kmac_profil(uint8_t profile)
{
	DEBUG_TRACE("SmdSat::%s: Load KMAC profile %u", __func__, profile);
	if (!send_command_2phase(SMDSAT_CMD_WRITE_KMAC_REQ, SMDSAT_CMD_WRITE_KMAC, &profile, 1)) {
		DEBUG_ERROR("SmdSat::%s: Failed to load KMAC profile", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSat::get_spi_status(uint8_t *status) {
	uint8_t rx[4] = {0};
	uint16_t rx_len = sizeof(rx);
	if (!send_command_auto(SMDSAT_CMD_SPI_STATUS, nullptr, 0, rx, &rx_len)) {
		DEBUG_ERROR("SmdSat::%s: Failed to read SPI status", __func__);
		*status = SMDSAT_SPICMD_ERROR;
		return;
	}
	*status = rx[0];
	DEBUG_TRACE("SmdSat::%s: status=%u", __func__, *status);
}

void SmdSat::get_kmac_status(uint8_t *status) {
	uint8_t rx[4] = {0};
	uint16_t rx_len = sizeof(rx);
	if (!send_command_auto(SMDSAT_CMD_MAC_STATUS, nullptr, 0, rx, &rx_len)) {
		DEBUG_ERROR("SmdSat::%s: Failed to read KMAC status", __func__);
		*status = MAC_ERROR;
		return;
	}
	*status = rx[0];
	DEBUG_TRACE("SmdSat::%s: status=%u", __func__, *status);
}

void SmdSat::set_radio_conf(smd_uint8_array_t *radio_conf) {
	DEBUG_TRACE("SmdSat::%s", __func__);
	if (radio_conf->size != SMDSAT_CMD_WRITECONF_LEN - 1) {
		DEBUG_ERROR("SmdSat::%s: Size mismatch %u != %u",
			__func__, radio_conf->size, SMDSAT_CMD_WRITECONF_LEN - 1);
		return;
	}
	if (!send_command_2phase(SMDSAT_CMD_WRITE_RCONF_REQ, SMDSAT_CMD_WRITE_RCONF,
	                         radio_conf->p_data, radio_conf->size)) {
		DEBUG_ERROR("SmdSat::%s: Failed to set radio conf", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSat::read_radio_conf(SmdArgosModulation *modulation) {
	DEBUG_TRACE("SmdSat::%s", __func__);
	uint8_t rx[SMDSAT_CMD_READCONF_LEN] = {0};
	uint16_t rx_len = sizeof(rx);
	if (!send_command_auto(SMDSAT_CMD_READ_RCONF, nullptr, 0, rx, &rx_len)) {
		DEBUG_ERROR("SmdSat::%s: Failed to read radio conf", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}

	uint32_t min_frequency = (rx[3] << 24) | (rx[2] << 16) | (rx[1] << 8) | rx[0];
	uint32_t max_frequency = (rx[7] << 24) | (rx[6] << 16) | (rx[5] << 8) | rx[4];
	int8_t rf_level = rx[8];
	*modulation = static_cast<SmdArgosModulation>(rx[9]);

	DEBUG_INFO("SmdSat::%s: Modulation: %u, Min Freq: %u, Max Freq: %u, RF Level %d", __func__,
		*modulation, min_frequency, max_frequency, rf_level);
}

bool SmdSat::save_radio_conf() {
	DEBUG_TRACE("SmdSat::%s", __func__);
	if (!send_command_auto(SMDSAT_CMD_SAVE_RCONF)) {
		DEBUG_WARN("SmdSat::%s: Radio conf failed to save", __func__);
		return false;
	}
	DEBUG_INFO("SmdSat::%s: Radio conf saved", __func__);
	return true;
}

void SmdSat::read_lpm(uint8_t *lpm_mode) {
	DEBUG_TRACE("SmdSat::%s", __func__);
	uint8_t rx[4] = {0};
	uint16_t rx_len = sizeof(rx);
	if (!send_command_auto(SMDSAT_CMD_READ_LPM, nullptr, 0, rx, &rx_len)) {
		DEBUG_ERROR("SmdSat::%s: Failed to read LPM", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
	*lpm_mode = rx[0];
	DEBUG_INFO("SmdSat::SMD LPM mode is: %u", *lpm_mode);
}

void SmdSat::write_lpm(uint8_t *lpm_mode) {
	DEBUG_TRACE("SmdSat::%s", __func__);
	if (!send_command_2phase(SMDSAT_CMD_WRITE_LPM_REQ, SMDSAT_CMD_WRITE_LPM, lpm_mode, 1)) {
		DEBUG_ERROR("SmdSat::%s: Failed to write LPM", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSat::read_version(uint8_t *version) {
	DEBUG_TRACE("SmdSat::%s", __func__);
	uint8_t rx[4] = {0};
	uint16_t rx_len = sizeof(rx);
	if (!send_command_auto(SMDSAT_CMD_READ_VERSION, nullptr, 0, rx, &rx_len)) {
		DEBUG_ERROR("SmdSat::%s: Failed to read version", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
	*version = rx[0];
	DEBUG_INFO("SmdSat::SMD SPI Version: %u", *version);
}

void SmdSat::read_address(smd_uint8_array_t *address) {
	DEBUG_TRACE("SmdSat::%s", __func__);
	if (address->size != SMDSAT_CMD_READ_ADDR_LEN) {
		DEBUG_ERROR("SmdSat::%s: Size mismatch %u != %u",
			__func__, address->size, SMDSAT_CMD_READ_ADDR_LEN);
		return;
	}
	uint16_t rx_len = address->size;
	if (!send_command_auto(SMDSAT_CMD_READ_ADDR, nullptr, 0, address->p_data, &rx_len)) {
		DEBUG_ERROR("SmdSat::%s: Failed to read address", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSat::set_address(smd_uint8_array_t *address) {
	DEBUG_TRACE("SmdSat::%s", __func__);
	if (address->size != SMDSAT_CMD_WRITE_ADDR_LEN - 1) {
		DEBUG_ERROR("SmdSat::%s: Size mismatch %u != %u",
			__func__, address->size, SMDSAT_CMD_WRITE_ADDR_LEN - 1);
		return;
	}
	if (!send_command_2phase(SMDSAT_CMD_WRITE_ADDR_REQ, SMDSAT_CMD_WRITE_ADDR,
	                         address->p_data, address->size)) {
		DEBUG_ERROR("SmdSat::%s: Failed to set address", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSat::read_seckey(smd_uint8_array_t *seckey) {
	DEBUG_TRACE("SmdSat::%s", __func__);
	if (seckey->size != SMDSAT_CMD_READ_SECKEY_LEN) {
		DEBUG_ERROR("SmdSat::%s: Size mismatch %u != %u",
			__func__, seckey->size, SMDSAT_CMD_READ_SECKEY_LEN);
		return;
	}
	uint16_t rx_len = seckey->size;
	if (!send_command_auto(SMDSAT_CMD_READ_SECKEY, nullptr, 0, seckey->p_data, &rx_len)) {
		DEBUG_ERROR("SmdSat::%s: Failed to read seckey", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSat::set_seckey(smd_uint8_array_t *seckey) {
	DEBUG_TRACE("SmdSat::%s", __func__);
	if (seckey->size != SMDSAT_CMD_WRITE_SECKEY_LEN - 1) {
		DEBUG_ERROR("SmdSat::%s: Size mismatch %u != %u",
			__func__, seckey->size, SMDSAT_CMD_WRITE_SECKEY_LEN);
		return;
	}
	if (!send_command_2phase(SMDSAT_CMD_WRITE_SECKEY_REQ, SMDSAT_CMD_WRITE_SECKEY,
	                         seckey->p_data, seckey->size)) {
		DEBUG_ERROR("SmdSat::%s: Failed to set seckey", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSat::read_id(uint32_t *id) {
	if (id == nullptr) {
		DEBUG_ERROR("SmdSat::%s: id ptr is null", __func__);
		return;
	}
	uint8_t rx[SMDSAT_CMD_READ_ID_LEN] = {0};
	uint16_t rx_len = sizeof(rx);
	if (!send_command_auto(SMDSAT_CMD_READ_ID, nullptr, 0, rx, &rx_len)) {
		DEBUG_ERROR("SmdSat::%s: Failed to read ID", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
	*id = 0;
	for (int i = 0; i < SMDSAT_CMD_READ_ID_LEN; i++) {
		*id |= rx[i] << (8 * i);
	}
	DEBUG_INFO("SmdSat::%s: SMD ID: %u", __func__, *id);
}

void SmdSat::set_id(uint32_t id) {
	DEBUG_TRACE("SmdSat::%s: id: %u", __func__, id);
	uint8_t data[4];
	memcpy(data, &id, sizeof(id));
	if (!send_command_2phase(SMDSAT_CMD_WRITE_ID_REQ, SMDSAT_CMD_WRITE_ID, data, sizeof(data))) {
		DEBUG_ERROR("SmdSat::%s: Failed to set ID", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSat::read_tcxo_warmup(uint8_t *time_ms) {
	if (time_ms == nullptr) {
		DEBUG_ERROR("SmdSat::%s: tcxo ptr is null", __func__);
		return;
	}
	uint8_t rx[SMDSAT_CMD_READ_ID_LEN] = {0};
	uint16_t rx_len = sizeof(rx);
	if (!send_command_auto(SMDSAT_CMD_READ_TCXO, nullptr, 0, rx, &rx_len)) {
		DEBUG_ERROR("SmdSat::%s: Failed to read TCXO warmup", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
	*time_ms = 0;
	for (int i = 0; i < SMDSAT_CMD_READ_ID_LEN; i++) {
		*time_ms |= rx[i] << (8 * i);
	}
	DEBUG_INFO("SmdSat::%s: SMD TCXO warmup time: %u", __func__, *time_ms);
}

void SmdSat::write_tcxo_warmup(uint32_t time_ms) {
	DEBUG_TRACE("SmdSat::%s: tcxo warmup time: %u", __func__, time_ms);
	uint8_t data[4];
	memcpy(data, &time_ms, sizeof(time_ms));
	if (!send_command_2phase(SMDSAT_CMD_WRITE_TCXO_REQ, SMDSAT_CMD_WRITE_TCXO, data, sizeof(data))) {
		DEBUG_ERROR("SmdSat::%s: Failed to write TCXO warmup", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSat::read_serial(smd_uint8_array_t *serial) {
	DEBUG_TRACE("SmdSat::%s", __func__);
	if (serial->size != SMDSAT_CMD_READ_SERIAL_LEN) {
		DEBUG_ERROR("SmdSat::%s: Size mismatch %u != %u",
			__func__, serial->size, SMDSAT_CMD_READ_SERIAL_LEN);
		return;
	}
	uint16_t rx_len = serial->size;
	if (!send_command_auto(SMDSAT_CMD_READ_SN, nullptr, 0, serial->p_data, &rx_len)) {
		DEBUG_ERROR("SmdSat::%s: Failed to read serial", __func__);
		throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSat::set_tcxo_warmup_internal(uint32_t time_s)
{
	DEBUG_TRACE("SmdSat::%s", __func__);
	this->write_tcxo_warmup(time_s * 1000);
}

void SmdSat::set_tcxo_control(bool state) {
	(void)state;
	DEBUG_TRACE("SmdSat::%s: Managed by SMD module", __func__);
}

void SmdSat::print_firmware_version() {
	// CMD_READ_VERSION (0x05) via A+ - returns firmware version string
	// (CMD_READ_FIRMWARE 0x06 is NOT supported in A+ by STM32 firmware)
	uint8_t rx[SPI_PROTOCOL_APLUS_MAX_DATA_LEN] = {0};
	uint16_t rx_len = sizeof(rx);

	if (!send_command_auto(SMDSAT_CMD_READ_VERSION, nullptr, 0, rx, &rx_len)) {
		DEBUG_ERROR("SmdSat::%s: CMD_READ_VERSION failed", __func__);
		return;
	}

	size_t len = 0;
	while (len < rx_len && rx[len] != 0 &&
	       rx[len] >= 0x20 && rx[len] < 0x7F) {
		len++;
	}

	if (len > 0) {
		DEBUG_INFO("SmdSat::%s: Firmware=%.*s", __func__, (int)len, (char*)rx);
	} else {
		DEBUG_WARN("SmdSat::%s: Empty firmware version (rx_len=%u)", __func__, rx_len);
	}
}

bool SmdSat::smd_ping()
{
	DEBUG_TRACE("SmdSat::%s", __func__);

	for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
		try {
			if (send_command_auto(SMDSAT_CMD_PING)) {
				DEBUG_INFO("SmdSat::%s: ACK from SMD received", __func__);
				return true;
			}
		} catch (...) {
			DEBUG_WARN("SmdSat::%s: Ping failed, retry %u/%u", __func__, retry + 1, SMDSAT_SPI_MAX_RETRIES);
		}
		nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
	}

	DEBUG_WARN("SmdSat::%s: Ping not received after %u retries", __func__, SMDSAT_SPI_MAX_RETRIES);
	return false;
}

bool SmdSat::is_tx_finished() {
	DEBUG_TRACE("SmdSat::%s", __func__);

	uint8_t rx[4] = {0};
	uint16_t rx_len = sizeof(rx);

	if (!send_command_auto(SMDSAT_CMD_READ_SPIMAC_STATE, nullptr, 0, rx, &rx_len)) {
		DEBUG_WARN("SmdSat::%s: Failed to read SPIMAC state", __func__);
		return false;
	}

	uint8_t spi_state = rx[0];
	uint8_t mac_status = rx[1];

	DEBUG_TRACE("SmdSat::%s: spiState=%02X macStatus=%02X", __func__, spi_state, mac_status);

	if (mac_status == MAC_TX_DONE) {
		DEBUG_INFO("SmdSat::%s: TX completed successfully", __func__);
		return true;
	}

	if (mac_status == MAC_TX_TIMEOUT) {
		DEBUG_WARN("SmdSat::%s: TX timeout (no satellite)", __func__);
		return true;
	}

	if (mac_status == MAC_ERROR) {
		DEBUG_ERROR("SmdSat::%s: TX error", __func__);
		return true;
	}

	return false;
}

bool SmdSat::is_tx_in_progress() {
	DEBUG_TRACE("SmdSat::%s", __func__);

	uint8_t rx[4] = {0};
	uint16_t rx_len = sizeof(rx);

	if (!send_command_auto(SMDSAT_CMD_READ_SPIMAC_STATE, nullptr, 0, rx, &rx_len)) {
		DEBUG_WARN("SmdSat::%s: Failed to read SPIMAC state", __func__);
		return false;
	}

	return (rx[1] == MAC_TX_IN_PROGRESS);
}

bool SmdSat::is_command_accepted() {
	return true;
}

bool SmdSat::is_firmware_ready() {
	return true;
}

void SmdSat::power_off() {
	DEBUG_TRACE("SmdSat::%s",__func__);
    if (!SMD_STATE_EQUAL(stopped)) {
    	m_stopping = true;
    }
}

void SmdSat::power_on() {
	DEBUG_TRACE("SmdSat::%s",__func__);

    if (m_state != SmdSat::stopped) {
    	m_stopping = false;
    	DEBUG_TRACE("SmdSat::%s: not disabled: state=%u", __func__, (unsigned int)m_state);
    	return;
    }

    m_state = SmdSatState::starting;
    m_stopping = false;

    DEBUG_TRACE("SmdSat::%s:: start state machine",__func__);
    state_machine();
}

void SmdSat::power_off_immediate()
{
	DEBUG_TRACE("SmdSat::%s",__func__);

    if (!SMD_STATE_EQUAL(stopped)) {
    	system_scheduler->cancel_task(m_task);
    	SMD_STATE_CHANGE(idle, stopped);
    }
}

SmdSat::SmdSat(unsigned int idle_shutdown_ms) {
	DEBUG_TRACE("SmdSat::%s",__func__);
	set_idle_timeout(idle_shutdown_ms);
    m_packet_buffer.clear();
	m_modulation = ARGOS_MOD_LDA2;
    m_nrf_spim = nullptr;
    m_state = SmdSatState::stopped;
    m_tcxo_warmup_time = DEFAULT_TCXO_WARMUP_TIME_SECONDS;
    m_tx_power = 0;
	// Protocol A+ initialization (forced, no legacy)
	m_protocol_mode = SpiProtocolMode::APLUS;
	m_sequence_number = 0;
	m_protocol_detected = true;
	// DFU initialization
	m_dfu_mode = false;
	memset(&m_dfu_info, 0, sizeof(m_dfu_info));
	this->shutdown();
	is_kmac_profil_loaded = false;
}

SmdSat::~SmdSat() {
	power_off_immediate();
}

void SmdSat::shutdown(void) {
	GPIOPins::init_pin(SAT_RESET);
	GPIOPins::clear(SAT_RESET);
	nrf_delay_ms(SMDSAT_DELAY_RST_MS);
	GPIOPins::clear(SAT_PWR_EN);
	nrf_gpio_cfg_default(BSP::GPIO_Inits[SAT_RESET].pin_number);  // Disconnect reset (ext pull-up)
#ifdef SMD_VPA_PIN
	GPIOPins::drive_low(SMD_VPA_PIN);
#endif
}

void SmdSat::state_machine(bool use_scheduler) {
	m_next_delay = 0;

	switch (m_state) {
	case SmdSat::starting:
		SMD_STATE_CALL(starting);
		break;
	case SmdSat::powering_on:
		SMD_STATE_CALL(powering_on);
		break;
	case SmdSat::load_kmac:
		SMD_STATE_CALL(load_kmac);
		break;
	case SmdSat::idle_pending:
		SMD_STATE_CALL(idle_pending);
		break;
	case SmdSat::idle:
		SMD_STATE_CALL(idle);
		break;
	case SmdSat::transmit_pending:
		SMD_STATE_CALL(transmit_pending);
		break;
	case SmdSat::transmitting:
		SMD_STATE_CALL(transmitting);
		break;
	case SmdSat::error:
		SMD_STATE_CALL(error);
		break;
	case SmdSat::stopped:
		SMD_STATE_CALL(stopped);
		break;
	default:
		break;
	}

	if (use_scheduler && !SMD_STATE_EQUAL(stopped)) {
		system_scheduler->cancel_task(m_task);
		m_task = system_scheduler->post_task_prio([this]() {
			state_machine();
		}, "SmdReceiverStateMachine", Scheduler::DEFAULT_PRIORITY, m_next_delay);
	}
}

void SmdSat::state_starting_enter() {}
void SmdSat::state_starting_exit() {}

void SmdSat::state_starting()
{
	m_is_first_tx = true;
	is_kmac_profil_loaded = false;
	// Force Protocol A+ mode after power cycle
	m_protocol_mode = SpiProtocolMode::APLUS;
	m_protocol_detected = true;
	m_sequence_number = 0;
	m_nrf_spim = new NrfSPIM(SPI_SATELLITE);
	m_state_counter = 3;
	m_next_delay = 0;
	SMD_STATE_CHANGE(starting, powering_on);
}

void SmdSat::state_error_enter() {
	uint8_t status = 0;
	get_kmac_status(&status);
	notify(KineisEventDeviceError({}));
	SMD_STATE_CHANGE(error, stopped);
}

void SmdSat::state_error_exit() {}

void SmdSat::state_error() {
	SMD_STATE_CHANGE(error, stopped);
}

void SmdSat::state_stopped_enter() {
	delete m_nrf_spim;
	m_nrf_spim = nullptr;

	nrf_gpio_cfg_output(BSP::SPI_Inits[SPI_SATELLITE].config.ss_pin);
	nrf_gpio_pin_clear(BSP::SPI_Inits[SPI_SATELLITE].config.ss_pin);
	nrf_gpio_cfg_input(BSP::SPI_Inits[SPI_SATELLITE].config.mosi_pin, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(BSP::SPI_Inits[SPI_SATELLITE].config.miso_pin, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(BSP::SPI_Inits[SPI_SATELLITE].config.sck_pin, NRF_GPIO_PIN_PULLDOWN);

	this->shutdown();
	GPIOPins::release_sensors_pwr();
	is_kmac_profil_loaded = false;
	// Keep Protocol A+ mode forced, just reset sequence number
	m_sequence_number = 0;

	m_packet_buffer.clear();

	notify(KineisEventPowerOff({}));
}

void SmdSat::state_stopped_exit() {}
void SmdSat::state_stopped() {}

void SmdSat::state_powering_on_enter() {}

void SmdSat::state_powering_on_exit() {
	m_next_delay = SMDSAT_DELAY_POWER_ON_MS;
}

void SmdSat::state_powering_on() {
	DEBUG_TRACE("SmdSat::%s: Starting power-on sequence", __func__);

	// Acquire VSENSORS before powering on SMD to ensure stable power rail
	GPIOPins::acquire_sensors_pwr();

	GPIOPins::init_pin(SAT_RESET);
	GPIOPins::clear(SAT_RESET);
	nrf_delay_ms(10);

	GPIOPins::set(SAT_PWR_EN);
#ifdef SMD_VPA_PIN
	// Release VPA to HIGH-Z so SMD can control it autonomously
	GPIOPins::release_to_highz(SMD_VPA_PIN);
#endif
	DEBUG_TRACE("SmdSat::%s: Power enabled, waiting for stabilization", __func__);

	nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);

	GPIOPins::release_to_highz(SAT_RESET);
	DEBUG_TRACE("SmdSat::%s: Reset released", __func__);

	SMD_STATE_CHANGE(powering_on, idle_pending);
}

void SmdSat::state_load_kmac_enter() {}
void SmdSat::state_load_kmac_exit() {}

void SmdSat::state_load_kmac() {
	uint8_t kmac_status = 0;
	get_kmac_status(&kmac_status);
	if (kmac_status == MAC_OK) {
		if (!is_kmac_profil_loaded) {
			load_kmac_profil(1);
			m_next_delay = SMDSAT_DELAY_LOAD_KMAC_MS;
			is_kmac_profil_loaded = true;
		} else {
			m_next_delay = SMDSAT_DELAY_CMD_MS;
		}
		SMD_STATE_CHANGE(load_kmac, idle_pending);
	} else {
		if (--m_state_counter == 0) {
			DEBUG_ERROR("SmdSat::%s: failed to enter load kmac state",__func__);
			SMD_STATE_CHANGE(load_kmac, error);
		} else {
			m_next_delay = SMDSAT_DELAY_TICK_INTERRUPT_MS;
		}
	}
}

void SmdSat::state_idle_pending_enter() {
	m_next_delay = SMDSAT_DELAY_CMD_MS;
	m_state_counter = 10;
}

void SmdSat::state_idle_pending_exit() {
	m_next_delay = SMDSAT_DELAY_CMD_MS;
}

void SmdSat::state_idle_pending() {
	if (smd_ping()) {
		if (!is_kmac_profil_loaded) {
			SMD_STATE_CHANGE(idle_pending, load_kmac);
		} else {
			SMD_STATE_CHANGE(idle_pending, idle);
		}
	} else {
		if (--m_state_counter == 0) {
			DEBUG_ERROR("SmdSat::%s: failed to enter IDLE state",__func__);
			SMD_STATE_CHANGE(idle_pending, error);
		} else {
			m_next_delay = SMDSAT_DELAY_CMD_MS;
		}
	}
}

void SmdSat::state_idle_enter() {
	m_next_delay = SMDSAT_DELAY_TICK_INTERRUPT_MS;
	m_state_counter = m_idle_timeout_ms / SMDSAT_DELAY_TICK_INTERRUPT_MS;
}

void SmdSat::state_idle_exit() {
	m_next_delay = SMDSAT_DELAY_CMD_MS;
}

void SmdSat::state_idle() {
	if (m_packet_buffer.length()) {
		m_tx_buffer = m_packet_buffer;
		m_packet_buffer.clear();
		SMD_STATE_CHANGE(idle, transmit_pending);
	} else if (m_stopping) {
		SMD_STATE_CHANGE(idle, stopped);
	} else {
		m_next_delay = SMDSAT_DELAY_TICK_INTERRUPT_MS;
		if (--m_state_counter == 0) {
		 	DEBUG_TRACE("SmdSat::%s: idle timeout elapsed",__func__);
			SMD_STATE_CHANGE(idle, stopped);
		}
		return;
	}

	m_state_counter = m_idle_timeout_ms / SMDSAT_DELAY_TICK_INTERRUPT_MS;
}

void SmdSat::state_transmit_pending_enter() {}

void SmdSat::state_transmit_pending_exit() {
	m_next_delay = SMDSAT_DELAY_CMD_TX + (m_tcxo_warmup_time*1000);
}

void SmdSat::state_transmit_pending() {
	if (m_tx_buffer.size()) {
		initiate_tx();
        notify(KineisEventTxStarted({}));
        SMD_STATE_CHANGE(transmit_pending, transmitting);
	} else {
		SMD_STATE_CHANGE(transmit_pending, idle);
		if (--m_state_counter == 0) {
			DEBUG_ERROR("SmdSat::%s: failed accept SEND command",__func__);
	        SMD_STATE_CHANGE(transmit_pending, error);
		} else
			m_next_delay = SMDSAT_DELAY_CMD_MS;
	}
}

void SmdSat::state_transmitting_enter() {
    DEBUG_TRACE("SmdSat::%s", __func__);
	// Calculate max poll attempts based on TCXO warmup + TX time + margin
	// TCXO warmup (default 2s) + TX time (~1-3s) + margin (2s) = ~7s total
	// Poll every SMDSAT_TIMING_POLL_MS (500ms)
	uint32_t total_timeout_ms = (m_tcxo_warmup_time * 1000) + 5000;  // TCXO warmup + 5s margin
	m_state_counter = (total_timeout_ms / SMDSAT_TIMING_POLL_MS) + 1;
	DEBUG_TRACE("SmdSat::%s: poll timeout=%ums, counter=%u", __func__, total_timeout_ms, m_state_counter);
}

void SmdSat::state_transmitting_exit() {
	m_is_first_tx = false;
}

void SmdSat::state_transmitting() {
	if (is_tx_finished()) {
        if (m_tx_buffer.size()) {
        	m_tx_buffer.clear();
        	notify(KineisEventTxComplete({}));
        }
        SMD_STATE_CHANGE(transmitting, stopped);
	} else {
		if (!m_tx_buffer.size()) {
			SMD_STATE_CHANGE(transmitting, stopped);
		} else if (--m_state_counter == 0) {
			DEBUG_ERROR("SmdSat::%s: TX timeout after polling",__func__);
			SMD_STATE_CHANGE(transmitting, error);
		} else {
			// Poll every SMDSAT_TIMING_POLL_MS (500ms)
			m_next_delay = SMDSAT_TIMING_POLL_MS;
		}
	}
}

void SmdSat::stop_send() {
	DEBUG_TRACE("SmdSat::%s",__func__);
	m_packet_buffer.clear();
	m_tx_buffer.clear();
}

void SmdSat::start_receive(const KineisModulation mode) {
	(void)mode;
	DEBUG_TRACE("SmdSat::%s: Not supported",__func__);
}

bool SmdSat::stop_receive() {
	DEBUG_TRACE("SmdSat::%s: Not supported",__func__);
	return false;
}

void SmdSat::set_frequency(double freq_mhz) {
	DEBUG_TRACE("SmdSat::%s",__func__);
	m_tx_freq = freq_mhz;
}

void SmdSat::initiate_tx() {
	DEBUG_TRACE("SmdSat::%s: Size: %u", __func__, m_tx_buffer.size());

	uint16_t size = m_tx_buffer.size();

	// Step 1: WRITE_TX_REQ (0x14) - Prepare for TX
	if (!send_command_auto(SMDSAT_CMD_WRITE_TX_REQ)) {
		DEBUG_ERROR("SmdSat::%s: TX REQ failed", __func__);
		return;
	}

	// Step 2: WRITE_TX_SIZE (0x15) - Send size (little-endian, matching Zephyr)
	uint8_t size_data[2];
	size_data[0] = size & 0xFF;
	size_data[1] = (size >> 8) & 0xFF;
	if (!send_command_auto(SMDSAT_CMD_WRITE_TX_SIZE, size_data, 2)) {
		DEBUG_ERROR("SmdSat::%s: TX SIZE failed", __func__);
		return;
	}

	// Step 3: WRITE_TX (0x16) - Send actual payload data
	if (!send_command_auto(SMDSAT_CMD_WRITE_TX, reinterpret_cast<const uint8_t*>(m_tx_buffer.data()), size)) {
		DEBUG_ERROR("SmdSat::%s: TX DATA failed", __func__);
		return;
	}

	// Post-TX delay for STM32 async processing (matches Zephyr ARGOS_SPI_POST_TX_DELAY_MS)
	nrf_delay_ms(SMDSAT_SPI_POST_TX_DELAY_MS);
}

void SmdSat::send(const KineisModulation mode, const KineisPacket& user_payload, const unsigned int payload_length)
{
	(void)mode;
    DEBUG_TRACE("SmdSat::%s: length %u", __func__, payload_length);

    unsigned int max_payload_size = 0;
    switch(m_modulation) {
        case ARGOS_MOD_LDA2:
        case ARGOS_MOD_LDA2L:
            max_payload_size = ARGOS_TX_LDA2_PAYLOAD_BYTE_SIZE;
            break;
        case ARGOS_MOD_LDK:
            max_payload_size = ARGOS_TX_LDK_PAYLOAD_BYTE_SIZE;
            break;
        case ARGOS_MOD_VLDA4:
            max_payload_size = ARGOS_TX_VLDA4_PAYLOAD_BYTE_SIZE;
            break;
        default:
            DEBUG_TRACE("SmdSat::%s: Unknown modulation type %d", __func__, m_modulation);
            return;
    }

    unsigned int effective_payload_length = std::min(payload_length,
                                                     static_cast<unsigned int>(user_payload.size()));
    if (effective_payload_length > max_payload_size) {
        DEBUG_TRACE("SmdSat::%s: Payload truncated from %u to %u bytes",
                    __func__, effective_payload_length, max_payload_size);
        effective_payload_length = max_payload_size;
    }

    m_packet_buffer.assign(max_payload_size, 0);
    std::copy(user_payload.begin(), user_payload.begin() + effective_payload_length, m_packet_buffer.begin());

    DEBUG_TRACE("SmdSat::%s: raw data[%u]=%s", __func__,
                effective_payload_length,
                Binascii::hexlify(m_packet_buffer).c_str());

    DEBUG_TRACE("SmdSat::%s::Packet size %u", __func__, static_cast<unsigned int>(m_packet_buffer.size()));

    power_on();
}

void SmdSat::set_tcxo_warmup_time(unsigned int time_s) {
	m_tcxo_warmup_time = time_s;
}

void SmdSat::set_tx_power(unsigned int power) {
	m_tx_power = power;
}

void SmdSat::set_credentials(unsigned int dec_id, unsigned int address, const std::string& seckey, const std::string& radioconf) {
	DEBUG_TRACE("SmdSat::%s",__func__);
	bool stop_spi = false;
	bool was_stopped = (m_state == SmdSatState::stopped);

	auto wait_between_commands = [this]() -> bool {
		nrf_delay_ms(SMDSAT_DELAY_CMD_MS * 2);
		return true;
	};

	if (was_stopped) {
		GPIOPins::acquire_sensors_pwr();
		GPIOPins::init_pin(SAT_RESET);
		GPIOPins::clear(SAT_RESET);
		GPIOPins::set(SAT_PWR_EN);
#ifdef SMD_VPA_PIN
		GPIOPins::release_to_highz(SMD_VPA_PIN);
#endif
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);
		GPIOPins::release_to_highz(SAT_RESET);
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);
	}
	if (m_nrf_spim == nullptr) {
		m_nrf_spim = new NrfSPIM(SPI_SATELLITE);
		stop_spi = true;
	}

	// Force A+ protocol mode
	m_protocol_mode = SpiProtocolMode::APLUS;
	m_protocol_detected = true;
	m_sequence_number = 0;

	{
		bool smd_ready = false;
		for (uint8_t attempt = 0; attempt < 10; attempt++) {
			nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS / 2);
			if (smd_ping()) {
				DEBUG_INFO("SmdSat::%s: SMD ready after %u attempts", __func__, attempt + 1);
				smd_ready = true;
				break;
			}
		}
		if (!smd_ready) {
			DEBUG_ERROR("SmdSat::%s: SMD not ready after power-on, aborting", __func__);
			goto cleanup;
		}
	}

	for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
		try {
			set_id(dec_id);
			break;
		} catch (...) {
			if (retry == SMDSAT_SPI_MAX_RETRIES - 1) {
				DEBUG_ERROR("SmdSat::%s: Failed to set ID", __func__);
				goto cleanup;
			}
			nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
		}
	}

	if (!wait_between_commands()) goto cleanup;

	{
		uint8_t address_data[4] = {
			static_cast<uint8_t>((address >> 24) & 0xFF),
			static_cast<uint8_t>((address >> 16) & 0xFF),
			static_cast<uint8_t>((address >> 8) & 0xFF),
			static_cast<uint8_t>(address & 0xFF)
		};
		smd_uint8_array_t address_val = {SMDSAT_CMD_WRITE_ADDR_LEN-1, address_data};

		for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
			try {
				set_address(&address_val);
				break;
			} catch (...) {
				if (retry == SMDSAT_SPI_MAX_RETRIES - 1) {
					DEBUG_ERROR("SmdSat::%s: Failed to set address", __func__);
					goto cleanup;
				}
				nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
			}
		}
	}

	if (!wait_between_commands()) goto cleanup;

	{
		std::string seckey_val = Binascii::unhexlify(seckey);
		smd_uint8_array_t seckey_struct = {static_cast<uint16_t>(seckey_val.size()),
		                               reinterpret_cast<uint8_t *>(seckey_val.data())};

		for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
			try {
				set_seckey(&seckey_struct);
				break;
			} catch (...) {
				if (retry == SMDSAT_SPI_MAX_RETRIES - 1) {
					DEBUG_ERROR("SmdSat::%s: Failed to set seckey", __func__);
					goto cleanup;
				}
				nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
			}
		}
	}

	if (!wait_between_commands()) goto cleanup;

	{
		std::string radioconf_val = Binascii::unhexlify(radioconf);
		smd_uint8_array_t radioconf_struct = {static_cast<uint16_t>(radioconf_val.size()),
		                                  reinterpret_cast<uint8_t *>(radioconf_val.data())};

		for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
			try {
				set_radio_conf(&radioconf_struct);
				break;
			} catch (...) {
				if (retry == SMDSAT_SPI_MAX_RETRIES - 1) {
					DEBUG_ERROR("SmdSat::%s: Failed to set radio conf", __func__);
					goto cleanup;
				}
				nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
			}
		}
	}

	wait_between_commands();

	for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
		try {
			if (save_radio_conf()) break;
			if (retry == SMDSAT_SPI_MAX_RETRIES - 1) goto cleanup;
		} catch (...) {
			if (retry == SMDSAT_SPI_MAX_RETRIES - 1) goto cleanup;
		}
		nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	}

	wait_between_commands();

	for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
		try {
			load_kmac_profil(1);
			break;
		} catch (...) {
			if (retry == SMDSAT_SPI_MAX_RETRIES - 1) goto cleanup;
			nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
		}
	}

	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	DEBUG_INFO("SmdSat::%s: Credentials set successfully", __func__);

cleanup:
	if (stop_spi) {
		delete m_nrf_spim;
		m_nrf_spim = nullptr;
	}
	if (was_stopped) {
		shutdown();
		GPIOPins::release_sensors_pwr();
	}
}

void SmdSat::read_credentials(unsigned int *dec_id, unsigned int *address, std::string *seckey, std::string *radioconf) {
	DEBUG_TRACE("SmdSat::%s",__func__);
	bool stop_spi = false;
	if (m_state == SmdSatState::stopped) {
		GPIOPins::acquire_sensors_pwr();
		GPIOPins::init_pin(SAT_RESET);
		GPIOPins::clear(SAT_RESET);
		GPIOPins::set(SAT_PWR_EN);
#ifdef SMD_VPA_PIN
		GPIOPins::release_to_highz(SMD_VPA_PIN);
#endif
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);
		GPIOPins::release_to_highz(SAT_RESET);
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);
	}
	if (m_nrf_spim == nullptr) {
		m_nrf_spim = new NrfSPIM(SPI_SATELLITE);
		stop_spi = true;
	}

	// Force A+ protocol mode
	m_protocol_mode = SpiProtocolMode::APLUS;
	m_protocol_detected = true;
	m_sequence_number = 0;

	nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);

	if (dec_id) {
		uint32_t dec_id_val = 0;
        read_id(&dec_id_val);
		*dec_id = static_cast<unsigned int>(dec_id_val);
		nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
    }

    if (address) {
        uint8_t address_data[SMDSAT_CMD_READ_ADDR_LEN] = {0};
        smd_uint8_array_t address_value = {SMDSAT_CMD_READ_ADDR_LEN, address_data};
        read_address(&address_value);
		nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
		*address  = (address_value.p_data[0] << 24) |
					(address_value.p_data[1] << 16) |
					(address_value.p_data[2] << 8)  |
					(address_value.p_data[3]);
    }

    if (seckey) {
        uint8_t seckey_data[SMDSAT_CMD_READ_SECKEY_LEN] = {0};
        smd_uint8_array_t seckey_value = {SMDSAT_CMD_READ_SECKEY_LEN, seckey_data};
        read_seckey(&seckey_value);
		nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
        *seckey = Binascii::hexlify(std::string(reinterpret_cast<char *>(seckey_data), SMDSAT_CMD_READ_SECKEY_LEN));
    }

	SmdArgosModulation modulation_dev;
	read_radio_conf(&modulation_dev);
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	(void)radioconf;

	if(stop_spi) {
		delete m_nrf_spim;
		m_nrf_spim = nullptr;
	}
	if (m_state == SmdSatState::stopped) {
		shutdown();
		GPIOPins::release_sensors_pwr();
	}
}

// ============================================================================
// DFU (Device Firmware Update) Implementation
// ============================================================================

// CRC-32 MPEG-2 calculation (polynomial 0x04C11DB7)
// Compatible with Zephyr argos-smd-driver
uint32_t SmdSat::calculate_crc32(const uint8_t *data, size_t len) {
	return spi_crc32_mpeg2(data, len);
}

// DFU-specific send command — matches Zephyr dfu_send_cmd exactly:
// 1. Build proper A+ CMD frame
// 2. Send CMD frame (ignore RX — it's previous response)
// 3. Wait command-specific delay
// 4. Send raw 0xAA idle bytes to read response (NOT a proper A+ NOP frame!)
// 5. Re-poll with 0xAA if no valid response (0x55) in first 8 bytes
// 6. Parse response
// 7. Increment sequence number ONCE
//
// Key differences from send_command_aplus (app-mode):
// - NOP frame is raw 0xAA idle bytes, not a valid A+ frame
// - Sequence number increments only once per command
// - Response retry checks for ANY non-0x55 (not just IDLE/BUSY patterns)
SmdDfuResponse SmdSat::dfu_send_command(uint8_t cmd, const uint8_t *data, uint16_t data_len,
                                        uint8_t *response_data, uint16_t *response_len) {
	DEBUG_TRACE("SmdSat::%s: cmd=0x%02X, data_len=%u, seq=%u", __func__, cmd, data_len, m_sequence_number);

	uint8_t tx_buf[SPI_PROTOCOL_APLUS_FRAME_SIZE];
	uint8_t rx_buf[SPI_PROTOCOL_APLUS_FRAME_SIZE];

	// ═══ Step 1: Build Protocol A+ request frame ═══
	// [0xAA][SEQ][CMD][LEN][DATA...][CRC8][padding 0xAA]
	// Note: Zephyr DFU pads with DFU_IDLE_PATTERN (0xAA), not 0xFF
	uint16_t idx = 0;
	tx_buf[idx++] = SPI_PROTOCOL_APLUS_MAGIC_REQUEST;
	tx_buf[idx++] = m_sequence_number;
	tx_buf[idx++] = cmd;
	tx_buf[idx++] = static_cast<uint8_t>(data_len);
	if (data != nullptr && data_len > 0) {
		memcpy(&tx_buf[idx], data, data_len);
		idx += data_len;
	}
	tx_buf[idx] = spi_crc8_ccitt(tx_buf, idx);
	idx++;
	// Pad rest with 0xAA (matches Zephyr DFU_IDLE_PATTERN padding)
	if (idx < SPI_PROTOCOL_APLUS_FRAME_SIZE) {
		memset(&tx_buf[idx], SPI_PROTOCOL_APLUS_IDLE_PATTERN, SPI_PROTOCOL_APLUS_FRAME_SIZE - idx);
	}

	// Inter-transaction delay
	nrf_delay_ms(SMDSAT_SPI_INTER_TX_DELAY_MS);

	// ═══ Step 2: Transaction 1 — Send command (RX is old data, ignore) ═══
	memset(rx_buf, 0, sizeof(rx_buf));
	int ret = m_nrf_spim->transfer(tx_buf, rx_buf, SPI_PROTOCOL_APLUS_FRAME_SIZE);
	if (ret) {
		DEBUG_ERROR("SmdSat::%s: SPI TX failed", __func__);
		return DFU_RSP_ERROR;
	}

	DEBUG_TRACE("SmdSat::%s: RX1 (ignore): %02X %02X %02X %02X",
	            __func__, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);

	// ═══ Step 3: Wait for slave to process command ═══
	uint32_t cmd_delay = get_command_delay(cmd);
	nrf_delay_ms(cmd_delay);

	// ═══ Step 4: Transaction 2 — Send raw 0xAA idle to read response ═══
	// (Zephyr DFU sends memset(tx_buf, DFU_IDLE_PATTERN, 64) — NOT a valid A+ frame!)
	memset(tx_buf, SPI_PROTOCOL_APLUS_IDLE_PATTERN, SPI_PROTOCOL_APLUS_FRAME_SIZE);
	memset(rx_buf, 0, sizeof(rx_buf));
	ret = m_nrf_spim->transfer(tx_buf, rx_buf, SPI_PROTOCOL_APLUS_FRAME_SIZE);
	if (ret) {
		DEBUG_ERROR("SmdSat::%s: SPI RX failed", __func__);
		return DFU_RSP_ERROR;
	}

	DEBUG_TRACE("SmdSat::%s: RX2: %02X %02X %02X %02X %02X %02X %02X %02X",
	            __func__, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3],
	            rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7]);

	// ═══ Step 4b: Re-poll until valid response (0x55) received ═══
	// Check first 8 bytes for response magic (matches Zephyr has_valid_response)
	constexpr uint8_t DFU_BUSY_RETRY_COUNT = 10;
	constexpr uint8_t DFU_BUSY_RETRY_DELAY_MS = 20;
	uint8_t busy_retries = 0;

	auto has_valid_response = [](const uint8_t *buf, uint8_t len) -> bool {
		uint8_t check_len = (len < 8) ? len : 8;
		for (uint8_t i = 0; i < check_len; i++) {
			if (buf[i] == SPI_PROTOCOL_APLUS_MAGIC_RESPONSE) {
				return true;
			}
		}
		return false;
	};

	while (!has_valid_response(rx_buf, SPI_PROTOCOL_APLUS_FRAME_SIZE) &&
	       busy_retries < DFU_BUSY_RETRY_COUNT) {
		DEBUG_TRACE("SmdSat::%s: No response yet (0x%02X), re-poll %u/%u",
		            __func__, rx_buf[0], busy_retries + 1, DFU_BUSY_RETRY_COUNT);

		nrf_delay_ms(DFU_BUSY_RETRY_DELAY_MS);

		// Re-poll: send idle pattern, read response
		memset(tx_buf, SPI_PROTOCOL_APLUS_IDLE_PATTERN, SPI_PROTOCOL_APLUS_FRAME_SIZE);
		memset(rx_buf, 0, sizeof(rx_buf));
		ret = m_nrf_spim->transfer(tx_buf, rx_buf, SPI_PROTOCOL_APLUS_FRAME_SIZE);
		if (ret) {
			DEBUG_ERROR("SmdSat::%s: SPI re-poll failed", __func__);
			return DFU_RSP_ERROR;
		}

		DEBUG_TRACE("SmdSat::%s: RX2 re-poll: %02X %02X %02X %02X %02X %02X %02X %02X",
		            __func__, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3],
		            rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7]);

		busy_retries++;
	}

	if (!has_valid_response(rx_buf, SPI_PROTOCOL_APLUS_FRAME_SIZE)) {
		DEBUG_ERROR("SmdSat::%s: No valid response after %u retries", __func__, DFU_BUSY_RETRY_COUNT);
		m_sequence_number++;
		return DFU_RSP_ERROR;
	}

	// ═══ Step 5: Parse response frame ═══
	// Find 0x55 magic, parse [0x55][SEQ][STATUS][LEN][DATA...][CRC8]
	SpiAplusResponse response;
	bool parsed = parse_aplus_response(rx_buf, SPI_PROTOCOL_APLUS_FRAME_SIZE, &response);

	// Increment sequence number ONCE (matches Zephyr dfu_send_cmd)
	m_sequence_number++;

	if (!parsed) {
		DEBUG_ERROR("SmdSat::%s: Failed to parse response", __func__);
		return DFU_RSP_ERROR;
	}

	// Copy response data if requested
	if (response_data != nullptr && response_len != nullptr && response.data_len > 0) {
		uint16_t copy_len = (*response_len < response.data_len) ? *response_len : response.data_len;
		memcpy(response_data, response.data, copy_len);
		*response_len = copy_len;
	}

	// Map Protocol A+ status directly to DFU response
	// (STATUS byte 0x00=OK, 0x01=ERROR, etc. — same codes as SmdDfuResponse)
	return static_cast<SmdDfuResponse>(response.status);
}

SmdDfuResponse SmdSat::dfu_ping() {
	DEBUG_TRACE("SmdSat::%s", __func__);

	uint8_t response_data[4];
	uint16_t response_len = sizeof(response_data);

	SmdDfuResponse result = dfu_send_command(SMDSAT_CMD_DFU_PING, nullptr, 0,
	                                         response_data, &response_len);

	if (result == DFU_RSP_OK && response_len >= 2) {
		DEBUG_INFO("SmdSat::%s: Bootloader v%u.%u", __func__,
		           response_data[0], response_data[1]);
	}

	return result;
}

SmdDfuResponse SmdSat::dfu_get_info(SmdDfuInfo *info) {
	DEBUG_TRACE("SmdSat::%s", __func__);

	if (info == nullptr) {
		return DFU_RSP_ERROR;
	}

	uint8_t response_data[32];
	uint16_t response_len = sizeof(response_data);

	SmdDfuResponse result = dfu_send_command(SMDSAT_CMD_DFU_GET_INFO, nullptr, 0,
	                                         response_data, &response_len);

	if (result == DFU_RSP_OK && response_len >= 15) {
		info->version_major = response_data[0];
		info->version_minor = response_data[1];
		info->version_patch = response_data[2];
		// Little-endian parsing (matches Zephyr argos_dfu_get_info)
		info->app_start_addr = response_data[3] | (response_data[4] << 8) |
		                       (response_data[5] << 16) | (response_data[6] << 24);
		info->app_max_size = response_data[7] | (response_data[8] << 8) |
		                     (response_data[9] << 16) | (response_data[10] << 24);
		info->flash_page_size = response_data[11] | (response_data[12] << 8) |
		                        (response_data[13] << 16) | (response_data[14] << 24);

		DEBUG_INFO("SmdSat::%s: Bootloader v%u.%u.%u, app_start=0x%08X, max_size=%u, page_size=%u",
		           __func__, info->version_major, info->version_minor, info->version_patch,
		           info->app_start_addr, info->app_max_size, info->flash_page_size);
	}

	return result;
}

SmdDfuResponse SmdSat::dfu_erase() {
	DEBUG_TRACE("SmdSat::%s", __func__);

	// Erase takes time, use longer timeout
	PMU::kick_watchdog();
	nrf_delay_ms(100);

	SmdDfuResponse result = dfu_send_command(SMDSAT_CMD_DFU_ERASE, nullptr, 0, nullptr, nullptr);

	if (result == DFU_RSP_OK) {
		// Wait for erase to complete (~3 seconds for 102 pages)
		// Kick watchdog periodically during the wait
		for (unsigned int i = 0; i < SMDSAT_DFU_ERASE_TIMEOUT_MS / 500; i++) {
			PMU::kick_watchdog();
			nrf_delay_ms(500);
		}
		DEBUG_INFO("SmdSat::%s: Flash erased", __func__);
	}

	return result;
}

SmdDfuResponse SmdSat::dfu_write_chunk(uint32_t addr, const uint8_t *data, uint16_t len) {
	DEBUG_TRACE("SmdSat::%s: addr=0x%08X, len=%u", __func__, addr, len);

	if (data == nullptr || len == 0 || len > SMDSAT_DFU_CHUNK_SIZE) {
		return DFU_RSP_SIZE_ERROR;
	}

	// Step 1: Send WRITE_REQ with address and length (little-endian, matches Zephyr)
	uint8_t req_data[6];
	req_data[0] = addr & 0xFF;
	req_data[1] = (addr >> 8) & 0xFF;
	req_data[2] = (addr >> 16) & 0xFF;
	req_data[3] = (addr >> 24) & 0xFF;
	req_data[4] = len & 0xFF;
	req_data[5] = (len >> 8) & 0xFF;

	SmdDfuResponse result = dfu_send_command(SMDSAT_CMD_DFU_WRITE_REQ, req_data, 6, nullptr, nullptr);
	if (result != DFU_RSP_OK) {
		DEBUG_ERROR("SmdSat::%s: WRITE_REQ failed", __func__);
		return result;
	}

	nrf_delay_ms(10);

	// Step 2: Send WRITE_DATA with actual data
	result = dfu_send_command(SMDSAT_CMD_DFU_WRITE_DATA, data, len, nullptr, nullptr);
	if (result != DFU_RSP_OK) {
		DEBUG_ERROR("SmdSat::%s: WRITE_DATA failed at addr 0x%08X", __func__, addr);
	}

	return result;
}

SmdDfuResponse SmdSat::dfu_read_chunk(uint32_t addr, uint8_t *data, uint16_t len) {
	DEBUG_TRACE("SmdSat::%s: addr=0x%08X, len=%u", __func__, addr, len);

	if (data == nullptr || len == 0) {
		return DFU_RSP_SIZE_ERROR;
	}

	// Step 1: Send READ_REQ with address and length (little-endian, matches Zephyr)
	uint8_t req_data[6];
	req_data[0] = addr & 0xFF;
	req_data[1] = (addr >> 8) & 0xFF;
	req_data[2] = (addr >> 16) & 0xFF;
	req_data[3] = (addr >> 24) & 0xFF;
	req_data[4] = len & 0xFF;
	req_data[5] = (len >> 8) & 0xFF;

	SmdDfuResponse result = dfu_send_command(SMDSAT_CMD_DFU_READ_REQ, req_data, 6, nullptr, nullptr);
	if (result != DFU_RSP_OK) {
		DEBUG_ERROR("SmdSat::%s: READ_REQ failed", __func__);
		return result;
	}

	nrf_delay_ms(10);

	// Step 2: Read data
	uint16_t response_len = len;
	result = dfu_send_command(SMDSAT_CMD_DFU_READ_DATA, nullptr, 0, data, &response_len);

	return result;
}

SmdDfuResponse SmdSat::dfu_verify(uint32_t crc32) {
	DEBUG_TRACE("SmdSat::%s: crc32=0x%08X", __func__, crc32);

	// Send CRC32 in little-endian format (matches Zephyr argos_dfu_verify)
	uint8_t crc_data[4];
	crc_data[0] = crc32 & 0xFF;
	crc_data[1] = (crc32 >> 8) & 0xFF;
	crc_data[2] = (crc32 >> 16) & 0xFF;
	crc_data[3] = (crc32 >> 24) & 0xFF;

	SmdDfuResponse result = dfu_send_command(SMDSAT_CMD_DFU_VERIFY, crc_data, 4, nullptr, nullptr);

	if (result == DFU_RSP_OK) {
		DEBUG_INFO("SmdSat::%s: CRC verification passed", __func__);
	} else {
		DEBUG_ERROR("SmdSat::%s: CRC verification failed", __func__);
	}

	return result;
}

SmdDfuResponse SmdSat::dfu_reset() {
	DEBUG_TRACE("SmdSat::%s", __func__);
	return dfu_send_command(SMDSAT_CMD_DFU_RESET, nullptr, 0, nullptr, nullptr);
}

SmdDfuResponse SmdSat::dfu_jump() {
	DEBUG_TRACE("SmdSat::%s", __func__);

	SmdDfuResponse result = dfu_send_command(SMDSAT_CMD_DFU_JUMP, nullptr, 0, nullptr, nullptr);

	if (result == DFU_RSP_OK) {
		m_dfu_mode = false;
		DEBUG_INFO("SmdSat::%s: Jumping to application", __func__);
		nrf_delay_ms(SMDSAT_DFU_RESET_DELAY_MS);
	}

	return result;
}

SmdDfuResponse SmdSat::dfu_get_status(uint8_t *status) {
	DEBUG_TRACE("SmdSat::%s", __func__);

	uint8_t response_data[4];
	uint16_t response_len = sizeof(response_data);

	SmdDfuResponse result = dfu_send_command(SMDSAT_CMD_DFU_GET_STATUS, nullptr, 0,
	                                         response_data, &response_len);

	if (result == DFU_RSP_OK && status != nullptr && response_len >= 1) {
		*status = response_data[0];
	}

	return result;
}

SmdDfuResponse SmdSat::dfu_abort() {
	DEBUG_TRACE("SmdSat::%s", __func__);
	return dfu_send_command(SMDSAT_CMD_DFU_ABORT, nullptr, 0, nullptr, nullptr);
}

SmdDfuResponse SmdSat::dfu_set_header(const uint8_t *header) {
	DEBUG_TRACE("SmdSat::%s", __func__);

	if (header == nullptr) {
		return DFU_RSP_ERROR;
	}

	return dfu_send_command(SMDSAT_CMD_DFU_SET_HEADER, header, SMDSAT_DFU_HEADER_SIZE, nullptr, nullptr);
}

// Reproduces exact Zephyr spi_dfu_test flow:
// STEP 1: SYNC (5x 0xFF frames, wait for 0xAA idle)
// STEP 2: PING APP (cmd 0x02, standard A+ protocol)
// STEP 3: DFU_ENTER (cmd 0x3F, send-only)
// STEP 4: WAIT 2s FOR STM32 RESET
// STEP 5: SYNC WITH BOOTLOADER (5x 0xFF frames)
// STEP 6: PING BOOTLOADER (cmd 0x30, DFU protocol)
bool SmdSat::dfu_enter() {
	DEBUG_TRACE("SmdSat::%s", __func__);

	bool was_stopped = (m_state == SmdSatState::stopped);
	bool stop_spi = false;

	if (was_stopped) {
		// Power on SMD WITHOUT asserting hardware reset.
		// DFU_ENTER (0x3F) is a software command to the running app —
		// the STM32 will software-reset into bootloader itself.
		// Just power on and let the STM32 internal POR handle clean boot.
		GPIOPins::acquire_sensors_pwr();
		GPIOPins::set(SAT_PWR_EN);
#ifdef SMD_VPA_PIN
		GPIOPins::release_to_highz(SMD_VPA_PIN);
#endif
		// SAT_RESET left disconnected (high-Z) — STM32 internal pull-up keeps it HIGH
		PMU::kick_watchdog();
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);
		PMU::kick_watchdog();
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);
	}

	if (m_nrf_spim == nullptr) {
		m_nrf_spim = new NrfSPIM(SPI_SATELLITE);
		stop_spi = true;
	}

	// Force A+ protocol mode
	m_protocol_mode = SpiProtocolMode::APLUS;
	m_protocol_detected = true;
	m_sequence_number = 0;

	// Wait for SMD application to be ready
	PMU::kick_watchdog();
	nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS / 2);

	// ═══ STEP 1: SYNC with APP (matches Zephyr spi_dfu_test STEP 1) ═══
	// Send 5 dummy 0xFF frames, check for 0xAA idle response
	DEBUG_INFO("SmdSat::%s: STEP 1 - SPI sync with app...", __func__);
	{
		uint8_t sync_tx[SPI_PROTOCOL_APLUS_FRAME_SIZE];
		uint8_t sync_rx[SPI_PROTOCOL_APLUS_FRAME_SIZE];
		memset(sync_tx, 0xFF, sizeof(sync_tx));

		m_sequence_number = 0;
		for (int i = 0; i < 5; i++) {
			PMU::kick_watchdog();
			memset(sync_rx, 0, sizeof(sync_rx));
			m_nrf_spim->transfer(sync_tx, sync_rx, SPI_PROTOCOL_APLUS_FRAME_SIZE);
			nrf_delay_ms(SMDSAT_SPI_DETECT_TIMEOUT_MS);

			DEBUG_TRACE("SmdSat::%s: Sync %d RX: %02X %02X %02X %02X %02X %02X %02X %02X",
			            __func__, i, sync_rx[0], sync_rx[1], sync_rx[2], sync_rx[3],
			            sync_rx[4], sync_rx[5], sync_rx[6], sync_rx[7]);
		}
	}
	PMU::kick_watchdog();
	nrf_delay_ms(100);

	// ═══ STEP 2: PING APP (matches Zephyr spi_dfu_test STEP 2) ═══
	// Use standard A+ protocol (send_command_aplus) with cmd 0x02
	DEBUG_INFO("SmdSat::%s: STEP 2 - Ping app (cmd 0x02)...", __func__);
	m_sequence_number = 0;
	{
		SpiAplusResponse app_response;
		bool app_ok = send_command_aplus(SMDSAT_CMD_PING, nullptr, 0, &app_response);
		if (app_ok && app_response.status == SPI_APLUS_STATUS_OK) {
			DEBUG_INFO("SmdSat::%s: APP PING OK!", __func__);
		} else {
			DEBUG_WARN("SmdSat::%s: APP PING failed, checking if already in bootloader...", __func__);
			// Try DFU ping - maybe already in bootloader mode
			m_sequence_number = 0;
			SmdDfuResponse bl_ping = dfu_ping();
			if (bl_ping == DFU_RSP_OK) {
				DEBUG_INFO("SmdSat::%s: Already in BOOTLOADER mode!", __func__);
				m_dfu_mode = true;
				return true;
			}
			DEBUG_WARN("SmdSat::%s: No response from app or bootloader, trying DFU_ENTER anyway...", __func__);
		}
	}
	PMU::kick_watchdog();
	nrf_delay_ms(200);

	// ═══ STEP 3: DFU_ENTER (matches Zephyr spi_dfu_test STEP 3) ═══
	// Send CMD 0x3F as send-only (STM32 resets immediately, can't respond)
	DEBUG_INFO("SmdSat::%s: STEP 3 - DFU_ENTER (cmd 0x3F, send-only)...", __func__);
	{
		uint8_t tx_buf[SPI_PROTOCOL_APLUS_FRAME_SIZE];
		uint8_t rx_buf[SPI_PROTOCOL_APLUS_FRAME_SIZE];

		build_aplus_frame(tx_buf, SMDSAT_CMD_DFU_ENTER, nullptr, 0);
		memset(rx_buf, 0, sizeof(rx_buf));
		m_nrf_spim->transfer(tx_buf, rx_buf, SPI_PROTOCOL_APLUS_FRAME_SIZE);
		m_sequence_number++;
	}

	// Reset DFU sequence number (matches Zephyr data->dfu_seq_num = 0)
	m_sequence_number = 0;

	// ═══ STEP 4: WAIT 2s FOR STM32 RESET (matches Zephyr spi_dfu_test STEP 4) ═══
	DEBUG_INFO("SmdSat::%s: STEP 4 - Waiting 2s for STM32 reset...", __func__);
	for (int i = 0; i < 4; i++) {
		PMU::kick_watchdog();
		nrf_delay_ms(500);
	}

	// ═══ STEP 5: SYNC WITH BOOTLOADER (matches Zephyr spi_dfu_test STEP 5) ═══
	DEBUG_INFO("SmdSat::%s: STEP 5 - SPI sync with bootloader...", __func__);
	{
		uint8_t sync_tx[SPI_PROTOCOL_APLUS_FRAME_SIZE];
		uint8_t sync_rx[SPI_PROTOCOL_APLUS_FRAME_SIZE];
		memset(sync_tx, 0xFF, sizeof(sync_tx));

		for (int i = 0; i < 5; i++) {
			PMU::kick_watchdog();
			memset(sync_rx, 0, sizeof(sync_rx));
			m_nrf_spim->transfer(sync_tx, sync_rx, SPI_PROTOCOL_APLUS_FRAME_SIZE);
			nrf_delay_ms(SMDSAT_SPI_DETECT_TIMEOUT_MS);

			DEBUG_TRACE("SmdSat::%s: BL Sync %d RX: %02X %02X %02X %02X %02X %02X %02X %02X",
			            __func__, i, sync_rx[0], sync_rx[1], sync_rx[2], sync_rx[3],
			            sync_rx[4], sync_rx[5], sync_rx[6], sync_rx[7]);
		}
	}
	PMU::kick_watchdog();
	nrf_delay_ms(200);

	// Reset sequence number for bootloader session
	m_sequence_number = 0;

	// ═══ STEP 6: PING BOOTLOADER (matches Zephyr spi_dfu_test STEP 6) ═══
	DEBUG_INFO("SmdSat::%s: STEP 6 - Ping bootloader (cmd 0x30)...", __func__);
	SmdDfuResponse ping_result = dfu_ping();
	if (ping_result == DFU_RSP_OK) {
		m_dfu_mode = true;
		DEBUG_INFO("SmdSat::%s: BOOTLOADER PING OK! DFU mode entered.", __func__);
		return true;
	}

	// If first ping fails, retry a few more times (matches Zephyr argos_dfu_wait_ready)
	DEBUG_WARN("SmdSat::%s: First bootloader ping failed, retrying...", __func__);
	for (uint8_t ping_retry = 0; ping_retry < 20; ping_retry++) {
		PMU::kick_watchdog();
		nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
		ping_result = dfu_ping();
		if (ping_result == DFU_RSP_OK) {
			m_dfu_mode = true;
			DEBUG_INFO("SmdSat::%s: BOOTLOADER PING OK (retry %u)! DFU mode entered.", __func__, ping_retry + 1);
			return true;
		}
	}

	DEBUG_ERROR("SmdSat::%s: Bootloader not responding after DFU_ENTER", __func__);

	// Cleanup on failure
	if (stop_spi) {
		delete m_nrf_spim;
		m_nrf_spim = nullptr;
	}
	if (was_stopped) {
		shutdown();
		GPIOPins::release_sensors_pwr();
	}

	return false;
}

bool SmdSat::dfu_exit() {
	DEBUG_TRACE("SmdSat::%s", __func__);

	if (!m_dfu_mode) {
		DEBUG_WARN("SmdSat::%s: Not in DFU mode", __func__);
		return true;  // Already in application mode
	}

	SmdDfuResponse result = dfu_jump();
	if (result == DFU_RSP_OK) {
		m_dfu_mode = false;
		return true;
	}

	return false;
}

bool SmdSat::dfu_get_bootloader_info(SmdDfuInfo *info) {
	if (!m_dfu_mode) {
		DEBUG_WARN("SmdSat::%s: Not in DFU mode", __func__);
		return false;
	}

	return (dfu_get_info(info) == DFU_RSP_OK);
}

SmdDfuResponse SmdSat::firmware_update(const uint8_t *firmware, size_t size,
                                       void (*progress_callback)(uint8_t percent)) {
	DEBUG_INFO("SmdSat::%s: Starting firmware update, size=%u bytes", __func__, (unsigned int)size);

	if (firmware == nullptr || size == 0) {
		DEBUG_ERROR("SmdSat::%s: Invalid firmware data", __func__);
		return DFU_RSP_ERROR;
	}

	SmdDfuResponse result;

	// Step 1: Enter DFU mode if not already
	if (!m_dfu_mode) {
		DEBUG_INFO("SmdSat::%s: Entering DFU mode...", __func__);
		if (!dfu_enter()) {
			DEBUG_ERROR("SmdSat::%s: Failed to enter DFU mode", __func__);
			return DFU_RSP_NOT_READY;
		}
	}

	if (progress_callback) progress_callback(5);

	// Step 2: Ping bootloader
	DEBUG_INFO("SmdSat::%s: Pinging bootloader...", __func__);
	result = dfu_ping();
	if (result != DFU_RSP_OK) {
		DEBUG_ERROR("SmdSat::%s: Bootloader not responding", __func__);
		return result;
	}

	if (progress_callback) progress_callback(10);

	// Step 3: Get bootloader info
	DEBUG_INFO("SmdSat::%s: Getting bootloader info...", __func__);
	result = dfu_get_info(&m_dfu_info);
	if (result != DFU_RSP_OK) {
		DEBUG_ERROR("SmdSat::%s: Failed to get bootloader info", __func__);
		return result;
	}

	// Validate firmware size
	if (size > m_dfu_info.app_max_size) {
		DEBUG_ERROR("SmdSat::%s: Firmware too large (%u > %u)", __func__,
		            (unsigned int)size, m_dfu_info.app_max_size);
		return DFU_RSP_SIZE_ERROR;
	}

	if (progress_callback) progress_callback(15);

	// Step 4: Erase application area
	DEBUG_INFO("SmdSat::%s: Erasing flash...", __func__);
	result = dfu_erase();
	if (result != DFU_RSP_OK) {
		DEBUG_ERROR("SmdSat::%s: Flash erase failed", __func__);
		return result;
	}

	if (progress_callback) progress_callback(25);

	// Step 5: Write firmware in chunks
	DEBUG_INFO("SmdSat::%s: Writing firmware...", __func__);
	uint32_t addr = m_dfu_info.app_start_addr;
	size_t remaining = size;
	size_t offset = 0;
	uint8_t last_progress = 25;

	while (remaining > 0) {
		uint16_t chunk_size = (remaining > SMDSAT_DFU_CHUNK_SIZE) ?
		                      SMDSAT_DFU_CHUNK_SIZE : (uint16_t)remaining;

		result = dfu_write_chunk(addr, &firmware[offset], chunk_size);
		if (result != DFU_RSP_OK) {
			DEBUG_ERROR("SmdSat::%s: Write failed at offset %u", __func__, (unsigned int)offset);
			dfu_abort();
			return result;
		}

		addr += chunk_size;
		offset += chunk_size;
		remaining -= chunk_size;

		// Update progress (25% to 85% for write phase)
		if (progress_callback) {
			uint8_t progress = 25 + (uint8_t)((offset * 60) / size);
			if (progress > last_progress) {
				progress_callback(progress);
				last_progress = progress;
			}
		}

		// Small delay between chunks
		nrf_delay_ms(5);
	}

	DEBUG_INFO("SmdSat::%s: Firmware written successfully", __func__);

	if (progress_callback) progress_callback(90);

	// Step 6: Verify CRC32
	DEBUG_INFO("SmdSat::%s: Verifying firmware CRC...", __func__);
	uint32_t crc = calculate_crc32(firmware, size);
	result = dfu_verify(crc);
	if (result != DFU_RSP_OK) {
		DEBUG_ERROR("SmdSat::%s: CRC verification failed", __func__);
		return result;
	}

	if (progress_callback) progress_callback(95);

	// Step 7: Jump to application
	DEBUG_INFO("SmdSat::%s: Jumping to application...", __func__);
	result = dfu_jump();
	if (result != DFU_RSP_OK) {
		DEBUG_ERROR("SmdSat::%s: Jump to application failed", __func__);
		return result;
	}

	if (progress_callback) progress_callback(100);

	DEBUG_INFO("SmdSat::%s: Firmware update completed successfully!", __func__);
	return DFU_RSP_OK;
}

SmdDfuResponse SmdSat::firmware_update(File *file, size_t size, uint32_t stm32_crc32,
                                       void (*progress_callback)(uint8_t percent)) {
	DEBUG_INFO("SmdSat::%s: Starting streamed firmware update, size=%u bytes, CRC32=0x%08X",
	           __func__, (unsigned int)size, stm32_crc32);

	if (file == nullptr || size == 0) {
		DEBUG_ERROR("SmdSat::%s: Invalid file or size", __func__);
		return DFU_RSP_ERROR;
	}

	SmdDfuResponse result;

	// Step 1: Enter DFU mode if not already
	if (!m_dfu_mode) {
		DEBUG_INFO("SmdSat::%s: Entering DFU mode...", __func__);
		if (!dfu_enter()) {
			DEBUG_ERROR("SmdSat::%s: Failed to enter DFU mode", __func__);
			return DFU_RSP_NOT_READY;
		}
	}

	if (progress_callback) progress_callback(5);

	// Step 2: Ping bootloader
	PMU::kick_watchdog();
	DEBUG_INFO("SmdSat::%s: Pinging bootloader...", __func__);
	result = dfu_ping();
	if (result != DFU_RSP_OK) {
		DEBUG_ERROR("SmdSat::%s: Bootloader not responding", __func__);
		return result;
	}

	if (progress_callback) progress_callback(10);

	// Step 3: Get bootloader info
	PMU::kick_watchdog();
	DEBUG_INFO("SmdSat::%s: Getting bootloader info...", __func__);
	result = dfu_get_info(&m_dfu_info);
	if (result != DFU_RSP_OK) {
		DEBUG_ERROR("SmdSat::%s: Failed to get bootloader info", __func__);
		return result;
	}

	// Validate firmware size
	if (size > m_dfu_info.app_max_size) {
		DEBUG_ERROR("SmdSat::%s: Firmware too large (%u > %u)", __func__,
		            (unsigned int)size, m_dfu_info.app_max_size);
		return DFU_RSP_SIZE_ERROR;
	}

	if (progress_callback) progress_callback(15);

	// Step 4: Erase application area (3+ seconds — dfu_erase kicks watchdog internally)
	DEBUG_INFO("SmdSat::%s: Erasing flash...", __func__);
	result = dfu_erase();
	if (result != DFU_RSP_OK) {
		DEBUG_ERROR("SmdSat::%s: Flash erase failed", __func__);
		return result;
	}

	if (progress_callback) progress_callback(25);

	// Step 5: Write firmware in chunks — streamed from file (no large heap allocation)
	DEBUG_INFO("SmdSat::%s: Writing firmware (streamed)...", __func__);
	uint32_t addr = m_dfu_info.app_start_addr;
	size_t remaining = size;
	size_t offset = 0;
	uint8_t last_progress = 25;
	uint8_t chunk_buf[SMDSAT_DFU_CHUNK_SIZE];

	while (remaining > 0) {
		uint16_t chunk_size = (remaining > SMDSAT_DFU_CHUNK_SIZE) ?
		                      SMDSAT_DFU_CHUNK_SIZE : (uint16_t)remaining;

		PMU::kick_watchdog();

		lfs_ssize_t bytes_read = file->read(chunk_buf, chunk_size);
		if (bytes_read != (lfs_ssize_t)chunk_size) {
			DEBUG_ERROR("SmdSat::%s: File read failed at offset %u (got %d, expected %u)",
			            __func__, (unsigned int)offset, (int)bytes_read, chunk_size);
			dfu_abort();
			return DFU_RSP_ERROR;
		}

		result = dfu_write_chunk(addr, chunk_buf, chunk_size);
		if (result != DFU_RSP_OK) {
			DEBUG_ERROR("SmdSat::%s: Write failed at offset %u", __func__, (unsigned int)offset);
			dfu_abort();
			return result;
		}

		addr += chunk_size;
		offset += chunk_size;
		remaining -= chunk_size;

		// Update progress (25% to 85% for write phase)
		if (progress_callback) {
			uint8_t progress = 25 + (uint8_t)((offset * 60) / size);
			if (progress > last_progress) {
				progress_callback(progress);
				last_progress = progress;
			}
		}

		// Small delay between chunks
		nrf_delay_ms(5);
	}

	DEBUG_INFO("SmdSat::%s: Firmware written successfully", __func__);

	if (progress_callback) progress_callback(90);

	// Step 6: Verify CRC32 — use pre-computed CRC from firmware header
	DEBUG_INFO("SmdSat::%s: Verifying firmware CRC (0x%08X)...", __func__, stm32_crc32);
	result = dfu_verify(stm32_crc32);
	if (result != DFU_RSP_OK) {
		DEBUG_ERROR("SmdSat::%s: CRC verification failed", __func__);
		return result;
	}

	if (progress_callback) progress_callback(95);

	// Step 7: Jump to application
	DEBUG_INFO("SmdSat::%s: Jumping to application...", __func__);
	result = dfu_jump();
	if (result != DFU_RSP_OK) {
		DEBUG_ERROR("SmdSat::%s: Jump to application failed", __func__);
		return result;
	}

	if (progress_callback) progress_callback(100);

	DEBUG_INFO("SmdSat::%s: Streamed firmware update completed successfully!", __func__);
	return DFU_RSP_OK;
}

SmdDfuResponse SmdSat::firmware_update_from_file(const std::string& filepath,
                                                 void (*progress_callback)(uint8_t percent)) {
	DEBUG_INFO("SmdSat::%s: Loading firmware from %s", __func__, filepath.c_str());

	// Note: File reading requires filesystem support
	// This is a placeholder - actual implementation depends on platform filesystem

	(void)filepath;
	(void)progress_callback;

	DEBUG_ERROR("SmdSat::%s: File-based update not implemented - use firmware_update() with data buffer", __func__);
	return DFU_RSP_ERROR;
}

std::string SmdSat::get_firmware_version() {
	DEBUG_TRACE("SmdSat::%s", __func__);

	bool was_stopped = (m_state == SmdSatState::stopped);
	bool need_spi_cleanup = false;

	// Initialize SPI BEFORE power-on (same as state_starting)
	// SPI pins must be configured before powering SMD to avoid floating pins issues
	if (m_nrf_spim == nullptr) {
		m_nrf_spim = new NrfSPIM(SPI_SATELLITE);
		need_spi_cleanup = true;
	}

	// Power on SMD if needed (same sequence as state_powering_on)
	if (was_stopped) {
		DEBUG_INFO("SmdSat::%s: Powering on SMD to read version", __func__);
		GPIOPins::acquire_sensors_pwr();
		GPIOPins::init_pin(SAT_RESET);
		GPIOPins::clear(SAT_RESET);
		nrf_delay_ms(10);
		GPIOPins::set(SAT_PWR_EN);
#ifdef SMD_VPA_PIN
		GPIOPins::release_to_highz(SMD_VPA_PIN);
#endif
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);
		GPIOPins::release_to_highz(SAT_RESET);
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);
	}

	std::string version = "";

	// Force Protocol A+ mode for ping
	m_protocol_detected = true;
	m_protocol_mode = SpiProtocolMode::APLUS;
	m_sequence_number = 0;

	try {
		// Ping via A+ to confirm SMD is ready
		bool smd_ready = false;
		for (uint8_t attempt = 0; attempt < 10; attempt++) {
			nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS / 2);
			if (smd_ping()) {
				DEBUG_INFO("SmdSat::%s: SMD ready after %u attempts", __func__, attempt + 1);
				smd_ready = true;
				break;
			}
		}

		if (!smd_ready) {
			DEBUG_ERROR("SmdSat::%s: SMD not ready, cannot read firmware version", __func__);
		} else {
			// CMD_READ_VERSION (0x05) via A+ - returns firmware version string
			// (CMD_READ_FIRMWARE 0x06 is NOT supported in A+ by STM32 firmware)
			uint8_t rx[SPI_PROTOCOL_APLUS_MAX_DATA_LEN] = {0};
			uint16_t rx_len = sizeof(rx);

			if (send_command_auto(SMDSAT_CMD_READ_VERSION, nullptr, 0, rx, &rx_len)) {
				size_t len = 0;
				while (len < rx_len && rx[len] != 0 &&
				       rx[len] >= 0x20 && rx[len] < 0x7F) {
					len++;
				}
				version = std::string((char*)rx, len);
				DEBUG_INFO("SmdSat::%s: Firmware version: %s (len=%zu)", __func__, version.c_str(), len);
			} else {
				DEBUG_ERROR("SmdSat::%s: CMD_READ_VERSION failed", __func__);
			}
		}
	} catch (...) {
		DEBUG_ERROR("SmdSat::%s: Failed to read firmware version", __func__);
	}

	// Cleanup if we initialized SPI
	if (need_spi_cleanup) {
		delete m_nrf_spim;
		m_nrf_spim = nullptr;
	}

	// Full power off SMD if we powered it on
	if (was_stopped) {
		shutdown();
		GPIOPins::release_sensors_pwr();
	}

	return version;
}

std::string SmdSat::smd_spi_test() {
	DEBUG_INFO("SmdSat::%s: === SMD SPI COMMAND TEST START ===", __func__);

	bool was_stopped = (m_state == SmdSatState::stopped);
	bool need_spi_cleanup = false;
	std::string result;

	// Initialize SPI if needed
	if (m_nrf_spim == nullptr) {
		m_nrf_spim = new NrfSPIM(SPI_SATELLITE);
		need_spi_cleanup = true;
	}

	// Power on SMD if needed
	if (was_stopped) {
		DEBUG_INFO("SmdSat::%s: Powering on SMD for test", __func__);
		GPIOPins::acquire_sensors_pwr();
		GPIOPins::init_pin(SAT_RESET);
		GPIOPins::clear(SAT_RESET);
		nrf_delay_ms(10);
		GPIOPins::set(SAT_PWR_EN);
#ifdef SMD_VPA_PIN
		GPIOPins::release_to_highz(SMD_VPA_PIN);
#endif
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);
		GPIOPins::release_to_highz(SAT_RESET);
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);
	}

	// Force A+ protocol
	m_protocol_detected = true;
	m_protocol_mode = SpiProtocolMode::APLUS;
	m_sequence_number = 0;

	// Helper: test a read command and log raw bytes
	struct CmdTest {
		uint8_t cmd;
		const char *name;
		uint16_t expected_len;  // 0 = no data expected (status-only)
	};

	CmdTest tests[] = {
		{ SMDSAT_CMD_PING,             "PING",             0 },
		{ SMDSAT_CMD_READ_VERSION,     "READ_VERSION",     1 },
		{ SMDSAT_CMD_READ_FIRMWARE,    "READ_FIRMWARE",    SMDSAT_CMD_READ_FIRMWARE_LEN },
		{ SMDSAT_CMD_SPI_STATUS,       "SPI_STATUS",       1 },
		{ SMDSAT_CMD_MAC_STATUS,       "MAC_STATUS",       1 },
		{ SMDSAT_CMD_READ_ID,          "READ_ID",          SMDSAT_CMD_READ_ID_LEN },
		{ SMDSAT_CMD_READ_ADDR,        "READ_ADDR",        SMDSAT_CMD_READ_ADDR_LEN },
		{ SMDSAT_CMD_READ_SN,          "READ_SN",          SMDSAT_CMD_READ_SERIAL_LEN },
		{ SMDSAT_CMD_READ_RCONF,       "READ_RCONF",       SMDSAT_CMD_READCONF_LEN },
		{ SMDSAT_CMD_READ_KMAC,        "READ_KMAC",        2 },
		{ SMDSAT_CMD_READ_LPM,         "READ_LPM",         1 },
		{ SMDSAT_CMD_READ_SECKEY,      "READ_SECKEY",      SMDSAT_CMD_READ_SECKEY_LEN },
		{ SMDSAT_CMD_READ_TCXO,        "READ_TCXO",        4 },
		{ SMDSAT_CMD_READ_SPIMAC_STATE,"READ_SPIMAC_STATE",2 },
	};

	uint8_t pass_count = 0;
	uint8_t fail_count = 0;
	uint8_t num_tests = sizeof(tests) / sizeof(tests[0]);

	try {
		// Wait for SMD to become ready with ping
		bool smd_ready = false;
		for (uint8_t attempt = 0; attempt < 10; attempt++) {
			nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS / 2);
			if (smd_ping()) {
				DEBUG_INFO("SmdSat::%s: SMD ready after %u attempts", __func__, attempt + 1);
				smd_ready = true;
				break;
			}
		}

		if (!smd_ready) {
			DEBUG_ERROR("SmdSat::%s: SMD not ready - aborting test", __func__);
			result = "FAIL: SMD not ready (ping failed)";
		} else {
			std::string fails;

			for (uint8_t i = 0; i < num_tests; i++) {
				SpiAplusResponse response = {};

				DEBUG_INFO("--- Test %u/%u: CMD 0x%02X (%s) ---",
				           i + 1, num_tests, tests[i].cmd, tests[i].name);

				bool ok = false;
				try {
					ok = send_command_aplus(tests[i].cmd, nullptr, 0, &response);
				} catch (...) {
					DEBUG_ERROR("  EXCEPTION during transfer");
				}

				if (ok && response.status == SPI_APLUS_STATUS_OK) {
					pass_count++;
					DEBUG_INFO("  PASS: status=%u len=%u", (unsigned)response.status, response.data_len);

					// Log raw data (up to 32 bytes)
					uint16_t print_len = response.data_len;
					if (print_len > 32) print_len = 32;
					if (print_len > 0) {
						char hex[97] = {0};  // 32*3 + 1
						for (uint16_t j = 0; j < print_len; j++) {
							uint8_t hi = response.data[j] >> 4;
							uint8_t lo = response.data[j] & 0x0F;
							hex[j * 3]     = hi < 10 ? ('0' + hi) : ('A' + hi - 10);
							hex[j * 3 + 1] = lo < 10 ? ('0' + lo) : ('A' + lo - 10);
							hex[j * 3 + 2] = ' ';
						}
						hex[print_len * 3] = 0;
						DEBUG_INFO("  DATA[%u]: %s", response.data_len, hex);
					}

					// For string-type commands, try to print as string
					if (tests[i].cmd == SMDSAT_CMD_READ_VERSION ||
					    tests[i].cmd == SMDSAT_CMD_READ_FIRMWARE ||
					    tests[i].cmd == SMDSAT_CMD_READ_SN) {
						size_t slen = 0;
						while (slen < response.data_len && response.data[slen] != 0 &&
						       response.data[slen] >= 0x20 && response.data[slen] < 0x7F) {
							slen++;
						}
						if (slen > 0) {
							DEBUG_INFO("  STR: \"%.*s\"", (int)slen, (char*)response.data);
						}
					}
				} else {
					fail_count++;
					DEBUG_ERROR("  FAIL: ok=%d status=%u len=%u",
					            ok, (unsigned)response.status, response.data_len);
					if (!fails.empty()) fails += " ";
					fails += tests[i].name;
				}

				// Small delay between commands
				nrf_delay_ms(50);
			}

			DEBUG_INFO("=== TEST SUMMARY: %u/%u PASSED, %u FAILED ===",
			           pass_count, num_tests, fail_count);

			// Compact DTE result: "13/14 OK" or "13/14 FAIL:READ_FIRMWARE"
			result = std::to_string(pass_count) + "/" + std::to_string(num_tests);
			if (fail_count == 0) {
				result += " ALL OK";
			} else {
				result += " FAIL:" + fails;
			}
		}
	} catch (...) {
		DEBUG_ERROR("SmdSat::%s: Exception during test", __func__);
		result = "FAIL: Exception";
	}

	// Cleanup
	if (need_spi_cleanup) {
		delete m_nrf_spim;
		m_nrf_spim = nullptr;
	}
	if (was_stopped) {
		shutdown();
		GPIOPins::release_sensors_pwr();
	}

	DEBUG_INFO("SmdSat::%s: === SMD SPI COMMAND TEST END ===", __func__);
	return result;
}
