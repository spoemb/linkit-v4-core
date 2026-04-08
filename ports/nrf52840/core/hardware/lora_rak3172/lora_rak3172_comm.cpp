#include "lora_rak3172_comm.hpp"
#include "nrf_libuarte_async.h"
#include "bsp.hpp"
#include "error.hpp"
#include "debug.hpp"
#include "interrupt_lock.hpp"
#include <list>
#include <cstring>

using namespace LoRa;

static void lora_nrf_libuarte_async_evt_handler(void * context, nrf_libuarte_async_evt_t * p_evt)
{
    LoRaComm *obj = (LoRaComm *)context;
    if (p_evt->type == NRF_LIBUARTE_ASYNC_EVT_TX_DONE) {
        obj->handle_tx_done();
    } else if (p_evt->type == NRF_LIBUARTE_ASYNC_EVT_RX_DATA) {
        obj->handle_rx_buffer(p_evt->data.rxtx.p_data, (uint16_t)p_evt->data.rxtx.length);
    } else if (p_evt->type == NRF_LIBUARTE_ASYNC_EVT_ERROR) {
        obj->handle_error((unsigned int)p_evt->data.errorsrc);
    } else if (p_evt->type == NRF_LIBUARTE_ASYNC_EVT_ALLOC_ERROR) {
        obj->handle_error(0x100);
    } else if (p_evt->type == NRF_LIBUARTE_ASYNC_EVT_OVERRUN_ERROR) {
        obj->handle_error(0x200);
    }
}

LoRaComm::LoRaComm(unsigned int instance) :
    m_uart_instance(instance),
    m_is_init(false),
    m_is_send_busy(false),
    m_uart_config{},
    m_isr_buf_len(0),
    m_isr_error(false),
    m_isr_error_type(0),
    m_isr_overflow(false)
{
}

void LoRaComm::init(void) {
    if (m_is_init) return;

    // Pre-reserve string buffers to avoid heap reallocation during processing.
    m_rx_buffer.reserve(256);
    m_tx_buffer.reserve(128);
    m_line_buffer.reserve(128);
    m_last_value.reserve(64);

    // Copy BSP config and override baudrate for RAK3172 (115200 baud)
    m_uart_config = BSP::UARTAsync_Inits[m_uart_instance].config;
    m_uart_config.baudrate = NRF_UARTE_BAUDRATE_115200;

    if (nrf_libuarte_async_init(
            BSP::UARTAsync_Inits[m_uart_instance].uart,
            &m_uart_config,
            lora_nrf_libuarte_async_evt_handler,
            (void *)this) != NRF_SUCCESS) {
        DEBUG_ERROR("LoRaComm::init: failed to initialize libuarte");
        throw ErrorCode::RESOURCE_NOT_AVAILABLE;
    };
    nrf_libuarte_async_start_rx(BSP::UARTAsync_Inits[m_uart_instance].uart);
    m_is_init = true;
    m_is_rx_started = true;
    m_isr_buf_len = 0;
    m_isr_error = false;
    m_rx_buffer.clear();
}

void LoRaComm::deinit(void)
{
    if(m_is_init) {
        nrf_libuarte_async_uninit(BSP::UARTAsync_Inits[m_uart_instance].uart);
        m_is_init = false;

        if(m_uart_instance == 1)
        {
            // Fix bug in Nordic SDK for UARTE1 - HF clk and DMA bus not closed on uninit
            *(volatile uint32_t *)0x40028FFC = 0;
            *(volatile uint32_t *)0x40028FFC;
            *(volatile uint32_t *)0x40028FFC = 1;
        }
    }
}

bool LoRaComm::send(ATCmd cmd, const std::optional<std::string>& params)
{
    if(m_is_send_busy)
    {
        DEBUG_ERROR("LoRaComm::send: already busy sending");
        return false;
    }
    m_is_send_busy = true;
    return send_at_cmd(cmd, params);
}

bool LoRaComm::send_raw(const uint8_t* data, size_t len)
{
    if (!m_is_init || len == 0)
        return false;

    if (!m_is_rx_started) {
        nrf_libuarte_async_start_rx(BSP::UARTAsync_Inits[m_uart_instance].uart);
        m_is_rx_started = true;
    }

    ret_code_t ret = nrf_libuarte_async_tx(BSP::UARTAsync_Inits[m_uart_instance].uart,
                                            const_cast<uint8_t*>(data), len);
    return (ret == NRF_SUCCESS);
}

void LoRaComm::set_passthrough(bool active, PassthroughCallback callback)
{
    {
        InterruptLock lock;
        m_passthrough_active = active;
        m_isr_buf_len = 0;
    }
    m_passthrough_callback = callback;
    if (active) {
        m_rx_buffer.clear();
    }
}

struct ATCmd_list {
    ATCmd type;
    std::string value;
    bool expected_params;
};

// AT command string table for RAK3172 RUI3
static const std::list<ATCmd_list> cmd_list = {
    {AT_TEST,       "AT\r\n",          false},
    {AT_RESET,      "ATZ\r\n",         false},
    {AT_GET_VER,    "AT+VER=?\r\n",    false},
    {AT_SET_NWM,    "AT+NWM=",         true},
    {AT_GET_NWM,    "AT+NWM=?\r\n",    false},
    {AT_SET_NJM,    "AT+NJM=",         true},
    {AT_GET_NJM,    "AT+NJM=?\r\n",    false},
    {AT_SET_BAND,   "AT+BAND=",        true},
    {AT_GET_BAND,   "AT+BAND=?\r\n",   false},
    {AT_SET_DEVEUI, "AT+DEVEUI=",      true},
    {AT_GET_DEVEUI, "AT+DEVEUI=?\r\n", false},
    {AT_SET_APPEUI, "AT+APPEUI=",      true},
    {AT_GET_APPEUI, "AT+APPEUI=?\r\n", false},
    {AT_SET_APPKEY, "AT+APPKEY=",      true},
    {AT_GET_APPKEY, "AT+APPKEY=?\r\n", false},
    {AT_SET_DEVADDR,"AT+DEVADDR=",     true},
    {AT_GET_DEVADDR,"AT+DEVADDR=?\r\n",false},
    {AT_SET_APPSKEY,"AT+APPSKEY=",     true},
    {AT_SET_NWKSKEY,"AT+NWKSKEY=",     true},
    {AT_SET_CLASS,  "AT+CLASS=",       true},
    {AT_GET_CLASS,  "AT+CLASS=?\r\n",  false},
    {AT_SET_DR,     "AT+DR=",          true},
    {AT_GET_DR,     "AT+DR=?\r\n",     false},
    {AT_SET_ADR,    "AT+ADR=",         true},
    {AT_GET_ADR,    "AT+ADR=?\r\n",    false},
    {AT_SET_TXP,    "AT+TXP=",         true},
    {AT_GET_TXP,    "AT+TXP=?\r\n",    false},
    {AT_SET_CFM,    "AT+CFM=",         true},
    {AT_GET_CFM,    "AT+CFM=?\r\n",    false},
    {AT_SET_RETY,   "AT+RETY=",        true},
    {AT_JOIN,       "AT+JOIN=",        true},
    {AT_GET_NJS,    "AT+NJS=?\r\n",    false},
    {AT_SEND,       "AT+SEND=",        true},
    {AT_GET_RECV,   "AT+RECV=?\r\n",   false},
    {AT_GET_RSSI,   "AT+RSSI=?\r\n",   false},
    {AT_GET_SNR,    "AT+SNR=?\r\n",    false},
    {AT_SET_DCS,    "AT+DCS=",         true},
    {AT_SET_MASK,   "AT+MASK=",        true},
    {AT_SET_CHS,    "AT+CHS=",         true},
    {AT_SET_RX1DL,  "AT+RX1DL=",      true},
    {AT_SET_RX2DR,  "AT+RX2DR=",      true},
    {AT_SET_PNM,    "AT+PNM=",         true},
    {AT_SET_LPM,    "AT+LPM=",         true},
    {AT_SET_SLEEP,  "AT+SLEEP=",       true},
};

bool LoRaComm::send_at_cmd(ATCmd cmd, const std::optional<std::string>& params)
{
    ret_code_t ret;

    if(!m_is_rx_started)
    {
        nrf_libuarte_async_start_rx(BSP::UARTAsync_Inits[m_uart_instance].uart);
        m_is_rx_started = true;
    }

    auto at_cmd = std::find_if(cmd_list.begin(), cmd_list.end(), [cmd](const ATCmd_list& element)
    {
        return element.type == cmd;
    });

    if (at_cmd == cmd_list.end())
    {
        m_is_send_busy = false;
        return false;
    }

    m_tx_buffer = at_cmd->value;

    if (at_cmd->expected_params)
    {
        if (params.has_value()) {
            m_tx_buffer.append(params.value());
        }
        m_tx_buffer += "\r\n";
    }

    DEBUG_TRACE("LoRaComm::send> %.*s", (int)m_tx_buffer.size() - 2, m_tx_buffer.c_str());

    ret = nrf_libuarte_async_tx(BSP::UARTAsync_Inits[m_uart_instance].uart,
                                 reinterpret_cast<uint8_t*>(m_tx_buffer.data()),
                                 m_tx_buffer.length());

    if(ret != NRF_SUCCESS)
    {
        m_is_send_busy = false;
        DEBUG_ERROR("LoRaComm::send_at_cmd: failed to send ret=%08x", (unsigned int)ret);
    }

    return (ret == NRF_SUCCESS);
}

/**
 * Parse a single line from the RAK3172 response.
 * RAK3172 RUI3 response format:
 *   - "OK\r\n"                           -> success
 *   - "AT_ERROR\r\n"                     -> generic error
 *   - "AT_PARAM_ERROR\r\n"               -> parameter error
 *   - "AT_BUSY_ERROR\r\n"                -> busy
 *   - "AT_NO_NETWORK_JOINED\r\n"         -> not joined
 *   - "+EVT:JOINED\r\n"                  -> join success
 *   - "+EVT:JOIN FAILED\r\n"             -> join failure
 *   - "+EVT:TX_DONE\r\n"                 -> TX complete (unconfirmed)
 *   - "+EVT:SEND CONFIRMED OK\r\n"       -> confirmed TX acknowledged
 *   - "+EVT:SEND CONFIRMED FAILED\r\n"   -> confirmed TX not acknowledged
 *   - "+EVT:RX_1:<rssi>:<snr>:UNICAST:<port>:<payload>\r\n" -> downlink
 *   - "<value>\r\n"                       -> value response (e.g., DEVEUI read)
 */
RespType LoRaComm::parse_rx_line(std::string& line)
{
    // Strip trailing CR/LF in-place (no copy)
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n'))
        line.pop_back();

    if (line.empty())
        return RESP_UNKNOWN;

    // Check for OK
    if (line == OK_RESP)
        return RESP_OK;

    // Check for errors
    if (line == ERR_RESP)
        return RESP_ERROR;
    if (line == PARAM_ERR_RESP)
        return RESP_PARAM_ERROR;
    if (line == BUSY_ERR_RESP)
        return RESP_BUSY_ERROR;
    if (line == NO_NET_RESP)
        return RESP_NO_NETWORK;

    // Check for async events
    if (line.compare(0, EVT_PREFIX.size(), EVT_PREFIX.data(), EVT_PREFIX.size()) == 0)
    {
        if (line == EVT_JOINED)
            return RESP_EVT_JOINED;
        if (line == EVT_JOIN_FAILED)
            return RESP_EVT_JOIN_FAILED;
        if (line == EVT_TX_DONE)
            return RESP_EVT_TX_DONE;
        if (line == EVT_SEND_CONF_OK)
            return RESP_EVT_SEND_CONFIRMED_OK;
        if (line == EVT_SEND_CONF_FAIL)
            return RESP_EVT_SEND_CONFIRMED_FAILED;

        // Check for RX data: +EVT:RX_1:<rssi>:<snr>:UNICAST:<port>:<payload>
        if (line.compare(0, EVT_RX_PREFIX.size(), EVT_RX_PREFIX.data(), EVT_RX_PREFIX.size()) == 0)
        {
            // Extract port and payload from last two colon-separated fields
            size_t last_colon = line.rfind(':');
            if (last_colon != std::string::npos && last_colon > 0) {
                size_t prev_colon = line.rfind(':', last_colon - 1);
                if (prev_colon != std::string::npos) {
                    // Extract payload directly into m_last_value (reuses capacity)
                    m_last_value.assign(line, last_colon + 1, std::string::npos);
                    // Extract port number
                    m_last_rx_port = 0;
                    for (size_t i = prev_colon + 1; i < last_colon; i++) {
                        if (line[i] >= '0' && line[i] <= '9')
                            m_last_rx_port = m_last_rx_port * 10 + (line[i] - '0');
                    }
                    DEBUG_TRACE("LoRaComm: RX port=%d payload=%s", m_last_rx_port, m_last_value.c_str());
                }
            }
            return RESP_EVT_RX;
        }

        // Unknown event - log it
        DEBUG_TRACE("LoRaComm: unknown EVT: %s", line.c_str());
        return RESP_UNKNOWN;
    }

    // Otherwise it's a value response (e.g., DEVEUI, band number, etc.)
    m_last_value.assign(line);
    return RESP_VALUE;
}

void LoRaComm::handle_tx_done(void)
{
    m_is_send_busy = false;
}

/**
 * ISR callback: copy raw data to ISR buffer and free DMA buffer.
 * No string operations or notifications here — all processing is deferred
 * to process_rx() which runs in main context.
 */
void LoRaComm::handle_rx_buffer(uint8_t * buffer, uint16_t length)
{
    uint16_t cur_len = m_isr_buf_len;
    uint16_t space = LoRa::ISR_BUF_SIZE - cur_len;
    uint16_t copy_len = (length < space) ? length : space;
    memcpy(m_isr_buf + cur_len, buffer, copy_len);
    m_isr_buf_len = (uint16_t)(cur_len + copy_len);
    if (copy_len < length)
        m_isr_overflow = true;

    nrf_libuarte_async_rx_free(BSP::UARTAsync_Inits[m_uart_instance].uart, buffer, length);
}

/**
 * ISR callback: set error flag for deferred handling.
 */
void LoRaComm::handle_error(unsigned int error_type) {
    nrf_libuarte_async_stop_rx(BSP::UARTAsync_Inits[m_uart_instance].uart);
    m_is_rx_started = false;
    m_isr_error = true;
    m_isr_error_type = error_type;
}

/**
 * Process ISR-buffered data in main context.
 * Must be called periodically (e.g., in send_AT polling loop or state machine).
 */
void LoRaComm::process_rx()
{
    // Check for UART error (deferred from ISR)
    if (m_isr_error) {
        unsigned int err_type = m_isr_error_type;
        m_isr_error = false;
        m_rx_buffer.clear();
        m_isr_buf_len = 0;
        notify(LoRaCommEventUartError(err_type));
        return;
    }

    // Check for ISR RX buffer overflow (deferred from ISR)
    if (m_isr_overflow) {
        m_isr_overflow = false;
        DEBUG_ERROR("LoRaComm: ISR RX buffer overflow — data lost, AT response may be incomplete");
    }

    // Copy ISR buffer under interrupt lock to prevent race
    if (m_isr_buf_len == 0)
        return;

    uint8_t local_buf[LoRa::ISR_BUF_SIZE];
    uint16_t local_len;
    {
        InterruptLock lock;
        local_len = m_isr_buf_len;
        memcpy(local_buf, m_isr_buf, local_len);
        m_isr_buf_len = 0;
    }

    // In passthrough mode, forward raw data directly to callback
    if (m_passthrough_active && m_passthrough_callback) {
        m_passthrough_callback(local_buf, local_len);
        return;
    }

    // Guard against unbounded growth if data arrives without line terminators
    if (m_rx_buffer.size() + local_len > LoRa::MAX_RX_BUFFER_SIZE) {
        DEBUG_ERROR("LoRaComm: RX buffer overflow (%u bytes) — flushing", (unsigned)(m_rx_buffer.size() + local_len));
        m_rx_buffer.clear();
    }

    // Append to line accumulator and process complete lines
    m_rx_buffer.append(reinterpret_cast<const char*>(local_buf), local_len);
    process_rx_lines();
}

/**
 * Parse complete lines from m_rx_buffer and emit events.
 * Runs in main context only.
 */
void LoRaComm::process_rx_lines()
{
    size_t pos;
    while ((pos = m_rx_buffer.find('\n')) != std::string::npos)
    {
        // Reuse m_line_buffer — assign() within reserved capacity is just memcpy
        m_line_buffer.assign(m_rx_buffer, 0, pos + 1);
        m_rx_buffer.erase(0, pos + 1);

        // Quick empty-line check without creating a copy
        bool is_empty = true;
        for (size_t i = 0; i < m_line_buffer.size(); i++) {
            if (m_line_buffer[i] != '\r' && m_line_buffer[i] != '\n') {
                is_empty = false;
                break;
            }
        }
        if (is_empty)
            continue;

        // parse_rx_line trims in-place and compares against constexpr string_view constants
        RespType resp = parse_rx_line(m_line_buffer);

        DEBUG_TRACE("LoRaComm::rx< %s", m_line_buffer.c_str());

        switch (resp) {
            case RESP_OK:
                notify<LoRaCommEventRespOk>({});
                break;
            case RESP_ERROR:
            case RESP_PARAM_ERROR:
            case RESP_BUSY_ERROR:
            case RESP_NO_NETWORK:
                notify(LoRaCommEventRespError(resp));
                break;
            case RESP_EVT_JOINED:
                notify<LoRaCommEventJoined>({});
                break;
            case RESP_EVT_JOIN_FAILED:
                notify<LoRaCommEventJoinFailed>({});
                break;
            case RESP_EVT_TX_DONE:
            case RESP_EVT_SEND_CONFIRMED_OK:
                notify<LoRaCommEventTxDone>({});
                break;
            case RESP_EVT_SEND_CONFIRMED_FAILED:
                notify(LoRaCommEventRespError(RESP_EVT_SEND_CONFIRMED_FAILED));
                break;
            case RESP_EVT_RX:
                // Port and payload already parsed by parse_rx_line
                notify(LoRaCommEventRxData(m_last_rx_port, m_last_value));
                break;
            case RESP_VALUE:
                // Value stored in m_last_value, OK will follow
                break;
            case RESP_UNKNOWN:
            default:
                break;
        }
    }
}
