#include "lora_rak3172_comm.hpp"
#include "nrf_libuarte_async.h"
#include "bsp.hpp"
#include "error.hpp"
#include "debug.hpp"
#include <list>
#include <cstring>

using namespace LoRa;

static void lora_nrf_libuarte_async_evt_handler(void * context, nrf_libuarte_async_evt_t * p_evt)
{
    LoRaComm *obj = (LoRaComm *)context;
    if (p_evt->type == NRF_LIBUARTE_ASYNC_EVT_TX_DONE) {
        obj->handle_tx_done();
    } else if (p_evt->type == NRF_LIBUARTE_ASYNC_EVT_RX_DATA) {
        obj->handle_rx_buffer(p_evt->data.rxtx.p_data, (uint8_t)p_evt->data.rxtx.length);
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
    m_uart_config{}
{
}

void LoRaComm::init(void) {
    if (m_is_init) return;

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

    if (at_cmd->expected_params && params.has_value())
    {
        m_tx_buffer.append(params.value());
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
RespType LoRaComm::parse_rx_line(const std::string& line)
{
    // Strip trailing CR/LF
    std::string trimmed = line;
    while (!trimmed.empty() && (trimmed.back() == '\r' || trimmed.back() == '\n'))
        trimmed.pop_back();

    if (trimmed.empty())
        return RESP_UNKNOWN;

    // Check for OK
    if (trimmed == OK_RESP)
        return RESP_OK;

    // Check for errors
    if (trimmed == ERR_RESP)
        return RESP_ERROR;
    if (trimmed == PARAM_ERR_RESP)
        return RESP_PARAM_ERROR;
    if (trimmed == BUSY_ERR_RESP)
        return RESP_BUSY_ERROR;
    if (trimmed == NO_NET_RESP)
        return RESP_NO_NETWORK;

    // Check for async events
    if (trimmed.compare(0, EVT_PREFIX.size(), EVT_PREFIX) == 0)
    {
        if (trimmed == EVT_JOINED)
            return RESP_EVT_JOINED;
        if (trimmed == EVT_JOIN_FAILED)
            return RESP_EVT_JOIN_FAILED;
        if (trimmed == EVT_TX_DONE)
            return RESP_EVT_TX_DONE;
        if (trimmed == EVT_SEND_CONF_OK)
            return RESP_EVT_SEND_CONFIRMED_OK;
        if (trimmed == EVT_SEND_CONF_FAIL)
            return RESP_EVT_SEND_CONFIRMED_FAILED;

        // Check for RX data: +EVT:RX_1:<rssi>:<snr>:UNICAST:<port>:<payload>
        if (trimmed.compare(0, EVT_RX_PREFIX.size(), EVT_RX_PREFIX) == 0)
        {
            // Parse RX event - find port and payload
            // Format: +EVT:RX_X:<rssi>:<snr>:UNICAST:<port>:<payload>
            // We look for the last two colon-separated fields
            size_t last_colon = trimmed.rfind(':');
            if (last_colon != std::string::npos && last_colon > 0) {
                std::string payload = trimmed.substr(last_colon + 1);
                size_t prev_colon = trimmed.rfind(':', last_colon - 1);
                if (prev_colon != std::string::npos) {
                    m_last_value = payload;
                    DEBUG_TRACE("LoRaComm: RX payload=%s", payload.c_str());
                }
            }
            return RESP_EVT_RX;
        }

        // Unknown event - log it
        DEBUG_TRACE("LoRaComm: unknown EVT: %s", trimmed.c_str());
        return RESP_UNKNOWN;
    }

    // Otherwise it's a value response (e.g., DEVEUI, band number, etc.)
    m_last_value = trimmed;
    return RESP_VALUE;
}

void LoRaComm::handle_tx_done(void)
{
    m_is_send_busy = false;
}

void LoRaComm::handle_rx_buffer(uint8_t * buffer, uint8_t length)
{
    // Append received data to RX buffer
    m_rx_buffer.append(reinterpret_cast<const char*>(buffer), length);

    // Process complete lines (terminated by \n)
    size_t pos;
    while ((pos = m_rx_buffer.find('\n')) != std::string::npos)
    {
        std::string line = m_rx_buffer.substr(0, pos + 1);
        m_rx_buffer.erase(0, pos + 1);

        // Skip empty lines (blank line between value and OK)
        std::string trimmed = line;
        while (!trimmed.empty() && (trimmed.back() == '\r' || trimmed.back() == '\n'))
            trimmed.pop_back();
        if (trimmed.empty())
            continue;

        DEBUG_TRACE("LoRaComm::rx< %s", trimmed.c_str());

        RespType resp = parse_rx_line(line);

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
                // RX data event - m_last_value contains payload
                notify(LoRaCommEventRxData(0, m_last_value));
                break;
            case RESP_VALUE:
                // Value stored in m_last_value, OK will follow
                break;
            case RESP_UNKNOWN:
            default:
                break;
        }
    }

    nrf_libuarte_async_rx_free(BSP::UARTAsync_Inits[m_uart_instance].uart, buffer, length);
}

void LoRaComm::handle_error(unsigned int error_type) {
    nrf_libuarte_async_stop_rx(BSP::UARTAsync_Inits[m_uart_instance].uart);
    m_is_rx_started = false;
    notify(LoRaCommEventUartError(error_type));
}
