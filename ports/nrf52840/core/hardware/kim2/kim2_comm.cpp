#include "kim2_comm.hpp"
#include "nrf_libuarte_async.h"
#include "bsp.hpp"
#include "error.hpp"
#include "debug.hpp"
#include <list>

using namespace KIM2;

static void kim2_nrf_libuarte_async_evt_handler(void * context, nrf_libuarte_async_evt_t * p_evt)
{
    KIM2Comm *obj = (KIM2Comm *)context;
    if (p_evt->type == NRF_LIBUARTE_ASYNC_EVT_TX_DONE) {
        obj->handle_tx_done();
    } else if (p_evt->type == NRF_LIBUARTE_ASYNC_EVT_RX_DATA) {
        obj->handle_rx_buffer(p_evt->data.rxtx.p_data, (uint8_t)p_evt->data.rxtx.length);
    } else if (p_evt->type == NRF_LIBUARTE_ASYNC_EVT_ERROR) {
        // Stop the receiver to prevent further errors
        obj->handle_error((unsigned int)p_evt->data.errorsrc);
        // DEBUG_INFO("KIM2Device::event_handler ERROR %d", p_evt->data.errorsrc);
    } else if (p_evt->type == NRF_LIBUARTE_ASYNC_EVT_ALLOC_ERROR) {
        // Stop the receiver to prevent further errors
        obj->handle_error(0x100);
        // DEBUG_INFO("KIM2Device::event_handler ALLOC ERROR");
    } else if (p_evt->type == NRF_LIBUARTE_ASYNC_EVT_OVERRUN_ERROR) {
        // Stop the receiver to prevent further errors
        obj->handle_error(0x200);
        // DEBUG_INFO("KIM2Device::event_handler OVERRUN ERROR");
    }
}

KIM2Comm::KIM2Comm(unsigned int instance) :
    m_uart_instance(instance),
    m_is_init(false),
    m_is_send_busy(false)
{
}

void KIM2Comm::init(void) {
    if (m_is_init) return;

    if (nrf_libuarte_async_init(
            BSP::UARTAsync_Inits[m_uart_instance].uart,
            &BSP::UARTAsync_Inits[m_uart_instance].config,
            kim2_nrf_libuarte_async_evt_handler,
            (void *)this) != NRF_SUCCESS) {
        DEBUG_ERROR("KIM2Comm::init: failed to initialize libuarte library");
        throw ErrorCode::RESOURCE_NOT_AVAILABLE;
    };
    nrf_libuarte_async_start_rx(BSP::UARTAsync_Inits[m_uart_instance].uart);
    m_is_init = true;
    m_is_rx_started = true;
}

void KIM2Comm::deinit(void)
{
    if(m_is_init) {
        nrf_libuarte_async_uninit(BSP::UARTAsync_Inits[m_uart_instance].uart);
        m_is_init = false;

        if(m_uart_instance == 1)
        {
            /*Fix bug in Nordic SDK for UARTE1 - TODO : move in lower layers
             * HF clk and DMA bus not closed on calling uart uninit
             * => need to force these register values
             */
            *(volatile uint32_t *)0x40028FFC = 0;
            *(volatile uint32_t *)0x40028FFC;
            *(volatile uint32_t *)0x40028FFC = 1;
        }
    }
}

bool KIM2Comm::send(ATCmd cmd, const std::optional<std::string>& params)
{
    if(m_is_send_busy)
    {
        DEBUG_ERROR("KIM2Comm::send: already busy sending");
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

const std::list<ATCmd_list> cmd_list = {
    {AT_PING, "AT+PING=?\r\n", false},
    {AT_GET_ID, "AT+ID=?\r\n", false},
    {AT_GET_ADDR, "AT+ADDR=?\r\n", false},
    {AT_SET_RCONF, "AT+RCONF=", true},
    {AT_SAVE_RCONF, "AT+SAVE_RCONF\r\n", false},
    {AT_SET_KMAC_BASIC, "AT+KMAC=1\r\n", false}, // TODO : add BLIND mode support
    {AT_SET_LPM, "AT+LPM=", true},
    {AT_TX, "AT+TX=", true}
};

bool KIM2Comm::send_at_cmd(ATCmd cmd, const std::optional<std::string>& params)
{
    bool at_cmd_valid = false;
    ret_code_t ret;

    if(!m_is_rx_started)
    {
        nrf_libuarte_async_start_rx(BSP::UARTAsync_Inits[m_uart_instance].uart);
        m_is_rx_started = true;
    }

    auto at_cmd = std::find_if(cmd_list.begin(), cmd_list.end(), [cmd](ATCmd_list element)
    {
        return element.type == cmd;
    });

    if (at_cmd != cmd_list.end())
    {
        at_cmd_valid = true;
        m_tx_buffer = at_cmd->value;

        if (at_cmd->expected_params && params.has_value())
        {
            m_tx_buffer.append(params.value());
            m_tx_buffer += "\r\n";
        }

        ret = nrf_libuarte_async_tx(BSP::UARTAsync_Inits[m_uart_instance].uart, reinterpret_cast<uint8_t*>(m_tx_buffer.data()), m_tx_buffer.length());
        
        if(ret != NRF_SUCCESS)
        {
            m_is_send_busy = false;
            DEBUG_ERROR("KIM2Comm::send_at_cmd: failed to send ret=%08x", (unsigned int)ret);
        }
    }

    return at_cmd_valid && (ret == NRF_SUCCESS);
}

RespType KIM2Comm::parse_rx_message(const std::string& buffer, uint8_t start_index, uint8_t * used_length)
{
    RespType resp_type = RESP_UNKNOWN;
    std::string message = buffer.substr(start_index, std::string::npos);
    size_t position;

    // Find SYNC char '+'
    position = message.find(SYNC_CHAR);
    if (position == std::string::npos) {
        *used_length = message.size();
        return RESP_UNKNOWN;
    }

    // SYNC found : get msg type and data
    // -- +OK\r\n
    if (message.compare(position, OK_RESPONSE.size(), OK_RESPONSE) == 0)
    {
        resp_type = RESP_OK;
    }
    // -- +ID=
    else if ((message.compare(position, ID_RESPONSE.size(), ID_RESPONSE) == 0)
            && (message.find(END_CHAR_1) == position + ID_RESPONSE.size() + ID_SIZE)
            && (message.find(END_CHAR_2) == position + ID_RESPONSE.size() + ID_SIZE + 1))
    {
        // Warn : can the ID be shorter than 6 characters ?
        char ascii_id[KIM2::ID_SIZE + 1] = {0};
        std::memcpy(ascii_id, message.c_str() + ID_RESPONSE.size(), ID_SIZE);
        ascii_id[ID_SIZE] = 0;
        char *end = nullptr;
        long id_val = strtol(ascii_id, &end, 10);
        m_kineis_id = (end != ascii_id && *end == '\0') ? (unsigned int)id_val : 0;
        resp_type = RESP_CONFIG;
    }
    // -- +ADDR=
    else if ((message.compare(position, ADDR_RESPONSE.size(), ADDR_RESPONSE) == 0)
            && (message.find(END_CHAR_1) == position + ADDR_RESPONSE.size() + ADDR_SIZE)
            && (message.find(END_CHAR_2) == position + ADDR_RESPONSE.size() + ADDR_SIZE + 1))
    {
        char ascii_addr[KIM2::ADDR_SIZE + 1] = {0};
        std::memcpy(ascii_addr, message.c_str() + ADDR_RESPONSE.size(), ADDR_SIZE);
        ascii_addr[ADDR_SIZE] = 0;
        char *end_addr = nullptr;
        unsigned long addr_val = strtoul(ascii_addr, &end_addr, 16);
        m_hex_addr = (end_addr != ascii_addr) ? (unsigned int)addr_val : 0;
        resp_type = RESP_CONFIG;
    }
    // -- +ERROR=
    else if ((message.compare(position, ERR_RESPONSE.size(), ERR_RESPONSE) == 0))
    {
        resp_type = RESP_ERROR;
    }
    // -- +TX=
    else if ((message.compare(position, TX_RESPONSE.size(), TX_RESPONSE) == 0)
            && (message.find(END_CHAR_1)) && (message.find(END_CHAR_2)))
    {
        std::string resp = message.substr(position + TX_RESPONSE.size()); // extract response after "+TX="
        m_tx_status = std::stoi(resp.substr(0, resp.find(','))); // extract first response field (error code) and convert into integer
        resp_type = RESP_TX_STATUS;
    }
    
    // Find end of message ('\n')
    position = message.find(END_CHAR_2);
    if(position == std::string::npos)
    {
        *used_length = message.size();
    }
    else
    {
        *used_length = position + 1;
    }

    return resp_type;
}

void KIM2Comm::handle_tx_done(void)
{
    m_is_send_busy = false;
}

void KIM2Comm::handle_rx_buffer(uint8_t * buffer, uint8_t length)
{
    // Guard against unbounded buffer growth from corrupted data
    if (length > 128) length = 128;
    m_rx_buffer.assign(reinterpret_cast<const char*>(buffer), length);
    
    // DEBUG_TRACE("KIM2Comm::handle_rx_buffer %s", buffer);

    uint8_t parsing_index = 0;
    uint8_t current_length = 0;
    RespType msg = RESP_UNKNOWN;

    while(parsing_index < m_rx_buffer.size())
    {
        msg = parse_rx_message(m_rx_buffer, parsing_index, &current_length);
        if (msg == RESP_OK)
        {
            notify<KIM2CommEventRespOk>({});
        }
        else if (msg == RESP_TX_STATUS)
        {
            notify<KIM2CommEventTxDone>({});
        }
        else if (msg == RESP_ERROR)
        {
            DEBUG_INFO("KIM2Comm::handle_rx_buffer %s", m_rx_buffer.substr(parsing_index, current_length).data());
            notify<KIM2CommEventRespError>({});
        }
        parsing_index += current_length;
    }

    nrf_libuarte_async_rx_free(BSP::UARTAsync_Inits[m_uart_instance].uart, buffer, length);
}

void KIM2Comm::handle_error(unsigned int error_type) {
	nrf_libuarte_async_stop_rx(BSP::UARTAsync_Inits[m_uart_instance].uart);
    m_is_rx_started = false;
	notify(KIM2CommEventUartError(error_type));
}