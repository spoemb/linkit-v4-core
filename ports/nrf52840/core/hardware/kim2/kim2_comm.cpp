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
        // obj->handle_error((unsigned int)p_evt->data.errorsrc);
        DEBUG_INFO("KIM2Device::event_handler ERROR %d", p_evt->data.errorsrc);
    } else if (p_evt->type == NRF_LIBUARTE_ASYNC_EVT_ALLOC_ERROR) {
        // Stop the receiver to prevent further errors
        // obj->handle_error(0x100);
        DEBUG_INFO("KIM2Device::event_handler ALLOC ERROR");
    } else if (p_evt->type == NRF_LIBUARTE_ASYNC_EVT_OVERRUN_ERROR) {
        // Stop the receiver to prevent further errors
        // obj->handle_error(0x200);
        DEBUG_INFO("KIM2Device::event_handler OVERRUN ERROR");
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
}

void KIM2Comm::deinit(void)
{
    nrf_libuarte_async_stop_rx(BSP::UARTAsync_Inits[m_uart_instance].uart);
    nrf_libuarte_async_uninit(BSP::UARTAsync_Inits[m_uart_instance].uart);
    m_is_init = false;
}

bool KIM2Comm::send(ATCmd cmd)
{
    if(m_is_send_busy)
    {
        DEBUG_ERROR("KIM2Comm::send: already busy sending");
        return false;
    }
    m_is_send_busy = true;
    return send_at_cmd(cmd, NULL, 0);
}

struct ATCmd_list {
    ATCmd type;
    std::string value;
    bool expected_params;
};

const std::list<ATCmd_list> cmd_list = {
    {AT_PING, "AT+PING=?\r\n", false},
    {AT_ID, "AT+ID=?\r\n", false},
    {AT_ADDR, "AT+ADDR=?\r\n", false}
};

bool KIM2Comm::send_at_cmd(ATCmd cmd, char * params, uint8_t params_size)
{
    bool at_cmd_valid = false;
    ret_code_t ret;

    auto at_cmd = std::find_if(cmd_list.begin(), cmd_list.end(), [cmd](ATCmd_list element)
    {
        return element.type == cmd;
    });

    if (at_cmd != cmd_list.end())
    {
        at_cmd_valid = true;
        if (at_cmd->expected_params == false)
        {
            memcpy(m_tx_buffer, at_cmd->value.c_str(), at_cmd->value.length());
        }
        
        ret = nrf_libuarte_async_tx(BSP::UARTAsync_Inits[m_uart_instance].uart, m_tx_buffer, 11);
        
        if(ret != NRF_SUCCESS)
        {
            m_is_send_busy = false;
            DEBUG_ERROR("KIM2Comm::send_at_cmd: failed to send ret=%08x", (unsigned int)ret);
        }
    }

    return at_cmd_valid && (ret == NRF_SUCCESS);
}

RespType KIM2Comm::parse_rx_message(unsigned char * buffer, uint8_t size, uint8_t * used_length)
{
    RespType rx_type = RX_UNKNOWN;
    std::string message(buffer, buffer+size);
    size_t position;

    // Find SYNC char '+'
    position = message.find(SYNC_CHAR);
    if (position == std::string::npos) {
        *used_length = size;
        return RX_UNKNOWN;
    }

    // SYNC found : get msg type and data
    // -- +OK
    if (message.compare(position, OK_RESPONSE.size(), OK_RESPONSE) == 0)
    {
        rx_type = RX_OK;
    }
    // -- +ID=
    else if ((message.compare(position, ID_RESPONSE.size(), ID_RESPONSE) == 0)
            && (message.find(END_CHAR_1) == position + ID_RESPONSE.size() + ID_SIZE)
            && (message.find(END_CHAR_2) == position + ID_RESPONSE.size() + ID_SIZE + 1))
    {
        std::memcpy(m_ascii_id, message.c_str() + ID_RESPONSE.size(), ID_SIZE);
        m_ascii_id[ID_SIZE] = 0; //terminal character
        rx_type = RX_CONFIG;
    }
    // -- +ADDR=
    else if ((message.compare(position, ADDR_RESPONSE.size(), ADDR_RESPONSE) == 0)
            && (message.find(END_CHAR_1) == position + ADDR_RESPONSE.size() + ADDR_SIZE)
            && (message.find(END_CHAR_2) == position + ADDR_RESPONSE.size() + ADDR_SIZE + 1))
    {
        std::memcpy(m_ascii_addr, message.c_str() + ADDR_RESPONSE.size(), ADDR_SIZE);
        m_ascii_addr[ADDR_SIZE] = 0; //terminal character
        rx_type = RX_CONFIG;
    }
    
    // Find end of message ('\n')
    position = message.find(END_CHAR_2);
    if(position == std::string::npos)
    {
        *used_length = size;
    }
    else
    {
        *used_length = position + 1;
    }

    return rx_type;
}

void KIM2Comm::handle_tx_done(void)
{
    m_is_send_busy = false;
    notify<KIM2CommEventTxDone>({});
}

void KIM2Comm::handle_rx_buffer(uint8_t * buffer, uint8_t length)
{   
    std::memcpy(m_rx_buffer, buffer, length);
    
    unsigned char * current_buffer = m_rx_buffer;
    uint8_t remaining_length = length;
    uint8_t current_length = 0;
    RespType msg = RX_UNKNOWN;

    do {
        current_buffer += current_length;
        current_length = 0;
        msg = parse_rx_message(current_buffer, remaining_length, &current_length);
        if (msg == RX_OK)
        {
            notify<KIM2CommEventOk>({});
        }
        remaining_length -= current_length;
    } while(remaining_length);

    nrf_libuarte_async_rx_free(BSP::UARTAsync_Inits[1].uart, buffer, length);
}
