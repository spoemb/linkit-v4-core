#include "nrf_usb.hpp"
#include "nrf_drv_clock.h"
#include "app_usbd.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"
#include "pmu.hpp"
#include <cstring>

#define _NRF_USB_QUEUE_PROCESS()  while(app_usbd_event_queue_process()) {}

#define CDC_ACM_COMM_INTERFACE 0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2
#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

// USB CDC read buffer size (single transfer)
#define USB_CDC_READ_SIZE       64

// Static member initialization
bool NrfUSB::m_port_open = false;
char NrfUSB::m_rx_buffer[NRF_USB_RX_BUFFER_SIZE];
volatile size_t NrfUSB::m_rx_write_idx = 0;
volatile size_t NrfUSB::m_rx_read_idx = 0;
volatile bool NrfUSB::m_line_ready = false;

// Temporary buffer for USB CDC reads
static char s_usb_rx_temp[USB_CDC_READ_SIZE];

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

/**
 * @brief CDC_ACM class instance
 * */
 extern "C" {
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250);
}

/**
 * @brief Start a new USB CDC read operation
 */
static void start_usb_read(void)
{
    if (NrfUSB::is_port_open()) {
        app_usbd_cdc_acm_read(&m_app_cdc_acm, s_usb_rx_temp, USB_CDC_READ_SIZE);
    }
}

/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t
 * */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
            NrfUSB::set_port_open(true);
            // Start first read operation
            start_usb_read();
            break;
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            NrfUSB::set_port_open(false);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            // Get the number of bytes received
            size_t rx_size = app_usbd_cdc_acm_rx_size(&m_app_cdc_acm);
            if (rx_size > 0) {
                // Add received data to the ring buffer
                NrfUSB::on_rx_data(s_usb_rx_temp, rx_size);
            }
            // Start next read operation
            start_usb_read();
            break;
        }
        default:
            break;
    }
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-enum"
static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            break;
        case APP_USBD_EVT_DRV_RESUME:
            break;
        case APP_USBD_EVT_STARTED:
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            app_usbd_start();
            break;
        default:
            break;
    }
}
#pragma GCC diagnostic pop


void NrfUSB::init(void)
{
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
		.ev_isr_handler = NULL,
        .ev_state_proc = usbd_user_ev_handler,
		.enable_sof = false,
    };

    // Initialize rx buffer indices
    m_rx_write_idx = 0;
    m_rx_read_idx = 0;
    m_line_ready = false;

    nrf_drv_clock_init();
    nrf_drv_clock_lfclk_request(NULL);
    while(!nrf_drv_clock_lfclk_is_running())
    {
        /* Just waiting */
    }

    app_usbd_serial_num_generate();

	ret = app_usbd_init(&usbd_config);
	APP_ERROR_CHECK(ret);

	app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

	ret = app_usbd_power_events_enable();
	APP_ERROR_CHECK(ret);

    // Poll at init in case USB is already plugged
    for(int ms = 0; ms < 500; ms++)
    {
        _NRF_USB_QUEUE_PROCESS();
        if(m_port_open)
            break;
        PMU::delay_ms(1);
    }
}

int NrfUSB::write(char *ptr, int len)
{
    app_usbd_cdc_acm_write(&m_app_cdc_acm, ptr, len);
    _NRF_USB_QUEUE_PROCESS();
	return len;
}

void NrfUSB::on_rx_data(const char *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        char c = data[i];

        // Calculate next write position
        size_t next_write = (m_rx_write_idx + 1) % NRF_USB_RX_BUFFER_SIZE;

        // Check for buffer overflow (don't overwrite unread data)
        if (next_write == m_rx_read_idx) {
            // Buffer full, discard incoming data
            break;
        }

        // Store character
        m_rx_buffer[m_rx_write_idx] = c;
        m_rx_write_idx = next_write;

        // Check for line terminator
        if (c == '\r' || c == '\n') {
            m_line_ready = true;
        }
    }
}

bool NrfUSB::has_data()
{
    return m_rx_write_idx != m_rx_read_idx;
}

int NrfUSB::read(char *ptr, int max_len)
{
    int count = 0;
    while (m_rx_read_idx != m_rx_write_idx && count < max_len) {
        ptr[count++] = m_rx_buffer[m_rx_read_idx];
        m_rx_read_idx = (m_rx_read_idx + 1) % NRF_USB_RX_BUFFER_SIZE;
    }
    return count;
}

std::string NrfUSB::read_line()
{
    std::string result;

    // Process USB events first
    _NRF_USB_QUEUE_PROCESS();

    if (!m_line_ready) {
        return result;  // Empty string - no complete line available
    }

    // Read characters until we find line terminator or run out of data
    while (m_rx_read_idx != m_rx_write_idx) {
        char c = m_rx_buffer[m_rx_read_idx];
        m_rx_read_idx = (m_rx_read_idx + 1) % NRF_USB_RX_BUFFER_SIZE;

        if (c == '\r' || c == '\n') {
            // Skip any following \n or \r (handle both \r\n and \n\r)
            if (m_rx_read_idx != m_rx_write_idx) {
                char next = m_rx_buffer[m_rx_read_idx];
                if ((next == '\r' || next == '\n') && next != c) {
                    m_rx_read_idx = (m_rx_read_idx + 1) % NRF_USB_RX_BUFFER_SIZE;
                }
            }
            break;
        }

        result += c;
    }

    // Check if there's another complete line in the buffer
    m_line_ready = false;
    size_t idx = m_rx_read_idx;
    while (idx != m_rx_write_idx) {
        char c = m_rx_buffer[idx];
        if (c == '\r' || c == '\n') {
            m_line_ready = true;
            break;
        }
        idx = (idx + 1) % NRF_USB_RX_BUFFER_SIZE;
    }

    return result;
}

void NrfUSB::process(void)
{
    _NRF_USB_QUEUE_PROCESS();
}

void NrfUSB::set_port_open(bool is_open)
{
    m_port_open = is_open;
    if (!is_open) {
        // Reset buffer when port closes
        m_rx_write_idx = 0;
        m_rx_read_idx = 0;
        m_line_ready = false;
    }
}
