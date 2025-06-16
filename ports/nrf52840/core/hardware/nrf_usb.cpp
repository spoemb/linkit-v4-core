#include "nrf_usb.hpp"
#include "nrf_drv_clock.h"
#include "app_usbd.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#define _NRF_USB_QUEUE_PROCESS()  while(app_usbd_event_queue_process()) {}

#define CDC_ACM_COMM_INTERFACE 0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2
#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

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
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t
 * */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    // app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
            NrfUSB::m_port_open = true;
            /*Setup first transfer*/
            // ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
            //                                        m_rx_buffer,
            //                                        READ_SIZE);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            NrfUSB::m_port_open = false;
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
            break;
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
            // NRF_LOG_INFO("USB power detected");
            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            // NRF_LOG_INFO("USB power removed");
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            // NRF_LOG_INFO("USB ready");
            app_usbd_start();
            break;
        default:
            break;
    }
}
#pragma GCC diagnostic pop

bool NrfUSB::m_port_open = false;

void NrfUSB::init(void)
{	
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
		.ev_isr_handler = NULL,
        .ev_state_proc = usbd_user_ev_handler,
		.enable_sof = false,
    };

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

    while(!NrfUSB::m_port_open)
        _NRF_USB_QUEUE_PROCESS(); //TODO : remove this blocking loop if no USB plugged !
}

int NrfUSB::write(char *ptr, int len)
{
    app_usbd_cdc_acm_write(&m_app_cdc_acm, ptr, len);
    _NRF_USB_QUEUE_PROCESS();
	return len;
}

bool NrfUSB::process(void)
{
    return app_usbd_event_queue_process();
}