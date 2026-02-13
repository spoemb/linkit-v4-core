#include "nrf_debug_uart.hpp"
#include "nrfx_uarte.h"
#include "nrf_gpio.h"
#include <cstring>

// Use UARTE instance 1 for debug (instance 0 is used by GPS)
static const nrfx_uarte_t m_debug_uarte = NRFX_UARTE_INSTANCE(1);

bool NrfDebugUart::m_is_init = false;

void NrfDebugUart::init(uint32_t tx_pin)
{
    if (m_is_init) return;

    nrfx_uarte_config_t config;
    memset(&config, 0, sizeof(config));
    config.pseltxd = tx_pin;
    config.pselrxd = NRF_UARTE_PSEL_DISCONNECTED;
    config.pselcts = NRF_UARTE_PSEL_DISCONNECTED;
    config.pselrts = NRF_UARTE_PSEL_DISCONNECTED;
    config.p_context = NULL;
    config.baudrate = NRF_UARTE_BAUDRATE_460800;
    config.interrupt_priority = NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY;
    config.hwfc = NRF_UARTE_HWFC_DISABLED;
    config.parity = NRF_UARTE_PARITY_EXCLUDED;

    nrfx_err_t err = nrfx_uarte_init(&m_debug_uarte, &config, NULL);
    if (err == NRFX_SUCCESS) {
        m_is_init = true;
    }
}

void NrfDebugUart::write(const char* data, int len)
{
    if (!m_is_init || len <= 0) return;

    // nrfx_uarte_tx is blocking when no event handler is provided
    nrfx_uarte_tx(&m_debug_uarte, (const uint8_t*)data, len);
}

bool NrfDebugUart::is_init()
{
    return m_is_init;
}
