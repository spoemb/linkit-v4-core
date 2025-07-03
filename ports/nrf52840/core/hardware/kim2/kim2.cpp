#include <string.h>

#include "kim2.hpp"
#include "kim2_comm.hpp"
#include "debug.hpp"
#include "error.hpp"

#include "nrf_delay.h"

using namespace KIM2;

KIM2Device::KIM2Device()
{
    //INIT variables, clear buffer
    //GPIOPins::clear(TOBEDEFINED); (SAT_EN, WAKEUP, INT)
    m_cmd_is_ok = false;
  
    m_kim2_comm.init();
    m_kim2_comm.subscribe(*this);
    detect_device();
}

KIM2Device::~KIM2Device()
{
    m_kim2_comm.deinit();
}

void KIM2Device::detect_device(void)
{
    DEBUG_TRACE("KIM2Device::detect_device");

    // Ping KIM2
    m_kim2_comm.send(AT_PING);
    for(uint16_t time = 0; !m_cmd_is_ok && time <= 1000; time += 10)
    {
        nrf_delay_ms(10);
    }
    if (!m_cmd_is_ok) {
        throw ErrorCode::RESOURCE_NOT_AVAILABLE;
    }
    m_cmd_is_ok = false;

    // Get KIM2 ID
    m_kim2_comm.send(AT_ID);
    while(m_cmd_is_ok == false)
    {
        nrf_delay_ms(10);
    }
    DEBUG_TRACE("KIM2Device::detect_device ID:%s", m_kim2_comm.m_ascii_id);
    m_cmd_is_ok = false;

    // Get KIM2 ADDR
    m_kim2_comm.send(AT_ADDR);
    while(m_cmd_is_ok == false)
    {
        nrf_delay_ms(10);
    }
    DEBUG_TRACE("KIM2Device::detect_device ADDR:%s", m_kim2_comm.m_ascii_addr);
    m_cmd_is_ok = false;
}

void KIM2Device::react (const KIM2CommEventOk&) {
    m_cmd_is_ok = true;
}


