#include "kim2.hpp"
#include "kim2_comm.hpp"
#include "bsp.hpp"
#include "gpio.hpp"
#include "pmu.hpp"
#include "debug.hpp"
#include "error.hpp"
#include "bitpack.hpp"
#include "binascii.hpp"
#include "config_store.hpp"
#include <stdint.h>
#include <string>

using namespace KIM2;

#define LDK_MAX_LENGTH_BITS      (16*8)
#define LDA2_MAX_LENGTH_BITS     (24*8)
#define VLDA4_MAX_LENGTH_BITS    (3*8)
#define KIM2_COMM_OK             (0)

#define KIM2_STATE_CHANGE(x, y)                     \
	do {                                             \
		DEBUG_TRACE("KIM2::KIM2_STATE_CHANGE: " #x " -> " #y ); \
		m_state = y;                                 \
		state_ ## x ##_exit();                       \
		state_ ## y ##_enter();                      \
		run_state_machine();						 \
	} while (0)

#define KIM2_STATE_EQUAL(x) \
	(m_state == x)

#define KIM2_STATE_CALL(x) \
	do {                    \
		state_ ## x();      \
	} while (0)

extern Scheduler *system_scheduler;
extern ConfigurationStore *configuration_store;

KIM2Device::KIM2Device()
{
    m_tx_buffer.clear();
    m_packet_buffer.clear();
    m_state = KIM2ManagerState::power_off;
    KIM2_STATE_CHANGE(power_off, power_on);
}

KIM2Device::~KIM2Device()
{
    power_off_immediate();
}

void KIM2Device::send(const KineisModulation mode, const KineisPacket& user_payload, const unsigned int payload_length)
{
    KineisPacket packet;
    uint16_t total_bits;
    uint16_t modulo;
    uint16_t stuffing_bits = 0;

    switch (mode)
    {
        case KineisModulation::LDK:
            if (payload_length > LDK_MAX_LENGTH_BITS)
            {
                DEBUG_ERROR("KIM2Device::send: LDK payload is too long : %d bits - MAX is %d bits",
                    payload_length, LDK_MAX_LENGTH_BITS);
                return;
            }
            stuffing_bits = LDK_MAX_LENGTH_BITS - payload_length;
            break;

        case KineisModulation::LDA2:
            if (payload_length > LDA2_MAX_LENGTH_BITS)
            {
                DEBUG_ERROR("KIM2Device::send: LDA2 payload is too long : %d bits - MAX is %d bits",
                    payload_length, LDA2_MAX_LENGTH_BITS);
                return;
            }
            // LDA2 payload size can be any multiple of 4 bytes
            modulo = payload_length % 32;
            stuffing_bits = modulo ? (32 - modulo) : 0;
            break;

        case KineisModulation::VLDA4:
            if (payload_length > VLDA4_MAX_LENGTH_BITS)
            {
                DEBUG_ERROR("KIM2Device::send: VLDA4 payload is too long : %d bits - MAX is %d bits",
                    payload_length,
                    VLDA4_MAX_LENGTH_BITS);
                return;
            }
            stuffing_bits = VLDA4_MAX_LENGTH_BITS - payload_length;
            break;

        default:
            DEBUG_ERROR("KIM2Device::send: Unknown modulation type");
            return;
    }
    DEBUG_TRACE("KIM2Device::send: adding %u stuffing bits for alignment", stuffing_bits);

    total_bits = payload_length + stuffing_bits;
    packet.assign(total_bits/8, 0);
    packet.replace(0, user_payload.size(), user_payload);

    // Add any stuffing bits
    unsigned int payload_bits_remaining = stuffing_bits;
    unsigned int op_offset = payload_length;
    while (payload_bits_remaining) {
        unsigned int bits = std::min(8U, payload_bits_remaining);
        payload_bits_remaining -= bits;
        PACK_BITS(0, packet, op_offset, bits);
    }

    // Setup TX mode
    DEBUG_TRACE("KIM2Device::send_packet: data[%u]=%s", total_bits, Binascii::hexlify(packet).c_str());
    m_packet_buffer = Binascii::hexlify(packet).c_str();
    m_tx_mode = mode; //TODO : use m_tx_mode

    // Request power on (if not already running)
    start_device();
}

void KIM2Device::stop_send() {
    DEBUG_TRACE("KIM2Device::stop_send");
    m_packet_buffer.clear();
    m_tx_buffer.clear();
}

void KIM2Device::react (const KIM2CommEventRespOk&) {
    m_cmd_is_ok = true;
}

void KIM2Device::react(const KIM2CommEventTxDone&) {
    m_tx_done = true;
}

void KIM2Device::react(const KIM2CommEventRespError&) {
    m_is_error = true;
}

void KIM2Device::react(const KIM2CommEventUartError& err) {
    m_is_error = true;
    system_scheduler->post_task_prio([this, err]() {
        DEBUG_INFO("KIM2CommEventUartError: type=%02x", err.error_type);
    }, "Debug");
}

void KIM2Device::initiate_timeout(unsigned int timeout_ms) {
	cancel_timeout();
	m_timeout.handle = system_scheduler->post_task_prio([this]() {
		on_timeout();
	}, "Timeout", Scheduler::DEFAULT_PRIORITY, timeout_ms);
}

void KIM2Device::on_timeout() {
    DEBUG_ERROR("KIM2Device::on_timeout");
    m_is_error = true;
}

void KIM2Device::cancel_timeout() {
	system_scheduler->cancel_task(m_timeout.handle);
}


void KIM2Device::start_device()
{
    // Device is already powered
    if (m_state != KIM2ManagerState::power_off) {
        m_stopping = false;  // Clear any pending stopping flag
        DEBUG_TRACE("KIM2::start: already running in state=%u", (unsigned int)m_state);
        run_state_machine(0);
        return;
    }

    // Set top-level state variables
    m_stopping = false;
    KIM2_STATE_CHANGE(power_off, power_on);
}

void KIM2Device::power_off_immediate(void)
{
    DEBUG_TRACE("KIM2Device::power_off_immediate");

    // We force a power off by cancelling the state machine and
    // forcing a stopped state
    if (!KIM2_STATE_EQUAL(power_off)) {
        system_scheduler->cancel_task(m_task);
        KIM2_STATE_CHANGE(idle, power_off);
    }
}

bool KIM2Device::send_AT(ATCmd cmd, const std::optional<std::string>& params)
{
    uint16_t timeout_ms = 1000; // CHANGE!?

    m_cmd_is_ok = false;
    m_is_error = false;

    m_kim2_comm.send(cmd, params);

    while(!m_cmd_is_ok && timeout_ms != 0)
    {
        PMU::delay_ms(1);
        timeout_ms --;
    }

    if(timeout_ms == 0)
        DEBUG_TRACE("KIM2Device::send_AT +OK not received");

    return !m_cmd_is_ok; // 0=OK - 1=error
}

void KIM2Device::state_machine(void)
{
	switch (m_state) {
	case KIM2ManagerState::power_off:
		KIM2_STATE_CALL(power_off);
		break;
	case KIM2ManagerState::power_on:
		KIM2_STATE_CALL(power_on);
		break;
	case KIM2ManagerState::init:
		KIM2_STATE_CALL(init);
		break;
	case KIM2ManagerState::idle:
		KIM2_STATE_CALL(idle);
		break;
	case KIM2ManagerState::transmit:
		KIM2_STATE_CALL(transmit);
		break;
	case KIM2ManagerState::error:
		KIM2_STATE_CALL(error);
		break;
	default:
		break;
	}
}

void KIM2Device::run_state_machine(uint16_t delay_ms)
{
    system_scheduler->cancel_task(m_task);
    m_task = system_scheduler->post_task_prio([this]() {
        state_machine();
    }, "KIM2StateMachine", Scheduler::DEFAULT_PRIORITY, delay_ms);
}

void KIM2Device::state_power_off_enter()
{
    DEBUG_INFO("KIM2Device::state_power_off_enter");
    m_kim2_comm.deinit();
    GPIOPins::clear(SAT_EXTWAKEUP);
    GPIOPins::clear(SAT_RESET);
    GPIOPins::clear(SAT_PWR_EN);
    m_tx_buffer.clear();
    m_packet_buffer.clear();

    //notify(ArticEventPowerOff({}))
}

void KIM2Device::state_power_off()
{
    ;
}

void KIM2Device::state_power_off_exit()
{
    ;
}

void KIM2Device::state_power_on_enter()
{
    GPIOPins::set(SAT_PWR_EN);
    GPIOPins::set(SAT_RESET);
    GPIOPins::set(SAT_EXTWAKEUP);
    m_kim2_comm.init();
    m_kim2_comm.subscribe(*this);
    if(m_packet_buffer.length() > 0)
        PMU::delay_ms(500); // Add delay if KIM2 was just powered ON for transmission
}

void KIM2Device::state_power_on()
{
    DEBUG_INFO("KIM2Device::state_power_on");
    if(send_AT(AT_PING) == KIM2_COMM_OK)
    {
        KIM2_STATE_CHANGE(power_on, init);
    }
    else
    {
        KIM2_STATE_CHANGE(power_on, error);
    }
}

void KIM2Device::state_power_on_exit()
{
    ;
}

void KIM2Device::state_init_enter()
{
    ;
}

void KIM2Device::state_init()
{
    if(m_kim2_comm.m_kineis_id == 0 && m_kim2_comm.m_hex_addr == 0)
    {
        bool at_error = send_AT(AT_GET_ID);
        if(!at_error)
        {
            DEBUG_TRACE("KIM2Device::state_init ID:%d", m_kim2_comm.m_kineis_id);
            configuration_store->write_param(ParamID::ARGOS_DECID, m_kim2_comm.m_kineis_id);

            at_error = send_AT(AT_GET_ADDR);
            if (!at_error)
            {
                DEBUG_TRACE("KIM2Device::state_init ADDR:%x", m_kim2_comm.m_hex_addr);
                configuration_store->write_param(ParamID::ARGOS_HEXID, m_kim2_comm.m_hex_addr);
            }
        }

        if(at_error)
        {
            DEBUG_ERROR("KIM2Device::state_init : can not read ID or ADDR");
            KIM2_STATE_CHANGE(init, error);
            return;
        }
    }

    // Read RCONF or similar from configuration_store ?
    std::string rconf = "03921fb104b92859209b18abd009de96"; // ESS4 - LDK - 27dBm
    bool rconf_error = send_AT(AT_SET_RCONF, rconf);
    bool kmac_error = send_AT(AT_SET_KMAC_BASIC);

    if(!rconf_error && !kmac_error)
    {
        DEBUG_TRACE("KIM2Device::state_init RCONF and KMAC set");
        KIM2_STATE_CHANGE(init, idle);
    }
    else
    {
        DEBUG_ERROR("KIM2Device::state_init : can not set RCONF=%d or KMAC=%d", rconf_error, kmac_error);
        KIM2_STATE_CHANGE(init, error);
    }

    // std::string lpm_standby = "0x01";
    // if(!(send_AT(AT_SET_LPM, lpm_standby)))
    // {
    //     DEBUG_TRACE("KIM2Device::state_init LPM=standby set");
    //     KIM2_STATE_CHANGE(init, idle);
    // }
    // else
    // {
    //     DEBUG_ERROR("KIM2Device::state_init : can not LPM");
    //     KIM2_STATE_CHANGE(init, error);
    // }
}

void KIM2Device::state_init_exit()
{
    ;
}

void KIM2Device::state_idle_enter()
{
    // GPIOPins::clear(SAT_EXTWAKEUP);
    // m_kim2_comm.deinit();
    // GPIOPins::clear(SAT_RESET);
}

void KIM2Device::state_idle()
{
    // Check for any new commands
	if (m_packet_buffer.length()) {
		m_tx_buffer = m_packet_buffer;
		m_packet_buffer.clear();
		KIM2_STATE_CHANGE(idle, transmit);
	}
    else
    {
        KIM2_STATE_CHANGE(idle, power_off);
    }
}

void KIM2Device::state_idle_exit()
{
    ;
}

void KIM2Device::state_transmit_enter()
{
    DEBUG_INFO("KIM2Device::state_transmit_enter");
    // GPIOPins::set(SAT_RESET);
    // PMU::delay_ms(10); // Check if delay necessary
    // GPIOPins::set(SAT_EXTWAKEUP);
    // PMU::delay_ms(10); // Check if delay necessary
	// use m_tx_mode ?

    m_tx_done = false;
    send_AT(AT_TX, m_tx_buffer);
    notify(KineisEventTxStarted({}));
    initiate_timeout(60000);     // Update this timeout once LPM and/or BLIND mode implemented
}

void KIM2Device::state_transmit()
{
    if(m_tx_done)
    {
        m_tx_done = false;
        DEBUG_TRACE("KIM2Device::state_transmit : TX status %d", m_kim2_comm.m_tx_status);
        if (m_tx_buffer.size()) {
            m_tx_buffer.clear();
            notify(KineisEventTxComplete({})); // check tx status ?
        }
        //TODO handle other cases
        KIM2_STATE_CHANGE(transmit, idle);
    }
    else if(m_is_error)
    {
        KIM2_STATE_CHANGE(transmit, error);
    }
    else
    {
        run_state_machine();
    }
}

void KIM2Device::state_transmit_exit()
{
    cancel_timeout();
}

void KIM2Device::state_error_enter()
{
    ;
}

void KIM2Device::state_error()
{
    DEBUG_ERROR("KIM2Device::state_error");
    notify(KineisEventDeviceError({}));
    KIM2_STATE_CHANGE(error, power_off);
}

void KIM2Device::state_error_exit()
{
    ;
}
