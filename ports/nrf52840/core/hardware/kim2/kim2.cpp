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

// Timing constants (ms)
#define KIM2_DELAY_POWER_ON_MS   500
#define KIM2_DELAY_POLL_MS       100
#define KIM2_TX_TIMEOUT_MS       60000
#define KIM2_IDLE_TICK_MS        100

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

// ============================================================================
// Constructor / Destructor
// ============================================================================

KIM2Device::KIM2Device()
{
    m_tx_buffer.clear();
    m_packet_buffer.clear();
    m_state = KIM2ManagerState::power_off;
    m_tx_mode = KineisModulation::LDK;
    m_current_rconf_mode = KineisModulation::LDK;
    m_timeout = {};
    m_stopping = false;
    m_cmd_is_ok = false;
    m_is_error = false;
    m_tx_done = false;
    m_tx_poll_counter = 0;
    // Start in power_off — device will power on when send() is called
}

KIM2Device::~KIM2Device()
{
    power_off_immediate();
}

// ============================================================================
// KineisDevice interface
// ============================================================================

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
                DEBUG_ERROR("KIM2Device::send: LDK payload too long: %d bits (max %d)",
                    payload_length, LDK_MAX_LENGTH_BITS);
                return;
            }
            stuffing_bits = LDK_MAX_LENGTH_BITS - payload_length;
            break;

        case KineisModulation::LDA2:
            if (payload_length > LDA2_MAX_LENGTH_BITS)
            {
                DEBUG_ERROR("KIM2Device::send: LDA2 payload too long: %d bits (max %d)",
                    payload_length, LDA2_MAX_LENGTH_BITS);
                return;
            }
            modulo = payload_length % 32;
            stuffing_bits = modulo ? (32 - modulo) : 0;
            break;

        case KineisModulation::VLDA4:
            if (payload_length > VLDA4_MAX_LENGTH_BITS)
            {
                DEBUG_ERROR("KIM2Device::send: VLDA4 payload too long: %d bits (max %d)",
                    payload_length, VLDA4_MAX_LENGTH_BITS);
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

    // Store as hex string for AT+TX command
    DEBUG_TRACE("KIM2Device::send: data[%u]=%s", total_bits, Binascii::hexlify(packet).c_str());
    m_packet_buffer = Binascii::hexlify(packet).c_str();
    m_tx_mode = mode;

    // Request power on (if not already running)
    start_device();
}

void KIM2Device::stop_send() {
    DEBUG_TRACE("KIM2Device::stop_send");
    m_packet_buffer.clear();
    m_tx_buffer.clear();
}

void KIM2Device::start_receive(const KineisModulation mode)
{
    (void)mode;
    DEBUG_WARN("KIM2Device::start_receive: RX not implemented");
}

bool KIM2Device::stop_receive()
{
    DEBUG_WARN("KIM2Device::stop_receive: RX not implemented");
    return false;
}

void KIM2Device::set_frequency(double freq_mhz)
{
    (void)freq_mhz;
    // KIM2 frequency is configured via RCONF, not a separate command
}

void KIM2Device::set_tcxo_warmup_time(unsigned int ms)
{
    (void)ms;
    // KIM2 TCXO warmup is managed internally by the module
}

// ============================================================================
// Runtime modulation switching
// ============================================================================

bool KIM2Device::switch_modulation(KineisModulation mode, const std::string& rconf_hex) {
    if (mode == m_current_rconf_mode) {
        DEBUG_TRACE("KIM2Device::%s: already in target modulation %d", __func__, (int)mode);
        return true;
    }

    DEBUG_INFO("KIM2Device::%s: switching %d -> %d", __func__, (int)m_current_rconf_mode, (int)mode);

    if (rconf_hex.size() != 32) {
        DEBUG_ERROR("KIM2Device::%s: invalid RCONF hex length %u", __func__, (unsigned)rconf_hex.size());
        return false;
    }

    // If device is powered off, just store the target mode.
    // state_init() will write the correct RCONF at next power-on.
    if (m_state == KIM2ManagerState::power_off) {
        DEBUG_INFO("KIM2Device::%s: device OFF, deferring to next init", __func__);
        m_current_rconf_mode = mode;
        return true;
    }

    if (!send_AT(AT_SET_RCONF, rconf_hex)) {
        DEBUG_ERROR("KIM2Device::%s: failed to set RCONF", __func__);
        return false;
    }

    if (!send_AT(AT_SET_KMAC_BASIC)) {
        DEBUG_ERROR("KIM2Device::%s: failed to reload KMAC", __func__);
        return false;
    }

    m_current_rconf_mode = mode;
    DEBUG_INFO("KIM2Device::%s: modulation switched OK", __func__);
    return true;
}

KineisModulation KIM2Device::get_current_modulation() const {
    return m_current_rconf_mode;
}

// ============================================================================
// KIM2Comm event handlers (ISR context)
// ============================================================================

void KIM2Device::react(const KIM2CommEventRespOk&) {
    m_cmd_is_ok = true;
}

void KIM2Device::react(const KIM2CommEventTxDone&) {
    m_tx_done = true;
}

void KIM2Device::react(const KIM2CommEventRespError&) {
    m_is_error = true;
}

void KIM2Device::react(const KIM2CommEventUartError& err) {
    system_scheduler->post_task_prio([this, err]() {
        DEBUG_INFO("KIM2CommEventUartError: type=%02x", err.error_type);
    }, "Debug");
}

// ============================================================================
// AT command helper
// ============================================================================

bool KIM2Device::send_AT(ATCmd cmd, const std::optional<std::string>& params, uint16_t timeout_ms)
{
    m_cmd_is_ok = false;
    m_is_error = false;

    m_kim2_comm.send(cmd, params);

    uint16_t wdt_kick_counter = 0;
    while(!m_cmd_is_ok && !m_is_error && timeout_ms != 0)
    {
        PMU::delay_ms(1);
        timeout_ms--;
        if (++wdt_kick_counter >= 10000) {
            PMU::kick_watchdog();
            wdt_kick_counter = 0;
        }
    }

    if(timeout_ms == 0) {
        DEBUG_WARN("KIM2Device::send_AT: timeout (cmd=%d)", (int)cmd);
    }

    return m_cmd_is_ok && !m_is_error;
}

// ============================================================================
// Timeout management
// ============================================================================

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

// ============================================================================
// Power management
// ============================================================================

void KIM2Device::start_device()
{
    if (m_state != KIM2ManagerState::power_off) {
        m_stopping = false;
        DEBUG_TRACE("KIM2Device::start: already running in state=%u", (unsigned int)m_state);
        run_state_machine(0);
        return;
    }

    m_stopping = false;
    KIM2_STATE_CHANGE(power_off, power_on);
}

void KIM2Device::power_off_immediate(void)
{
    DEBUG_TRACE("KIM2Device::power_off_immediate");

    if (!KIM2_STATE_EQUAL(power_off)) {
        system_scheduler->cancel_task(m_task);
        cancel_timeout();
        KIM2_STATE_CHANGE(idle, power_off);
    }
}

// ============================================================================
// State machine
// ============================================================================

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

// ============================================================================
// State: power_off
// ============================================================================

void KIM2Device::state_power_off_enter()
{
    DEBUG_INFO("KIM2Device::state_power_off_enter");
    m_kim2_comm.deinit();
    GPIOPins::clear(SAT_EXTWAKEUP);
    GPIOPins::clear(SAT_PWR_EN);
    m_tx_buffer.clear();
    m_packet_buffer.clear();

    notify(KineisEventPowerOff({}));
}

void KIM2Device::state_power_off()
{
    ;
}

void KIM2Device::state_power_off_exit()
{
    ;
}

// ============================================================================
// State: power_on
// ============================================================================

void KIM2Device::state_power_on_enter()
{
    GPIOPins::set(SAT_PWR_EN);
    GPIOPins::set(SAT_EXTWAKEUP);
    m_kim2_comm.init();
    m_kim2_comm.subscribe(*this);
    PMU::delay_ms(KIM2_DELAY_POWER_ON_MS);
}

void KIM2Device::state_power_on()
{
    DEBUG_INFO("KIM2Device::state_power_on");
    if(send_AT(AT_PING))
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

// ============================================================================
// State: init
// ============================================================================

void KIM2Device::state_init_enter()
{
    ;
}

void KIM2Device::state_init()
{
    // Read credentials from module if not already known
    if(m_kim2_comm.m_kineis_id == 0 && m_kim2_comm.m_hex_addr == 0)
    {
        if(!send_AT(AT_GET_ID))
        {
            DEBUG_ERROR("KIM2Device::state_init: can not read ID");
            KIM2_STATE_CHANGE(init, error);
            return;
        }
        DEBUG_TRACE("KIM2Device::state_init ID:%d", m_kim2_comm.m_kineis_id);
        configuration_store->write_param(ParamID::ARGOS_DECID, m_kim2_comm.m_kineis_id);

        if(!send_AT(AT_GET_ADDR))
        {
            DEBUG_ERROR("KIM2Device::state_init: can not read ADDR");
            KIM2_STATE_CHANGE(init, error);
            return;
        }
        DEBUG_TRACE("KIM2Device::state_init ADDR:%x", m_kim2_comm.m_hex_addr);
        configuration_store->write_param(ParamID::ARGOS_HEXID, m_kim2_comm.m_hex_addr);
    }

    // Configure RCONF + KMAC
    // When adaptive modulation is ON, use the per-modulation RCONF matching
    // m_current_rconf_mode (set by switch_modulation() while device was OFF).
    // When adaptive is OFF, use the master RCONF entered by the user.
    std::string rconf;
    bool adaptive = configuration_store->read_param<bool>(ParamID::ARGOS_ADAPTIVE_MODULATION);
    if (adaptive) {
        switch (m_current_rconf_mode) {
            case KineisModulation::LDK:
                rconf = configuration_store->read_param<std::string>(ParamID::ARGOS_RADIOCONF_LDK);
                break;
            case KineisModulation::VLDA4:
                rconf = configuration_store->read_param<std::string>(ParamID::ARGOS_RADIOCONF_VLDA4);
                break;
            case KineisModulation::LDA2:
            default:
                rconf = configuration_store->read_param<std::string>(ParamID::ARGOS_RADIOCONF_LDA2);
                break;
        }
        DEBUG_INFO("KIM2Device::state_init: adaptive ON, using RCONF for mode %d", (int)m_current_rconf_mode);
    } else {
        rconf = configuration_store->read_param<std::string>(ParamID::ARGOS_RADIOCONF);
    }
    if (rconf.empty()) {
        rconf = "03921fb104b92859209b18abd009de96"; // Default: ESS4 - LDK - 27dBm
        DEBUG_WARN("KIM2Device::state_init: RCONF empty, using default");
    }

    if(!send_AT(AT_SET_RCONF, rconf))
    {
        DEBUG_ERROR("KIM2Device::state_init: can not set RCONF");
        KIM2_STATE_CHANGE(init, error);
        return;
    }

    if(!send_AT(AT_SET_KMAC_BASIC))
    {
        DEBUG_ERROR("KIM2Device::state_init: can not set KMAC");
        KIM2_STATE_CHANGE(init, error);
        return;
    }
    DEBUG_TRACE("KIM2Device::state_init RCONF and KMAC set");
    if (!adaptive) {
        configuration_store->write_param(ParamID::ARGOS_RADIOCONF, rconf);
    }

    std::string lpm_standby = "0x0F";
    if(!send_AT(AT_SET_LPM, lpm_standby))
    {
        DEBUG_ERROR("KIM2Device::state_init: can not set LPM");
        KIM2_STATE_CHANGE(init, error);
        return;
    }
    DEBUG_TRACE("KIM2Device::state_init LPM=standby set");
    KIM2_STATE_CHANGE(init, idle);
}

void KIM2Device::state_init_exit()
{
    ;
}

// ============================================================================
// State: idle (with configurable timeout like SMD)
// ============================================================================

void KIM2Device::state_idle_enter()
{
    if (m_idle_timeout_ms > 0) {
        m_tx_poll_counter = m_idle_timeout_ms / KIM2_IDLE_TICK_MS;
    } else {
        m_tx_poll_counter = 0; // No idle timeout — power off immediately if no packet
    }
}

void KIM2Device::state_idle()
{
    // Check for pending TX
	if (m_packet_buffer.length()) {
		m_tx_buffer = m_packet_buffer;
		m_packet_buffer.clear();
		KIM2_STATE_CHANGE(idle, transmit);
	}
    else if (m_stopping) {
        KIM2_STATE_CHANGE(idle, power_off);
    }
    else if (m_idle_timeout_ms == 0) {
        // No idle timeout configured — power off immediately
        KIM2_STATE_CHANGE(idle, power_off);
    }
    else if (m_tx_poll_counter == 0) {
        DEBUG_TRACE("KIM2Device::state_idle: idle timeout elapsed");
        KIM2_STATE_CHANGE(idle, power_off);
    }
    else {
        m_tx_poll_counter--;
        run_state_machine(KIM2_IDLE_TICK_MS);
    }
}

void KIM2Device::state_idle_exit()
{
    ;
}

// ============================================================================
// State: transmit
// ============================================================================

void KIM2Device::state_transmit_enter()
{
    DEBUG_INFO("KIM2Device::state_transmit_enter: mode=%u", (unsigned int)m_tx_mode);

    if (m_tx_mode != m_current_rconf_mode) {
        DEBUG_WARN("KIM2Device::state_transmit_enter: TX mode %u != RCONF mode %u",
            (unsigned int)m_tx_mode, (unsigned int)m_current_rconf_mode);
    }

    m_tx_done = false;
    m_is_error = false;
    send_AT(AT_TX, m_tx_buffer);
    notify(KineisEventTxStarted({}));
    initiate_timeout(KIM2_TX_TIMEOUT_MS);

    // Poll counter for TX completion
    m_tx_poll_counter = KIM2_TX_TIMEOUT_MS / KIM2_DELAY_POLL_MS;
}

void KIM2Device::state_transmit()
{
    if(m_tx_done)
    {
        m_tx_done = false;
        DEBUG_TRACE("KIM2Device::state_transmit: TX status %d", m_kim2_comm.m_tx_status);
        if (m_tx_buffer.size()) {
            m_tx_buffer.clear();
            if (m_kim2_comm.m_tx_status == 0) {
                notify(KineisEventTxComplete({}));
            } else {
                DEBUG_WARN("KIM2Device::state_transmit: TX failed status=%d", m_kim2_comm.m_tx_status);
                notify(KineisEventDeviceError({}));
            }
        }
        KIM2_STATE_CHANGE(transmit, idle);
    }
    else if(m_is_error)
    {
        DEBUG_ERROR("KIM2Device::state_transmit: error during TX");
        m_tx_buffer.clear();
        KIM2_STATE_CHANGE(transmit, error);
    }
    else
    {
        if (--m_tx_poll_counter == 0) {
            DEBUG_ERROR("KIM2Device::state_transmit: TX poll timeout");
            m_tx_buffer.clear();
            KIM2_STATE_CHANGE(transmit, error);
        } else {
            run_state_machine(KIM2_DELAY_POLL_MS);
        }
    }
}

void KIM2Device::state_transmit_exit()
{
    cancel_timeout();
}

// ============================================================================
// State: error
// ============================================================================

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
