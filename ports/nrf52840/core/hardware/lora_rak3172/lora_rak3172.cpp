#include "lora_rak3172.hpp"
#include "lora_rak3172_comm.hpp"
#include "bsp.hpp"
#include "gpio.hpp"
#include "pmu.hpp"
#include "debug.hpp"
#include "error.hpp"
#include "binascii.hpp"
#include "config_store.hpp"
#include <stdint.h>
#include <string>

using namespace LoRa;

#define LORA_COMM_OK  (0)

#define LORA_STATE_CHANGE(x, y)                                 \
    do {                                                        \
        DEBUG_TRACE("LoRa::STATE_CHANGE: " #x " -> " #y);      \
        m_state = State::y;                                     \
        state_ ## x ##_exit();                                  \
        state_ ## y ##_enter();                                 \
        run_state_machine();                                    \
    } while (0)

#define LORA_STATE_EQUAL(x) \
    (m_state == State::x)

#define LORA_STATE_CALL(x) \
    do {                   \
        state_ ## x();     \
    } while (0)

extern Scheduler *system_scheduler;
extern ConfigurationStore *configuration_store;

LoRaDevice::LoRaDevice()
{
    m_packet_buffer.clear();
    m_state = State::power_off;
    m_joined = false;
    m_join_failed = false;
    m_tx_done = false;
    m_config_step = 0;
    load_config_from_store();
    LORA_STATE_CHANGE(power_off, power_on);
}

LoRaDevice::~LoRaDevice()
{
    power_off_immediate();
}

/**
 * Send a packet via LoRaWAN.
 *
 * The modulation parameter is ignored (LoRa uses its own DR settings).
 * The packet data is hex-encoded and sent on the configured FPort.
 */
void LoRaDevice::send(const KineisModulation mode, const KineisPacket& user_payload, const unsigned int payload_length)
{
    (void)mode;  // LoRa handles modulation via DR setting

    // Convert binary payload to hex string
    unsigned int payload_bytes = (payload_length + 7) / 8;
    KineisPacket packet(user_payload.begin(), user_payload.begin() + payload_bytes);
    m_packet_buffer = Binascii::hexlify(packet);

    DEBUG_TRACE("LoRaDevice::send: payload[%u bits]=%s", payload_length, m_packet_buffer.c_str());

    // Request power on (if not already running)
    start_device();
}

void LoRaDevice::stop_send() {
    DEBUG_TRACE("LoRaDevice::stop_send");
    m_packet_buffer.clear();
}

// ========================================================================
// Event handlers from LoRaComm
// ========================================================================

void LoRaDevice::react(const LoRaCommEventRespOk&) {
    m_cmd_is_ok = true;
}

void LoRaDevice::react(const LoRaCommEventRespError& err) {
    DEBUG_INFO("LoRaDevice: error response type=%d", (int)err.error_type);
    m_is_error = true;
}

void LoRaDevice::react(const LoRaCommEventJoined&) {
    DEBUG_INFO("LoRaDevice: JOINED");
    m_joined = true;
}

void LoRaDevice::react(const LoRaCommEventJoinFailed&) {
    DEBUG_INFO("LoRaDevice: JOIN FAILED");
    m_join_failed = true;
}

void LoRaDevice::react(const LoRaCommEventTxDone&) {
    DEBUG_TRACE("LoRaDevice: TX_DONE");
    m_tx_done = true;
}

void LoRaDevice::react(const LoRaCommEventRxData& rx) {
    DEBUG_INFO("LoRaDevice: RX port=%d data=%s", rx.port, rx.payload.c_str());
    // Downlink data received - could be forwarded to a service if needed
}

void LoRaDevice::react(const LoRaCommEventUartError& err) {
    system_scheduler->post_task_prio([this, err]() {
        DEBUG_INFO("LoRaDevice: UART error type=%02x", err.error_type);
    }, "Debug");
}

// ========================================================================
// Timeout management
// ========================================================================

void LoRaDevice::initiate_timeout(unsigned int timeout_ms) {
    cancel_timeout();
    m_timeout.handle = system_scheduler->post_task_prio([this]() {
        on_timeout();
    }, "LoRaTimeout", Scheduler::DEFAULT_PRIORITY, timeout_ms);
}

void LoRaDevice::on_timeout() {
    DEBUG_ERROR("LoRaDevice::on_timeout");
    m_is_error = true;
}

void LoRaDevice::cancel_timeout() {
    system_scheduler->cancel_task(m_timeout.handle);
}

// ========================================================================
// State machine infrastructure
// ========================================================================

void LoRaDevice::start_device()
{
    if (m_state != State::power_off) {
        DEBUG_TRACE("LoRa::start: already running state=%u", (unsigned int)m_state);
        run_state_machine(0);
        return;
    }
    LORA_STATE_CHANGE(power_off, power_on);
}

void LoRaDevice::power_off_immediate(void)
{
    DEBUG_TRACE("LoRaDevice::power_off_immediate");
    if (!LORA_STATE_EQUAL(power_off)) {
        system_scheduler->cancel_task(m_task);
        LORA_STATE_CHANGE(idle, power_off);
    }
}

/**
 * Blocking send of an AT command with polling for OK response.
 * Returns 0 on success (OK received), non-zero on error/timeout.
 */
bool LoRaDevice::send_AT(ATCmd cmd, const std::optional<std::string>& params)
{
    uint16_t timeout_ms = 2000;

    m_cmd_is_ok = false;
    m_is_error = false;

    m_lora_comm.send(cmd, params);

    while(!m_cmd_is_ok && !m_is_error && timeout_ms != 0)
    {
        PMU::delay_ms(1);
        timeout_ms--;
    }

    if(timeout_ms == 0) {
        DEBUG_TRACE("LoRaDevice::send_AT: timeout (no OK received)");
    }

    return !m_cmd_is_ok;  // 0=OK, 1=error
}

void LoRaDevice::state_machine(void)
{
    switch (m_state) {
    case State::power_off:   LORA_STATE_CALL(power_off);  break;
    case State::power_on:    LORA_STATE_CALL(power_on);   break;
    case State::configure:   LORA_STATE_CALL(configure);  break;
    case State::joining:     LORA_STATE_CALL(joining);    break;
    case State::idle:        LORA_STATE_CALL(idle);       break;
    case State::transmit:    LORA_STATE_CALL(transmit);   break;
    case State::error:       LORA_STATE_CALL(error);      break;
    default: break;
    }
}

void LoRaDevice::run_state_machine(uint16_t delay_ms)
{
    system_scheduler->cancel_task(m_task);
    m_task = system_scheduler->post_task_prio([this]() {
        state_machine();
    }, "LoRaStateMachine", Scheduler::DEFAULT_PRIORITY, delay_ms);
}

// ========================================================================
// State: power_off
// ========================================================================

void LoRaDevice::state_power_off_enter()
{
    DEBUG_INFO("LoRaDevice::state_power_off_enter");
    m_lora_comm.deinit();
    GPIOPins::clear(SAT_EXTWAKEUP);
    GPIOPins::clear(SAT_PWR_EN);
    m_packet_buffer.clear();
}

void LoRaDevice::state_power_off() {
    ;
}

void LoRaDevice::state_power_off_exit() {
    ;
}

// ========================================================================
// State: power_on
//   - Enable power to module
//   - Init UART at 115200 baud
//   - Wait for module boot
//   - Verify communication with AT ping
// ========================================================================

void LoRaDevice::state_power_on_enter()
{
    GPIOPins::set(SAT_PWR_EN);
    GPIOPins::set(SAT_EXTWAKEUP);
    m_lora_comm.init();
    m_lora_comm.subscribe(*this);
    m_joined = false;
    m_join_failed = false;

    // Wait for RAK3172 boot (typically ~1.5s)
    PMU::delay_ms(2000);
}

void LoRaDevice::state_power_on()
{
    DEBUG_INFO("LoRaDevice::state_power_on");

    // Ping module with AT command (retry up to 3 times)
    for (int retry = 0; retry < 3; retry++)
    {
        if (send_AT(AT_TEST) == LORA_COMM_OK)
        {
            DEBUG_TRACE("LoRaDevice: AT ping OK (attempt %d)", retry + 1);
            LORA_STATE_CHANGE(power_on, configure);
            return;
        }
        PMU::delay_ms(500);
    }

    DEBUG_ERROR("LoRaDevice: AT ping failed after 3 attempts");
    LORA_STATE_CHANGE(power_on, error);
}

void LoRaDevice::state_power_on_exit() {
    ;
}

// ========================================================================
// State: configure
//   - Set LoRaWAN parameters: join mode, band, keys, class, DR, ADR, etc.
//   - Each parameter is sent as a blocking AT command
// ========================================================================

void LoRaDevice::state_configure_enter()
{
    m_config_step = 0;
}

void LoRaDevice::state_configure()
{
    DEBUG_INFO("LoRaDevice::state_configure step=%u", m_config_step);
    bool at_error = false;

    switch (m_config_step)
    {
        case 0:
            // Set join mode (OTAA or ABP)
            at_error = send_AT(AT_SET_NJM, std::to_string(m_config.njm));
            break;

        case 1:
            // Set frequency band
            at_error = send_AT(AT_SET_BAND, std::to_string(m_config.band));
            break;

        case 2:
            // Set or read Device EUI
            if (m_config.deveui.empty()) {
                // Read factory DEVEUI from module
                at_error = send_AT(AT_GET_DEVEUI);
                if (!at_error) {
                    m_config.deveui = m_lora_comm.m_last_value;
                    DEBUG_INFO("LoRaDevice: DEVEUI=%s", m_config.deveui.c_str());
                }
            } else {
                at_error = send_AT(AT_SET_DEVEUI, m_config.deveui);
            }
            break;

        case 3:
            // Set Application EUI
            if (!m_config.appeui.empty()) {
                at_error = send_AT(AT_SET_APPEUI, m_config.appeui);
            }
            break;

        case 4:
            // Set Application Key (OTAA) or session keys (ABP)
            if (m_config.njm == 1 && !m_config.appkey.empty()) {
                at_error = send_AT(AT_SET_APPKEY, m_config.appkey);
            } else if (m_config.njm == 0) {
                // ABP: set DevAddr, NwkSKey, AppSKey
                if (!m_config.devaddr.empty())
                    at_error = send_AT(AT_SET_DEVADDR, m_config.devaddr);
                if (!at_error && !m_config.nwkskey.empty())
                    at_error = send_AT(AT_SET_NWKSKEY, m_config.nwkskey);
                if (!at_error && !m_config.appskey.empty())
                    at_error = send_AT(AT_SET_APPSKEY, m_config.appskey);
            }
            break;

        case 5:
            // Set device class
            at_error = send_AT(AT_SET_CLASS, std::string(1, (char)m_config.device_class));
            break;

        case 6:
            // Set data rate
            at_error = send_AT(AT_SET_DR, std::to_string(m_config.dr));
            break;

        case 7:
            // Set ADR
            at_error = send_AT(AT_SET_ADR, std::to_string(m_config.adr));
            break;

        case 8:
            // Set TX power
            at_error = send_AT(AT_SET_TXP, std::to_string(m_config.txp));
            break;

        case 9:
            // Set confirmed/unconfirmed mode
            at_error = send_AT(AT_SET_CFM, std::to_string(m_config.cfm));
            break;

        case 10:
            // Configuration complete - check if already joined
            at_error = send_AT(AT_GET_NJS);
            if (!at_error && m_lora_comm.m_last_value == "1") {
                DEBUG_INFO("LoRaDevice: already joined from previous session");
                m_joined = true;
                LORA_STATE_CHANGE(configure, idle);
                return;
            }
            // Not joined - proceed to joining state
            LORA_STATE_CHANGE(configure, joining);
            return;

        default:
            LORA_STATE_CHANGE(configure, joining);
            return;
    }

    if (at_error)
    {
        DEBUG_ERROR("LoRaDevice::state_configure: failed at step %u", m_config_step);
        LORA_STATE_CHANGE(configure, error);
        return;
    }

    m_config_step++;
    run_state_machine(50);  // Continue configuration
}

void LoRaDevice::state_configure_exit() {
    ;
}

// ========================================================================
// State: joining
//   - Initiate LoRaWAN join procedure
//   - Wait for +EVT:JOINED or +EVT:JOIN FAILED
//   - Timeout after 120 seconds
// ========================================================================

void LoRaDevice::state_joining_enter()
{
    DEBUG_INFO("LoRaDevice::state_joining_enter");
    m_joined = false;
    m_join_failed = false;
    m_is_error = false;

    // AT+JOIN=<start>:<auto_join>:<interval>:<attempts>
    // start=1, auto_join=1, interval=10s, attempts=0 (infinite)
    send_AT(AT_JOIN, "1:1:10:0");

    // Timeout for join procedure (120 seconds)
    initiate_timeout(120000);
}

void LoRaDevice::state_joining()
{
    if (m_joined)
    {
        DEBUG_INFO("LoRaDevice: network joined successfully");
        LORA_STATE_CHANGE(joining, idle);
    }
    else if (m_join_failed || m_is_error)
    {
        DEBUG_ERROR("LoRaDevice: join failed");
        LORA_STATE_CHANGE(joining, error);
    }
    else
    {
        // Still waiting for join event
        run_state_machine(500);
    }
}

void LoRaDevice::state_joining_exit()
{
    cancel_timeout();
}

// ========================================================================
// State: idle
//   - Check for pending TX packets
//   - Power off if nothing to send
// ========================================================================

void LoRaDevice::state_idle_enter() {
    ;
}

void LoRaDevice::state_idle()
{
    if (m_packet_buffer.length()) {
        LORA_STATE_CHANGE(idle, transmit);
    } else {
        // Nothing to send - power off
        // RAK3172 saves session state in flash, so rejoin after power-up is fast
        LORA_STATE_CHANGE(idle, power_off);
    }
}

void LoRaDevice::state_idle_exit() {
    ;
}

// ========================================================================
// State: transmit
//   - Send hex payload via AT+SEND=<fport>:<hex_data>
//   - Wait for +EVT:TX_DONE
// ========================================================================

void LoRaDevice::state_transmit_enter()
{
    DEBUG_INFO("LoRaDevice::state_transmit_enter");

    m_tx_done = false;
    m_is_error = false;

    // Format: AT+SEND=<port>:<hex_payload>
    std::string send_params = std::to_string(m_config.fport) + ":" + m_packet_buffer;
    send_AT(AT_SEND, send_params);

    notify(KineisEventTxStarted({}));

    // TX timeout: 30 seconds should be enough for any DR
    initiate_timeout(30000);
}

void LoRaDevice::state_transmit()
{
    if (m_tx_done)
    {
        m_tx_done = false;
        DEBUG_TRACE("LoRaDevice::state_transmit: TX complete");
        m_packet_buffer.clear();
        notify(KineisEventTxComplete({}));
        LORA_STATE_CHANGE(transmit, idle);
    }
    else if (m_is_error)
    {
        DEBUG_ERROR("LoRaDevice::state_transmit: TX error");
        LORA_STATE_CHANGE(transmit, error);
    }
    else
    {
        run_state_machine(100);
    }
}

void LoRaDevice::state_transmit_exit()
{
    cancel_timeout();
}

// ========================================================================
// State: error
// ========================================================================

void LoRaDevice::state_error_enter() {
    ;
}

void LoRaDevice::state_error()
{
    DEBUG_ERROR("LoRaDevice::state_error");
    notify(KineisEventDeviceError({}));
    LORA_STATE_CHANGE(error, power_off);
}

void LoRaDevice::state_error_exit() {
    ;
}

// ========================================================================
// Config store integration
// ========================================================================

void LoRaDevice::load_config_from_store()
{
    m_config.deveui  = configuration_store->read_param<std::string>(ParamID::LORA_DEVEUI);
    m_config.appeui  = configuration_store->read_param<std::string>(ParamID::LORA_APPEUI);
    m_config.appkey  = configuration_store->read_param<std::string>(ParamID::LORA_APPKEY);
    m_config.devaddr = configuration_store->read_param<std::string>(ParamID::LORA_DEVADDR);
    m_config.appskey = configuration_store->read_param<std::string>(ParamID::LORA_APPSKEY);
    m_config.nwkskey = configuration_store->read_param<std::string>(ParamID::LORA_NWKSKEY);
    m_config.njm     = (uint8_t)configuration_store->read_param<unsigned int>(ParamID::LORA_NJM);
    m_config.band    = (uint8_t)configuration_store->read_param<unsigned int>(ParamID::LORA_BAND);
    m_config.dr      = (uint8_t)configuration_store->read_param<unsigned int>(ParamID::LORA_DR);
    m_config.adr     = configuration_store->read_param<bool>(ParamID::LORA_ADR) ? 1 : 0;
    m_config.txp     = (uint8_t)configuration_store->read_param<unsigned int>(ParamID::LORA_TXP);
    m_config.cfm     = configuration_store->read_param<bool>(ParamID::LORA_CFM) ? 1 : 0;
    m_config.fport   = (uint8_t)configuration_store->read_param<unsigned int>(ParamID::LORA_FPORT);

    // Class: stored as 0/1/2 → convert to 'A'/'B'/'C'
    unsigned int lora_class = configuration_store->read_param<unsigned int>(ParamID::LORA_CLASS);
    m_config.device_class = 'A' + (uint8_t)lora_class;

    DEBUG_INFO("LoRaDevice: config loaded from store NJM=%u BAND=%u DR=%u ADR=%u TXP=%u CFM=%u FPORT=%u CLASS=%c",
               m_config.njm, m_config.band, m_config.dr, m_config.adr, m_config.txp, m_config.cfm, m_config.fport, m_config.device_class);
    if (!m_config.deveui.empty())
        DEBUG_INFO("LoRaDevice: DEVEUI=%s", m_config.deveui.c_str());
    if (!m_config.appeui.empty())
        DEBUG_INFO("LoRaDevice: APPEUI=%s", m_config.appeui.c_str());
}

// ========================================================================
// KineisDevice interface - unused methods for LoRa
// ========================================================================

void LoRaDevice::start_receive(const KineisModulation mode)
{
    (void)mode;
    DEBUG_WARN("LoRaDevice::start_receive: not implemented (use Class C for continuous RX)");
}

bool LoRaDevice::stop_receive()
{
    DEBUG_WARN("LoRaDevice::stop_receive: not implemented");
    return false;
}

void LoRaDevice::set_frequency(double freq_mhz)
{
    (void)freq_mhz;
    DEBUG_TRACE("LoRaDevice::set_frequency: %.3f MHz (ignored - LoRa uses band/channel config)", freq_mhz);
}

void LoRaDevice::set_tcxo_warmup_time(unsigned int ms)
{
    (void)ms;
    DEBUG_TRACE("LoRaDevice::set_tcxo_warmup_time: %u ms (managed by RAK3172)", ms);
}
