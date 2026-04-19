/**
 * @file lora_rak3172.cpp
 * @brief LoRa RAK3172-SiP device driver — state machine, TX, join, config.
 */

#include "lora_rak3172.hpp"
#include "lora_rak3172_comm.hpp"
#include "bsp.hpp"
#include "gpio.hpp"
#include "pmu.hpp"
#include "debug.hpp"
#include "error.hpp"
#include "binascii.hpp"
#include "config_store.hpp"
#include <cstdint>
#include <string>

using namespace LoRa;

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
    m_is_configured = false;
    m_power_on_retry = 0;
    m_nwm_reboot_pending = false;
    m_consecutive_errors = 0;
    load_config_from_store();
    LORA_STATE_CHANGE(power_off, power_on);
}

LoRaDevice::~LoRaDevice()
{
    stop_bridge();
    power_off_immediate();
}

bool LoRaDevice::start_bridge(LoRaComm::PassthroughCallback rx_callback)
{
    if (m_bridge_active)
        return true;

    // Cancel any pending state machine task
    cancel_timeout();
    system_scheduler->cancel_task(m_task);

    // Ensure UART is initialized
    m_lora_comm.init();

    // Enable passthrough: raw UART RX goes to callback
    m_lora_comm.set_passthrough(true, rx_callback);
    m_bridge_active = true;

    DEBUG_INFO("LoRaDevice: bridge mode ACTIVE");
    return true;
}

void LoRaDevice::stop_bridge()
{
    if (!m_bridge_active)
        return;

    m_lora_comm.set_passthrough(false);
    m_bridge_active = false;

    DEBUG_INFO("LoRaDevice: bridge mode STOPPED");

    // Restart state machine from current state
    run_state_machine();
}

bool LoRaDevice::bridge_send(const uint8_t* data, size_t len)
{
    if (!m_bridge_active)
        return false;
    return m_lora_comm.send_raw_data(data, len);
}

void LoRaDevice::bridge_process_rx()
{
    if (m_bridge_active)
        m_lora_comm.process_rx();
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

    // Reject if already transmitting to avoid overwriting in-flight packet
    if (m_state == State::transmit) {
        DEBUG_WARN("LoRaDevice::send: already transmitting, ignoring");
        return;
    }

    // Defense in depth: reject oversized payloads before handing to the module.
    // Service layer should already clamp to LoRaPayloadLimits::max_payload_for_dr,
    // but a stale config cache or programming error must not be able to wedge the
    // module with an AT_PARAM_ERROR that falls through the error retry path.
    unsigned int payload_bytes = (payload_length + 7) / 8;
    static constexpr unsigned int dr_limits[] = { 51, 51, 51, 115, 222, 222 };
    unsigned int max_bytes = (m_config.dr < 6) ? dr_limits[m_config.dr] : dr_limits[0];
    if (payload_bytes > max_bytes) {
        DEBUG_ERROR("LoRaDevice::send: payload %u bytes > DR%u limit %u — rejecting",
                    payload_bytes, m_config.dr, max_bytes);
        notify(KineisEventDeviceError({}));
        return;
    }

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
    DEBUG_INFO("LoRaDevice: error response type=%d", static_cast<int>(err.error_type));
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
    DEBUG_INFO("LoRaDevice: UART error type=%02x", err.error_type);
    m_is_error = true;
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
    if (m_state == State::standby) {
        // Quick wake from LPM Stop2 via AT ping (~10ms)
        // RAK3172 wakes on first UART byte, responds to AT within a few ms
        // Retry up to 3 times before power cycling (module may be slow to wake)
        bool wake_ok = false;
        for (int attempt = 0; attempt < 3 && !wake_ok; attempt++) {
            if (send_AT(AT_TEST))
                wake_ok = true;
        }
        if (wake_ok) {
            DEBUG_TRACE("LoRa: wake from standby OK");
            LORA_STATE_CHANGE(standby, idle);
        } else {
            // Module not responding after 3 attempts — full power cycle
            DEBUG_WARN("LoRa: standby wake failed after 3 attempts, power cycling");
            std::string saved_packet = m_packet_buffer;
            power_off_immediate();  // This clears m_packet_buffer
            m_packet_buffer = saved_packet;
            LORA_STATE_CHANGE(power_off, power_on);
        }
        return;
    }

    if (m_state != State::power_off) {
        DEBUG_TRACE("LoRa::start: already running state=%u", static_cast<unsigned>(m_state));
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
        cancel_timeout();
        m_state = State::power_off;
        state_power_off_enter();
    }
}

/// @brief Blocking send of an AT command with polling for OK response.
/// @param cmd        AT command type.
/// @param params     Optional parameter string.
/// @param timeout_ms Maximum wait time in ms.
/// @return true on success (OK received), false on error/timeout.
bool LoRaDevice::send_AT(ATCmd cmd, const std::optional<std::string>& params, uint16_t timeout_ms)
{
    m_cmd_is_ok = false;
    m_is_error = false;

    m_lora_comm.send(cmd, params);

    while (!m_cmd_is_ok && !m_is_error && timeout_ms != 0) {
        PMU::delay_ms(1);
        m_lora_comm.process_rx();
        timeout_ms--;
    }

    if (timeout_ms == 0)
        DEBUG_WARN("LoRaDevice::send_AT: timeout (cmd=%d)", static_cast<int>(cmd));

    return m_cmd_is_ok && !m_is_error;
}

void LoRaDevice::state_machine(void)
{
    switch (m_state) {
    case State::power_off:   LORA_STATE_CALL(power_off);  break;
    case State::power_on:    LORA_STATE_CALL(power_on);   break;
    case State::configure:   LORA_STATE_CALL(configure);  break;
    case State::joining:     LORA_STATE_CALL(joining);    break;
    case State::idle:        LORA_STATE_CALL(idle);       break;
    case State::standby:     LORA_STATE_CALL(standby);    break;
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
    m_lora_comm.init();  // RAK3172 RUI3 factory default 115200 8N1
    m_lora_comm.subscribe(*this);
    m_joined = false;
    m_join_failed = false;
    m_power_on_retry = 0;
    // Non-blocking: schedule first state_power_on() after module boot delay
}

void LoRaDevice::state_power_on()
{
    if (m_power_on_retry == 0) {
        // First call: wait for RAK3172 boot (typically ~1.5s)
        m_power_on_retry = 1;
        run_state_machine(2000);  // Non-blocking 2s boot wait
        return;
    }

    // Drain boot banner / noise before the first AT ping. The RAK3172 prints a
    // welcome banner at power-on which would otherwise trigger a spurious
    // framing error and confuse the next send_AT's polling loop.
    if (m_power_on_retry == 1)
        m_lora_comm.process_rx();

    DEBUG_INFO("LoRaDevice::state_power_on retry=%u", m_power_on_retry);

    // Try AT ping
    if (send_AT(AT_TEST))
    {
        DEBUG_TRACE("LoRaDevice: AT ping OK (attempt %u)", m_power_on_retry);
        LORA_STATE_CHANGE(power_on, configure);
        return;
    }

    if (m_power_on_retry < 4) {
        m_power_on_retry++;
        run_state_machine(500);  // Non-blocking 500ms retry delay
        return;
    }

    DEBUG_ERROR("LoRaDevice: AT ping failed after %u attempts — check wiring/module", m_power_on_retry);
    LORA_STATE_CHANGE(power_on, error);
}

void LoRaDevice::state_power_on_exit() {
    ;
}

// ========================================================================
// State: configure
//   - First boot: full configuration (all LoRaWAN parameters via AT commands)
//   - Subsequent boots: fast path — RAK3172 persists config to flash,
//     so we only read DEVEUI and check join status
// ========================================================================

void LoRaDevice::state_configure_enter()
{
    m_nwm_reboot_pending = false;
    if (m_is_configured) {
        // Fast path: RAK3172 persists config to flash, skip to DEVEUI read + join check
        m_config_step = 100;
        DEBUG_INFO("LoRaDevice: fast config (module already configured)");
    } else {
        m_config_step = 0;
        DEBUG_INFO("LoRaDevice: full config (first boot)");
    }
}

void LoRaDevice::state_configure()
{
    DEBUG_TRACE("LoRaDevice::state_configure step=%u", m_config_step);
    bool at_error = false;

    switch (m_config_step)
    {
        // ==== Full configuration (first boot only) ====

        case 0:
        {
            // Ensure LoRaWAN network mode (AT+NWM=1)
            if (m_nwm_reboot_pending) {
                m_nwm_reboot_pending = false;
                at_error = !send_AT(AT_TEST);
                if (at_error) {
                    DEBUG_ERROR("LoRaDevice: module not responding after NWM reboot");
                }
                break;
            }
            at_error = !send_AT(AT_GET_NWM);
            if (!at_error && m_lora_comm.m_last_value == "1") {
                DEBUG_TRACE("LoRaDevice: already in LoRaWAN mode");
                break;
            }
            DEBUG_INFO("LoRaDevice: setting NWM=1 (module will reboot)");
            if (!m_lora_comm.send(AT_SET_NWM, "1")) {
                DEBUG_ERROR("LoRaDevice: failed to issue AT+NWM=1 (UART busy/deinit)");
                at_error = true;
                break;
            }
            m_nwm_reboot_pending = true;
            run_state_machine(3000);  // Wait for module autoreboot after NWM change
            return;
        }

        case 1:
            // Set join mode (OTAA or ABP)
            at_error = !send_AT(AT_SET_NJM, std::to_string(m_config.njm));
            break;

        case 2:
            // Set frequency band
            at_error = !send_AT(AT_SET_BAND, std::to_string(m_config.band));
            break;

        case 3:
            // Set Device EUI — try: 1) config store, 2) module, 3) nRF52840 FICR
            if (m_config.deveui.empty()) {
                at_error = !send_AT(AT_GET_DEVEUI);
                if (!at_error && !m_lora_comm.m_last_value.empty() &&
                    m_lora_comm.m_last_value != "0000000000000000") {
                    m_config.deveui = m_lora_comm.m_last_value;
                    DEBUG_INFO("LoRaDevice: module DEVEUI=%s", m_config.deveui.c_str());
                } else {
                    // Generate from nRF52840 FICR unique device ID
                    char buf[17];
#ifdef NRF52840_XXAA
                    snprintf(buf, sizeof(buf), "%08lX%08lX",
                             (unsigned long)NRF_FICR->DEVICEID[1],
                             (unsigned long)NRF_FICR->DEVICEID[0]);
#else
                    snprintf(buf, sizeof(buf), "%08lX%08lX",
                             (unsigned long)0, (unsigned long)PMU::device_identifier());
#endif
                    m_config.deveui = std::string(buf);
                    DEBUG_INFO("LoRaDevice: generated DEVEUI=%s", m_config.deveui.c_str());
                    at_error = !send_AT(AT_SET_DEVEUI, m_config.deveui);
                }
                // Persist to config store so user can read via PARMR (LRP01)
                configuration_store->write_param(ParamID::LORA_DEVEUI, m_config.deveui);
            } else {
                at_error = !send_AT(AT_SET_DEVEUI, m_config.deveui);
            }
            break;

        case 4:
            // Set Application EUI (OTAA only)
            if (m_config.njm == 1 && !m_config.appeui.empty()) {
                if (m_config.appeui.size() != 16) {
                    DEBUG_ERROR("LoRaDevice: invalid APPEUI length %u (expected 16 hex chars)", static_cast<unsigned>(m_config.appeui.size()));
                    at_error = true;
                    break;
                }
                at_error = !send_AT(AT_SET_APPEUI, m_config.appeui);
            }
            break;

        case 5:
            // Set Application Key (OTAA) or session keys (ABP)
            if (m_config.njm == 1 && !m_config.appkey.empty()) {
                if (m_config.appkey.size() != 32) {
                    DEBUG_ERROR("LoRaDevice: invalid APPKEY length %u (expected 32 hex chars)", static_cast<unsigned>(m_config.appkey.size()));
                    at_error = true;
                    break;
                }
                at_error = !send_AT(AT_SET_APPKEY, m_config.appkey);
            } else if (m_config.njm == 0) {
                if (!m_config.devaddr.empty()) {
                    if (m_config.devaddr.size() != 8) {
                        DEBUG_ERROR("LoRaDevice: invalid DEVADDR length %u (expected 8 hex chars)", static_cast<unsigned>(m_config.devaddr.size()));
                        at_error = true;
                        break;
                    }
                    at_error = !send_AT(AT_SET_DEVADDR, m_config.devaddr);
                }
                if (!at_error && !m_config.nwkskey.empty()) {
                    if (m_config.nwkskey.size() != 32) {
                        DEBUG_ERROR("LoRaDevice: invalid NWKSKEY length %u (expected 32 hex chars)", static_cast<unsigned>(m_config.nwkskey.size()));
                        at_error = true;
                        break;
                    }
                    at_error = !send_AT(AT_SET_NWKSKEY, m_config.nwkskey);
                }
                if (!at_error && !m_config.appskey.empty()) {
                    if (m_config.appskey.size() != 32) {
                        DEBUG_ERROR("LoRaDevice: invalid APPSKEY length %u (expected 32 hex chars)", static_cast<unsigned>(m_config.appskey.size()));
                        at_error = true;
                        break;
                    }
                    at_error = !send_AT(AT_SET_APPSKEY, m_config.appskey);
                }
            }
            break;

        case 6:
            // Set device class (AT+CLASS=A/B/C)
            at_error = !send_AT(AT_SET_CLASS, std::string(1, static_cast<char>(m_config.device_class)));
            break;

        case 7:
            // Set data rate
            at_error = !send_AT(AT_SET_DR, std::to_string(m_config.dr));
            break;

        case 8:
            // Set ADR
            at_error = !send_AT(AT_SET_ADR, std::to_string(m_config.adr));
            break;

        case 9:
            // Set TX power
            at_error = !send_AT(AT_SET_TXP, std::to_string(m_config.txp));
            break;

        case 10:
            // Set confirmed/unconfirmed mode
            at_error = !send_AT(AT_SET_CFM, std::to_string(m_config.cfm));
            break;

        case 11:
            // Set retry count (only relevant if CFM=1)
            if (m_config.cfm) {
                at_error = !send_AT(AT_SET_RETY, std::to_string(LoRa::DEFAULT_RETY));
            }
            break;

        case 12:
            // Apply user-configured low-power mode (LORA_LP_MODE / LRP15):
            //   0 = shutdown (0 µA idle, ~2.5 s reboot on next TX)
            //   1 = standby Stop2 (~1.7 µA idle, ~10 ms wake)
            at_error = !send_AT(AT_SET_LPM, std::to_string(m_config.lp_mode));
            break;

        case 13:
            // Full config done — mark configured, jump to common path
            m_is_configured = true;
            m_config_step = 99;  // Will be incremented to 100
            break;

        // ==== Common path (runs every boot) ====

        case 100:
            // Read Device EUI from module and save to config store
            at_error = !send_AT(AT_GET_DEVEUI);
            if (!at_error) {
                m_config.deveui = m_lora_comm.m_last_value;
                DEBUG_INFO("LoRaDevice: DEVEUI=%s", m_config.deveui.c_str());
                configuration_store->write_param(ParamID::LORA_DEVEUI, m_config.deveui);
            }
            break;

        case 101:
            // Check join status / initiate join
            if (m_config.njm == 0) {
                DEBUG_INFO("LoRaDevice: ABP mode - no join required");
                m_joined = true;
                LORA_STATE_CHANGE(configure, idle);
                return;
            }
            at_error = !send_AT(AT_GET_NJS);
            if (!at_error && m_lora_comm.m_last_value == "1") {
                DEBUG_INFO("LoRaDevice: already joined from previous session");
                m_joined = true;
                LORA_STATE_CHANGE(configure, idle);
                return;
            }
            LORA_STATE_CHANGE(configure, joining);
            return;

        default:
            LORA_STATE_CHANGE(configure, error);
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
//   - Timeout after 90 seconds
// ========================================================================

void LoRaDevice::state_joining_enter()
{
    DEBUG_INFO("LoRaDevice::state_joining_enter: NJM=%u", m_config.njm);

    // ABP mode should never reach here, but handle gracefully
    if (m_config.njm == 0) {
        DEBUG_INFO("LoRaDevice: ABP mode - skipping join");
        m_joined = true;
        return;
    }

    m_joined = false;
    m_join_failed = false;
    m_is_error = false;

    // AT+JOIN=<start>:<auto_join>:<interval>:<attempts>
    // start=1, auto_join=0, interval=10s, attempts=8
    // - auto_join=0: no persistent rejoin on failure (saves battery)
    // - attempts=8: ~80s max join window, suitable for surfacing duration
    if (!send_AT(AT_JOIN, "1:0:10:8")) {
        DEBUG_ERROR("LoRaDevice: AT+JOIN command failed");
        m_is_error = true;
        return;
    }

    // Timeout for join procedure (90 seconds = 8 attempts * 10s + margin)
    initiate_timeout(90000);
}

void LoRaDevice::state_joining()
{
    m_lora_comm.process_rx();  // Process any ISR-buffered async events

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
    } else if (m_config.lp_mode == 1) {
        // Standby: module stays powered, LPM Stop2 ~1.7µA, wake ~10ms
        LORA_STATE_CHANGE(idle, standby);
    } else {
        // Shutdown: full power off = 0µA, but 2.5s reboot on next TX
        LORA_STATE_CHANGE(idle, power_off);
    }
}

void LoRaDevice::state_idle_exit() {
    ;
}

// ========================================================================
// State: standby
//   - Module powered, UART initialized, LPM Stop2 active (~1.7µA)
//   - RAK3172 auto-sleeps via LPM=1 when no UART activity
//   - Wake on UART RX (any byte wakes the MCU from Stop2 in ~15µs)
//   - Waiting for send() to be called
// ========================================================================

void LoRaDevice::state_standby_enter() {
    DEBUG_INFO("LoRaDevice: standby (LPM Stop2 ~1.7uA)");
}

void LoRaDevice::state_standby() {
    // Nothing to do - module is sleeping, waiting for send() call
}

void LoRaDevice::state_standby_exit() {
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
    if (!send_AT(AT_SEND, send_params)) {
        DEBUG_ERROR("LoRaDevice: AT+SEND command failed");
        m_is_error = true;
        return;
    }

    notify(KineisEventTxStarted({}));

    // TX timeout based on data rate — low DRs have very long air times
    // EU868 worst case: DR0 (SF12/125kHz) ~2.5s per byte, 51 bytes max = ~130s
    // Add margin for RX windows (1s + 2s) and processing
    static constexpr unsigned int tx_timeout_ms[] = {
        150000,  // DR0 (SF12) — ~130s air time + RX windows
        80000,   // DR1 (SF11) — ~65s air time
        45000,   // DR2 (SF10) — ~35s air time
        25000,   // DR3 (SF9)  — ~18s air time
        15000,   // DR4 (SF8)  — ~10s air time
        10000,   // DR5 (SF7)  — ~5s air time
    };
    unsigned int dr = m_config.dr;
    unsigned int timeout = (dr < 6) ? tx_timeout_ms[dr] : 10000;
    initiate_timeout(timeout);
}

void LoRaDevice::state_transmit()
{
    m_lora_comm.process_rx();  // Process any ISR-buffered async events

    if (m_tx_done)
    {
        m_tx_done = false;
        m_consecutive_errors = 0;  // Reset error counter on successful TX
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
    m_consecutive_errors++;
    if (m_consecutive_errors <= MAX_CONSECUTIVE_ERRORS) {
        // Transient error — retry from idle (module may still be responsive)
        DEBUG_WARN("LoRaDevice::state_error: retry %u/%u", m_consecutive_errors, MAX_CONSECUTIVE_ERRORS);
        m_is_error = false;
        LORA_STATE_CHANGE(error, idle);
    } else {
        // Persistent error — power cycle
        DEBUG_ERROR("LoRaDevice::state_error: max retries reached, powering off");
        m_consecutive_errors = 0;
        notify(KineisEventDeviceError({}));
        LORA_STATE_CHANGE(error, power_off);
    }
}

void LoRaDevice::state_error_exit() {
    ;
}

// ========================================================================
// Config store integration
// ========================================================================

/// @brief Load LoRaWAN configuration from ConfigStore.
void LoRaDevice::load_config_from_store()
{
    m_config.deveui  = configuration_store->read_param<std::string>(ParamID::LORA_DEVEUI);
    m_config.appeui  = configuration_store->read_param<std::string>(ParamID::LORA_APPEUI);
    m_config.appkey  = configuration_store->read_param<std::string>(ParamID::LORA_APPKEY);
    m_config.devaddr = configuration_store->read_param<std::string>(ParamID::LORA_DEVADDR);
    m_config.appskey = configuration_store->read_param<std::string>(ParamID::LORA_APPSKEY);
    m_config.nwkskey = configuration_store->read_param<std::string>(ParamID::LORA_NWKSKEY);
    m_config.njm     = static_cast<uint8_t>(configuration_store->read_param<unsigned int>(ParamID::LORA_NJM));
    m_config.band    = static_cast<uint8_t>(configuration_store->read_param<unsigned int>(ParamID::LORA_BAND));
    m_config.dr      = static_cast<uint8_t>(configuration_store->read_param<unsigned int>(ParamID::LORA_DR));
    m_config.adr     = configuration_store->read_param<bool>(ParamID::LORA_ADR) ? 1 : 0;
    m_config.txp     = static_cast<uint8_t>(configuration_store->read_param<unsigned int>(ParamID::LORA_TXP));
    m_config.cfm     = configuration_store->read_param<bool>(ParamID::LORA_CFM) ? 1 : 0;
    m_config.fport   = static_cast<uint8_t>(configuration_store->read_param<unsigned int>(ParamID::LORA_FPORT));
    m_config.lp_mode = static_cast<uint8_t>(configuration_store->read_param<unsigned int>(ParamID::LORA_LP_MODE));

    // Class: stored as 0/1/2 → convert to 'A'/'B'/'C'
    unsigned int lora_class = configuration_store->read_param<unsigned int>(ParamID::LORA_CLASS);
    m_config.device_class = 'A' + static_cast<uint8_t>(lora_class);

    // Compute the largest packet the current config will ever produce, and bump
    // LORA_DR to the minimum rate that guarantees it fits. This replaces the
    // previous "silent skip" behavior: every packet the service emits is now
    // transmittable. Only applies to packet types actually enabled by config.
    //
    // EU868 max MAC payload: DR0-2 = 51 B, DR3 = 115 B, DR4-5 = 222 B.
    unsigned int max_packet_bytes = 2;  // Status packet floor (14 bits)

    // CloudLocate MEAS50 is only emitted when FASTLOC_MODE=CLOUDLOCATE + fmt=MEAS50
    unsigned int fastloc_mode = configuration_store->read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE);
    unsigned int cl_format    = configuration_store->read_param<unsigned int>(ParamID::GNSS_CLOUDLOCATE_FORMAT);
    if (fastloc_mode == (unsigned int)BaseFastlocMode::CLOUDLOCATE &&
        cl_format    == (unsigned int)BaseCloudLocateFormat::MEAS50) {
        // type(3)+fmt(2)+flags(4)+volt(7)+50B = 16+400 = 416 bits = 52 B
        if (52 > max_packet_bytes) max_packet_bytes = 52;
    }

    // Sensor packet: header + mask + always-present GPS_FULL + enabled sensors
    unsigned int sensor_bits = 14 + 6 + 86;  // header + mask + GPS_FULL
    if (fastloc_mode >= (unsigned int)BaseFastlocMode::DEGRADED_PVT)
        sensor_bits += 60;                   // BITS_FASTLOC_QUALITY
#if ENABLE_ALS_SENSOR
    if (configuration_store->read_param<bool>(ParamID::ALS_SENSOR_ENABLE))
        sensor_bits += 17;
#endif
#if ENABLE_PH_SENSOR
    if (configuration_store->read_param<bool>(ParamID::PH_SENSOR_ENABLE))
        sensor_bits += 14;
#endif
#if ENABLE_PRESSURE_SENSOR
    if (configuration_store->read_param<bool>(ParamID::PRESSURE_SENSOR_ENABLE))
        sensor_bits += 29;                   // pressure + press_temp
#endif
    // SEA_TEMP and THERMISTOR share the same 14-bit slot (mutually exclusive at build)
#if ENABLE_SEA_TEMP_SENSOR
    if (configuration_store->read_param<bool>(ParamID::SEA_TEMP_SENSOR_ENABLE))
        sensor_bits += 14;
#endif
#if ENABLE_THERMISTOR_SENSOR
    if (configuration_store->read_param<bool>(ParamID::THERMISTOR_SENSOR_ENABLE))
        sensor_bits += 14;
#endif
#if ENABLE_AXL_SENSOR
    if (configuration_store->read_param<bool>(ParamID::AXL_SENSOR_ENABLE))
        sensor_bits += 67;                   // AXL temp + 3 axis + act
#endif
    unsigned int sensor_bytes = (sensor_bits + 7) / 8;
    if (sensor_bytes > max_packet_bytes) max_packet_bytes = sensor_bytes;

    // GPS multi is always size-clamped by max_gps_entries(), so it never
    // exceeds the current DR — no contribution to max_packet_bytes.

    // Map byte threshold → min required DR
    uint8_t min_dr = 0;
    if      (max_packet_bytes > 115) min_dr = 4;  // DR4 (222 B)
    else if (max_packet_bytes > 51)  min_dr = 3;  // DR3 (115 B)
    else                             min_dr = 0;  // any DR

    if (m_config.dr < min_dr) {
        DEBUG_WARN("LoRaDevice: max packet %u B requires DR≥%u — bumping LORA_DR %u → %u",
                   max_packet_bytes, min_dr, m_config.dr, min_dr);
        m_config.dr = min_dr;
    }

    DEBUG_INFO("LoRaDevice: config NJM=%u BAND=%u DR=%u ADR=%u TXP=%u CFM=%u FPORT=%u CLASS=%c LP=%u",
               m_config.njm, m_config.band, m_config.dr, m_config.adr,
               m_config.txp, m_config.cfm, m_config.fport, m_config.device_class, m_config.lp_mode);
    if (!m_config.deveui.empty())
        DEBUG_INFO("LoRaDevice: DEVEUI=%s", m_config.deveui.c_str());
    if (!m_config.appeui.empty())
        DEBUG_INFO("LoRaDevice: APPEUI=%s", m_config.appeui.c_str());
}

/// @brief Read LoRa credentials from RAK3172 module via AT commands.
/// Module must be powered on (idle/standby/configure state).
LoRaDevice::LoRaCredentials LoRaDevice::read_lora_credentials()
{
    LoRaCredentials creds;
    creds.read_ok = false;

    if (m_state == State::power_off) {
        DEBUG_WARN("LoRaDevice::read_lora_credentials: module is powered off");
        return creds;
    }

    // Read NJM
    if (send_AT(AT_GET_NJM)) {
        creds.njm = static_cast<uint8_t>(std::stoul(m_lora_comm.m_last_value));
    } else {
        DEBUG_WARN("LoRaDevice::read_lora_credentials: failed to read NJM");
        return creds;
    }

    // Read DEVEUI
    if (send_AT(AT_GET_DEVEUI)) {
        creds.deveui = m_lora_comm.m_last_value;
    }

    if (creds.njm == 1) {
        // OTAA: read APPEUI + APPKEY
        if (send_AT(AT_GET_APPEUI)) {
            creds.appeui = m_lora_comm.m_last_value;
        }
        if (send_AT(AT_GET_APPKEY)) {
            creds.appkey = m_lora_comm.m_last_value;
        }
    } else {
        // ABP: read DEVADDR (NWKSKEY/APPSKEY are write-only on RAK3172)
        if (send_AT(AT_GET_DEVADDR)) {
            creds.devaddr = m_lora_comm.m_last_value;
        }
    }

    creds.read_ok = true;
    return creds;
}

/// @brief Write LoRa credentials from the config store to the RAK3172 module.
/// Module must be powered on (idle/standby/configure state). Caller must reload
/// the module config (e.g. via read_lora_credentials()) afterwards to refresh cache.
/// @note Changing NJM triggers a RAK3172 reboot; subsequent AT writes in the same
///       call will fail. Callers should only change NJM outside this path or
///       re-invoke after the reboot completes.
bool LoRaDevice::write_credentials_from_config()
{
    if (m_state == State::power_off) {
        DEBUG_WARN("LoRaDevice::write_credentials_from_config: module is powered off");
        return false;
    }

    // Load latest values from config store into m_config (keeps local cache coherent)
    load_config_from_store();

    // Set join mode first — module may reboot if NJM changes
    if (!send_AT(AT_SET_NJM, std::to_string(m_config.njm))) {
        DEBUG_ERROR("LoRaDevice::write_credentials_from_config: AT_SET_NJM failed");
        return false;
    }

    if (!m_config.deveui.empty()) {
        if (m_config.deveui.size() != 16) {
            DEBUG_ERROR("LoRaDevice::write_credentials_from_config: invalid DEVEUI length %u",
                        static_cast<unsigned>(m_config.deveui.size()));
            return false;
        }
        if (!send_AT(AT_SET_DEVEUI, m_config.deveui)) {
            DEBUG_ERROR("LoRaDevice::write_credentials_from_config: AT_SET_DEVEUI failed");
            return false;
        }
    }

    if (m_config.njm == 1) {
        // OTAA: APPEUI + APPKEY
        if (!m_config.appeui.empty()) {
            if (m_config.appeui.size() != 16) {
                DEBUG_ERROR("LoRaDevice::write_credentials_from_config: invalid APPEUI length %u",
                            static_cast<unsigned>(m_config.appeui.size()));
                return false;
            }
            if (!send_AT(AT_SET_APPEUI, m_config.appeui)) {
                DEBUG_ERROR("LoRaDevice::write_credentials_from_config: AT_SET_APPEUI failed");
                return false;
            }
        }
        if (!m_config.appkey.empty()) {
            if (m_config.appkey.size() != 32) {
                DEBUG_ERROR("LoRaDevice::write_credentials_from_config: invalid APPKEY length %u",
                            static_cast<unsigned>(m_config.appkey.size()));
                return false;
            }
            if (!send_AT(AT_SET_APPKEY, m_config.appkey)) {
                DEBUG_ERROR("LoRaDevice::write_credentials_from_config: AT_SET_APPKEY failed");
                return false;
            }
        }
    } else {
        // ABP: DEVADDR (session keys NWKSKEY/APPSKEY are write-only, skipped here)
        if (!m_config.devaddr.empty()) {
            if (m_config.devaddr.size() != 8) {
                DEBUG_ERROR("LoRaDevice::write_credentials_from_config: invalid DEVADDR length %u",
                            static_cast<unsigned>(m_config.devaddr.size()));
                return false;
            }
            if (!send_AT(AT_SET_DEVADDR, m_config.devaddr)) {
                DEBUG_ERROR("LoRaDevice::write_credentials_from_config: AT_SET_DEVADDR failed");
                return false;
            }
        }
    }

    DEBUG_INFO("LoRaDevice::write_credentials_from_config: credentials written (njm=%u)", m_config.njm);
    return true;
}

/// @brief Start RF Continuous Wave on the RAK3172 (AT+CW=freq,pwr,duration).
bool LoRaDevice::cw_start(uint32_t freq_hz, uint16_t power_dbm, uint16_t duration_s)
{
    if (m_state == State::power_off) {
        DEBUG_WARN("LoRaDevice::cw_start: module is powered off");
        return false;
    }
    if (duration_s == 0) duration_s = 60;   // RUI3 requires a duration
    std::string params = std::to_string(freq_hz) + "," +
                         std::to_string(power_dbm) + "," +
                         std::to_string(duration_s);
    DEBUG_INFO("LoRaDevice::cw_start: %s", params.c_str());
    return send_AT(AT_SET_CW, params);
}

/// @brief Stop CW — RUI3 does not expose AT+CW stop; reset the module.
bool LoRaDevice::cw_stop()
{
    DEBUG_INFO("LoRaDevice::cw_stop (ATZ reset)");
    return send_AT(AT_RESET);
}

/// @brief Read RAK3172 RUI3 firmware version. Module must be powered on.
std::string LoRaDevice::get_firmware_version()
{
    if (m_state == State::power_off) {
        DEBUG_WARN("LoRaDevice::get_firmware_version: module is powered off");
        return "";
    }
    if (!send_AT(AT_GET_VER)) {
        DEBUG_WARN("LoRaDevice::get_firmware_version: AT+VER=? failed");
        return "";
    }
    return m_lora_comm.m_last_value;
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
