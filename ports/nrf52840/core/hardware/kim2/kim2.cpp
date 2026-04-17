/**
 * @file kim2.cpp
 * @brief KIM2 satellite module — state machine, AT command flow, TX management.
 */

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
#include <cstdint>
#include <string>

using namespace KIM2;

/// @name Payload size limits per modulation (bits)
/// @{
static constexpr uint16_t LDK_MAX_LENGTH_BITS  = 16 * 8;
static constexpr uint16_t LDA2_MAX_LENGTH_BITS  = 24 * 8;
static constexpr uint16_t VLDA4_MAX_LENGTH_BITS  = 3 * 8;
/// @}

/// @name Timing constants (ms)
/// @{
static constexpr uint16_t KIM2_DELAY_POWER_ON_MS = 500;   ///< Wait after power-on before ping
static constexpr uint16_t KIM2_DELAY_POLL_MS     = 100;   ///< TX poll interval
static constexpr uint32_t KIM2_TX_TIMEOUT_MS     = 60000; ///< Max time for TX complete
static constexpr uint16_t KIM2_IDLE_TICK_MS      = 100;   ///< Idle state poll interval
/// @brief Delay after AT+RCONF= so the module can process the new radio config
///        (analogous to SMD's 80ms STM32 RCONF processing window).
static constexpr uint16_t KIM2_DELAY_AFTER_RCONF_MS = 80;
/// @brief Delay after AT+KMAC=1 so the MAC layer can re-initialize with the
///        new RCONF. Sending AT+TX before the MAC is ready causes the module
///        to reject the frame (observed as +ERROR=5 / KNS_STATUS_BAD_LEN).
///        SMD uses 200 ms here; same value reused for KIM2.
static constexpr uint16_t KIM2_DELAY_AFTER_KMAC_MS  = 200;
/// @brief Timeout for the synchronous +OK ACK of AT+TX. The KIM2 firmware
///        can take up to ~3 s to emit +OK (even though the +TX=<status>
///        completion arrives later via the async path). The 1 s default is
///        too short and causes a spurious "send_AT: timeout" warning while
///        the TX is actually succeeding. Match SMD AT's 5 s budget.
static constexpr uint16_t KIM2_TX_ACK_TIMEOUT_MS    = 5000;
/// @}

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
    stop_bridge();
    power_off_immediate();
}

// ============================================================================
// Bridge/passthrough mode — raw UART access for DTE KIMBR command
// ============================================================================

bool KIM2Device::start_bridge(KIM2Comm::PassthroughCallback rx_callback)
{
    if (m_bridge_active)
        return true;

    // Cancel any pending state machine task / timeout — bridge owns the UART
    cancel_timeout();
    system_scheduler->cancel_task(m_task);

    // Power on module hardware if not already on (required for UART communication).
    // If already powered (state=init/idle/transmit), we reuse the current GPIO state.
    bool was_off = (m_state == KIM2ManagerState::power_off);
    if (was_off) {
        GPIOPins::set(SAT_PWR_EN);
        GPIOPins::set(SAT_EXTWAKEUP);
    }

    // Ensure UART is initialized (may be off if no TX happened yet)
    m_kim2_comm.init();

    // Wait for module boot if we just powered it on
    if (was_off) {
        PMU::delay_ms(KIM2_DELAY_POWER_ON_MS);
    }

    // Enable passthrough: raw UART RX goes to callback (line-framed with CRLF)
    m_kim2_comm.set_passthrough(true, rx_callback);
    m_bridge_active = true;

    DEBUG_INFO("KIM2Device: bridge mode ACTIVE (was_off=%u)", was_off ? 1 : 0);
    return true;
}

void KIM2Device::stop_bridge()
{
    if (!m_bridge_active)
        return;

    m_kim2_comm.set_passthrough(false);
    m_bridge_active = false;

    DEBUG_INFO("KIM2Device: bridge mode STOPPED");

    // Force power_off — bridge usage may have left the module in an arbitrary
    // AT protocol state. Next send() will do a clean power-on cycle via
    // start_device(). This avoids resuming into stale idle/transmit states.
    cancel_timeout();
    system_scheduler->cancel_task(m_task);
    m_kim2_comm.unsubscribe(*this);
    m_state = KIM2ManagerState::power_off;
    state_power_off_enter();  // Deinit UART, cut GPIOs, clear buffers
}

bool KIM2Device::bridge_send(const uint8_t* data, size_t len)
{
    if (!m_bridge_active)
        return false;
    return m_kim2_comm.send_raw_data(data, len);
}

void KIM2Device::bridge_process_rx()
{
    if (m_bridge_active)
        m_kim2_comm.process_rx();
}

// ============================================================================
// KineisDevice interface
// ============================================================================

/// @brief Pack payload with modulation-specific stuffing bits, queue for TX.
/// @param mode            Modulation (LDK, LDA2, VLDA4).
/// @param user_payload    Raw payload bytes.
/// @param payload_length  Payload length in bits.
void KIM2Device::send(const KineisModulation mode, const KineisPacket& user_payload, const unsigned int payload_length)
{
    // Reject TX while bridge is active — bridge owns the UART exclusively.
    // Stop the bridge first via stop_bridge() if a TX is needed.
    if (m_bridge_active) {
        DEBUG_WARN("KIM2Device::send: rejected — bridge mode active");
        notify(KineisEventDeviceError({}));
        return;
    }

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
    // KIM2 has no configurable TCXO AT command — warmup is handled
    // internally by the module firmware. The observed ~3 s delay between
    // AT+TX and the +OK ACK is the module's internal TCXO warmup + actual
    // RF transmission, which is why KIM2_TX_ACK_TIMEOUT_MS is set to 5 s.
}

// ============================================================================
// Runtime modulation switching
// ============================================================================

/// @brief Runtime modulation switch: write RCONF then reload KMAC.
/// @param mode       Target modulation.
/// @param rconf_hex  32-char hex RCONF string for the target modulation.
/// @return true on success, false on AT command failure.
bool KIM2Device::switch_modulation(KineisModulation mode, const std::string& rconf_hex) {
    if (mode == m_current_rconf_mode) {
        DEBUG_TRACE("KIM2Device::%s: already in target modulation %d", __func__, static_cast<int>(mode));
        return true;
    }

    DEBUG_INFO("KIM2Device::%s: switching %d -> %d", __func__, static_cast<int>(m_current_rconf_mode), static_cast<int>(mode));

    if (rconf_hex.size() != 32) {
        DEBUG_ERROR("KIM2Device::%s: invalid RCONF hex length %u", __func__, static_cast<unsigned>(rconf_hex.size()));
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
    PMU::delay_ms(KIM2_DELAY_AFTER_RCONF_MS);

    if (!send_AT(AT_SET_KMAC_BASIC)) {
        DEBUG_ERROR("KIM2Device::%s: failed to reload KMAC", __func__);
        return false;
    }
    PMU::delay_ms(KIM2_DELAY_AFTER_KMAC_MS);

    m_current_rconf_mode = mode;
    DEBUG_INFO("KIM2Device::%s: modulation switched OK", __func__);
    return true;
}

KineisModulation KIM2Device::get_current_modulation() const {
    return m_current_rconf_mode;
}

// ============================================================================
// Credential read-back (SATVF)
// ============================================================================

/// @brief Actively read ID, address and decoded RCONF from the KIM2 module.
///
/// When the device is stopped, this does a synchronous power-on / ping /
/// power-off cycle so SATVF always returns fresh values (not stale cache).
/// If the state machine is currently running (init/idle/transmit) we reuse
/// the live UART session and leave power state untouched.
///
/// @note KIM2 exposes no AT+SECKEY command (unlike SMD), so @p seckey is
///       always cleared. @p radioconf carries the decoded
///       "freq_min,freq_max,mod_type,rf_level" string from AT+RCONF=? —
///       useful for SATVF diagnostics (e.g. confirming the active modulation).
void KIM2Device::read_credentials(unsigned int *dec_id, unsigned int *address,
                                   std::string *seckey, std::string *radioconf)
{
    DEBUG_TRACE("KIM2Device::read_credentials");

    // Zero outputs up-front so callers see a clean state on early return.
    if (dec_id)    *dec_id = 0;
    if (address)   *address = 0;
    if (seckey)    seckey->clear();
    if (radioconf) radioconf->clear();

    // Bridge owns the UART — can't issue AT reads. Fall back to cached values.
    if (m_bridge_active) {
        DEBUG_WARN("KIM2Device::read_credentials: bridge active, returning cached values");
        if (dec_id)    *dec_id    = m_kim2_comm.m_kineis_id;
        if (address)   *address   = m_kim2_comm.m_hex_addr;
        if (radioconf) *radioconf = m_kim2_comm.m_rconf_info;
        return;
    }

    const bool was_off = (m_state == KIM2ManagerState::power_off);

    if (was_off) {
        // Freeze the state machine so it doesn't race with our synchronous reads.
        cancel_timeout();
        system_scheduler->cancel_task(m_task);

        GPIOPins::set(SAT_PWR_EN);
        GPIOPins::set(SAT_EXTWAKEUP);
        m_kim2_comm.init();
        m_kim2_comm.subscribe(*this);
        PMU::delay_ms(KIM2_DELAY_POWER_ON_MS);

        if (!send_AT(AT_PING)) {
            DEBUG_WARN("KIM2Device::read_credentials: ping failed — module not responding");
            m_kim2_comm.unsubscribe(*this);
            m_kim2_comm.deinit();
            GPIOPins::clear(SAT_EXTWAKEUP);
            GPIOPins::clear(SAT_PWR_EN);
            return;
        }
    }

    if (send_AT(AT_GET_ID)) {
        if (dec_id) *dec_id = m_kim2_comm.m_kineis_id;
    } else {
        DEBUG_WARN("KIM2Device::read_credentials: AT+ID=? failed");
    }

    if (send_AT(AT_GET_ADDR)) {
        if (address) *address = m_kim2_comm.m_hex_addr;
    } else {
        DEBUG_WARN("KIM2Device::read_credentials: AT+ADDR=? failed");
    }

    if (send_AT(AT_GET_RCONF)) {
        if (radioconf) *radioconf = m_kim2_comm.m_rconf_info;
    } else {
        DEBUG_WARN("KIM2Device::read_credentials: AT+RCONF=? failed");
    }

    if (was_off) {
        m_kim2_comm.unsubscribe(*this);
        m_kim2_comm.deinit();
        GPIOPins::clear(SAT_EXTWAKEUP);
        GPIOPins::clear(SAT_PWR_EN);
    }
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

/// @brief Send AT command and busy-wait for +OK or +ERROR response.
/// @param cmd         AT command type.
/// @param params      Optional parameter string.
/// @param timeout_ms  Max wait time in ms (default 1000).
/// @return true if +OK received before timeout.
/// @note Enforces the KIM2 Integration Manual v0.8 timing constraint:
///       "User shall wait at minimum 10ms before sending a new command
///       after previous is completed." The 10ms gap is applied after the
///       response is received so the next send_AT() call is already clear
///       to transmit.
bool KIM2Device::send_AT(ATCmd cmd, const std::optional<std::string>& params, uint16_t timeout_ms)
{
    m_cmd_is_ok = false;
    m_is_error = false;

    m_kim2_comm.send(cmd, params);

    // Busy-wait for UART response — blocking but bounded.
    // AT protocol is synchronous; async would require full state machine rewrite.
    while (!m_cmd_is_ok && !m_is_error && timeout_ms != 0) {
        PMU::delay_ms(1);
        m_kim2_comm.process_rx();  // Drain ISR buffer → parse → notify
        timeout_ms--;
    }

    if(timeout_ms == 0) {
        DEBUG_WARN("KIM2Device::send_AT: timeout (cmd=%d)", static_cast<int>(cmd));
    }

    // Mandatory inter-command gap (KIM2 manual v0.8, §3.A timing constraints).
    PMU::delay_ms(10);

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

/// @brief Power on module and start state machine (no-op if already running).
void KIM2Device::start_device()
{
    if (m_state != KIM2ManagerState::power_off) {
        m_stopping = false;
        DEBUG_TRACE("KIM2Device::start: already running in state=%u", static_cast<unsigned int>(m_state));
        run_state_machine(0);
        return;
    }

    m_stopping = false;
    KIM2_STATE_CHANGE(power_off, power_on);
}

/// @brief Immediate power off — cancel tasks, uninit UART, cut GPIO power.
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

/// @brief Dispatch to the current state handler.
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

/// @brief Schedule the next state machine tick after delay_ms.
/// @param delay_ms  Delay before next tick (default 100 ms).
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

/// @brief Power off: uninit UART, cut GPIO power, clear buffers.
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

/// @brief Power on: enable SAT_PWR_EN + EXTWAKEUP, init UART, wait 500 ms.
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

/// @brief Init: read ID/ADDR, write RCONF + KMAC → transition to idle.
/// @note Per KIM2 Integration Manual v0.8, RCONF is kept in RAM and must be
///       reapplied after every power-on (SAVE_RCONF is discouraged for
///       normal use). KMAC=1 (basic MAC profile) must be set after RCONF
///       and before any AT+TX.
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

    // Configure RCONF for the target modulation, then start basic MAC profile.
    // When adaptive modulation is ON, use the per-modulation RCONF matching
    // m_current_rconf_mode (set by switch_modulation() while device was OFF).
    // When adaptive is OFF, use the master RCONF entered by the user.
    bool adaptive = configuration_store->read_param<bool>(ParamID::ARGOS_ADAPTIVE_MODULATION);
    std::string rconf = load_rconf_for_mode(m_current_rconf_mode);
    if (adaptive) {
        DEBUG_INFO("KIM2Device::state_init: adaptive ON, using RCONF for mode %d", static_cast<int>(m_current_rconf_mode));
    }
    if (rconf.empty()) {
        rconf = "03921fb104b92859209b18abd009de96"; // Default: ESS4 - LDK - 27dBm
        m_current_rconf_mode = KineisModulation::LDK; // default matches LDK
        DEBUG_WARN("KIM2Device::state_init: RCONF empty, using default LDK fallback");
    }

    if(!send_AT(AT_SET_RCONF, rconf))
    {
        DEBUG_ERROR("KIM2Device::state_init: can not set RCONF");
        KIM2_STATE_CHANGE(init, error);
        return;
    }
    PMU::delay_ms(KIM2_DELAY_AFTER_RCONF_MS);

    // Diagnostic: query what the module actually stored (freq_min,freq_max,mod_type,rf_level).
    // Useful to confirm the encrypted RCONF decoded to the expected modulation.
    if (send_AT(AT_GET_RCONF)) {
        DEBUG_INFO("KIM2Device::state_init: module RCONF=%s", m_kim2_comm.m_rconf_info.c_str());
    } else {
        DEBUG_WARN("KIM2Device::state_init: AT+RCONF=? query failed");
    }

    if(!send_AT(AT_SET_KMAC_BASIC))
    {
        DEBUG_ERROR("KIM2Device::state_init: can not set KMAC");
        KIM2_STATE_CHANGE(init, error);
        return;
    }
    PMU::delay_ms(KIM2_DELAY_AFTER_KMAC_MS);
    DEBUG_TRACE("KIM2Device::state_init RCONF set and KMAC=1 activated");
    if (!adaptive) {
        configuration_store->write_param(ParamID::ARGOS_RADIOCONF, rconf);
    }

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

/// @brief Idle: check for pending TX, or power off after timeout.
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

/// @brief Start TX: send AT+TX, set timeout, start polling for +TX= response.
void KIM2Device::state_transmit_enter()
{
    DEBUG_INFO("KIM2Device::state_transmit_enter: mode=%u", static_cast<unsigned int>(m_tx_mode));

    // Auto-recovery: if the module's RCONF doesn't match the requested TX
    // modulation, rewrite it before AT+TX. Otherwise KIM rejects the TX with
    // +ERROR=5 (KNS_STATUS_BAD_LEN, per v0.8) because the payload length no
    // longer matches the active modulation's allowed sizes. Happens when
    // ensure_modulation() wasn't called by the service or when state_init
    // fell back to a default RCONF for a different mode than the caller wants.
    if (m_tx_mode != m_current_rconf_mode) {
        DEBUG_WARN("KIM2Device::state_transmit_enter: TX mode %u != RCONF mode %u, realigning",
            static_cast<unsigned int>(m_tx_mode), static_cast<unsigned int>(m_current_rconf_mode));

        std::string rconf = load_rconf_for_mode(m_tx_mode);
        if (rconf.size() != 32) {
            DEBUG_ERROR("KIM2Device::state_transmit_enter: no valid RCONF for mode %u (len=%u) — aborting TX",
                static_cast<unsigned int>(m_tx_mode), static_cast<unsigned int>(rconf.size()));
            m_tx_buffer.clear();
            KIM2_STATE_CHANGE(transmit, error);
            return;
        }
        if (!send_AT(AT_SET_RCONF, rconf)) {
            DEBUG_ERROR("KIM2Device::state_transmit_enter: RCONF write failed — aborting TX");
            m_tx_buffer.clear();
            KIM2_STATE_CHANGE(transmit, error);
            return;
        }
        PMU::delay_ms(KIM2_DELAY_AFTER_RCONF_MS);

        // Diagnostic: log what the module decoded from the RCONF hex.
        if (send_AT(AT_GET_RCONF)) {
            DEBUG_INFO("KIM2Device::state_transmit_enter: module RCONF=%s",
                m_kim2_comm.m_rconf_info.c_str());
        }

        if (!send_AT(AT_SET_KMAC_BASIC)) {
            DEBUG_ERROR("KIM2Device::state_transmit_enter: KMAC reload failed — aborting TX");
            m_tx_buffer.clear();
            KIM2_STATE_CHANGE(transmit, error);
            return;
        }
        PMU::delay_ms(KIM2_DELAY_AFTER_KMAC_MS);
        m_current_rconf_mode = m_tx_mode;
        DEBUG_INFO("KIM2Device::state_transmit_enter: RCONF realigned to mode %u",
            static_cast<unsigned int>(m_tx_mode));
    }

    m_tx_done = false;
    m_is_error = false;
    DEBUG_INFO("KIM2Device::state_transmit_enter: AT+TX=%s (hex_len=%u bytes=%u)",
        m_tx_buffer.c_str(),
        static_cast<unsigned>(m_tx_buffer.size()),
        static_cast<unsigned>(m_tx_buffer.size() / 2));
    send_AT(AT_TX, m_tx_buffer, KIM2_TX_ACK_TIMEOUT_MS);
    notify(KineisEventTxStarted({}));
    initiate_timeout(KIM2_TX_TIMEOUT_MS);

    // Poll counter for TX completion
    m_tx_poll_counter = KIM2_TX_TIMEOUT_MS / KIM2_DELAY_POLL_MS;
}

/// @brief Poll TX: check for +TX= done, error, or poll timeout.
void KIM2Device::state_transmit()
{
    m_kim2_comm.process_rx();  // Drain ISR buffer for async TX events

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

/// @brief Error: notify listener and transition to power_off.
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

// ============================================================================
// Internal helpers
// ============================================================================

std::string KIM2Device::load_rconf_for_mode(KineisModulation mode)
{
    bool adaptive = configuration_store->read_param<bool>(ParamID::ARGOS_ADAPTIVE_MODULATION);
    if (!adaptive) {
        return configuration_store->read_param<std::string>(ParamID::ARGOS_RADIOCONF);
    }
    switch (mode) {
        case KineisModulation::LDK:
            return configuration_store->read_param<std::string>(ParamID::ARGOS_RADIOCONF_LDK);
        case KineisModulation::VLDA4:
            return configuration_store->read_param<std::string>(ParamID::ARGOS_RADIOCONF_VLDA4);
        case KineisModulation::LDA2:
        default:
            return configuration_store->read_param<std::string>(ParamID::ARGOS_RADIOCONF_LDA2);
    }
}
