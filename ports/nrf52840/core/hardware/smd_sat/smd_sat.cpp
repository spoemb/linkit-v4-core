#include <stdint.h>
#include <cstring>
#include <cmath>

#include "bsp.hpp"
#include "error.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include "scheduler.hpp"
#include "timer.hpp"
#include "binascii.hpp"
#include "smd_sat.hpp"
#include "pmu.hpp"

#include "nrf_delay.h"
#include "nrf_gpio.h"

extern Scheduler *system_scheduler;
extern Timer *system_timer;

#include "config_store.hpp"
extern ConfigurationStore *configuration_store;

// ============================================================================
// Constructor / Destructor
// ============================================================================

SmdSat::SmdSat(SmdSatCmd& cmd, unsigned int idle_shutdown_ms)
	: m_cmd(cmd)
{
	DEBUG_TRACE("SmdSat::%s",__func__);
	set_idle_timeout(idle_shutdown_ms);
	m_packet_buffer.clear();
	m_modulation = ARGOS_MOD_LDA2;
	m_state = SmdSatState::stopped;
	m_tcxo_warmup_time = DEFAULT_TCXO_WARMUP_TIME_SECONDS;
	m_tx_power = 0;
	this->shutdown();
	is_kmac_profil_loaded = false;
}

SmdSat::~SmdSat() {
	power_off_immediate();
}

// ============================================================================
// Power management
// ============================================================================

void SmdSat::shutdown(void) {
	GPIOPins::init_pin(SAT_RESET);
	GPIOPins::clear(SAT_RESET);
	nrf_delay_ms(SMDSAT_DELAY_RST_MS);
	GPIOPins::clear(SAT_PWR_EN);
#ifdef SMD_VPA_PIN
	GPIOPins::drive_low(SMD_VPA_PIN);
#endif
}

void SmdSat::power_off() {
	DEBUG_TRACE("SmdSat::%s",__func__);
	if (!SMD_STATE_EQUAL(stopped)) {
		m_stopping = true;
	}
}

void SmdSat::power_on() {
	DEBUG_TRACE("SmdSat::%s",__func__);

	if (m_state != SmdSat::stopped) {
		m_stopping = false;
		DEBUG_TRACE("SmdSat::%s:: already running",__func__);
		return;
	}

	m_stopping = false;
	SMD_STATE_CHANGE(stopped, starting);
	DEBUG_TRACE("SmdSat::%s:: start state machine",__func__);
	state_machine();
}

void SmdSat::power_off_immediate()
{
	DEBUG_TRACE("SmdSat::%s",__func__);

	if (!SMD_STATE_EQUAL(stopped)) {
		system_scheduler->cancel_task(m_task);
		switch (m_state) {
		case SmdSatState::starting:         state_starting_exit(); break;
		case SmdSatState::powering_on:      state_powering_on_exit(); break;
		case SmdSatState::load_kmac:        state_load_kmac_exit(); break;
		case SmdSatState::idle_pending:     state_idle_pending_exit(); break;
		case SmdSatState::idle:             state_idle_exit(); break;
		case SmdSatState::transmit_pending: state_transmit_pending_exit(); break;
		case SmdSatState::transmitting:     state_transmitting_exit(); break;
		case SmdSatState::error:            state_error_exit(); break;
		case SmdSatState::stopped:          break;
		default: break;
		}
		m_state = SmdSatState::stopped;
		state_stopped_enter();
	}

	// Clean up transport in case state was already stopped
	m_cmd.deinit();
}

// ============================================================================
// State machine
// ============================================================================

void SmdSat::state_machine(bool use_scheduler) {
	m_next_delay = 0;

	switch (m_state) {
	case SmdSat::starting:
		SMD_STATE_CALL(starting);
		break;
	case SmdSat::powering_on:
		SMD_STATE_CALL(powering_on);
		break;
	case SmdSat::load_kmac:
		SMD_STATE_CALL(load_kmac);
		break;
	case SmdSat::idle_pending:
		SMD_STATE_CALL(idle_pending);
		break;
	case SmdSat::idle:
		SMD_STATE_CALL(idle);
		break;
	case SmdSat::transmit_pending:
		SMD_STATE_CALL(transmit_pending);
		break;
	case SmdSat::transmitting:
		SMD_STATE_CALL(transmitting);
		break;
	case SmdSat::error:
		SMD_STATE_CALL(error);
		break;
	case SmdSat::stopped:
		SMD_STATE_CALL(stopped);
		break;
	default:
		break;
	}

	if (use_scheduler && !SMD_STATE_EQUAL(stopped)) {
		system_scheduler->cancel_task(m_task);
		m_task = system_scheduler->post_task_prio([this]() {
			state_machine();
		}, "SmdReceiverStateMachine", Scheduler::DEFAULT_PRIORITY, m_next_delay);
	}
}

void SmdSat::state_starting_enter() {}
void SmdSat::state_starting_exit() {}

void SmdSat::state_starting()
{
	m_is_first_tx = true;
	is_kmac_profil_loaded = false;
	m_credentials_written = false;
	m_cmd.init();
	m_state_counter = 3;
	m_next_delay = 0;
	SMD_STATE_CHANGE(starting, powering_on);
}

void SmdSat::state_error_enter() {
	try {
		uint8_t status = 0;
		m_cmd.get_kmac_status(&status);
	} catch (...) {
		DEBUG_WARN("SmdSat::%s: read failed in error handler", __func__);
	}
	notify(KineisEventDeviceError({}));
	SMD_STATE_CHANGE(error, stopped);
}

void SmdSat::state_error_exit() {}

void SmdSat::state_error() {
	SMD_STATE_CHANGE(error, stopped);
}

void SmdSat::state_stopped_enter() {
	m_cmd.deinit();

	this->shutdown();
	GPIOPins::release_sensors_pwr();
	nrf_gpio_cfg_default(BSP::GPIO_Inits[SAT_RESET].pin_number);
	is_kmac_profil_loaded = false;

	m_packet_buffer.clear();

	notify(KineisEventPowerOff({}));
}

void SmdSat::state_stopped_exit() {}
void SmdSat::state_stopped() {}

void SmdSat::state_powering_on_enter() {}

void SmdSat::state_powering_on_exit() {
	m_next_delay = SMDSAT_DELAY_POWER_ON_MS;
}

void SmdSat::state_powering_on() {
	DEBUG_TRACE("SmdSat::%s: Starting power-on sequence", __func__);

	GPIOPins::acquire_sensors_pwr();

	GPIOPins::init_pin(SAT_RESET);
	GPIOPins::clear(SAT_RESET);
	nrf_delay_ms(10);

	GPIOPins::set(SAT_PWR_EN);
#ifdef SMD_VPA_PIN
	GPIOPins::release_to_highz(SMD_VPA_PIN);
#endif
	DEBUG_TRACE("SmdSat::%s: Power enabled | waiting for stabilization", __func__);

	nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);

	GPIOPins::release_to_highz(SAT_RESET);
	DEBUG_TRACE("SmdSat::%s: Reset released", __func__);

	SMD_STATE_CHANGE(powering_on, idle_pending);
}

void SmdSat::state_load_kmac_enter() {}
void SmdSat::state_load_kmac_exit() {}

void SmdSat::state_load_kmac() {
	// Write credentials from config store only if they changed (PARMW on IDP12/IDT06/IDP13/IDP14)
	if (!m_credentials_written) {
		m_credentials_written = true;
		if (configuration_store && configuration_store->is_credentials_dirty()) {
			if (!write_credentials_from_config()) {
				DEBUG_ERROR("SmdSat::%s: credential write failed", __func__);
				SMD_STATE_CHANGE(load_kmac, error);
				return;
			}
			configuration_store->clear_credentials_dirty();
		} else {
			DEBUG_TRACE("SmdSat::%s: credentials unchanged, skipping write", __func__);
		}
	}

	uint8_t kmac_status = 0;
	m_cmd.get_kmac_status(&kmac_status);
	if (kmac_status == MAC_OK) {
		if (!is_kmac_profil_loaded) {
			m_cmd.load_kmac_profil(1);
			m_next_delay = SMDSAT_DELAY_LOAD_KMAC_MS;
			is_kmac_profil_loaded = true;
		} else {
			m_next_delay = SMDSAT_DELAY_CMD_MS;
		}
		SMD_STATE_CHANGE(load_kmac, idle_pending);
	} else {
		if (--m_state_counter == 0) {
			DEBUG_ERROR("SmdSat::%s: failed to enter load kmac state",__func__);
			SMD_STATE_CHANGE(load_kmac, error);
		} else {
			m_next_delay = SMDSAT_DELAY_TICK_INTERRUPT_MS;
		}
	}
}

void SmdSat::state_idle_pending_enter() {
	m_next_delay = SMDSAT_DELAY_CMD_MS;
	m_state_counter = 10;
}

void SmdSat::state_idle_pending_exit() {
	m_next_delay = SMDSAT_DELAY_CMD_MS;
}

void SmdSat::state_idle_pending() {
	if (m_cmd.ping()) {
		if (!is_kmac_profil_loaded) {
			SMD_STATE_CHANGE(idle_pending, load_kmac);
		} else {
			SMD_STATE_CHANGE(idle_pending, idle);
		}
	} else {
		if (--m_state_counter == 0) {
			DEBUG_ERROR("SmdSat::%s: failed to enter IDLE state",__func__);
			SMD_STATE_CHANGE(idle_pending, error);
		} else {
			m_next_delay = SMDSAT_DELAY_CMD_MS;
		}
	}
}

void SmdSat::state_idle_enter() {
	m_next_delay = SMDSAT_DELAY_TICK_INTERRUPT_MS;
	m_state_counter = m_idle_timeout_ms / SMDSAT_DELAY_TICK_INTERRUPT_MS;
}

void SmdSat::state_idle_exit() {
	m_next_delay = SMDSAT_DELAY_CMD_MS;
}

void SmdSat::state_idle() {
	if (m_packet_buffer.length()) {
		m_tx_buffer = m_packet_buffer;
		m_packet_buffer.clear();
		SMD_STATE_CHANGE(idle, transmit_pending);
	} else if (m_stopping) {
		SMD_STATE_CHANGE(idle, stopped);
	} else {
		m_next_delay = SMDSAT_DELAY_TICK_INTERRUPT_MS;
		if (--m_state_counter == 0) {
			DEBUG_TRACE("SmdSat::%s: idle timeout elapsed",__func__);
			SMD_STATE_CHANGE(idle, stopped);
		}
		return;
	}

	m_state_counter = m_idle_timeout_ms / SMDSAT_DELAY_TICK_INTERRUPT_MS;
}

void SmdSat::state_transmit_pending_enter() {
	m_state_counter = 3;
}

void SmdSat::state_transmit_pending_exit() {
	m_next_delay = SMDSAT_DELAY_CMD_TX + (m_tcxo_warmup_time*1000);
}

void SmdSat::state_transmit_pending() {
	if (m_tx_buffer.size()) {
		if (!m_cmd.initiate_tx(m_tx_buffer)) {
			DEBUG_ERROR("SmdSat::%s: initiate_tx failed, aborting TX", __func__);
			m_tx_buffer.clear();
			SMD_STATE_CHANGE(transmit_pending, error);
			return;
		}
		notify(KineisEventTxStarted({}));
		SMD_STATE_CHANGE(transmit_pending, transmitting);
	} else if (--m_state_counter == 0) {
		DEBUG_ERROR("SmdSat::%s: failed accept SEND command",__func__);
		SMD_STATE_CHANGE(transmit_pending, error);
	} else {
		m_next_delay = SMDSAT_DELAY_CMD_MS;
	}
}

void SmdSat::state_transmitting_enter() {
	DEBUG_TRACE("SmdSat::%s", __func__);
	uint32_t total_timeout_ms = (m_tcxo_warmup_time * 1000) + 5000;
	m_state_counter = (total_timeout_ms / SMDSAT_TIMING_POLL_MS) + 1;
	DEBUG_TRACE("SmdSat::%s: poll timeout=%ums | counter=%u", __func__, total_timeout_ms, m_state_counter);
}

void SmdSat::state_transmitting_exit() {
	m_is_first_tx = false;
}

void SmdSat::state_transmitting() {
	if (m_cmd.is_tx_finished()) {
		if (m_tx_buffer.size()) {
			m_tx_buffer.clear();
			DEBUG_INFO("SmdSat::%s: notify KineisEventTxComplete", __func__);
			notify(KineisEventTxComplete({}));
		}
		SMD_STATE_CHANGE(transmitting, stopped);
	} else {
		if (!m_tx_buffer.size()) {
			SMD_STATE_CHANGE(transmitting, stopped);
		} else if (--m_state_counter == 0) {
			DEBUG_ERROR("SmdSat::%s: TX timeout after polling",__func__);
			SMD_STATE_CHANGE(transmitting, error);
		} else {
			m_next_delay = SMDSAT_TIMING_POLL_MS;
		}
	}
}

// ============================================================================
// KineisDevice interface
// ============================================================================

void SmdSat::stop_send() {
	DEBUG_TRACE("SmdSat::%s",__func__);
	m_packet_buffer.clear();
	m_tx_buffer.clear();
}

void SmdSat::start_receive(const KineisModulation mode) {
	(void)mode;
	DEBUG_TRACE("SmdSat::%s: Not supported",__func__);
}

bool SmdSat::stop_receive() {
	DEBUG_TRACE("SmdSat::%s: Not supported",__func__);
	return false;
}

void SmdSat::set_frequency(double freq_mhz) {
	DEBUG_TRACE("SmdSat::%s",__func__);
	m_tx_freq = freq_mhz;
}

void SmdSat::send(const KineisModulation mode, const KineisPacket& user_payload, const unsigned int payload_length)
{
	DEBUG_TRACE("SmdSat::%s: length %u mode=%d current=%d", __func__, payload_length, (int)mode, (int)m_modulation);
	SmdArgosModulation requested = kineis_to_smd_mod(mode);
	if (requested != m_modulation) {
		DEBUG_WARN("SmdSat::%s: TX mode %d != current modulation %d — call switch_modulation() first",
		           __func__, (int)requested, (int)m_modulation);
	}

	unsigned int max_payload_size = 0;
	switch(m_modulation) {
		case ARGOS_MOD_LDA2:
			max_payload_size = ARGOS_TX_LDA2_PAYLOAD_BYTE_SIZE;
			break;
		case ARGOS_MOD_LDK:
			max_payload_size = ARGOS_TX_LDK_PAYLOAD_BYTE_SIZE;
			break;
		case ARGOS_MOD_VLDA4:
			max_payload_size = ARGOS_TX_VLDA4_PAYLOAD_BYTE_SIZE;
			break;
		default:
			DEBUG_TRACE("SmdSat::%s: Unknown modulation type %d", __func__, m_modulation);
			return;
	}

	unsigned int effective_payload_length = std::min(payload_length,
	                                                 static_cast<unsigned int>(user_payload.size()));
	if (effective_payload_length > max_payload_size) {
		DEBUG_ERROR("SmdSat::%s: Payload truncated from %u to %u bytes",
		            __func__, effective_payload_length, max_payload_size);
		effective_payload_length = max_payload_size;
	}

	unsigned int padded_size = max_payload_size;
	if (m_modulation == ARGOS_MOD_LDA2) {
		padded_size = ((effective_payload_length + ARGOS_TX_LDA2_SIZE_STEP - 1) /
		                ARGOS_TX_LDA2_SIZE_STEP) * ARGOS_TX_LDA2_SIZE_STEP;
		if (padded_size < ARGOS_TX_LDA2_MIN_BYTE_SIZE)
			padded_size = ARGOS_TX_LDA2_MIN_BYTE_SIZE;
		if (padded_size > max_payload_size)
			padded_size = max_payload_size;
		DEBUG_TRACE("SmdSat::%s: LDA2 padded %u -> %u bytes", __func__, effective_payload_length, padded_size);
	}

	m_packet_buffer.assign(padded_size, 0);
	std::copy(user_payload.begin(), user_payload.begin() + effective_payload_length, m_packet_buffer.begin());

	DEBUG_TRACE("SmdSat::%s: raw data[%u]=%s", __func__,
	            effective_payload_length,
	            Binascii::hexlify(m_packet_buffer).c_str());

	DEBUG_TRACE("SmdSat::%s::Packet size %u", __func__, static_cast<unsigned int>(m_packet_buffer.size()));

	power_on();
}

void SmdSat::set_tcxo_warmup_time(unsigned int time_s) {
	m_tcxo_warmup_time = time_s;
}

void SmdSat::set_tx_power(unsigned int power) {
	m_tx_power = power;
}

// ============================================================================
// Auto-write credentials from config store at boot
// ============================================================================

bool SmdSat::write_credentials_from_config() {
	if (!configuration_store) return false;

	auto wait_cmd = []() { nrf_delay_ms(SMDSAT_DELAY_CMD_MS * 2); };

	unsigned int dec_id = configuration_store->read_param<unsigned int>(ParamID::ARGOS_DECID);
	unsigned int address = configuration_store->read_param<unsigned int>(ParamID::ARGOS_HEXID);
	std::string seckey = configuration_store->read_param<std::string>(ParamID::ARGOS_SECKEY);
	std::string radioconf = configuration_store->read_param<std::string>(ParamID::ARGOS_RADIOCONF);

	// Skip if credentials are not configured yet
	if (seckey.empty() || radioconf.empty()) {
		DEBUG_WARN("SmdSat::%s: credentials not configured (seckey=%u rconf=%u) | skipping",
		           __func__, (unsigned)seckey.size(), (unsigned)radioconf.size());
		return true;  // Not an error, just nothing to write
	}

	DEBUG_INFO("SmdSat::%s: writing credentials from config store (id=%u addr=0x%08X)",
	           __func__, dec_id, address);

	// Set ID
	try { m_cmd.set_id(dec_id); } catch (...) {
		DEBUG_ERROR("SmdSat::%s: failed to set ID", __func__); return false;
	}
	wait_cmd();

	// Set Address
	{
		uint8_t address_data[4] = {
			static_cast<uint8_t>((address >> 24) & 0xFF),
			static_cast<uint8_t>((address >> 16) & 0xFF),
			static_cast<uint8_t>((address >> 8) & 0xFF),
			static_cast<uint8_t>(address & 0xFF)
		};
		smd_uint8_array_t address_val = {SMDSAT_CMD_WRITE_ADDR_LEN-1, address_data};
		try { m_cmd.set_address(&address_val); } catch (...) {
			DEBUG_ERROR("SmdSat::%s: failed to set address", __func__); return false;
		}
	}
	wait_cmd();

	// Set Security Key
	{
		std::string seckey_bin = Binascii::unhexlify(seckey);
		smd_uint8_array_t seckey_struct = {static_cast<uint16_t>(seckey_bin.size()),
		                                   reinterpret_cast<uint8_t *>(seckey_bin.data())};
		try { m_cmd.set_seckey(&seckey_struct); } catch (...) {
			DEBUG_ERROR("SmdSat::%s: failed to set seckey", __func__); return false;
		}
	}
	wait_cmd();

	// Set Radio Configuration
	{
		std::string rconf_bin = Binascii::unhexlify(radioconf);
		smd_uint8_array_t rconf_struct = {static_cast<uint16_t>(rconf_bin.size()),
		                                  reinterpret_cast<uint8_t *>(rconf_bin.data())};
		try { m_cmd.set_radio_conf(&rconf_struct); } catch (...) {
			DEBUG_ERROR("SmdSat::%s: failed to set radio conf", __func__); return false;
		}
	}
	wait_cmd();

	// Save RCONF to flash
	try {
		if (!m_cmd.save_radio_conf()) {
			DEBUG_ERROR("SmdSat::%s: failed to save RCONF", __func__); return false;
		}
	} catch (...) {
		DEBUG_ERROR("SmdSat::%s: save_radio_conf exception", __func__); return false;
	}
	wait_cmd();

	DEBUG_INFO("SmdSat::%s: credentials written successfully", __func__);
	return true;
}

// ============================================================================
// Credentials (legacy API, kept for direct hardware access if needed)
// ============================================================================

void SmdSat::set_credentials(unsigned int dec_id, unsigned int address, const std::string& seckey, const std::string& radioconf) {
	DEBUG_TRACE("SmdSat::%s",__func__);
	bool was_stopped = (m_state == SmdSatState::stopped);

	auto wait_between_commands = []() -> bool {
		nrf_delay_ms(SMDSAT_DELAY_CMD_MS * 2);
		return true;
	};

	if (was_stopped) {
		GPIOPins::acquire_sensors_pwr();
		GPIOPins::init_pin(SAT_RESET);
		GPIOPins::clear(SAT_RESET);
		GPIOPins::set(SAT_PWR_EN);
#ifdef SMD_VPA_PIN
		GPIOPins::release_to_highz(SMD_VPA_PIN);
#endif
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);
		GPIOPins::release_to_highz(SAT_RESET);
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);
	}
	m_cmd.init();

	{
		bool smd_ready = false;
		for (uint8_t attempt = 0; attempt < 10; attempt++) {
			nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS / 2);
			if (m_cmd.ping()) {
				DEBUG_INFO("SmdSat::%s: SMD ready after %u attempts", __func__, attempt + 1);
				smd_ready = true;
				break;
			}
		}
		if (!smd_ready) {
			DEBUG_ERROR("SmdSat::%s: SMD not ready after power-on | aborting", __func__);
			goto cleanup;
		}
	}

	for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
		try {
			m_cmd.set_id(dec_id);
			break;
		} catch (...) {
			if (retry == SMDSAT_SPI_MAX_RETRIES - 1) {
				DEBUG_ERROR("SmdSat::%s: Failed to set ID", __func__);
				goto cleanup;
			}
			nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
		}
	}

	if (!wait_between_commands()) goto cleanup;

	{
		uint8_t address_data[4] = {
			static_cast<uint8_t>((address >> 24) & 0xFF),
			static_cast<uint8_t>((address >> 16) & 0xFF),
			static_cast<uint8_t>((address >> 8) & 0xFF),
			static_cast<uint8_t>(address & 0xFF)
		};
		smd_uint8_array_t address_val = {SMDSAT_CMD_WRITE_ADDR_LEN-1, address_data};

		for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
			try {
				m_cmd.set_address(&address_val);
				break;
			} catch (...) {
				if (retry == SMDSAT_SPI_MAX_RETRIES - 1) {
					DEBUG_ERROR("SmdSat::%s: Failed to set address", __func__);
					goto cleanup;
				}
				nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
			}
		}
	}

	if (!wait_between_commands()) goto cleanup;

	{
		std::string seckey_val = Binascii::unhexlify(seckey);
		smd_uint8_array_t seckey_struct = {static_cast<uint16_t>(seckey_val.size()),
		                               reinterpret_cast<uint8_t *>(seckey_val.data())};

		for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
			try {
				m_cmd.set_seckey(&seckey_struct);
				break;
			} catch (...) {
				if (retry == SMDSAT_SPI_MAX_RETRIES - 1) {
					DEBUG_ERROR("SmdSat::%s: Failed to set seckey", __func__);
					goto cleanup;
				}
				nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
			}
		}
	}

	if (!wait_between_commands()) goto cleanup;

	{
		std::string radioconf_val = Binascii::unhexlify(radioconf);
		smd_uint8_array_t radioconf_struct = {static_cast<uint16_t>(radioconf_val.size()),
		                                  reinterpret_cast<uint8_t *>(radioconf_val.data())};

		for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
			try {
				m_cmd.set_radio_conf(&radioconf_struct);
				break;
			} catch (...) {
				if (retry == SMDSAT_SPI_MAX_RETRIES - 1) {
					DEBUG_ERROR("SmdSat::%s: Failed to set radio conf", __func__);
					goto cleanup;
				}
				nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
			}
		}
	}

	wait_between_commands();

	for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
		try {
			if (m_cmd.save_radio_conf()) break;
			if (retry == SMDSAT_SPI_MAX_RETRIES - 1) goto cleanup;
		} catch (...) {
			if (retry == SMDSAT_SPI_MAX_RETRIES - 1) goto cleanup;
		}
		nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	}

	wait_between_commands();

	for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
		try {
			m_cmd.load_kmac_profil(1);
			break;
		} catch (...) {
			if (retry == SMDSAT_SPI_MAX_RETRIES - 1) goto cleanup;
			nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
		}
	}

	wait_between_commands();

	{
		uint8_t rconf_raw[SMDSAT_CMD_READ_RCONF_RAW_LEN] = {0};
		uint16_t rconf_raw_len = 0;
		try {
			m_cmd.read_rconf_raw(rconf_raw, &rconf_raw_len);
			char hex[SMDSAT_CMD_READ_RCONF_RAW_LEN * 2 + 1] = {0};
			for (uint16_t i = 0; i < rconf_raw_len && i < SMDSAT_CMD_READ_RCONF_RAW_LEN; i++) {
				uint8_t hi = rconf_raw[i] >> 4;
				uint8_t lo = rconf_raw[i] & 0x0F;
				hex[i * 2]     = hi < 10 ? ('0' + hi) : ('a' + hi - 10);
				hex[i * 2 + 1] = lo < 10 ? ('0' + lo) : ('a' + lo - 10);
			}
			DEBUG_INFO("SmdSat::%s: RCONF_RAW readback OK: %s", __func__, hex);
		} catch (...) {
			DEBUG_WARN("SmdSat::%s: Failed to read back raw radio config", __func__);
		}
	}

	DEBUG_INFO("SmdSat::%s: Credentials set successfully", __func__);

cleanup:
	if (was_stopped) {
		m_cmd.deinit();
		shutdown();
		GPIOPins::release_sensors_pwr();
		nrf_gpio_cfg_default(BSP::GPIO_Inits[SAT_RESET].pin_number);
	}
}

void SmdSat::read_credentials(unsigned int *dec_id, unsigned int *address, std::string *seckey, std::string *radioconf) {
	DEBUG_TRACE("SmdSat::%s",__func__);
	bool was_stopped = (m_state == SmdSatState::stopped);

	if (was_stopped) {
		GPIOPins::acquire_sensors_pwr();
		GPIOPins::init_pin(SAT_RESET);
		GPIOPins::clear(SAT_RESET);
		GPIOPins::set(SAT_PWR_EN);
#ifdef SMD_VPA_PIN
		GPIOPins::release_to_highz(SMD_VPA_PIN);
#endif
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);
		GPIOPins::release_to_highz(SAT_RESET);
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);
	}
	m_cmd.init();

	nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);

	try {
		if (dec_id) {
			uint32_t dec_id_val = 0;
			m_cmd.read_id(&dec_id_val);
			*dec_id = static_cast<unsigned int>(dec_id_val);
			nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
		}

		if (address) {
			uint8_t address_data[SMDSAT_CMD_READ_ADDR_LEN] = {0};
			smd_uint8_array_t address_value = {SMDSAT_CMD_READ_ADDR_LEN, address_data};
			m_cmd.read_address(&address_value);
			nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
			*address  = ((uint32_t)address_value.p_data[0] << 24) |
			            ((uint32_t)address_value.p_data[1] << 16) |
			            ((uint32_t)address_value.p_data[2] << 8)  |
			            (address_value.p_data[3]);
		}

		if (seckey) {
			uint8_t seckey_data[SMDSAT_CMD_READ_SECKEY_LEN] = {0};
			smd_uint8_array_t seckey_value = {SMDSAT_CMD_READ_SECKEY_LEN, seckey_data};
			m_cmd.read_seckey(&seckey_value);
			nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
			*seckey = Binascii::hexlify(std::string(reinterpret_cast<char *>(seckey_data), SMDSAT_CMD_READ_SECKEY_LEN));
		}

		if (radioconf) {
			uint8_t rconf_raw[SMDSAT_CMD_READ_RCONF_RAW_LEN] = {0};
			uint16_t rconf_raw_len = 0;
			m_cmd.read_rconf_raw(rconf_raw, &rconf_raw_len);
			nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
			*radioconf = Binascii::hexlify(std::string(reinterpret_cast<char *>(rconf_raw),
			                               rconf_raw_len > 0 ? rconf_raw_len : SMDSAT_CMD_READ_RCONF_RAW_LEN));
			DEBUG_INFO("SmdSat::%s: RCONF_RAW: %s", __func__, radioconf->c_str());
		}
	} catch (...) {
		DEBUG_ERROR("SmdSat::%s: error during credential read", __func__);
		if (was_stopped) {
			m_cmd.deinit();
			shutdown();
			GPIOPins::release_sensors_pwr();
			nrf_gpio_cfg_default(BSP::GPIO_Inits[SAT_RESET].pin_number);
		}
		throw;
	}

	if (was_stopped) {
		m_cmd.deinit();
		shutdown();
		GPIOPins::release_sensors_pwr();
		nrf_gpio_cfg_default(BSP::GPIO_Inits[SAT_RESET].pin_number);
	}
}

// ============================================================================
// Runtime modulation switching
// ============================================================================

static SmdArgosModulation kineis_to_smd_mod(KineisModulation mode) {
	switch (mode) {
		case KineisModulation::LDK:  return ARGOS_MOD_LDK;
		case KineisModulation::VLDA4: return ARGOS_MOD_VLDA4;
		case KineisModulation::LDA2:
		default:                      return ARGOS_MOD_LDA2;
	}
}

static KineisModulation smd_to_kineis_mod(SmdArgosModulation mode) {
	switch (mode) {
		case ARGOS_MOD_LDK:  return KineisModulation::LDK;
		case ARGOS_MOD_VLDA4: return KineisModulation::VLDA4;
		case ARGOS_MOD_LDA2:
		default:              return KineisModulation::LDA2;
	}
}

bool SmdSat::switch_modulation(KineisModulation mode, const std::string& rconf_hex) {
	SmdArgosModulation target = kineis_to_smd_mod(mode);
	if (target == m_modulation) {
		DEBUG_TRACE("SmdSat::%s: already in target modulation %d", __func__, (int)target);
		return true;
	}

	DEBUG_INFO("SmdSat::%s: switching %d -> %d", __func__, (int)m_modulation, (int)target);

	if (rconf_hex.size() != 32) {
		DEBUG_ERROR("SmdSat::%s: invalid RCONF hex length %u (expected 32)", __func__, (unsigned)rconf_hex.size());
		return false;
	}

	// SMD must be powered on (not stopped) for RCONF write
	bool was_stopped = (m_state == SmdSatState::stopped);
	if (was_stopped) {
		DEBUG_ERROR("SmdSat::%s: SMD is stopped, cannot switch modulation", __func__);
		return false;
	}

	auto wait_cmd = []() { nrf_delay_ms(SMDSAT_DELAY_CMD_MS * 2); };

	// 1. Write RCONF
	std::string rconf_bin = Binascii::unhexlify(rconf_hex);
	smd_uint8_array_t rconf_struct = {static_cast<uint16_t>(rconf_bin.size()),
	                                  reinterpret_cast<uint8_t *>(rconf_bin.data())};
	try {
		m_cmd.set_radio_conf(&rconf_struct);
	} catch (...) {
		DEBUG_ERROR("SmdSat::%s: failed to write RCONF", __func__);
		return false;
	}

	wait_cmd();

	// 2. Save RCONF to flash
	try {
		if (!m_cmd.save_radio_conf()) {
			DEBUG_ERROR("SmdSat::%s: failed to save RCONF", __func__);
			return false;
		}
	} catch (...) {
		DEBUG_ERROR("SmdSat::%s: save_radio_conf exception", __func__);
		return false;
	}

	wait_cmd();

	// 3. Reload KMAC profile 1
	try {
		m_cmd.load_kmac_profil(1);
	} catch (...) {
		DEBUG_ERROR("SmdSat::%s: failed to reload KMAC", __func__);
		return false;
	}

	wait_cmd();

	// Update cached modulation
	m_modulation = target;
	is_kmac_profil_loaded = true;

	DEBUG_INFO("SmdSat::%s: modulation switched to %d OK", __func__, (int)target);
	return true;
}

KineisModulation SmdSat::get_current_modulation() const {
	return smd_to_kineis_mod(m_modulation);
}

// ============================================================================
// DFU (delegates to command layer)
// ============================================================================

bool SmdSat::dfu_enter() {
	DEBUG_TRACE("SmdSat::%s", __func__);

	bool was_stopped = (m_state == SmdSatState::stopped);

	if (was_stopped) {
		GPIOPins::acquire_sensors_pwr();
		GPIOPins::set(SAT_PWR_EN);
#ifdef SMD_VPA_PIN
		GPIOPins::release_to_highz(SMD_VPA_PIN);
#endif
		PMU::kick_watchdog();
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);
		PMU::kick_watchdog();
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);
	}

	m_cmd.init();

	PMU::kick_watchdog();
	nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS / 2);

	bool result = m_cmd.dfu_enter();

	if (!result) {
		if (was_stopped) {
			m_cmd.deinit();
			shutdown();
			GPIOPins::release_sensors_pwr();
			nrf_gpio_cfg_default(BSP::GPIO_Inits[SAT_RESET].pin_number);
		}
	}

	return result;
}

bool SmdSat::dfu_exit() {
	return m_cmd.dfu_exit();
}

bool SmdSat::dfu_get_bootloader_info(SmdDfuInfo *info) {
	return m_cmd.dfu_get_bootloader_info(info);
}

SmdDfuResponse SmdSat::firmware_update(const uint8_t *firmware, size_t size,
                                       void (*progress_callback)(uint8_t percent)) {
	DEBUG_INFO("SmdSat::%s: Starting firmware update | size=%u bytes", __func__, (unsigned int)size);

	if (firmware == nullptr || size == 0) {
		DEBUG_ERROR("SmdSat::%s: Invalid firmware data", __func__);
		return DFU_RSP_ERROR;
	}

	SmdDfuResponse result;

	if (!m_cmd.dfu_supported()) {
		DEBUG_ERROR("SmdSat::%s: DFU not supported by command layer", __func__);
		return DFU_RSP_ERROR;
	}

	// Enter DFU mode
	if (!dfu_enter()) {
		DEBUG_ERROR("SmdSat::%s: Failed to enter DFU mode", __func__);
		return DFU_RSP_NOT_READY;
	}

	if (progress_callback) progress_callback(5);

	// Get bootloader info
	SmdDfuInfo dfu_info;
	if (!m_cmd.dfu_get_bootloader_info(&dfu_info)) {
		DEBUG_ERROR("SmdSat::%s: Failed to get bootloader info", __func__);
		return DFU_RSP_ERROR;
	}

	if (size > dfu_info.app_max_size) {
		DEBUG_ERROR("SmdSat::%s: Firmware too large (%u > %u)", __func__,
		            (unsigned int)size, dfu_info.app_max_size);
		return DFU_RSP_SIZE_ERROR;
	}

	if (progress_callback) progress_callback(15);

	// Erase
	result = m_cmd.dfu_erase();
	if (result != DFU_RSP_OK) {
		DEBUG_ERROR("SmdSat::%s: Flash erase failed", __func__);
		return result;
	}

	if (progress_callback) progress_callback(25);

	// Write in chunks
	uint32_t addr = dfu_info.app_start_addr;
	size_t remaining = size;
	size_t offset = 0;
	uint8_t last_progress = 25;

	while (remaining > 0) {
		uint16_t chunk_size = (remaining > SMDSAT_DFU_CHUNK_SIZE) ?
		                      SMDSAT_DFU_CHUNK_SIZE : (uint16_t)remaining;

		PMU::kick_watchdog();

		result = m_cmd.dfu_write_chunk(addr, &firmware[offset], chunk_size);
		if (result != DFU_RSP_OK) {
			DEBUG_ERROR("SmdSat::%s: Write failed at offset %u", __func__, (unsigned int)offset);
			return result;
		}

		addr += chunk_size;
		offset += chunk_size;
		remaining -= chunk_size;

		if (progress_callback) {
			uint8_t progress = 25 + (uint8_t)((offset * 60) / size);
			if (progress > last_progress) {
				progress_callback(progress);
				last_progress = progress;
			}
		}

		nrf_delay_ms(5);
	}

	if (progress_callback) progress_callback(90);

	// Verify CRC32
	PMU::kick_watchdog();
	uint32_t crc = m_cmd.calculate_crc32(firmware, size);
	result = m_cmd.dfu_verify(crc);
	if (result != DFU_RSP_OK) {
		DEBUG_ERROR("SmdSat::%s: CRC verification failed", __func__);
		return result;
	}

	if (progress_callback) progress_callback(95);

	// Jump to application
	m_cmd.dfu_jump();

	// Hardware reset
	PMU::kick_watchdog();
	GPIOPins::init_pin(SAT_RESET);
	GPIOPins::clear(SAT_RESET);
	nrf_delay_ms(50);
	GPIOPins::release_to_highz(SAT_RESET);
	nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);
	PMU::kick_watchdog();

	// Read new firmware version
	m_cmd.init();
	bool app_ready = false;
	for (uint8_t attempt = 0; attempt < 10; attempt++) {
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS / 2);
		PMU::kick_watchdog();
		if (m_cmd.ping()) {
			app_ready = true;
			break;
		}
	}

	if (app_ready) {
		m_new_firmware_version = get_firmware_version();
	}

	if (progress_callback) progress_callback(100);

	DEBUG_INFO("SmdSat::%s: Firmware update completed!", __func__);
	return DFU_RSP_OK;
}

SmdDfuResponse SmdSat::firmware_update(File *file, size_t size, uint32_t stm32_crc32,
                                       void (*progress_callback)(uint8_t percent)) {
	DEBUG_INFO("SmdSat::%s: Starting streamed firmware update | size=%u bytes | CRC32=0x%08X",
	           __func__, (unsigned int)size, stm32_crc32);

	if (file == nullptr || size == 0) {
		return DFU_RSP_ERROR;
	}

	if (!m_cmd.dfu_supported()) {
		return DFU_RSP_ERROR;
	}

	SmdDfuResponse result;

	if (!dfu_enter()) {
		return DFU_RSP_NOT_READY;
	}

	if (progress_callback) progress_callback(5);

	SmdDfuInfo dfu_info;
	if (!m_cmd.dfu_get_bootloader_info(&dfu_info)) {
		return DFU_RSP_ERROR;
	}

	if (size > dfu_info.app_max_size) {
		return DFU_RSP_SIZE_ERROR;
	}

	if (progress_callback) progress_callback(15);

	result = m_cmd.dfu_erase();
	if (result != DFU_RSP_OK) return result;

	if (progress_callback) progress_callback(25);

	uint32_t addr = dfu_info.app_start_addr;
	size_t remaining = size;
	size_t offset = 0;
	uint8_t last_progress = 25;
	uint8_t chunk_buf[SMDSAT_DFU_CHUNK_SIZE];

	while (remaining > 0) {
		uint16_t chunk_size = (remaining > SMDSAT_DFU_CHUNK_SIZE) ?
		                      SMDSAT_DFU_CHUNK_SIZE : (uint16_t)remaining;

		PMU::kick_watchdog();

		lfs_ssize_t bytes_read = file->read(chunk_buf, chunk_size);
		if (bytes_read != (lfs_ssize_t)chunk_size) {
			return DFU_RSP_ERROR;
		}

		result = m_cmd.dfu_write_chunk(addr, chunk_buf, chunk_size);
		if (result != DFU_RSP_OK) return result;

		addr += chunk_size;
		offset += chunk_size;
		remaining -= chunk_size;

		if (progress_callback) {
			uint8_t progress = 25 + (uint8_t)((offset * 60) / size);
			if (progress > last_progress) {
				progress_callback(progress);
				last_progress = progress;
			}
		}

		nrf_delay_ms(5);
	}

	if (progress_callback) progress_callback(90);

	result = m_cmd.dfu_verify(stm32_crc32);
	if (result != DFU_RSP_OK) return result;

	if (progress_callback) progress_callback(95);

	m_cmd.dfu_jump();

	PMU::kick_watchdog();
	GPIOPins::init_pin(SAT_RESET);
	GPIOPins::clear(SAT_RESET);
	nrf_delay_ms(50);
	GPIOPins::release_to_highz(SAT_RESET);
	nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);
	PMU::kick_watchdog();

	m_cmd.init();
	bool app_ready = false;
	for (uint8_t attempt = 0; attempt < 10; attempt++) {
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS / 2);
		PMU::kick_watchdog();
		if (m_cmd.ping()) {
			app_ready = true;
			break;
		}
	}

	if (app_ready) {
		m_new_firmware_version = get_firmware_version();
	}

	if (progress_callback) progress_callback(100);
	return DFU_RSP_OK;
}

SmdDfuResponse SmdSat::firmware_update_from_file(const std::string& filepath,
                                                 void (*progress_callback)(uint8_t percent)) {
	(void)filepath;
	(void)progress_callback;
	DEBUG_ERROR("SmdSat::%s: Not implemented", __func__);
	return DFU_RSP_ERROR;
}

std::string SmdSat::get_firmware_version() {
	DEBUG_TRACE("SmdSat::%s", __func__);

	bool was_stopped = (m_state == SmdSatState::stopped);

	if (was_stopped) {
		GPIOPins::acquire_sensors_pwr();
		GPIOPins::init_pin(SAT_RESET);
		GPIOPins::clear(SAT_RESET);
		nrf_delay_ms(10);
		GPIOPins::set(SAT_PWR_EN);
#ifdef SMD_VPA_PIN
		GPIOPins::release_to_highz(SMD_VPA_PIN);
#endif
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);
		GPIOPins::release_to_highz(SAT_RESET);
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);
	}

	m_cmd.init();

	std::string version = "";

	try {
		bool smd_ready = false;
		for (uint8_t attempt = 0; attempt < 10; attempt++) {
			nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS / 2);
			if (m_cmd.ping()) {
				smd_ready = true;
				break;
			}
		}

		if (smd_ready) {
			uint8_t ver[64] = {0};
			m_cmd.read_version(ver);
			size_t len = 0;
			while (len < sizeof(ver) && ver[len] != 0 && ver[len] >= 0x20 && ver[len] < 0x7F) {
				len++;
			}
			version = std::string((char*)ver, len);
			DEBUG_INFO("SmdSat::%s: Firmware version: %s", __func__, version.c_str());
		}
	} catch (...) {
		DEBUG_ERROR("SmdSat::%s: Failed to read firmware version", __func__);
	}

	if (was_stopped) {
		m_cmd.deinit();
		shutdown();
		GPIOPins::release_sensors_pwr();
		nrf_gpio_cfg_default(BSP::GPIO_Inits[SAT_RESET].pin_number);
	}

	return version;
}

std::string SmdSat::smd_spi_test() {
	return m_cmd.run_command_test();
}
