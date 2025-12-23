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
#include "bitpack.hpp"

#include "nrf_delay.h"
#include "nrf_gpio.h"

//TODO: warmup time not managed by firmware right now

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
extern Scheduler *system_scheduler;
extern Timer *system_timer;

static constexpr const char *const spistatus_string[] =
{
    "SPI_UNKNOWN",    
    "SPI_INIT",    
    "SPI_IDLE",   
    "SPI_PROCESS_CMD", 
    "SPI_WAITING_RX",
    "SPI_WAITING_TX",
    "SPI_WAITING_MAC_EVT", 
    "SPI_ERROR"
};
static constexpr const char *const kmacstatus_string[] =
{
    "MAC_UNKNOWN",    
    "MAC_OK",   
    "MAC_TX_DONE", 
    "MAC_TX_SIZE_ERROR",
    "MAC_TXACK_DONE",
    "MAC_TX_TIMEOUT", 
    "MAC_TXACK_TIMEOUT",
    "MAC_RX_ERROR",
    "MAC_RX_TIMEOUT",
    "MAC_ERROR"
};

void SmdSat::read_byte(uint8_t *byte_read) {
	// Read byte is done after a send command, wait before to request read.
    nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
    uint8_t read_cmd = SMDSAT_CMD_NONE;
    int ret;

    ret = m_nrf_spim->transfer(&read_cmd, byte_read, sizeof(read_cmd));
    if (ret) {
       throw ErrorCode::SPI_COMMS_ERROR;
	}
	return;
}

void SmdSat::send_command(uint8_t command) {
    uint8_t buffer_read;
    int ret;
	DEBUG_TRACE("%s::Send %u",__func__, command);

    ret = m_nrf_spim->transfer(&command, &buffer_read, sizeof(command));
    if (ret) {
       throw ErrorCode::SPI_COMMS_ERROR;
	}
	return;
}

void SmdSat::send_command(const uint8_t *tx_data, uint8_t *rx_data, uint16_t size) {
    int ret = m_nrf_spim->transfer(tx_data, rx_data, size);
    if (ret) {
       throw ErrorCode::SPI_COMMS_ERROR;
	}
	return;
}

void SmdSat::load_kmac_profil(uint8_t profile)
{
	DEBUG_TRACE("SmdSat::%s:Load KMAC profile %u",__func__, profile);
	uint8_t tx[SMDSAT_CMD_WRITEKMAC_LEN] = {0};
	uint8_t rx[SMDSAT_CMD_WRITEKMAC_LEN] = {0};
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	send_command(SMDSAT_CMD_WRITE_KMAC_REQ);
	tx[0] = SMDSAT_CMD_WRITE_KMAC;
	tx[1] = profile;
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	send_command(tx, rx, SMDSAT_CMD_WRITEKMAC_LEN);
	return;
}

void SmdSat::get_spi_status(uint8_t *status) {
	send_command(SMDSAT_CMD_SPI_STATUS);
	read_byte(status);
	DEBUG_TRACE("SmdSat::%s=%s", __func__, (char *)spistatus_string[*status]);
	return;
}

void SmdSat::get_kmac_status(uint8_t *status) {
	DEBUG_TRACE("SmdSat::%s:%s", __func__, (char *)kmacstatus_string[*status]);
	send_command(SMDSAT_CMD_MAC_STATUS);
	read_byte(status);
	return;
}

// ============================================================================
// Robust SPI Communication Helpers
// ============================================================================

bool SmdSat::is_spi_ready() {
	uint8_t status = 0;
	get_spi_status(&status);
	return (status == SMDSAT_SPICMD_IDLE || status == SMDSAT_SPICMD_INIT);
}

bool SmdSat::is_spi_error() {
	uint8_t status = 0;
	get_spi_status(&status);
	return (status == SMDSAT_SPICMD_ERROR);
}

bool SmdSat::wait_for_spi_ready(uint32_t timeout_ms) {
	uint32_t elapsed = 0;
	const uint32_t poll_interval = SMDSAT_SPI_RETRY_DELAY_MS;

	while (elapsed < timeout_ms) {
		uint8_t status = 0;
		get_spi_status(&status);

		if (status == SMDSAT_SPICMD_IDLE || status == SMDSAT_SPICMD_INIT) {
			return true;
		}

		if (status == SMDSAT_SPICMD_ERROR) {
			DEBUG_WARN("SmdSat::%s: SPI in ERROR state, waiting for recovery", __func__);
		}

		nrf_delay_ms(poll_interval);
		elapsed += poll_interval;
	}

	DEBUG_ERROR("SmdSat::%s: Timeout waiting for SPI ready (waited %u ms)", __func__, timeout_ms);
	return false;
}

bool SmdSat::send_command_with_retry(uint8_t command, uint8_t max_retries) {
	for (uint8_t retry = 0; retry < max_retries; retry++) {
		// Wait for SPI to be ready before sending
		if (!wait_for_spi_ready()) {
			DEBUG_WARN("SmdSat::%s: SPI not ready, retry %u/%u", __func__, retry + 1, max_retries);
			nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
			continue;
		}

		try {
			send_command(command);
			return true;
		} catch (...) {
			DEBUG_WARN("SmdSat::%s: Command 0x%02X failed, retry %u/%u", __func__, command, retry + 1, max_retries);
			nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
		}
	}

	DEBUG_ERROR("SmdSat::%s: Command 0x%02X failed after %u retries", __func__, command, max_retries);
	return false;
}

bool SmdSat::send_command_with_retry(const uint8_t *tx_data, uint8_t *rx_data, uint16_t size, uint8_t max_retries) {
	for (uint8_t retry = 0; retry < max_retries; retry++) {
		// Wait for SPI to be ready before sending
		if (!wait_for_spi_ready()) {
			DEBUG_WARN("SmdSat::%s: SPI not ready, retry %u/%u", __func__, retry + 1, max_retries);
			nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
			continue;
		}

		try {
			send_command(tx_data, rx_data, size);
			return true;
		} catch (...) {
			DEBUG_WARN("SmdSat::%s: Command failed, retry %u/%u", __func__, retry + 1, max_retries);
			nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
		}
	}

	DEBUG_ERROR("SmdSat::%s: Command failed after %u retries", __func__, max_retries);
	return false;
}
void SmdSat::set_radio_conf(uint8_array_t *radio_conf) {
	DEBUG_TRACE("SmdSat::%s",__func__);
	// - 1 size to remove cmd written
	if(radio_conf->size != SMDSAT_CMD_WRITECONF_LEN - 1){
		DEBUG_ERROR("SmdSat::%s:Size allocated to set radio conf %u != from %u",
			__func__, radio_conf->size, SMDSAT_CMD_WRITECONF_LEN - 1);
		return;
	}
	//spi_write(spiTxBuffer, 1);
	send_command(SMDSAT_CMD_WRITE_RCONF_REQ);
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	uint8_t tx[SMDSAT_CMD_WRITECONF_LEN] = {0};
	tx[0] = SMDSAT_CMD_WRITE_RCONF;
	memcpy(&(tx[1]), radio_conf->p_data, radio_conf->size);
	send_command(tx, NULL, SMDSAT_CMD_WRITECONF_LEN);
	return;
}


void SmdSat::read_radio_conf(ArgosModulation *modulation){
	DEBUG_TRACE("%s",__func__);
	send_command(SMDSAT_CMD_READ_RCONF);
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	uint8_t rx[SMDSAT_CMD_READCONF_LEN] = {0};
	uint8_t tx[SMDSAT_CMD_READCONF_LEN] = {0};
	send_command(tx, rx, SMDSAT_CMD_READCONF_LEN);

    uint32_t min_frequency = (rx[3] << 24) |
                             (rx[2] << 16) |
                             (rx[1] << 8)  |
                              rx[0];

    uint32_t max_frequency = (rx[7] << 24) |
                             (rx[6] << 16) |
                             (rx[5] << 8)  |
                              rx[4];

    int8_t rf_level = rx[8];

    *modulation = static_cast<ArgosModulation>(rx[9]); 


    // Print the decoded values
    DEBUG_INFO("SmdSat::%s:Modulation: %u, Min Frequency: %u, Max Frequency: %u, RF Level %u", __func__, 
		*modulation, min_frequency, max_frequency, rf_level);
	return;
}

bool SmdSat::save_radio_conf(){
	DEBUG_TRACE("%s",__func__);
	send_command(SMDSAT_CMD_SAVE_RCONF);
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	uint8_t ack = 0;
	read_byte(&ack);
	if (ack == 1) {
		DEBUG_INFO("SmdSat::%s:radio conf saved",__func__);
		return true;
	} else {
		DEBUG_WARN("SmdSat::%s:radio conf failed to save",__func__);
		return false;
	}
}
void SmdSat::read_lpm(uint8_t *lpm_mode) {
	DEBUG_TRACE("SmdSat::%s",__func__);
	send_command(SMDSAT_CMD_READ_LPM);
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	read_byte(lpm_mode);
	DEBUG_INFO("SmdSat::SMD LPM mode is: %u", *lpm_mode);
	return;
}

void SmdSat::write_lpm(uint8_t *lpm_mode){
	DEBUG_TRACE("SmdSat::%s",__func__);
	send_command(SMDSAT_CMD_WRITE_LPM_REQ);
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	uint8_t tx[SMDSAT_CMD_WRITE_LPM_LEN], rx[SMDSAT_CMD_WRITE_LPM_LEN] = {0};
	tx[0] = SMDSAT_CMD_WRITE_LPM;
	tx[1] = *lpm_mode;
	send_command(tx, rx, SMDSAT_CMD_WRITE_LPM_LEN);
	DEBUG_INFO("SmdSat::SMD LPM mode is: %u", *lpm_mode);
	return;
}
void SmdSat::read_version(uint8_t *version){
	DEBUG_TRACE("SmdSat::%s",__func__);
	send_command(SMDSAT_CMD_READ_VERSION);
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	read_byte(version);
	DEBUG_INFO("SmdSat::SMD SPI Version: %u", *version);
	return;
}

void SmdSat::read_address(uint8_array_t *address){
	DEBUG_TRACE("%s",__func__);
	if(address->size != SMDSAT_CMD_READ_ADDR_LEN){
		DEBUG_ERROR("SmdSat::%s:Size allocated to store address %u != from %u",
			__func__, address->size, SMDSAT_CMD_READ_ADDR_LEN);
		return;
	}
	send_command(SMDSAT_CMD_READ_ADDR);
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);

	uint8_t tx[SMDSAT_CMD_READ_ADDR_LEN] = {0};
	send_command(tx, address->p_data, address->size);
	DEBUG_INFO("SmdSat::SMD Address: 0x%02X%02X%02X%02X",
               address->p_data[0],
               address->p_data[1],
               address->p_data[2],
               address->p_data[3]);
	return;
}


void SmdSat::set_address(uint8_array_t *address){
	DEBUG_TRACE("%s:: ",__func__);
	// - 1 size to remove cmd written
	if(address->size != SMDSAT_CMD_WRITE_ADDR_LEN-1){
		DEBUG_ERROR("SmdSat::%s:Size allocated to store address %u != from %u",
			__func__, address->size, SMDSAT_CMD_WRITE_ADDR_LEN-1);
		return;
	}
	send_command(SMDSAT_CMD_WRITE_ADDR_REQ);
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	uint8_t tx[SMDSAT_CMD_WRITE_ADDR_LEN], rx[SMDSAT_CMD_WRITE_ADDR_LEN] = {0};
	tx[0] = SMDSAT_CMD_WRITE_ADDR;
	memcpy(&(tx[1]), address->p_data, address->size);

	send_command(tx, rx, (uint16_t)SMDSAT_CMD_WRITE_ADDR_LEN);
	
	// Create a buffer to build the output string.
    char buffer[128];
    int offset = 0;

    // Start the message
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "SMD Address write:");

	for (size_t i = 0; i < address->size; i++) {
        offset += snprintf(buffer + offset, sizeof(buffer) - offset, " 0x%02X", address->p_data[i]);
    } 
	DEBUG_INFO("%s", buffer);
	return;
}

void SmdSat::read_seckey(uint8_array_t *seckey){
	DEBUG_TRACE("SmdSat::%s",__func__);
	if(seckey->size != SMDSAT_CMD_READ_SECKEY_LEN){
		DEBUG_ERROR("SmdSat::%s:Size allocated to store address %u != from %u",
			__func__, seckey->size, SMDSAT_CMD_READ_SECKEY_LEN);
		return;
	}
	send_command(SMDSAT_CMD_READ_SECKEY);
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);

	uint8_t tx[SMDSAT_CMD_READ_SECKEY_LEN] = {0};
	send_command(tx, seckey->p_data, seckey->size);
	// Create a buffer to build the output string.
    char buffer[128];
    int offset = 0;

    // Start the message
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "SMD SecKey read:");

    // Loop through each of the 16 bytes and append them in hexadecimal format.
    for (size_t i = 0; i < seckey->size; i++) {
        offset += snprintf(buffer + offset, sizeof(buffer) - offset, " 0x%02X", seckey->p_data[i]);
    } 
	DEBUG_INFO("%s", buffer);
	return;
}

void SmdSat::set_seckey(uint8_array_t *seckey){
	DEBUG_TRACE("%s:: ",__func__);
	if(seckey->size != SMDSAT_CMD_WRITE_SECKEY_LEN - 1){
		DEBUG_ERROR("SmdSat::%s:Size allocated to store address %u != from %u",
			__func__, seckey->size, SMDSAT_CMD_WRITE_SECKEY_LEN);
		return;
	}
	send_command(SMDSAT_CMD_WRITE_SECKEY_REQ);
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	uint8_t tx[SMDSAT_CMD_WRITE_SECKEY_LEN], rx[SMDSAT_CMD_WRITE_SECKEY_LEN] = {0};
	tx[0] = SMDSAT_CMD_WRITE_SECKEY;
	memcpy(&(tx[1]), seckey->p_data, seckey->size);
	send_command(tx, rx, SMDSAT_CMD_WRITE_SECKEY_LEN);
	
	// Create a buffer to build the output string.
    char buffer[128];
    int offset = 0;

    // Start the message
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "SMD SecKey write:");

	for (size_t i = 0; i < seckey->size; i++) {
        offset += snprintf(buffer + offset, sizeof(buffer) - offset, " 0x%02X", seckey->p_data[i]);
    } 
	DEBUG_INFO("%s", buffer);
	return;
}

void SmdSat::read_id(uint32_t *id){
	if (id == nullptr) 
	{
		DEBUG_ERROR("SmdSat::%s:id ptr is null",__func__);
		return;
	}
	send_command(SMDSAT_CMD_READ_ID);
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	uint8_t tx[SMDSAT_CMD_READ_ID_LEN] = {0};
	uint8_t rx[SMDSAT_CMD_READ_ID_LEN] = {0};
	send_command(tx, rx,SMDSAT_CMD_READ_ID_LEN);
	for (int i = 0; i < SMDSAT_CMD_READ_ID_LEN; i++) {
		*id |= rx[i] << (8 * i);
	 }
	DEBUG_INFO("SmdSat::%s:SMD ID: 0x%u", __func__, *id);
	return;
}

void SmdSat::set_id(uint32_t id){
	DEBUG_TRACE("SmdSat::%s: id: %u",__func__, id);
	send_command(SMDSAT_CMD_WRITE_ID_REQ);
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	uint8_t tx[SMDSAT_CMD_WRITE_ID_LEN], rx[SMDSAT_CMD_WRITE_ID_LEN] = {0};
	tx[0] = SMDSAT_CMD_WRITE_ID;
	memcpy(&(tx[1]), &id, sizeof(id));
	send_command(tx, rx, SMDSAT_CMD_WRITE_ID_LEN);
	return;
}

void SmdSat::read_tcxo_warmup(uint8_t *time_ms){
	if (time_ms == nullptr) 
	{
		DEBUG_ERROR("SmdSat::%s:tcxo ptr is null",__func__);
		return;
	}
	send_command(SMDSAT_CMD_READ_ID);
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	uint8_t tx[SMDSAT_CMD_READ_ID_LEN] = {0};
	uint8_t rx[SMDSAT_CMD_READ_ID_LEN] = {0};
	send_command(tx, rx,SMDSAT_CMD_READ_ID_LEN);
	for (int i = 0; i < SMDSAT_CMD_READ_ID_LEN; i++) {
		*time_ms |= rx[i] << (8 * i);
	 }
	DEBUG_INFO("SmdSat::%s:SMD TCXO wamrup time: %u", __func__, *time_ms);
	return;
}

void SmdSat::write_tcxo_warmup(uint32_t time_ms){
	DEBUG_TRACE("SmdSat::%s: tcxo warmup time: %u",__func__, time_ms);
	send_command(SMDSAT_CMD_WRITE_TCXO_REQ);
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	uint8_t tx[SMDSAT_CMD_WRITE_TCXO_LEN], rx[SMDSAT_CMD_WRITE_TCXO_LEN] = {0};
	tx[0] = SMDSAT_CMD_WRITE_TCXO;
	memcpy(&(tx[1]), &time_ms, sizeof(time_ms));
	send_command(tx, rx, SMDSAT_CMD_WRITE_ID_LEN);
	
	return;
}

void SmdSat::read_serial(uint8_array_t *serial){
	DEBUG_TRACE("%s",__func__);
	if(serial->size != SMDSAT_CMD_READ_SERIAL_LEN){
		DEBUG_ERROR("SmdSat::%s:Size allocated to store address %u != from %u",
			__func__, serial->size, SMDSAT_CMD_READ_SERIAL_LEN);
		return;
	}
	send_command(SMDSAT_CMD_READ_SN);
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	
	uint8_t tx[SMDSAT_CMD_READ_SERIAL_LEN] = {0};
	send_command(tx, serial->p_data, serial->size);
	DEBUG_INFO("SmdSat::%s:SMD SN: %.*s", __func__, serial->size, serial->p_data);
	return;
}

void SmdSat::set_tcxo_warmup(uint32_t time_s)
{
	DEBUG_TRACE("SmdSat::%s: Not managed right now",__func__);
	this->write_tcxo_warmup(time_s*1000);
	return;
}

void SmdSat::set_tcxo_control(bool state) {
	DEBUG_TRACE("SmdSat::%s: Managed by SMD module",__func__);
}

void SmdSat::print_firmware_version() {
	send_command(SMDSAT_CMD_READ_FIRMWARE);
	uint8_t tx[SMDSAT_CMD_READ_FIRMWARE_LEN] = {0};
	uint8_t rx[SMDSAT_CMD_READ_FIRMWARE_LEN] = {0};
	send_command(tx, rx, (uint16_t)SMDSAT_CMD_READ_FIRMWARE_LEN);
    DEBUG_TRACE("SmdSat::%s: %s", __func__, rx);
	return;
}


bool SmdSat::smd_ping()
{
	DEBUG_TRACE("SmdSat::%s",__func__);

	// Use retry mechanism for ping - critical for establishing communication
	for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
		try {
			send_command(SMDSAT_CMD_PING);
			uint8_t ping = 0;
			read_byte(&ping);
			if (ping == 1) {
				DEBUG_INFO("SmdSat::%s: ACK from SMD received",__func__);
				return true;
			} else {
				DEBUG_TRACE("SmdSat::%s: Ping response: %u (retry %u/%u)", __func__, ping, retry + 1, SMDSAT_SPI_MAX_RETRIES);
			}
		} catch (...) {
			DEBUG_WARN("SmdSat::%s: Ping failed, retry %u/%u", __func__, retry + 1, SMDSAT_SPI_MAX_RETRIES);
		}
		nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
	}

	DEBUG_WARN("SmdSat::%s: Ping not received after %u retries", __func__, SMDSAT_SPI_MAX_RETRIES);
	return false;
}

bool SmdSat::is_tx_finished() {
	DEBUG_TRACE("SmdSat::%s",__func__);
    uint8_t kmac_status = 0;
	get_kmac_status(&kmac_status);
	if(kmac_status == MAC_TX_DONE)
	{
		return true;
	} else {
		return false;
	}	
}
bool SmdSat::is_tx_in_progress() {
	DEBUG_TRACE("SmdSat::%s",__func__);
	
	return true;
}

bool SmdSat::is_command_accepted() {
	DEBUG_TRACE("SmdSat::%s",__func__);
	
	return true;
}

bool SmdSat::is_firmware_ready() {
	DEBUG_TRACE("SmdSat::%s",__func__);
	
	return true;
}
void SmdSat::power_off() {
	DEBUG_TRACE("SmdSat::%s",__func__);

    // The state machine will safely transition to stopped as soon
    // as all pending activities have completed
    if (!SMD_STATE_EQUAL(stopped)) {
    	m_stopping = true;
    }
	return;
}
void SmdSat::power_on() {
	DEBUG_TRACE("SmdSat::%s",__func__);

    // Device is already powered
    if (m_state != SmdSat::stopped) {
    	m_stopping = false;  // Clear any pending stopping flag
    	DEBUG_TRACE("SmdSat::%s: not disabled: state=%u", __func__, (unsigned int)m_state);
    	return;
    }
	
    // Set top-level state variables
    m_state = SmdSatState::starting; // Force a reset when the task starts
    m_stopping = false;

    // This will run the state machine perpetually until a power_off is called
    DEBUG_TRACE("SmdSat::%s:: start state machine",__func__);
    state_machine();
	return;
}


void SmdSat::power_off_immediate()
{
	DEBUG_TRACE("SmdSat::%s",__func__);

    // We force a power off by cancelling the state machine and
    // forcing a stopped state
    if (!SMD_STATE_EQUAL(stopped)) {
    	system_scheduler->cancel_task(m_task);
    	SMD_STATE_CHANGE(idle, stopped);
    }
	return;
}



SmdSat::SmdSat(unsigned int idle_shutdown_ms) {
	DEBUG_TRACE("SmdSat::%s",__func__);
	set_idle_timeout(idle_shutdown_ms);
    m_ack_buffer.clear();
    m_packet_buffer.clear();
	m_modulation = ARGOS_MOD_LDA2;
    m_nrf_spim = nullptr;
    m_state = SmdSatState::stopped;
    m_tcxo_warmup_time = DEFAULT_TCXO_WARMUP_TIME_SECONDS;
	this->shutdown();
	is_kmac_profil_loaded = false; 
	return;
}
SmdSat::~SmdSat() {
	power_off_immediate();
	return;
}
void SmdSat::shutdown(void) {
	GPIOPins::clear(SAT_RESET);
	nrf_delay_ms(SMDSAT_DELAY_RST_MS); // Wait for the reset to take effect
	GPIOPins::clear(SAT_PWR_EN);
	return;
}

void SmdSat::state_machine(bool use_scheduler) {
	// Assume no delay between state machine invocations
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
		default:
			break;
		}

		if (use_scheduler && !SMD_STATE_EQUAL(stopped)) {
			// Invoke ourselves again if we are not stopped
			//DEBUG_TRACE("SmdSat::%s: reschedule in %u ms", __func__, m_next_delay);
			system_scheduler->cancel_task(m_task);
			m_task = system_scheduler->post_task_prio([this]() {
				state_machine();
			}, "SmdReceiverStateMachine", Scheduler::DEFAULT_PRIORITY, m_next_delay);
		}
		return;
	}


	void SmdSat::state_starting_enter() {
		return;
	}

	void SmdSat::state_starting_exit() {
		return;
	}
	void SmdSat::state_starting() 
	{
		m_is_first_tx = true;
		is_kmac_profil_loaded = false;
		m_nrf_spim = new NrfSPIM(SPI_SATELLITE);
		m_state_counter = 3;
		m_next_delay = 0;
		SMD_STATE_CHANGE(starting, powering_on);
		return;
	}

	void SmdSat::state_error_enter() {
		// Dump device status
		uint8_t status = 0;
		get_kmac_status(&status);
		notify(ArticEventDeviceError({}));

		SMD_STATE_CHANGE(error, stopped);
		return;
	}

	void SmdSat::state_error_exit() {
		return;
	}

	void SmdSat::state_error() {
		SMD_STATE_CHANGE(error, stopped);
		return;
	}


	void SmdSat::state_stopped_enter() {
		// Cleanup the SPIM instance
		delete m_nrf_spim;
		m_nrf_spim = nullptr; // Invalidate this pointer so if we call this function again it doesn't call delete on an invalid pointer

		
		// FIXME: should this be moved into the NrfSPIM driver?
		nrf_gpio_cfg_output(BSP::SPI_Inits[SPI_SATELLITE].config.ss_pin);
		nrf_gpio_pin_clear(BSP::SPI_Inits[SPI_SATELLITE].config.ss_pin);
		nrf_gpio_cfg_input(BSP::SPI_Inits[SPI_SATELLITE].config.mosi_pin, NRF_GPIO_PIN_PULLDOWN);
		nrf_gpio_cfg_input(BSP::SPI_Inits[SPI_SATELLITE].config.miso_pin, NRF_GPIO_PIN_PULLDOWN);
		nrf_gpio_cfg_input(BSP::SPI_Inits[SPI_SATELLITE].config.sck_pin, NRF_GPIO_PIN_PULLDOWN);

		// Power down the device
		this->shutdown();
		is_kmac_profil_loaded = false; 

		// Clear down any leftover flags
		//m_rx_pending = false;
		m_packet_buffer.clear();
		m_ack_buffer.clear();

		notify(ArticEventPowerOff({}));
		return;
	}

	void SmdSat::state_stopped_exit() {
		return;
	}

	void SmdSat::state_stopped() {
		return;
	}

	void SmdSat::state_powering_on_enter() {
		return;
	}

	void SmdSat::state_powering_on_exit() {
		m_next_delay = SMDSAT_DELAY_POWER_ON_MS;
		return;
	}

	void SmdSat::state_powering_on() {
		// Correct power-up sequence:
		// 1. Hold STM32WL in reset to minimize inrush current
		// 2. Enable power to SMD module
		// 3. Wait for power rails to stabilize
		// 4. Release reset
		DEBUG_TRACE("SmdSat::%s: Starting power-on sequence", __func__);

		GPIOPins::clear(SAT_RESET);  // Hold reset LOW - keeps STM32WL in reset (low power)
		nrf_delay_ms(10);            // Small delay before enabling power

		GPIOPins::set(SAT_PWR_EN);   // Enable power to SMD module via TPS22904
		DEBUG_TRACE("SmdSat::%s: Power enabled, waiting for stabilization", __func__);

		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);  // Wait for power to stabilize (500ms)

		GPIOPins::set(SAT_RESET);    // Release reset (HIGH = run)
		DEBUG_TRACE("SmdSat::%s: Reset released", __func__);

		SMD_STATE_CHANGE(powering_on, idle_pending);
		return;
	}

	void SmdSat::state_load_kmac_enter() {
		return;
	}

	void SmdSat::state_load_kmac_exit() {
		return;
	}

	void SmdSat::state_load_kmac() {
		uint8_t kmac_status = 0;
		get_kmac_status(&kmac_status);
		if (kmac_status == MAC_OK) {
			if (!is_kmac_profil_loaded) {
				load_kmac_profil(1);
				m_next_delay = SMDSAT_DELAY_LOAD_KMAC_MS;
				is_kmac_profil_loaded = true;
			} else {
				m_next_delay = SMDSAT_DELAY_CMD_MS;
				DEBUG_TRACE("SmdSat::%s: KMAC profil already loaded", __func__);
			}
			SMD_STATE_CHANGE(load_kmac, idle_pending);
		} else {
			// Check retries
			if (--m_state_counter == 0) {
				// Failed to go IDLE
				DEBUG_ERROR("SmdSat::%s: failed to enter load kmac state",__func__);
				SMD_STATE_CHANGE(load_kmac, error);
			} else {
				m_next_delay = SMDSAT_DELAY_TICK_INTERRUPT_MS;
			}
		}
		return;
	}
	//TODO check if we use LPM or just turn ON/OFF device WAKEUP PIN USED in this case no CMD
	void SmdSat::state_idle_pending_enter() {
	m_next_delay = SMDSAT_DELAY_CMD_MS;
	m_state_counter = 10;
	return;
}

void SmdSat::state_idle_pending_exit() {
	m_next_delay = SMDSAT_DELAY_CMD_MS;
	return;
}

void SmdSat::state_idle_pending() {
	if (smd_ping()) {
		if (!is_kmac_profil_loaded)
		{
			SMD_STATE_CHANGE(idle_pending, load_kmac);
		} else {
			SMD_STATE_CHANGE(idle_pending, idle);
		}
	} else {
		// Check retries
		if (--m_state_counter == 0) {
			// Failed to go IDLE
			DEBUG_ERROR("SmdSat::%s: failed to enter IDLE state",__func__);
			SMD_STATE_CHANGE(idle_pending, error);
		} else {
			m_next_delay = SMDSAT_DELAY_CMD_MS;
		}
	}
	return;
}

void SmdSat::state_idle_enter() {
	//TODO Check if we are already IDLE or not
	m_next_delay = SMDSAT_DELAY_TICK_INTERRUPT_MS;
	m_state_counter = m_idle_timeout_ms / SMDSAT_DELAY_TICK_INTERRUPT_MS;
	return;
}

void SmdSat::state_idle_exit() {
	m_next_delay = SMDSAT_DELAY_CMD_MS;
	return;
}

void SmdSat::state_idle() {
	//DEBUG_TRACE("SmdSat::%s::enter %u",__func__, m_packet_buffer.length());
	// Check for any new commands
	if (m_packet_buffer.length()) {
		m_tx_buffer = m_packet_buffer;
		m_packet_buffer.clear();
		SMD_STATE_CHANGE(idle, transmit_pending);
	} else if (m_ack_buffer.length()) {
		m_tx_buffer = m_ack_buffer;
		m_ack_buffer.clear();
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

	// Reset state timeout counter
	m_state_counter = m_idle_timeout_ms / SMDSAT_DELAY_TICK_INTERRUPT_MS;
	return;
	
}
void SmdSat::state_transmit_pending_enter() {
/* 	m_state_counter = 100;
	if (m_tx_buffer.size())
		initiate_tx();
	else {
		SMD_STATE_CHANGE(transmit_pending, idle);
	 }*/
	return;
}

void SmdSat::state_transmit_pending_exit() {
	m_next_delay = SMDSAT_DELAY_CMD_TX + (m_tcxo_warmup_time*1000);
	return;
	
}

void SmdSat::state_transmit_pending() {
	if (m_tx_buffer.size()) {
		initiate_tx();
		// Clear the interrupt
        notify(ArticEventTxStarted({}));
        SMD_STATE_CHANGE(transmit_pending, transmitting);
	} else {
		SMD_STATE_CHANGE(transmit_pending, idle);
		// Check retries
		if (--m_state_counter == 0) {
			DEBUG_ERROR("ArticSat::%s: failed accept SEND command",__func__);
	        SMD_STATE_CHANGE(transmit_pending, error);
		} else
			m_next_delay = SMDSAT_DELAY_CMD_MS;
	}
	return;
}

void SmdSat::state_transmitting_enter() {
    DEBUG_TRACE("SmdSat::%s", __func__);
	// Timeout calculation must allow for TCXO warm-up time on first TX, otherwise
	// use a nominal timeout since no TCXO warm-up will be used
	// TODO: NO TCXO timeout
	//m_state_counter = m_is_first_tx ? (500 + (m_tcxo_warmup_time * 100)) : 500;
	m_state_counter = 5;
	
	//send_command(ARTIC_CMD_START_TX_1M_SLEEP);
	return;
}

void SmdSat::state_transmitting_exit() {
	//TODO: First tx not used
	m_is_first_tx = false; 
	return;
}

void SmdSat::state_transmitting() {
	if (is_tx_finished()) {
		// Clear the interrupt
		// Check if transmission was aborted by user
        if (m_tx_buffer.size()) {
        	m_tx_buffer.clear();
        	notify(ArticEventTxComplete({}));
        }
        SMD_STATE_CHANGE(transmitting, stopped);
	} else {
		if (!m_tx_buffer.size()) {
			// Abort transmission
			SMD_STATE_CHANGE(transmitting, stopped);
		} else if (--m_state_counter == 0) {
			DEBUG_ERROR("SmdSat::%s: failed to complete TX",__func__);
			SMD_STATE_CHANGE(transmitting, error);
		} else
			m_next_delay = SMDSAT_DELAY_CMD_MS;
	}
	return;
}
void SmdSat::stop_send() {
	DEBUG_TRACE("SmdSat::%s",__func__);
	m_ack_buffer.clear();
	m_packet_buffer.clear();
	m_tx_buffer.clear();
	return;
}

void SmdSat::start_receive(const ArgosMode mode) {
	DEBUG_TRACE("SmdSat::%s",__func__);
	return;
}
bool SmdSat::stop_receive() {
	DEBUG_TRACE("SmdSat::%s",__func__);
	return false;
}

void SmdSat::set_frequency(const double freq) {
	DEBUG_TRACE("SmdSat::%s",__func__);
	m_tx_freq = freq;
	return;
}
void SmdSat::initiate_tx() {
	DEBUG_TRACE("SmdSat::%s:Size : %u",__func__, m_tx_buffer.size());

	// Wait for SPI to be ready before starting TX sequence
	if (!wait_for_spi_ready()) {
		DEBUG_ERROR("SmdSat::%s: SPI not ready, aborting TX", __func__);
		return;
	}

	// Send TX request command with retry
	if (!send_command_with_retry(SMDSAT_CMD_WRITE_TX_REQ)) {
		DEBUG_ERROR("SmdSat::%s: Failed to send TX request", __func__);
		return;
	}

	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);

	uint16_t size = m_tx_buffer.size();
	uint8_t tx_req[SMDSAT_CMD_WRITETX_LEN] = {0};
	uint8_t rx_req[SMDSAT_CMD_WRITETX_LEN] = {0};
	tx_req[0] = SMDSAT_CMD_WRITE_TX_SIZE;
	tx_req[1] = (size >> 8) & 0xFF;
	tx_req[2] = (size) & 0xFF;

	// Send TX size with retry
	if (!send_command_with_retry(tx_req, rx_req, SMDSAT_CMD_WRITETX_LEN)) {
		DEBUG_ERROR("SmdSat::%s: Failed to send TX size", __func__);
		return;
	}

	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);

	uint8_t *tx_msg = NULL;
	uint8_t *rx_msg = NULL;
	tx_msg = static_cast<uint8_t*>(calloc(size + 1, sizeof(uint8_t)));
	rx_msg = static_cast<uint8_t*>(calloc(size + 1, sizeof(uint8_t)));

	if (tx_msg == NULL || rx_msg == NULL) {
		DEBUG_ERROR("SmdSat::%s: Memory allocation failed for TX/RX buffers", __func__);
		free(tx_msg);
		free(rx_msg);
		return;
	}

	tx_msg[0] = SMDSAT_CMD_WRITE_TX;

	// Copy the content of m_tx_buffer into tx_msg starting at index 1
	memcpy(tx_msg + 1, m_tx_buffer.data(), size);

	// Send TX payload with retry
	if (!send_command_with_retry(tx_msg, rx_msg, size + 1)) {
		DEBUG_ERROR("SmdSat::%s: Failed to send TX payload", __func__);
	}

	// Free allocated memory
	free(tx_msg);
	free(rx_msg);
	return;
}
void SmdSat::send(const ArgosMode mode, const ArticPacket& user_payload, const unsigned int payload_length)
{
	// Log the payload length to be sent
    DEBUG_TRACE("SmdSat::%s: length %u", __func__, payload_length);
;
    // Determine the maximum allowed payload size based on the current modulation type
    unsigned int max_payload_size = 0;
    switch(m_modulation) {
        case ARGOS_MOD_LDA2:
            max_payload_size = ARGOS_TX_LDA2_PAYLOAD_BYTE_SIZE;
            break;
        case ARGOS_MOD_LDA2L:
            // If you have a different value for LDA2L, use it here.
            // Otherwise, assuming it is the same as LDA2:
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

	// Determine the effective payload length:
    //  - Use the minimum of the provided payload_length and the size of user_payload.
    //  - If that value is greater than max_payload_size, we truncate it.
    unsigned int effective_payload_length = std::min(payload_length,
                                                     static_cast<unsigned int>(user_payload.size()));
    if (effective_payload_length > max_payload_size) {
        DEBUG_TRACE("SmdSat::%s: Payload size (%u bytes) exceeds maximum allowed (%u bytes) for modulation %d; truncating",
                    __func__, effective_payload_length, max_payload_size, m_modulation);
        effective_payload_length = max_payload_size;
    }


    // Initialize (or reinitialize) m_packet_buffer with the proper payload length.
    // This assigns max_payload_size bytes initialized to zero.
    m_packet_buffer.assign(max_payload_size, 0);

    // Copy the actual user payload into m_packet_buffer.
    // This copies only 'payload_length' bytes.
    std::copy(user_payload.begin(), user_payload.begin() + effective_payload_length, m_packet_buffer.begin());

    // Debug log: print the raw hexified packet buffer.
    DEBUG_TRACE("SmdSat::%s: raw data[%u]=%s", __func__,
                effective_payload_length,
                Binascii::hexlify(m_packet_buffer).c_str());

    // Set the TX mode
    m_tx_mode = mode;
    DEBUG_TRACE("SmdSat::%s::Packet size %u", __func__, static_cast<unsigned int>(m_packet_buffer.size()));

    // Power on if not already running
    power_on();

	return;
}

void SmdSat::send_ack(const ArgosMode mode, const unsigned int a_dcs, const unsigned int dl_msg_id, const unsigned int exec_report)
{
	DEBUG_TRACE("SmdSat::send_ack : Not implemented for the moment");
	return;
}
void SmdSat::set_tcxo_warmup_time(const unsigned int time_s) {
	m_tcxo_warmup_time = time_s;
	return;
}

void SmdSat::set_tx_power(const BaseArgosPower power) {
	m_tx_power = power;
	return;
}

void SmdSat::set_credentials(const unsigned int dec_id, const unsigned int address, const std::string seckey, const std::string radioconf) {
	DEBUG_TRACE("SmdSat::%s",__func__);
	bool stop_spi = false;
	bool was_stopped = (m_state == SmdSatState::stopped);

	// Helper lambda to wait between commands - use fixed delay to avoid watchdog issues on SMD
	auto wait_between_commands = [this]() -> bool {
		nrf_delay_ms(SMDSAT_DELAY_CMD_MS * 2);
		return true;  // Just wait, don't poll - polling can trigger watchdog on SMD
	};

	if (was_stopped)
	{
		// Power-up sequence: enable power, then release reset with proper timing
		GPIOPins::clear(SAT_RESET);  // Hold reset LOW while powering up
		GPIOPins::set(SAT_PWR_EN);   // Enable power to SMD module
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);  // Wait for power to stabilize
		GPIOPins::set(SAT_RESET);    // Release reset (HIGH = run)
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);  // Wait for STM32WL to boot and init SPI
	}
	if (m_nrf_spim == nullptr)
	{
		m_nrf_spim = new NrfSPIM(SPI_SATELLITE);
		stop_spi = true;
	}

	// Wait for SMD to be fully ready using ping with retries
	// The SPI status can return IDLE while the SMD is still processing internal queues (KNS_Q)
	// Ping ensures the firmware is actually ready to accept commands
	{
		bool smd_ready = false;
		for (uint8_t attempt = 0; attempt < 10; attempt++) {
			nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS / 2);  // 500ms between attempts
			if (smd_ping()) {
				DEBUG_INFO("SmdSat::%s: SMD ready after %u attempts", __func__, attempt + 1);
				smd_ready = true;
				break;
			}
			DEBUG_TRACE("SmdSat::%s: Waiting for SMD to be ready (attempt %u/10)", __func__, attempt + 1);
		}
		if (!smd_ready) {
			DEBUG_ERROR("SmdSat::%s: SMD not ready after power-on, aborting", __func__);
			goto cleanup;
		}
	}

	// Set the ID with retry
	for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
		try {
			set_id(dec_id);
			break;
		} catch (...) {
			if (retry == SMDSAT_SPI_MAX_RETRIES - 1) {
				DEBUG_ERROR("SmdSat::%s: Failed to set ID after %u retries", __func__, SMDSAT_SPI_MAX_RETRIES);
				goto cleanup;
			}
			DEBUG_WARN("SmdSat::%s: set_id retry %u/%u", __func__, retry + 1, SMDSAT_SPI_MAX_RETRIES);
			nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
		}
	}

	if (!wait_between_commands()) {
		DEBUG_ERROR("SmdSat::%s: SPI not ready after set_id", __func__);
		goto cleanup;
	}

	// Set address with retry
	{
		uint8_t address_data[4] = {
			static_cast<uint8_t>((address >> 24) & 0xFF),
			static_cast<uint8_t>((address >> 16) & 0xFF),
			static_cast<uint8_t>((address >> 8) & 0xFF),
			static_cast<uint8_t>(address & 0xFF)
		};
		uint8_array_t address_val = {SMDSAT_CMD_WRITE_ADDR_LEN-1, address_data};

		for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
			try {
				set_address(&address_val);
				break;
			} catch (...) {
				if (retry == SMDSAT_SPI_MAX_RETRIES - 1) {
					DEBUG_ERROR("SmdSat::%s: Failed to set address after %u retries", __func__, SMDSAT_SPI_MAX_RETRIES);
					goto cleanup;
				}
				DEBUG_WARN("SmdSat::%s: set_address retry %u/%u", __func__, retry + 1, SMDSAT_SPI_MAX_RETRIES);
				nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
			}
		}
	}

	if (!wait_between_commands()) {
		DEBUG_ERROR("SmdSat::%s: SPI not ready after set_address", __func__);
		goto cleanup;
	}

	// Set seckey with retry
	{
		std::string seckey_val = Binascii::unhexlify(seckey);
		uint8_array_t seckey_struct = {static_cast<uint16_t>(seckey_val.size()),
		                               reinterpret_cast<uint8_t *>(seckey_val.data())};

		for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
			try {
				set_seckey(&seckey_struct);
				break;
			} catch (...) {
				if (retry == SMDSAT_SPI_MAX_RETRIES - 1) {
					DEBUG_ERROR("SmdSat::%s: Failed to set seckey after %u retries", __func__, SMDSAT_SPI_MAX_RETRIES);
					goto cleanup;
				}
				DEBUG_WARN("SmdSat::%s: set_seckey retry %u/%u", __func__, retry + 1, SMDSAT_SPI_MAX_RETRIES);
				nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
			}
		}
	}

	if (!wait_between_commands()) {
		DEBUG_ERROR("SmdSat::%s: SPI not ready after set_seckey", __func__);
		goto cleanup;
	}

	// Set radio config with retry
	{
		std::string radioconf_val = Binascii::unhexlify(radioconf);
		uint8_array_t radioconf_struct = {static_cast<uint16_t>(radioconf_val.size()),
		                                  reinterpret_cast<uint8_t *>(radioconf_val.data())};

		for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
			try {
				set_radio_conf(&radioconf_struct);
				break;
			} catch (...) {
				if (retry == SMDSAT_SPI_MAX_RETRIES - 1) {
					DEBUG_ERROR("SmdSat::%s: Failed to set radio conf after %u retries", __func__, SMDSAT_SPI_MAX_RETRIES);
					goto cleanup;
				}
				DEBUG_WARN("SmdSat::%s: set_radio_conf retry %u/%u", __func__, retry + 1, SMDSAT_SPI_MAX_RETRIES);
				nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
			}
		}
	}

	// Wait before save - flash operations take more time
	wait_between_commands();

	// Save radio config with retry
	for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
		try {
			if (save_radio_conf()) {
				break;
			}
			// save_radio_conf returned false - retry
			if (retry == SMDSAT_SPI_MAX_RETRIES - 1) {
				DEBUG_ERROR("SmdSat::%s: save_radio_conf failed after %u retries", __func__, SMDSAT_SPI_MAX_RETRIES);
				goto cleanup;
			}
		} catch (...) {
			if (retry == SMDSAT_SPI_MAX_RETRIES - 1) {
				DEBUG_ERROR("SmdSat::%s: Failed to save radio conf after %u retries", __func__, SMDSAT_SPI_MAX_RETRIES);
				goto cleanup;
			}
		}
		DEBUG_WARN("SmdSat::%s: save_radio_conf retry %u/%u", __func__, retry + 1, SMDSAT_SPI_MAX_RETRIES);
		nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	}

	// Wait after flash save
	wait_between_commands();

	// Load KMAC profile with retry
	for (uint8_t retry = 0; retry < SMDSAT_SPI_MAX_RETRIES; retry++) {
		try {
			load_kmac_profil(1);
			break;
		} catch (...) {
			if (retry == SMDSAT_SPI_MAX_RETRIES - 1) {
				DEBUG_ERROR("SmdSat::%s: Failed to load KMAC profile after %u retries", __func__, SMDSAT_SPI_MAX_RETRIES);
				goto cleanup;
			}
			DEBUG_WARN("SmdSat::%s: load_kmac_profil retry %u/%u", __func__, retry + 1, SMDSAT_SPI_MAX_RETRIES);
			nrf_delay_ms(SMDSAT_SPI_RETRY_DELAY_MS);
		}
	}

	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	DEBUG_INFO("SmdSat::%s: Credentials set successfully", __func__);

cleanup:
	if (stop_spi)
	{
		delete m_nrf_spim;
		m_nrf_spim = nullptr;
	}
	if (was_stopped)
	{
		shutdown();
	}
}

// TODO: implement read of raw radio conf value
void SmdSat::read_credentials(unsigned int *dec_id, unsigned int *address, std::string *seckey, std::string *radioconf) {
	DEBUG_TRACE("SmdSat::%s",__func__);
	bool stop_spi = false;
	if (m_state == SmdSatState::stopped)
	{
		// Power-up sequence: enable power, then release reset with proper timing
		GPIOPins::clear(SAT_RESET);  // Hold reset LOW while powering up
		GPIOPins::set(SAT_PWR_EN);   // Enable power to SMD module
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);  // Wait for power to stabilize
		GPIOPins::set(SAT_RESET);    // Release reset (HIGH = run)
		nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);  // Wait for STM32WL to boot and init SPI
	}
	if (m_nrf_spim == nullptr)
	{
		m_nrf_spim = new NrfSPIM(SPI_SATELLITE);
		stop_spi = true;
	}
	nrf_delay_ms(SMDSAT_DELAY_POWER_ON_MS);

	// Read the ID
	 if (dec_id) {
		uint32_t dec_id_val = 0;
        read_id(&dec_id_val);
		*dec_id = static_cast<unsigned int>(dec_id_val);

		nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
    }

    // Read the Address
    if (address) {
        uint8_t address_data[SMDSAT_CMD_READ_ADDR_LEN] = {0};
        uint8_array_t address_value = {SMDSAT_CMD_READ_ADDR_LEN, address_data};
        read_address(&address_value);
		nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
		*address  = (address_value.p_data[0] << 24) |
					(address_value.p_data[1] << 16) |
					(address_value.p_data[2] << 8)  |
					(address_value.p_data[3]);
    }

    // Read the Security Key and Convert to Hex String
    if (seckey) {
        uint8_t seckey_data[SMDSAT_CMD_READ_SECKEY_LEN] = {0};
        uint8_array_t seckey_value = {SMDSAT_CMD_READ_SECKEY_LEN, seckey_data};
        read_seckey(&seckey_value);
		nrf_delay_ms(SMDSAT_DELAY_CMD_MS);

        // Convert binary seckey data to hex and assign it to *seckey
        *seckey = Binascii::hexlify(std::string(reinterpret_cast<char *>(seckey_data), SMDSAT_CMD_READ_SECKEY_LEN));
    }

    // Read the Radio Configuration and Convert to Hex String
	ArgosModulation modulation_dev;
	read_radio_conf(&modulation_dev);
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);

	if(stop_spi)
	{
		delete m_nrf_spim;
		m_nrf_spim = nullptr; // Invalidate this pointer so if we call this function again it doesn't call delete on an invalid pointer
	}
	if (m_state == SmdSatState::stopped) {
		shutdown();
	}
}