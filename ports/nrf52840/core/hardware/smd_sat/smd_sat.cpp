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
}

void SmdSat::send_command(uint8_t command) {
    uint8_t buffer_read;
    int ret;
	DEBUG_TRACE("%s::Send %u",__func__, command);

    ret = m_nrf_spim->transfer(&command, &buffer_read, sizeof(command));
    if (ret) {
       throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSat::send_command(const uint8_t *tx_data, uint8_t *rx_data, uint16_t size) {
    int ret = m_nrf_spim->transfer(tx_data, rx_data, size);
    if (ret) {
       throw ErrorCode::SPI_COMMS_ERROR;
	}
}

void SmdSat::reload_kmac_profil()
{
	// send_command(SMDSAT_CMD_WRITE_KMAC_REQ);

	uint8_t tx[SMDSAT_CMD_WRITEKMAC_LEN] = {0};
	uint8_t rx[SMDSAT_CMD_WRITEKMAC_LEN] = {0};
	// tx[0] = SMDSAT_CMD_WRITE_KMAC;
	// tx[1] = 0;
	// nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	// send_command(tx, rx, (uint16_t)SMDSAT_CMD_WRITEKMAC_LEN);
	
	// nrf_delay_ms(SMDSAT_DELAY_LOAD_KMAC_MS);
	
	send_command(SMDSAT_CMD_WRITE_KMAC_REQ);
	tx[0] = SMDSAT_CMD_WRITE_KMAC;
	tx[1] = 1;
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	send_command(tx, rx, SMDSAT_CMD_WRITEKMAC_LEN);
}

void SmdSat::get_spi_status(uint8_t *status) {
	send_command(SMDSAT_CMD_SPI_STATUS);
	read_byte(status);
	DEBUG_TRACE("SmdSat::spi status=%s", (char *)spistatus_string[*status]);
}

void SmdSat::get_kmac_status(uint8_t *status) {
	send_command(SMDSAT_CMD_MAC_STATUS);
	read_byte(status);
	DEBUG_TRACE("SmdSat::kmac status=%s", (char *)kmacstatus_string[*status]);
}

void SmdSat::set_tcxo_warmup(uint32_t time_s)
{
    DEBUG_TRACE("SmdSat::set_tcxo_warmup:TODO");
}
void SmdSat::set_tcxo_control(bool state) {
    DEBUG_TRACE("SmdSat::set_tcxo_control:TODO");
}
void SmdSat::print_firmware_version() {
	send_command(SMDSAT_CMD_READ_FIRMWARE);
	uint8_t tx[SMDSAT_CMD_READ_FIRMWARE_LEN] = {0};
	uint8_t rx[SMDSAT_CMD_READ_FIRMWARE_LEN] = {0};
	send_command(tx, rx, (uint16_t)SMDSAT_CMD_READ_FIRMWARE_LEN);
    DEBUG_TRACE("SmdSat::print_firmware_version: %s", rx);
}

bool SmdSat::is_idle_state() {
    DEBUG_TRACE("SmdSat::%s:Waiting spi=WAITING_RX and kmac=MAC_OK",__func__);
	uint8_t status = 0;
    get_spi_status(&status);
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	
	uint8_t kmac_status = 0;
    get_kmac_status(&kmac_status);
	
	if ((status == SMDSAT_SPICMD_WAITING_RX) && (kmac_status == MAC_OK)) {
		return true;
	} else {
		return false;
	}
}
bool SmdSat::is_idle() {
    DEBUG_TRACE("SmdSat::%s",__func__);
	uint8_t status = 0;
    get_spi_status(&status);
	if (status == SMDSAT_SPICMD_WAITING_RX) {
		return true;
	} else {
		return false;
	}
}

bool SmdSat::smd_ping()
{
    DEBUG_TRACE("SmdSat::%s",__func__);
	send_command(SMDSAT_CMD_PING);
	uint8_t ping = 0;
	read_byte(&ping);
	if (ping == 1) {
		DEBUG_INFO("SmdSat::Ping ACK from SMD received");
		return true;
	} else {
		DEBUG_INFO("SmdSat::Ping not received : received : %u", ping);
		return false; 
	}
}
bool SmdSat::is_tx_finished() {
    DEBUG_TRACE("SmdSat::is_tx_finished:");
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
    DEBUG_TRACE("SmdSat::is_tx_in_progress:TODO");
	
	return true;
}

bool SmdSat::is_command_accepted() {
    DEBUG_TRACE("SmdSat::is_command_accepted:TODO");
	
	return true;
}

bool SmdSat::is_firmware_ready() {
    DEBUG_TRACE("SmdSat::is_firmware_ready:TODO");
	
	return true;
}
void SmdSat::power_off() {
    DEBUG_TRACE("ArsgosSmd::power_off");

    // The state machine will safely transition to stopped as soon
    // as all pending activities have completed
    if (!SMD_STATE_EQUAL(stopped)) {
    	m_stopping = true;
    }
}
void SmdSat::power_on() {
    DEBUG_TRACE("SmdSat::power_on");

    // Device is already powered
    if (m_state != SmdSat::stopped) {
    	m_stopping = false;  // Clear any pending stopping flag
    	DEBUG_TRACE("SmdSat::power_on: not disabled: state=%u", (unsigned int)m_state);
    	return;
    }
	
    // Set top-level state variables
    m_state = SmdSatState::starting; // Force a reset when the task starts
    m_stopping = false;

    // This will run the state machine perpetually until a power_off is called
    DEBUG_TRACE("SmdSat::start state machine");
    state_machine();
}


void SmdSat::power_off_immediate()
{
    DEBUG_TRACE("SmdSat::power_off_immediate");

    // We force a power off by cancelling the state machine and
    // forcing a stopped state
    if (!SMD_STATE_EQUAL(stopped)) {
    	system_scheduler->cancel_task(m_task);
    	SMD_STATE_CHANGE(idle, stopped);
    }
}



SmdSat::SmdSat(unsigned int idle_shutdown_ms) {
	DEBUG_TRACE("SmdSat::Constructor");
	set_idle_timeout(idle_shutdown_ms);
    m_ack_buffer.clear();
    m_packet_buffer.clear();
	m_modulation = ARGOS_MOD_LDA2;
    m_nrf_spim = nullptr;
    m_state = SmdSatState::stopped;
    m_tcxo_warmup_time = DEFAULT_TCXO_WARMUP_TIME_SECONDS;
	this->shutdown();
	is_kmac_profil_loaded = false; 
}
SmdSat::~SmdSat() {
	power_off_immediate();
}
void SmdSat::shutdown(void) {
	GPIOPins::clear(SAT_RESET);
	GPIOPins::clear(SAT_PWR_EN);
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
		DEBUG_TRACE("SmdSat::state_machine: reschedule in %u ms", m_next_delay);
		system_scheduler->cancel_task(m_task);
		m_task = system_scheduler->post_task_prio([this]() {
			state_machine();
		}, "SmdReceiverStateMachine", Scheduler::DEFAULT_PRIORITY, m_next_delay);
	}
}


void SmdSat::state_starting_enter() {
}

void SmdSat::state_starting_exit() {
}
void SmdSat::state_starting() 
{
    m_is_first_tx = true;
	is_kmac_profil_loaded = false;
	m_nrf_spim = new NrfSPIM(SPI_SATELLITE);
	m_state_counter = 3;
	m_next_delay = 0;
	SMD_STATE_CHANGE(starting, powering_on);
}

void SmdSat::state_error_enter() {
	// Dump device status
	uint8_t status = 0;
	get_spi_status(&status);

	notify(ArticEventDeviceError({}));

	SMD_STATE_CHANGE(error, stopped);
}

void SmdSat::state_error_exit() {
}

void SmdSat::state_error() {
	SMD_STATE_CHANGE(error, stopped);
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
}

void SmdSat::state_stopped_exit() {
}

void SmdSat::state_stopped() {
}

void SmdSat::state_powering_on_enter() {
}

void SmdSat::state_powering_on_exit() {
	m_next_delay = SMDSAT_DELAY_POWER_ON_MS;
}

void SmdSat::state_powering_on() {
    GPIOPins::set(SAT_PWR_EN);
    GPIOPins::set(SAT_RESET);
    SMD_STATE_CHANGE(powering_on, load_kmac);
}

void SmdSat::state_load_kmac_enter() {
}

void SmdSat::state_load_kmac_exit() {
	m_next_delay = SMDSAT_DELAY_LOAD_KMAC_MS;
}

void SmdSat::state_load_kmac() {
	if (smd_ping()) {
		if (!is_kmac_profil_loaded) {
			nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
			reload_kmac_profil();
			is_kmac_profil_loaded = true;
		} else {
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
}
//TODO check if we use LPM or just turn ON/OFF device WAKEUP PIN USED in this case no CMD
void SmdSat::state_idle_pending_enter() {
	if (!is_kmac_profil_loaded)
	{
		SMD_STATE_CHANGE(idle_pending, load_kmac);
	}
	m_next_delay = SMDSAT_DELAY_CMD_MS;
}

void SmdSat::state_idle_pending_exit() {
	m_next_delay = SMDSAT_DELAY_CMD_MS;
}

void SmdSat::state_idle_pending() {
	if (is_idle_state()) {
		SMD_STATE_CHANGE(idle_pending, idle);
	} else {
		// Check retries
		if (--m_state_counter == 0) {
			// Failed to go IDLE
			DEBUG_ERROR("SmdSat::state_machine: failed to enter IDLE state");
			SMD_STATE_CHANGE(idle_pending, error);
		} else {
			m_next_delay = SMDSAT_DELAY_CMD_MS;
		}
	}
}

void SmdSat::state_idle_enter() {
	//TODO Check if we are already IDLE or not
	m_next_delay = SMDSAT_DELAY_TICK_INTERRUPT_MS;
	m_state_counter = m_idle_timeout_ms / SMDSAT_DELAY_TICK_INTERRUPT_MS;
}

void SmdSat::state_idle_exit() {
	m_next_delay = SMDSAT_DELAY_CMD_MS;
}

void SmdSat::state_idle() {
	DEBUG_TRACE("%s::enter %u",__func__, m_packet_buffer.length());
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
		 	DEBUG_TRACE("ArticSat::state_idle: idle timeout elapsed");
			SMD_STATE_CHANGE(idle, stopped);
		}
		return;
	}

	// Reset state timeout counter
	m_state_counter = m_idle_timeout_ms / SMDSAT_DELAY_TICK_INTERRUPT_MS;
	
}
void SmdSat::state_transmit_pending_enter() {
/* 	m_state_counter = 100;
	if (m_tx_buffer.size())
		initiate_tx();
	else {
		SMD_STATE_CHANGE(transmit_pending, idle);
	 }*/
}

void SmdSat::state_transmit_pending_exit() {
	m_next_delay = SMDSAT_DELAY_CMD_TX;
	
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
			DEBUG_ERROR("ArticSat::state_machine: failed accept SEND command");
	        SMD_STATE_CHANGE(transmit_pending, error);
		} else
			m_next_delay = SMDSAT_DELAY_CMD_MS;
	}
}

void SmdSat::state_transmitting_enter() {
	// Timeout calculation must allow for TCXO warm-up time on first TX, otherwise
	// use a nominal timeout since no TCXO warm-up will be used
	// TODO: NO TCXO timeout
	//m_state_counter = m_is_first_tx ? (500 + (m_tcxo_warmup_time * 100)) : 500;
	m_state_counter = 100; 
	
	//send_command(ARTIC_CMD_START_TX_1M_SLEEP);
    DEBUG_TRACE("SmdSat::state_transmitting_enter:TODO");
}

void SmdSat::state_transmitting_exit() {
	//TODO: First tx not used
	m_is_first_tx = false; 
}

void SmdSat::state_transmitting() {
	if (is_tx_finished()) {
		// Clear the interrupt
		// Check if transmission was aborted by user
        if (m_tx_buffer.size()) {
        	m_tx_buffer.clear();
        	notify(ArticEventTxComplete({}));
        }
        SMD_STATE_CHANGE(transmitting, idle);
	} else {
		if (!m_tx_buffer.size()) {
			// Abort transmission
			SMD_STATE_CHANGE(transmitting, stopped);
		} else if (--m_state_counter == 0) {
			DEBUG_ERROR("SmdSat::state_machine: failed to complete TX");
			SMD_STATE_CHANGE(transmitting, error);
		} else
			m_next_delay = SMDSAT_DELAY_CMD_MS;
	}
}
void SmdSat::stop_send() {
	DEBUG_TRACE("SmdSat::stop_send: TODO");
	m_ack_buffer.clear();
	m_packet_buffer.clear();
	m_tx_buffer.clear();
}

void SmdSat::start_receive(const ArticMode mode) {
	DEBUG_TRACE("SmdState::start_receive: not implemented");
}
bool SmdSat::stop_receive() {
	DEBUG_TRACE("SmdSat::stop_receive: not implemented");
	return false;
}

void SmdSat::set_frequency(const double freq) {
	DEBUG_TRACE("SmdSat::set_frequency: Configure radiomodule conf");
	m_tx_freq = freq;
}
void SmdSat::initiate_tx() {
	DEBUG_TRACE("SmdSat::%s:Size : %u",__func__, m_tx_buffer.size());
	//TODO check modulation according correct size.
	//LDA2 used for the moment. 
	send_command(SMDSAT_CMD_WRITE_TX_REQ);

	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	//uint16_t size = m_tx_buffer.size();
	uint16_t size = m_tx_buffer.size();
	uint8_t tx_req[SMDSAT_CMD_WRITETX_LEN] = {0};
	uint8_t rx_req[SMDSAT_CMD_WRITETX_LEN] = {0};
	tx_req[0] = SMDSAT_CMD_WRITE_TX_SIZE;
	tx_req[1] =  (size >>8) & 0xFF;
	tx_req[2] =  (size) & 0xFF;

	send_command(tx_req, rx_req, SMDSAT_CMD_WRITETX_LEN);
	nrf_delay_ms(SMDSAT_DELAY_CMD_MS);
	
	uint8_t *tx_msg = NULL;
    uint8_t *rx_msg = NULL;
    tx_msg = static_cast<uint8_t*>(calloc(size + 1, sizeof(uint8_t)));
    rx_msg = static_cast<uint8_t*>(calloc(size + 1, sizeof(uint8_t)));

    if (tx_msg == NULL || rx_msg == NULL) {
        DEBUG_TRACE("Memory allocation failed for TX/RX buffers");
        free(tx_msg);
        free(rx_msg);
        return;
    }

    tx_msg[0] = SMDSAT_CMD_WRITE_TX;

	
    // Copy the content of m_tx_buffer into tx_msg starting at index 1
    memcpy(tx_msg + 1, m_tx_buffer.data(), size);
    //memcpy(tx_msg + 1, TXpayload_LDA2, size);

    send_command(tx_msg, rx_msg, size + 1);

    // Free allocated memory
    free(tx_msg);
    free(rx_msg);

}
void SmdSat::send(const ArticMode mode, const ArticPacket& user_payload, const unsigned int payload_length)
{
	// Log the payload length to be sent
    DEBUG_TRACE("SmdSat::send: length %u", payload_length);

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
            DEBUG_TRACE("SmdSat::send: Unknown modulation type %d", m_modulation);
            return;
    }

	// Determine the effective payload length:
    //  - Use the minimum of the provided payload_length and the size of user_payload.
    //  - If that value is greater than max_payload_size, we truncate it.
    unsigned int effective_payload_length = std::min(payload_length,
                                                     static_cast<unsigned int>(user_payload.size()));
    if (effective_payload_length > max_payload_size) {
        DEBUG_TRACE("SmdSat::send: Payload size (%u bytes) exceeds maximum allowed (%u bytes) for modulation %d; truncating",
                    effective_payload_length, max_payload_size, m_modulation);
        effective_payload_length = max_payload_size;
    }


    // Initialize (or reinitialize) m_packet_buffer with the proper payload length.
    // This assigns max_payload_size bytes initialized to zero.
    m_packet_buffer.assign(max_payload_size, 0);

    // Copy the actual user payload into m_packet_buffer.
    // This copies only 'payload_length' bytes.
    std::copy(user_payload.begin(), user_payload.begin() + effective_payload_length, m_packet_buffer.begin());

    // Debug log: print the raw hexified packet buffer.
    DEBUG_TRACE("ArticSat::send_packet: raw data[%u]=%s",
                effective_payload_length,
                Binascii::hexlify(m_packet_buffer).c_str());

    // Set the TX mode
    m_tx_mode = mode;
    DEBUG_TRACE("Packet size %u", static_cast<unsigned int>(m_packet_buffer.size()));

    // Power on if not already running
    power_on();

	// Setup TX operation
// 	DEBUG_TRACE("SmdSat::send: length %u", payload_length);

// 	m_packet_buffer = user_payload; 
//     // Directly copy the raw payload into m_packet_buffer
// //    m_packet_buffer.assign(user_payload, user_payload + payload_length);

//     // Debug log for confirmation
//     DEBUG_TRACE("ArticSat::send_packet: raw data[%u]=%s", 
//                 payload_length, 
//                 Binascii::hexlify(m_packet_buffer).c_str());

//     // Setup TX mode
//     m_tx_mode = mode;
//     DEBUG_TRACE("Packet size %u", m_packet_buffer.size());

//     // Power on if not already running
//     power_on();	
/* 	ArticPacket packet;
	unsigned int total_bits;
	unsigned int stuffing_bits = 0;
 */
	// // Only A2/A3 mode is supported
	// // TODO: check modulation add LDK / VLDA4 / A2 stuffing bit if necessary
	// if ((payload_length) & 31) {
	// 	// Stuff zeros at the end to align to nearest boundary
	// 	stuffing_bits = 32 - ((payload_length + 8) % 32);
	// 	DEBUG_TRACE("ArticSat::send_packet: adding %u stuffing bits for alignment", stuffing_bits);
	// }

	// // uint8_t length_encoded[] = { 0x0, 0x3, 0x5, 0x6, 0x9, 0xA, 0xC, 0xF };
	// // unsigned int length_idx = (stuffing_bits + payload_length - 8) / 32;
	// // unsigned int length_enc = length_encoded[length_idx];

	// unsigned int num_tail_bits = 0; // A2 mode
	// // if (mode == ArticMode::A3) {
	// // 	uint8_t tail_bits[] = { 7, 8, 9, 7, 8, 9, 7, 8 };
	// // 	num_tail_bits = tail_bits[length_idx];
	// // }
	// unsigned int op_offset = 0;
	// unsigned int ip_offset = 0;

	// // Transmission is:
	// // MSG_LEN (4)
	// // ARGOSID (28)
	// // USER_PAYLOAD (n*32 - 8)
	// // STUFFING_BITS
	// // TAIL_BITS (7,8,9)
	// //total_bits = 4 + 28 + payload_length + stuffing_bits + num_tail_bits;
	// total_bits = payload_length + stuffing_bits + num_tail_bits;

	// // Assign the buffer to include 24-bit length indicator and rounded-up to 3 bytes for XMEM alignment
	// //packet.assign(((((total_bits + 24) + 23) / 24) * 24) / 8, 0);
	// packet.assign()

	// // Set total length
	// //PACK_BITS(total_bits, packet, op_offset, 24);

	// // Set header
	// //PACK_BITS(length_enc, packet, op_offset, 4);
	// //PACK_BITS(m_device_identifier, packet, op_offset, 28);

	// // Append user payload
	// unsigned int payload_bits_remaining = payload_length;
	// uint8_t byte;
	// while (payload_bits_remaining) {
	// 	unsigned int bits = std::min(8U, payload_bits_remaining);
	// 	payload_bits_remaining -= bits;
	// 	EXTRACT_BITS(byte, user_payload, ip_offset, bits);
	// 	PACK_BITS(byte, packet, op_offset, bits);
	// }

	// // Add any stuffing bits
	// payload_bits_remaining = stuffing_bits;
	// while (payload_bits_remaining) {
	// 	unsigned int bits = std::min(8U, payload_bits_remaining);
	// 	payload_bits_remaining -= bits;
	// 	PACK_BITS(0, packet, op_offset, bits);
	// }

	// // Add tail bits
	// //PACK_BITS(0, packet, op_offset, num_tail_bits);

	// // Setup TX operation
	// DEBUG_TRACE("ArticSat::send_packet: data[%u]=%s tail=%u",
	// 		total_bits, Binascii::hexlify(packet).c_str(),
	// 		num_tail_bits);

	// // Setup TX mode
	// m_tx_mode = mode;
	// DEBUG_TRACE("Packet size %u", packet.length())
	// //m_packet_buffer = packet;
	// m_packet_buffer = user_payload;

	// // Request power on (if not already running)
	// power_on();
}

void SmdSat::send_ack(const ArticMode mode, const unsigned int a_dcs, const unsigned int dl_msg_id, const unsigned int exec_report)
{
	DEBUG_TRACE("SmdSat::send_ack : Not implemented for the moment");
}
void SmdSat::set_tcxo_warmup_time(const unsigned int time_s) {
	m_tcxo_warmup_time = time_s;
}

void SmdSat::set_tx_power(const BaseArgosPower power) {
	m_tx_power = power;
}