#include <stdint.h>
#include <cstring>
#include <cmath>

#include "argos_smd.hpp"
#include "bsp.hpp"
#include "error.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include "scheduler.hpp"
#include "timer.hpp"
#include "binascii.hpp"
#include "crc8.hpp"
#include "crc16.hpp"
#include "rtc.hpp"

#include "nrf_delay.h"
#include "nrf_gpio.h"

#define SMD_DELAY_TICK_INTERRUPT_MS 10
#define INVALID_MEM_SELECTION (0xFF)

#ifndef DEFAULT_TCXO_WARMUP_TIME_SECONDS
#define DEFAULT_TCXO_WARMUP_TIME_SECONDS 5
#endif

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define TX_FREQUENCY_ARGOS_2_3_BAND_START		401.62

static constexpr const char *const status_string[] =
{
    "IDLE",                                   // The firmware is idle and ready to accept commands.
    "RX_IN_PROGRESS",                         // The firmware is receiving.
    "TX_IN_PROGRESS",                         // The firmware is transmitting.
    "BUSY",                                   // The firmware is changing state.
};

extern Scheduler *system_scheduler;
extern Timer *system_timer;
extern RTC *rtc;

static constexpr const char *const atcmd_string[] =
{
    "AT+VERSION",       // Get firmware version
    "AT+PING",          // Ping the module
    "AT+FW",            // Get firmware version
    "AT+ADDR",          // Get MAC address
    "AT+SN",            // Get serial number
    "AT+ID",            // Get device ID
    "AT+RCONF",         // Get/SET radio configuration
    "AT+SAVE_RCONF",    // SAVE RCONF to non-volatile memory
    "AT+LPM",           // GET/SET low power mode command
    "AT+TX=",           // Send raw data
    "AT+CW=",           // Set Continuous wave transmission command
    "AT+PREPASS_EN",    // Get/Set prepass 
    "AT+UDATE",         // UTC datetime update
    "AT+ATXRP"          // get/set repetition commands
};

void ArgosSmd::send_command(uint8_t command)
{
    DEBUG_TRACE("ArgosSmd::send_command: TODO send command");
    //uint8_t buffer_read;
    //int ret;

    //ret = m_nrf_spim->transfer(&command, &buffer_read, sizeof(command));
    //if (ret) {
    //    throw ErrorCode::SPI_COMMS_ERROR;
}
    
void ArgosSmd::print_firmware_version()
{
    DEBUG_TRACE("ArgosSmd::print_firmware_version: TODO send firmware request");
}
void ArgosSmd::power_off()
{
    DEBUG_TRACE("ArsgosSmd::power_off");

    // The state machine will safely transition to stopped as soon
    // as all pending activities have completed
    if (!SMD_STATE_EQUAL(stopped)) {
    	m_stopping = true;
    }
}

void ArgosSmd::power_off_immediate()
{
    DEBUG_TRACE("ArgosSmd::power_off_immediate");

    // We force a power off by cancelling the state machine and
    // forcing a stopped state
    if (!SMD_STATE_EQUAL(stopped)) {
    	system_scheduler->cancel_task(m_task);
    	SMD_STATE_CHANGE(idle, stopped);
    }
}

void ArgosSmd::send_packet(ArgosPacket const& user_payload, unsigned int payload_length, const ArgosMode mode)
{
    DEBUG_TRACE("ArgosSmd::send_packet: TODO implemente me");
	// ArgosPacket packet;
	// unsigned int total_bits;
	// unsigned int stuffing_bits = 0;

	// // Only A2/A3 mode is supported
	// if ((payload_length + 8) & 31) {
	// 	// Stuff zeros at the end to align to nearest boundary
	// 	stuffing_bits = 32 - ((payload_length + 8) % 32);
	// 	DEBUG_TRACE("ArgosSmd::send_packet: adding %u stuffing bits for alignment", stuffing_bits);
	// }

	// uint8_t length_encoded[] = { 0x0, 0x3, 0x5, 0x6, 0x9, 0xA, 0xC, 0xF };
	// unsigned int length_idx = (stuffing_bits + payload_length - 8) / 32;
	// unsigned int length_enc = length_encoded[length_idx];

	// unsigned int num_tail_bits = 0; // A2 mode
	// if (mode == ArgosMode::ARGOS_3) {
	// 	uint8_t tail_bits[] = { 7, 8, 9, 7, 8, 9, 7, 8 };
	// 	num_tail_bits = tail_bits[length_idx];
	// }
	// unsigned int op_offset = 0;
	// unsigned int ip_offset = 0;

	// // Transmission is:
	// // MSG_LEN (4)
	// // ARGOSID (28)
	// // USER_PAYLOAD (n*32 - 8)
	// // STUFFING_BITS
	// // TAIL_BITS (7,8,9)
	// total_bits = 4 + 28 + payload_length + stuffing_bits + num_tail_bits;

	// // Assign the buffer to include 24-bit length indicator and rounded-up to 3 bytes for XMEM alignment
	// packet.assign(((((total_bits + 24) + 23) / 24) * 24) / 8, 0);

	// // Set total length
	// PACK_BITS(total_bits, packet, op_offset, 24);

	// // Set header
	// PACK_BITS(length_enc, packet, op_offset, 4);
	// PACK_BITS(m_argos_id, packet, op_offset, 28);

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
	// PACK_BITS(0, packet, op_offset, num_tail_bits);

	// // Setup TX operation
	// DEBUG_TRACE("ArticTransceiver::send_packet: data[%u]=%s tail=%u",
	// 		total_bits, Binascii::hexlify(packet).c_str(),
	// 		num_tail_bits);

	// // Setup TX mode
	// m_tx_mode = mode;
	// m_packet_buffer = packet;
}

void ArgosSmd::send_ack(const unsigned int a_dcs, const unsigned int dl_msg_id, const unsigned int exec_report, const ArgosMode mode)
{
	// ArgosPacket packet, crc_packet;
	// unsigned int stuffing_bits = 0;
	// unsigned int payload_length = 96;
	// unsigned int length_enc = 0x5;
	// unsigned int service_id = 0x00EBA;
	// unsigned int total_bits;

	DEBUG_TRACE("ArgosSmd::send_ack : Implement me");

	// unsigned int num_tail_bits = 0; // A2 mode
	// if (mode == ArgosMode::ARGOS_3)
	// 	num_tail_bits = 7;

	// // Transmission is:
	// // LENGTH (24)
	// // MSG_LEN (4)
	// // SERVICEID (20)  + *
	// // FCS (16)        |
	// // ADCS (4)        | *
	// // PMTID (28)      | *
	// // DLMSGID (16)    | *
	// // EXECRPT(4)      | *
	// // DATA (28)       + *
	// // STUFFING (0)
	// // TAIL_BITS (7)
	// total_bits = 24 + 4 + 20 + payload_length + stuffing_bits + num_tail_bits;

	// unsigned int crc_offset = 0;
	// unsigned int op_offset = 0;

	// // Assign a temporary buffer for computing the FCS over the required* fields
	// crc_packet.assign(13, 0); // 13 bytes, 100 bits
	// PACK_BITS(service_id, crc_packet, crc_offset, 20);
	// PACK_BITS(a_dcs, crc_packet, crc_offset, 4);
	// PACK_BITS(m_argos_id, crc_packet, crc_offset, 28);
	// PACK_BITS(dl_msg_id, crc_packet, crc_offset, 16);
	// PACK_BITS(exec_report, crc_packet, crc_offset, 4);
	// PACK_BITS(0, crc_packet, crc_offset, 28);

	// // Compute FCS
	// uint16_t fcs = CRC16::checksum(crc_packet, 100);

	// // Assign the buffer rounded-up to 3 bytes for XMEM alignment
	// packet.assign((((total_bits + 23) / 24) * 24) / 8, 0);

	// PACK_BITS(total_bits, packet, op_offset, 24);
	// PACK_BITS(length_enc, packet, op_offset, 4);
	// PACK_BITS(service_id, packet, op_offset, 20);
	// PACK_BITS(fcs, packet, op_offset, 16);
	// PACK_BITS(a_dcs, packet, op_offset, 4);
	// PACK_BITS(m_argos_id, packet, op_offset, 28);
	// PACK_BITS(dl_msg_id, packet, op_offset, 16);
	// PACK_BITS(exec_report, packet, op_offset, 4);
	// PACK_BITS(0, packet, op_offset, 28);
	// PACK_BITS(0, packet, op_offset, stuffing_bits);
	// PACK_BITS(0, packet, op_offset, num_tail_bits);

	// // Setup ACK mode
	// m_ack_mode = mode;
	// m_ack_buffer = packet;
}

void ArgosSmd::read_packet(ArgosPacket& packet, unsigned int& size) {
	DEBUG_INFO("ArgosSmd::read_packet : No downlink with SMD");
}

void ArgosSmd::set_rx_mode(const ArgosMode mode, const std::time_t stop_time) {

	DEBUG_TRACE("ArgosSmd::set_rx_mode: No downlink with SMD");
}
uint64_t ArgosSmd::get_rx_time_on() {
	DEBUG_TRACE("ArgosSmd::get_rx_time_on: No downlink with SMD");
	return 0;
}

void ArgosSmd::set_frequency(const double freq) {
	DEBUG_TRACE("ArgosSmd::set_frequency: Configure radiomodule conf");
	m_tx_freq = freq;
}

void ArgosSmd::set_tx_power(const BaseArgosPower power) {
	DEBUG_TRACE("ArgosSmd::set_tx_power: Configure radiomodule conf");
}

void ArgosSmd::set_tcxo_warmup_time(const unsigned int time_s) {
	DEBUG_TRACE("ArgosSmd::set_tcxo_warmup_time: Not ready to be configured right now");
}

ArgosSmd::ArgosSmd() {
    m_vuart = nullptr;
    m_state = ArgosSmdState::stopped;
    m_tcxo_warmup_time = DEFAULT_TCXO_WARMUP_TIME_SECONDS;
	GPIOPins::clear(SAT_RESET);
	GPIOPins::clear(SAT_PWR_EN);
}

void ArgosSmd::power_on(
		const unsigned int argos_id,
		std::function<void(ArgosAsyncEvent)> notification_callback)
{
    DEBUG_TRACE("ArgosSmd::power_on");

    // Device is already powered
    if (m_state != ArgosSmdState::stopped) {
    	m_stopping = false;  // Clear any pending stopping flag
    	DEBUG_TRACE("ArgosSmdState::power_on: not disabled: state=%u", (unsigned int)m_state);
    	return;
    }

    // Set top-level state variables
    m_argos_id = argos_id;
    m_notification_callback = notification_callback;
    m_state = ArgosSmdState::starting; // Force a reset when the task starts
    m_stopping = false;

    // Reset state variables which can be used prior to being fully programmed
    m_ack_buffer.clear();
    m_packet_buffer.clear();
    //m_rx_packet.clear();

    // This will run the state machine perpetually until a power_off is called
    state_machine();
}

void ArgosSmd::state_machine() 
{

	// Assume no delay between state machine invocations
	m_next_delay = 0;

	switch (m_state) {
	case ArgosSmd::starting:
		SMD_STATE_CALL(starting);
		break;
	case ArgosSmd::powering_on:
		SMD_STATE_CALL(powering_on);
		break;
	case ArgosSmd::reset_assert:
		SMD_STATE_CALL(reset_assert);
		break;
	case ArgosSmd::reset_deassert:
		SMD_STATE_CALL(reset_deassert);
		break;
	case ArgosSmd::idle_pending:
		SMD_STATE_CALL(idle_pending);
		break;
	case ArgosSmd::idle:
		SMD_STATE_CALL(idle);
		break;
	case ArgosSmd::transmit_pending:
		SMD_STATE_CALL(transmit_pending);
		break;
	case ArgosSmd::transmitting:
		SMD_STATE_CALL(transmitting);
		break;
	case ArgosSmd::error:
		SMD_STATE_CALL(error);
		break;
	case ArgosSmd::stopped:
		SMD_STATE_CALL(stopped);
	default:
		break;
	}

	if (!SMD_STATE_EQUAL(stopped)) {
		// Invoke ourselves again if we are not stopped
		DEBUG_TRACE("ArgosSmd::state_machine: reschedule in %u ms", m_next_delay);
		m_task = system_scheduler->post_task_prio([this]() {
			state_machine();
		}, "ArgosSmdStateMachine", Scheduler::DEFAULT_PRIORITY, m_next_delay);
	}
}


void ArgosSmd::state_starting_enter() {
}

void ArgosSmd::state_starting_exit() {
}

void ArgosSmd::state_starting() {
    m_is_first_tx = true;
	m_vuart = new VirtualUART(SMD_RX, SMD_TX);
	SMD_STATE_CHANGE(starting, powering_on);
}

void ArgosSmd::state_error_enter() {
	// Dump device status
	get_and_print_status();

	// The calling code's callback should call "power_off" and
	// do any necessary cleanup that it needs to otherwise
	// we will keep invoking the callback!
	m_notification_callback(ArgosAsyncEvent::ERROR);
}

void ArgosSmd::state_error_exit() {
}

void ArgosSmd::state_error() {
	if (m_stopping) {
		SMD_STATE_CHANGE(error, stopped);
	}
}

void ArgosSmd::state_stopped_enter() {
	// Cleanup the SPIM instance
	delete m_vuart;
    m_vuart = nullptr; // Invalidate this pointer so if we call this function again it doesn't call delete on an invalid pointer

    // Power down the device
	GPIOPins::clear(SAT_RESET);
	GPIOPins::clear(SAT_PWR_EN);

	m_notification_callback(ArgosAsyncEvent::OFF);
}

void ArgosSmd::state_stopped_exit() {
}

void ArgosSmd::state_stopped() {
}

void ArgosSmd::state_powering_on_enter() {
}

void ArgosSmd::state_powering_on_exit() {
	m_next_delay = SMD_DELAY_POWER_ON_MS;
}

void ArgosSmd::state_powering_on() {
    GPIOPins::set(SAT_PWR_EN);
    GPIOPins::set(SAT_RESET);
    SMD_STATE_CHANGE(powering_on, reset_assert);
}

void ArgosSmd::state_reset_assert_enter() {
}

void ArgosSmd::state_reset_assert_exit() {
	m_next_delay = SMD_DELAY_RST_MS;
}

void ArgosSmd::state_reset_assert() {
    GPIOPins::clear(SAT_RESET);
    SMD_STATE_CHANGE(reset_assert, reset_deassert);
}


void ArgosSmd::state_reset_deassert_enter() {
	m_next_delay = SMD_DELAY_RST_MS;
}

void ArgosSmd::state_reset_deassert_exit() {
	m_next_delay = SMD_DELAY_RST_MS;
}

void ArgosSmd::state_reset_deassert() {
    GPIOPins::set(SAT_RESET);
    SMD_STATE_CHANGE(reset_deassert, idle);
}

//TODO check if we use LPM or just turn ON/OFF device WAKEUP PIN USED in this case no CMD
void ArgosSmd::state_idle_pending_enter() {
	m_polling_counter = 100;
	//send_command(ARTIC_CMD_SLEEP);
}

void ArgosSmd::state_idle_pending_exit() {
}

void ArgosSmd::state_idle_pending() {

	if (is_idle_state()) {
		// Clear the interrupt
        SMD_STATE_CHANGE(idle_pending, idle);
	}
}

void ArgosSmd::state_idle_enter() {
	// Check if we are already IDLE or not
	if (is_idle()) {
		return;
	}

	SMD_STATE_CHANGE(idle, idle_pending);
}

void ArgosSmd::state_idle_exit() {
}

void ArgosSmd::state_idle() {
	// Check for any new commands
	if (m_packet_buffer.length()) {
		m_tx_event = ArgosAsyncEvent::TX_DONE;
		m_tx_buffer = m_packet_buffer;
		m_packet_buffer.clear();
		SMD_STATE_CHANGE(idle, transmit_pending);
	} else if (m_ack_buffer.length()) {
		m_tx_event = ArgosAsyncEvent::ACK_DONE;
		m_tx_buffer = m_ack_buffer;
		m_ack_buffer.clear();
		SMD_STATE_CHANGE(idle, transmit_pending);
	}
	else if (m_stopping) {
		SMD_STATE_CHANGE(idle, stopped);
	}
}