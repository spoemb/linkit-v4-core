#pragma once

#include <stdint.h>
#include "argos_scheduler.hpp"
#include "virtual_uart.hpp"

#define SMD_DELAY_RST_MS (5)
#define SMD_DELAY_POWER_ON_MS (1000)


#define SMD_STATE_CHANGE(x, y)                               \
	do {                                             \
		DEBUG_TRACE("SMD_STATE_CHANGE: " #x " -> " #y ); \
		m_state = y;                                 \
		state_ ## x ##_exit();                       \
		state_ ## y ##_enter();                      \
	} while (0)

#define SMD_STATE_EQUAL(x) \
	(m_state == x)

#define SMD_STATE_CALL(x)  \
	do {                    \
		state_ ## x();      \
	} while (0)


class ArgosSmd : public ArgosScheduler {
	public:
		enum class AtCommand {
			VERSION,
			PING,
			FW,
			ADDR,
			SN,
			ID,
			RCONF,
			SAVE_RCONF,
			LPM,
			TX,
			CW,
			PREPASS_EN,
			UDATE,
			ATXRP
		};

		static const char* atCommandToString(AtCommand cmd) {
			switch (cmd) {
				case AtCommand::VERSION: return "AT+VERSION";
				case AtCommand::PING: return "AT+PING";
				case AtCommand::FW: return "AT+FW";
				case AtCommand::ADDR: return "AT+ADDR";
				case AtCommand::SN: return "AT+SN";
				case AtCommand::ID: return "AT+ID";
				case AtCommand::RCONF: return "AT+RCONF";
				case AtCommand::SAVE_RCONF: return "AT+SAVE_RCONF";
				case AtCommand::LPM: return "AT+LPM";
				case AtCommand::TX: return "AT+TX=";
				case AtCommand::CW: return "AT+CX=";
				case AtCommand::PREPASS_EN: return "AT+PREPASS_EN";
				case AtCommand::UDATE: return "AT+UDATE";
				case AtCommand::ATXRP: return "AT+ATXRP";
				default: return "";
			};
		};
		
		enum status_flag
		{
			IDLE,                                   // The firmware is idle and ready to accept commands.
			RX_IN_PROGRESS,                         // The firmware is receiving.
			TX_IN_PROGRESS,                         // The firmware is transmitting.
			BUSY,                                   // The firmware is changing state.
		};
	private:
		enum ArgosSmdState {
			stopped,
			starting,
			powering_on,
			reset_assert,
			reset_deassert,
			idle_pending,
			idle,
			transmit_pending,
			transmitting,
			error
		};
	
	// Top-level state
	Scheduler::TaskHandle m_task;
	ArgosSmdState m_state;
	std::function<void(ArgosAsyncEvent)> m_notification_callback;
	VirtualUART *m_vuart;
	unsigned int m_argos_id;
	unsigned int m_polling_counter;
	unsigned int m_next_delay;
	bool m_stopping;

	// Argos TX state
	ArgosPacket m_tx_buffer;
	ArgosPacket m_ack_buffer;
	ArgosPacket m_packet_buffer;
	ArgosMode   m_tx_mode;
	ArgosMode   m_ack_mode;
	ArgosAsyncEvent m_tx_event;
	double      m_tx_freq;
	bool        m_is_first_tx;
	uint32_t    m_tcxo_warmup_time; // managed by STM

	// Support functionality
	void send_command(uint8_t command);
	void print_status(uint32_t status);
	void get_and_print_status();
	void print_firmware_version();
	bool is_idle();
	bool is_idle_state();
	bool is_tx_finished();
	bool is_tx_in_progress();
	bool is_command_accepted();
	bool is_firmware_ready();
	void initiate_tx();

	// State machine functionality
	void state_machine();
	void state_starting();
	void state_starting_enter();
	void state_starting_exit();
	void state_stopped();
	void state_stopped_enter();
	void state_stopped_exit();
	void state_error();
	void state_error_enter();
	void state_error_exit();
	void state_powering_on();
	void state_powering_on_enter();
	void state_powering_on_exit();
	void state_reset_assert();
	void state_reset_assert_enter();
	void state_reset_assert_exit();
	void state_reset_deassert();
	void state_reset_deassert_enter();
	void state_reset_deassert_exit();
	void state_idle_pending();
	void state_idle_pending_enter();
	void state_idle_pending_exit();
	void state_idle();
	void state_idle_enter();
	void state_idle_exit();
	void state_transmit_pending();
	void state_transmit_pending_enter();
	void state_transmit_pending_exit();
	void state_transmitting();
	void state_transmitting_enter();
	void state_transmitting_exit();

public:
	ArgosSmd();
	void power_off() override;
	void power_off_immediate() override;
	void power_on(const unsigned int argos_id, std::function<void(ArgosAsyncEvent)> notification_callback) override;
	void send_packet(ArgosPacket const& user_payload, unsigned int payload_length, const ArgosMode mode) override;
	void send_ack(const unsigned int a_dcs, const unsigned int dl_msg_id, const unsigned int exec_report, const ArgosMode mode) override;
	void read_packet(ArgosPacket& packet, unsigned int& size) override;
	void set_rx_mode(const ArgosMode mode, const std::time_t stop_time) override;
	uint64_t get_rx_time_on() override;
	void set_frequency(const double freq) override;
	void set_tx_power(const BaseArgosPower power) override;
	void set_tcxo_warmup_time(const unsigned int time_s) override;

};
