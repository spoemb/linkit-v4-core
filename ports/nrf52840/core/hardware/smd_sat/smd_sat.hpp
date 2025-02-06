#pragma once

#include <stdint.h>

#include "nrf_spim.hpp"
#include "smd_sat_registers.hpp"
#include "artic_device.hpp"
#include "scheduler.hpp"

#define SMD_STATE_CHANGE(x, y)                       \
	do {                                             \
		DEBUG_TRACE("SmdSat::SMD_STATE_CHANGE: " #x " -> " #y ); \
		m_state = y;                                 \
		state_ ## x ##_exit();                       \
		state_ ## y ##_enter();                      \
	} while (0)

#define SMD_STATE_EQUAL(x) \
	(m_state == x)

#define SMD_STATE_CALL(x)   \
	do {                    \
		state_ ## x();      \
	} while (0)


class SmdSat : public ArticDevice {

private:
	enum SmdSatState {
		stopped,
		starting,
		powering_on,
		load_kmac,
		idle_pending,
		idle,
		transmit_pending,
		transmitting,
		error
	};
	
	// Top-level state
	Scheduler::TaskHandle m_task;
	SmdSatState m_state;
	NrfSPIM *m_nrf_spim;
	unsigned int m_state_counter;
	unsigned int m_next_delay;
	bool m_stopping;
	bool is_kmac_profil_loaded = false;

	// Argos TX state
	ArticPacket m_tx_buffer;
	ArticPacket m_ack_buffer;
	ArticPacket m_packet_buffer;
	ArticMode   m_tx_mode;
	ArticMode   m_ack_mode;
	ArgosModulation m_modulation;
	BaseArgosPower m_tx_power;
	double      m_tx_freq;
	bool        m_is_first_tx;
	uint32_t    m_tcxo_warmup_time;

	// Support functionality
	void read_byte(uint8_t *byte_read);
	void send_command(uint8_t command);
	void send_command(const uint8_t *tx_data, uint8_t *rx_data, uint16_t size);
	void get_spi_status(uint8_t *status);
	void get_kmac_status(uint8_t *status);

	void reload_kmac_profil();
	void set_tcxo_warmup(uint32_t time_s);
	void set_tcxo_control(bool state);
	void print_firmware_version();
	bool is_idle();
	bool is_idle_state();
	bool smd_ping();
	
	bool is_tx_finished();
	bool is_tx_in_progress();
	bool is_command_accepted();
	bool is_firmware_ready();
	void power_off();
	void power_on();
	void power_off_immediate();
	
	void initiate_tx();


	// State machine functionality
	void state_machine(bool use_scheduler=true);
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
	void state_load_kmac_enter();
	void state_load_kmac_exit();
	void state_load_kmac();
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
	SmdSat(unsigned int idle_shutdown_timeout_ms = 1000);
	~SmdSat();
	void send(const ArticMode mode, const ArticPacket& packet, const unsigned int size_bits) override;
	void send_ack(const ArticMode mode, const unsigned int a_dcs, const unsigned int dl_msg_id, const unsigned int exec_report) override;
	void stop_send() override;
	void start_receive(const ArticMode mode) override;
	bool stop_receive() override;
	void set_frequency(const double freq) override;
	void set_tcxo_warmup_time(const unsigned int time) override;
	void set_tx_power(const BaseArgosPower power) override;
	static void shutdown(void);
};