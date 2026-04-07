#pragma once

#include <cstdint>
#include "ubx_comms.hpp"
#include "gps.hpp"
#include "scheduler.hpp"

class M10QAsyncReceiver : public UBXCommsEventListener, public GPSDevice {
public:
	M10QAsyncReceiver();
	~M10QAsyncReceiver();
	void power_on(const GPSNavSettings& nav_settings) override;
	void power_off() override;

private:
	GPSNavSettings m_nav_settings;
	UBXComms m_ubx_comms;
	uint8_t  m_navigation_database[16384];
	unsigned int m_ano_database_len;
    unsigned int m_ana_database_len;
	unsigned int m_expected_dbd_messages;
	unsigned int m_mga_ack_count;
	unsigned int m_step;
	unsigned int m_retries;
	unsigned int m_ano_start_pos;
	unsigned int m_uart_error_count;
	unsigned int m_num_consecutive_fixes;
	unsigned int m_num_power_on;
	unsigned int m_num_nav_samples;
	unsigned int m_num_sat_samples;
	bool m_powering_off;
	bool m_fix_was_found;
	bool m_unrecoverable_error;
	bool m_database_overflow;

	// GNSS device info (cached from configure phase)
	char m_gnss_sw_version[30];
	char m_gnss_hw_version[10];
	uint8_t m_gnss_unique_id[5];
	bool m_gnss_info_valid;

	// ISR-to-scheduler data buffers (replaces static locals to avoid data races)
	UBXCommsEventNavReport m_pending_nav;
	UBXCommsEventSatReport m_pending_sat;

	struct Timeout {
		Scheduler::TaskHandle handle;
	} m_timeout;
	Scheduler::TaskHandle m_state_machine_handle;

	enum State {
		idle,
		poweron,
		configure,
		senddatabase,
		sendofflinedatabase,
		startreceive,
		receive,
		stopreceive,
		fetchdatabase,
		poweroff,
	};

	enum OpState {
		IDLE,
		PENDING,
		SUCCESS,
		TIMEOUT,
		NACK,
		ERROR
	};

	State m_state;
	OpState m_op_state;

	// State machine
	void state_idle_enter();
	void state_idle();
	void state_idle_exit();
	void state_poweron_enter();
	void state_poweron();
	void state_poweron_exit();
	void state_configure_enter();
	void state_configure();
	void state_configure_exit();
	void state_senddatabase_enter();
	void state_senddatabase();
	void state_senddatabase_exit();
	void state_sendofflinedatabase_enter();
	void state_sendofflinedatabase();
	void state_sendofflinedatabase_exit();
	void state_startreceive_enter();
	void state_startreceive();
	void state_startreceive_exit();
	void state_receive_enter();
	void state_receive();
	void state_receive_exit();
	void state_stopreceive_enter();
	void state_stopreceive();
	void state_stopreceive_exit();
	void state_fetchdatabase_enter();
	void state_fetchdatabase();
	void state_fetchdatabase_exit();
	void state_poweroff_enter();
	void state_poweroff();
	void state_poweroff_exit();
	void state_machine();
	void run_state_machine(unsigned int time_ms = 0);

	// Helpers
	void cancel_timeout();
	void initiate_timeout(unsigned int timeout_ms = 1000);
	void on_timeout();
	void save_config();
	void soft_reset();
	void setup_uart_port();
	void setup_gnss_channel_sharing();
	void setup_power_management();
	void setup_continuous_mode();
	void setup_simple_navigation_settings();
	void setup_expert_navigation_settings();
	void supply_time_assistance();
	void supply_position_assistance();
	void disable_odometer();
	void disable_timepulse_output();
	void enable_nav_pvt_message();
	void enable_nav_dop_message();
	void enable_nav_status_message();
    void enable_nav_sat_message();
	void disable_nav_pvt_message();
	void disable_nav_dop_message();
	void disable_nav_status_message();
    void disable_nav_sat_message();
	void fetch_navigation_database();
	void query_mon_ver();
	void query_sec_uniqid();
	void sync_baud_rate(unsigned int baud);
	void dump_navigation_database(unsigned int);
	void save_dbd_to_flash();
	bool load_dbd_from_flash();

	// Power management
	void enter_shutdown();
	void exit_shutdown();
	void check_for_power_off();
#ifdef GPS_FAKE_POSITION
	void generate_fake_fix();
#endif

	// Events
	void react(const UBXCommsEventSendComplete&) override;
	void react(const UBXCommsEventAckNack&) override;
	void react(const UBXCommsEventCfgValget&) override;
    void react(const UBXCommsEventSatReport& sat) override;
	void react(const UBXCommsEventNavReport&) override;
	void react(const UBXCommsEventMgaAck&) override;
	void react(const UBXCommsEventMgaDBD&) override;
	void react(const UBXCommsEventMonVer&) override;
	void react(const UBXCommsEventSecUniqId&) override;
	void react(const UBXCommsEventDebug&) override;
	void react(const UBXCommsEventError&) override;

public:
	GNSSDeviceInfo get_device_info() const override;
	GNSSAlmanacStatus get_almanac_status() const override;

	// Bridge/passthrough mode: direct USB ↔ GNSS UART access
	bool start_bridge(PassthroughCallback rx_callback) override;
	void stop_bridge() override;
	bool is_bridge_active() const override { return m_bridge_active; }
	bool bridge_send(const uint8_t* data, size_t len) override;
	void bridge_process_rx() override;

private:
	bool m_bridge_active = false;
};
