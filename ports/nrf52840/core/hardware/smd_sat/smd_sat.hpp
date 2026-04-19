/**
 * @file smd_sat.hpp
 * @brief SMD satellite device driver — KineisDevice interface + state machine.
 */

#pragma once

#include <cstdint>

#include "smd_sat_cmd.hpp"
#include "kineis_device.hpp"
#include "scheduler.hpp"
#include "filesystem.hpp"

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


class SmdSat : public KineisDevice {

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

	// Command layer (SPI or AT, selected at compile time)
	SmdSatCmd& m_cmd;

	// Top-level state
	Scheduler::TaskHandle m_task;
	SmdSatState m_state;
	unsigned int m_state_counter;
	unsigned int m_next_delay;
	bool m_stopping;
	bool is_kmac_profil_loaded = false;       ///< Routes idle_pending → load_kmac for boot config (TCXO, LPM)
	bool m_needs_explicit_kmac_load = true;   ///< True when RCONF changed and explicit load_kmac_profil SPI command is needed.
	                                          ///< False when RCONF unchanged — STM32 auto-inits MAC from flash at POR.
	bool m_credentials_written = false;
	bool m_rconf_recovery_attempted = false;

	/// @brief Consecutive error count — after MAX errors, enter long cooldown to prevent SPI spam.
	unsigned int m_error_count = 0;
	static constexpr unsigned int SMD_MAX_CONSECUTIVE_ERRORS = 5;
	static constexpr unsigned int SMD_ERROR_COOLDOWN_MS = 30 * 60 * 1000;  ///< 30 min cooldown
	uint64_t m_cooldown_until = 0;  ///< Timestamp (ms) until which SMD operations are blocked

	// TX state
	KineisPacket m_tx_buffer;
	KineisPacket m_packet_buffer;
	SmdArgosModulation m_modulation;
	std::string m_pending_rconf;  // Deferred RCONF for next power-on
	unsigned int m_tx_power;
	double      m_tx_freq;
	bool        m_is_first_tx;
	uint32_t    m_tcxo_warmup_time;
	uint8_t     m_lpm_mode;  // SMD LPM bitmap written at every boot

	// New firmware version (cached after DFU)
	std::string m_new_firmware_version;

	void power_off();
	void power_on();
	void power_on_blocking();  // Synchronous boot: power + reset + SPI init + wait for ping + VPA release
	bool write_credentials_from_config();

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
	SmdSat(SmdSatCmd& cmd, unsigned int idle_shutdown_timeout_ms = 1000);
	~SmdSat();
	void send(const KineisModulation mode, const KineisPacket& packet, const unsigned int size_bits) override;
	void stop_send() override;
	void start_receive(const KineisModulation mode) override;
	bool stop_receive() override;
	void set_frequency(double freq_mhz) override;
	void set_tcxo_warmup_time(unsigned int time) override;
	void set_tx_power(unsigned int power) override;
	void set_lpm_mode(uint8_t lpm_bitmap);
	void set_credentials(unsigned int dec_id, unsigned int address,
	                     const std::string& seckey, const std::string& radioconf) override;
	void read_credentials(unsigned int *dec_id, unsigned int *address,
	                      std::string *seckey, std::string *radioconf) override;
	static void shutdown(void);

	// DFU Public API
	void power_off_immediate();

	bool dfu_enter();
	bool dfu_exit();
	bool is_dfu_mode() const { return m_cmd.is_dfu_mode(); }

	bool dfu_get_bootloader_info(SmdDfuInfo *info);

	SmdDfuResponse firmware_update(const uint8_t *firmware, size_t size,
	                               void (*progress_callback)(uint8_t percent) = nullptr);

	SmdDfuResponse firmware_update(File *file, size_t size, uint32_t stm32_crc32,
	                               void (*progress_callback)(uint8_t percent) = nullptr);

	SmdDfuResponse firmware_update_from_file(const std::string& filepath,
	                                         void (*progress_callback)(uint8_t percent) = nullptr);

	std::string get_firmware_version();
	std::string get_new_firmware_version() const { return m_new_firmware_version; }
	std::string smd_spi_test();

	/// @brief Start Continuous Wave transmission for RF testing / certification.
	/// @param freq_hz    Carrier frequency in Hz (must be in a licensed SMD band).
	/// @param power_dbm  TX power in dBm.
	/// @param duration_s Duration in seconds (0 = continuous until cw_stop()).
	/// @return true on success, false if module not ready or AT+CW failed.
	bool cw_start(uint32_t freq_hz, uint16_t power_dbm, uint16_t duration_s = 0);

	/// @brief Stop any active Continuous Wave transmission.
	bool cw_stop();

	// Runtime modulation switching (RCONF + save + KMAC reload, no power cycle)
	bool switch_modulation(KineisModulation mode, const std::string& rconf_hex) override;
	KineisModulation get_current_modulation() const override;
};
