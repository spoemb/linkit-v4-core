#pragma once

#include <stdint.h>

#include "nrf_spim.hpp"
#include "smd_sat_registers.hpp"
#include "kineis_device.hpp"
#include "scheduler.hpp"
#include "filesystem.hpp"

// ============================================================================
// Protocol A+ Response Structure
// ============================================================================
struct SpiAplusResponse {
    uint8_t seq;                                    // Sequence number
    SpiAplusStatus status;                          // Response status
    uint8_t data[SPI_PROTOCOL_APLUS_MAX_DATA_LEN];  // Response data
    uint16_t data_len;                              // Data length
    bool valid;                                     // CRC validation result
};

// Protocol mode enumeration
enum class SpiProtocolMode {
    LEGACY,      // Legacy protocol (direct command bytes)
    APLUS,       // Protocol A+ (framed with magic, seq, CRC)
    AUTO_DETECT  // Auto-detect based on first byte response
};

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

	// Top-level state
	Scheduler::TaskHandle m_task;
	SmdSatState m_state;
	NrfSPIM *m_nrf_spim;
	unsigned int m_state_counter;
	unsigned int m_next_delay;
	bool m_stopping;
	bool is_kmac_profil_loaded = false;

	// TX state
	KineisPacket m_tx_buffer;
	KineisPacket m_packet_buffer;
	SmdArgosModulation m_modulation;
	unsigned int m_tx_power;
	double      m_tx_freq;
	bool        m_is_first_tx;
	uint32_t    m_tcxo_warmup_time;

	// Protocol A+ state
	SpiProtocolMode m_protocol_mode;
	uint8_t m_sequence_number;
	bool m_protocol_detected;

	// DFU state
	bool m_dfu_mode;           // True when in bootloader DFU mode
	SmdDfuInfo m_dfu_info;     // Bootloader info (cached after DFU_GET_INFO)
	std::string m_new_firmware_version;  // Version read after last successful DFU

	// Legacy SPI support (kept for raw byte transfer)
	void read_byte(uint8_t *byte_read);
	void send_command(uint8_t command);
	void send_command(const uint8_t *tx_data, uint8_t *rx_data, uint16_t size);

	// SPI status queries (A+ protocol)
	void get_spi_status(uint8_t *status);
	void get_kmac_status(uint8_t *status);

	// Protocol A+ core (based on Zephyr argos-smd-driver)
	uint16_t build_aplus_frame(uint8_t *frame, uint8_t cmd, const uint8_t *data, uint16_t data_len);
	bool parse_aplus_response(const uint8_t *rx_buffer, uint16_t rx_len, SpiAplusResponse *response);
	bool send_command_aplus(uint8_t command, const uint8_t *tx_data, uint16_t tx_len,
	                        SpiAplusResponse *response);
	bool send_command_auto(uint8_t command, const uint8_t *tx_data = nullptr, uint16_t tx_len = 0,
	                       uint8_t *rx_data = nullptr, uint16_t *rx_len = nullptr);
	bool send_command_2phase(uint8_t req_cmd, uint8_t write_cmd,
	                         const uint8_t *data, uint16_t len);

	// High-level SPI commands (all use A+ protocol)
	void set_radio_conf(smd_uint8_array_t *radio_conf);
	void read_radio_conf(SmdArgosModulation *modulation);
	bool save_radio_conf();
	void read_version(uint8_t *version);
	void print_firmware_version();
	void read_address(smd_uint8_array_t *address);
	void set_address(smd_uint8_array_t *address);
	void read_seckey(smd_uint8_array_t *seckey);
	void set_seckey(smd_uint8_array_t *seckey);
	void read_id(uint32_t *id);
	void set_id(uint32_t id);
	void read_serial(smd_uint8_array_t *serial);
	void read_lpm(uint8_t *lpm_mode);
	void write_lpm(uint8_t *lpm_mode);
	void read_tcxo_warmup(uint32_t *time_ms);
	void write_tcxo_warmup(uint32_t time_ms);

	void load_kmac_profil(uint8_t profile);
	void read_kmac(uint8_t *profile);
	void read_rconf_raw(uint8_t *rconf_raw, uint16_t *len);
	void read_spimac_state(uint8_t *spi_state, uint8_t *mac_state);
	void read_firmware_info(uint8_t *info, uint16_t *len);

	// CW (Continuous Wave) commands
	void read_cw(uint8_t *cw_data, uint16_t *len);
	void write_cw(const uint8_t *cw_data, uint16_t len);

	// Prepass commands
	void read_prepassen(uint8_t *prepass_data, uint16_t *len);
	void write_prepassen(const uint8_t *prepass_data, uint16_t len);

	// UTC date commands
	void read_udate(uint8_t *udate_data, uint16_t *len);
	void write_udate(const uint8_t *udate_data, uint16_t len);

	void spi_sync();
	void set_tcxo_warmup_internal(uint32_t time_s);
	void set_tcxo_control(bool state);
	bool smd_ping();

	// DFU low-level operations (private)
	SmdDfuResponse dfu_send_command(uint8_t cmd, const uint8_t *data = nullptr, uint16_t data_len = 0,
	                                uint8_t *response_data = nullptr, uint16_t *response_len = nullptr);
	SmdDfuResponse dfu_send_with_retry(uint8_t cmd, const uint8_t *data = nullptr, uint16_t data_len = 0,
	                                   uint8_t *response_data = nullptr, uint16_t *response_len = nullptr);
	SmdDfuResponse dfu_ping();
	SmdDfuResponse dfu_get_info(SmdDfuInfo *info);
	SmdDfuResponse dfu_erase();
	SmdDfuResponse dfu_write_chunk(uint32_t addr, const uint8_t *data, uint16_t len);
	SmdDfuResponse dfu_read_chunk(uint32_t addr, uint8_t *data, uint16_t len);
	SmdDfuResponse dfu_verify(uint32_t crc32);
	SmdDfuResponse dfu_reset();
	SmdDfuResponse dfu_jump();
	SmdDfuResponse dfu_get_status(uint8_t *status);
	SmdDfuResponse dfu_abort();
	SmdDfuResponse dfu_set_header(const uint8_t *header);
	uint32_t calculate_crc32(const uint8_t *data, size_t len);

	bool is_tx_finished();
	bool is_tx_in_progress();
	void power_off();
	void power_on();

	bool initiate_tx();

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
	SmdSat(unsigned int idle_shutdown_timeout_ms = 1000);
	~SmdSat();
	void send(const KineisModulation mode, const KineisPacket& packet, const unsigned int size_bits) override;
	void stop_send() override;
	void start_receive(const KineisModulation mode) override;
	bool stop_receive() override;
	void set_frequency(double freq_mhz) override;
	void set_tcxo_warmup_time(unsigned int time) override;
	void set_tx_power(unsigned int power) override;
	void set_credentials(unsigned int dec_id, unsigned int address,
	                     const std::string& seckey, const std::string& radioconf) override;
	void read_credentials(unsigned int *dec_id, unsigned int *address,
	                      std::string *seckey, std::string *radioconf) override;
	static void shutdown(void);

	// ========================================================================
	// DFU (Device Firmware Update) Public API
	// ========================================================================

	/**
	 * @brief Immediately power off the SMD and set state to stopped
	 * @note Used before DFU to ensure clean power-on sequence
	 */
	void power_off_immediate();

	/**
	 * @brief Enter DFU mode - switches SMD from application to bootloader
	 * @return true if DFU mode entered successfully, false otherwise
	 * @note This triggers a reset on the SMD module
	 */
	bool dfu_enter();

	/**
	 * @brief Exit DFU mode - jumps from bootloader to application
	 * @return true if jump successful, false otherwise
	 */
	bool dfu_exit();

	/**
	 * @brief Check if currently in DFU bootloader mode
	 * @return true if in DFU mode, false if in normal application mode
	 */
	bool is_dfu_mode() const { return m_dfu_mode; }

	/**
	 * @brief Get DFU bootloader information
	 * @param info Pointer to structure to fill with bootloader info
	 * @return true if info retrieved successfully
	 */
	bool dfu_get_bootloader_info(SmdDfuInfo *info);

	/**
	 * @brief Perform complete firmware update from binary data
	 * @param firmware Pointer to firmware binary data
	 * @param size Size of firmware in bytes
	 * @param progress_callback Optional callback for progress reporting (0-100%)
	 * @return DFU_RSP_OK on success, error code otherwise
	 *
	 * This performs the complete update sequence:
	 * 1. Enter DFU mode (if not already)
	 * 2. Ping bootloader
	 * 3. Get bootloader info
	 * 4. Erase application area
	 * 5. Write firmware in chunks
	 * 6. Verify CRC32
	 * 7. Jump to application
	 */
	SmdDfuResponse firmware_update(const uint8_t *firmware, size_t size,
	                               void (*progress_callback)(uint8_t percent) = nullptr);

	/**
	 * @brief Perform firmware update streaming from a File (no large heap allocation)
	 * @param file Open file positioned at start of firmware data
	 * @param size Size of firmware data in bytes
	 * @param stm32_crc32 Pre-computed STM32 CRC32-MPEG2 from firmware header
	 * @param progress_callback Optional callback for progress reporting (0-100%)
	 * @return DFU_RSP_OK on success, error code otherwise
	 */
	SmdDfuResponse firmware_update(File *file, size_t size, uint32_t stm32_crc32,
	                               void (*progress_callback)(uint8_t percent) = nullptr);

	/**
	 * @brief Perform firmware update from file
	 * @param filepath Path to firmware binary file
	 * @param progress_callback Optional callback for progress reporting (0-100%)
	 * @return DFU_RSP_OK on success, error code otherwise
	 */
	SmdDfuResponse firmware_update_from_file(const std::string& filepath,
	                                         void (*progress_callback)(uint8_t percent) = nullptr);

	/**
	 * @brief Get SMD firmware version string
	 * @return Firmware version string, or empty string on error
	 * @note This requires the SMD to be powered on
	 */
	std::string get_firmware_version();

	/**
	 * @brief Get firmware version read after last DFU update
	 * @return Version string, or empty if not available
	 */
	std::string get_new_firmware_version() const { return m_new_firmware_version; }

	/**
	 * @brief Debug: test all SPI A+ commands and report results
	 * @return Multi-line string with pass/fail for each command
	 */
	std::string smd_spi_test();
};
