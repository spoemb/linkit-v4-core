#pragma once

/**
 * @file kim2.hpp
 * @brief KIM2 satellite module device driver (AT commands over UART).
 *
 * Implements the KineisDevice interface for the CLS KIM2 module.
 * State machine: power_off → power_on → init (ping, ID, RCONF, KMAC) → idle → transmit.
 * Power-managed: module is powered off between TX sessions to save battery.
 */

#include "kim2_comm.hpp"
#include "kineis_device.hpp"
#include "nrfx_uarte.h"
#include "scheduler.hpp"
#include <atomic>

class KIM2Device : public KIM2CommEventListener, public KineisDevice {
private:
	KIM2Comm m_kim2_comm;  ///< UART AT command layer

public:
	/// @throws ErrorCode if module does not respond to ping at first power-on.
	KIM2Device();
	~KIM2Device();

	/// @brief Pack payload, stuff bits, queue for TX.  Powers on module if needed.
	/// @param mode       Modulation type (LDK, LDA2, VLDA4).
	/// @param packet     Raw payload bytes.
	/// @param size_bits  Payload size in bits.
	void send(const KineisModulation mode, const KineisPacket& packet, const unsigned int size_bits) override;

	/// @brief Cancel any pending TX.
	void stop_send() override;

	/// @brief Not implemented on KIM2.
	void start_receive(const KineisModulation mode) override;

	/// @brief Not implemented on KIM2.
	/// @return Always false.
	bool stop_receive() override;

	/// @brief No-op — KIM2 frequency is set via RCONF.
	void set_frequency(double freq_mhz) override;

	/// @brief No-op — KIM2 manages TCXO warmup internally.
	void set_tcxo_warmup_time(unsigned int ms) override;

	// Bridge/passthrough mode: direct USB ↔ UART access for raw AT commands
	bool start_bridge(KIM2Comm::PassthroughCallback rx_callback);
	void stop_bridge();
	bool is_bridge_active() const { return m_bridge_active; }
	bool bridge_send(const uint8_t* data, size_t len);
	void bridge_process_rx();  // Call periodically to pump UART RX

private:
	/// @brief KIM2 state machine states.
	enum KIM2ManagerState {
		power_off,
		power_on,
		init,
		idle,
		transmit,
		error
	};

	/// @name Top-level state
	/// @{
	Scheduler::TaskHandle m_task;           ///< Scheduler task for state machine ticks
	KIM2ManagerState      m_state;          ///< Current state
	bool                  m_stopping;       ///< True when stop requested
	std::atomic<bool>     m_cmd_is_ok;      ///< Set by ISR on +OK response
	std::atomic<bool>     m_is_error;       ///< Set by ISR on +ERROR or timeout
	struct Timeout {
		Scheduler::TaskHandle handle;
	} m_timeout;
	/// @}

	/// @brief Bridge mode active flag — pauses state machine, routes raw UART.
	bool m_bridge_active = false;

	/// @name TX state
	/// @{
	KineisPacket m_tx_buffer;                ///< Hex string queued for AT+TX
	KineisPacket m_packet_buffer;            ///< Incoming packet from send()
	KineisModulation m_tx_mode;              ///< Modulation for current TX
	KineisModulation m_current_rconf_mode;   ///< Last RCONF modulation written
	std::string m_last_saved_rconf;          ///< Last saved RCONF hex (skip re-write if unchanged)
	std::atomic<bool> m_tx_done;             ///< Set by ISR on +TX= response
	unsigned int m_tx_poll_counter;          ///< Remaining TX poll ticks before timeout
	/// @}

	/// @name State machine
	/// @{
	void state_machine();                              ///< Dispatch to current state handler
	void run_state_machine(uint16_t delay_ms = 100);   ///< Schedule next state machine tick
	// State handlers: each state has enter/exit/tick functions
	void state_power_off();       void state_power_off_enter();   void state_power_off_exit();
	void state_power_on();        void state_power_on_enter();    void state_power_on_exit();
	void state_init();            void state_init_enter();        void state_init_exit();
	void state_idle();            void state_idle_enter();        void state_idle_exit();
	void state_transmit();        void state_transmit_enter();    void state_transmit_exit();
	void state_error();           void state_error_enter();       void state_error_exit();
	/// @}

	/// @name UART event handlers (ISR context → atomic flags)
	/// @{
	void react(const KIM2CommEventRespOk&) override;
	void react(const KIM2CommEventTxDone&) override;
	void react(const KIM2CommEventRespError&) override;
	void react(const KIM2CommEventUartError&) override;
	/// @}

	/// @name Runtime modulation switching (KineisDevice interface)
	/// @{

	/// @brief Switch RCONF + KMAC for a different modulation.
	/// @param mode       Target modulation.
	/// @param rconf_hex  32-char hex RCONF string.
	/// @return true on success.
	bool switch_modulation(KineisModulation mode, const std::string& rconf_hex) override;

	/// @return Current RCONF modulation.
	KineisModulation get_current_modulation() const override;
	/// @}

	/// @name Internal helpers
	/// @{

	/// @brief Send AT command and busy-wait for response (up to timeout_ms).
	/// @param cmd         AT command type.
	/// @param params      Optional parameter string.
	/// @param timeout_ms  Max wait in ms (default 1000).
	/// @return true if +OK received, false on timeout or +ERROR.
	bool send_AT(KIM2::ATCmd cmd, const std::optional<std::string>& params = std::nullopt,
	             uint16_t timeout_ms = 1000);

	/// @brief Power on module and start state machine (no-op if already running).
	void start_device();

	/// @brief Immediate power off — cancel all tasks, uninit UART, cut power.
	void power_off_immediate();

	void cancel_timeout();                              ///< Cancel pending AT response timeout
	void initiate_timeout(unsigned int timeout_ms = 1000); ///< Schedule AT response timeout
	void on_timeout();                                   ///< Timeout handler — sets m_is_error
	/// @}
};
