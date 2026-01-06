#pragma once

#include "kim2_comm.hpp"
#include "kineis_device.hpp"
#include "nrfx_uarte.h"
#include "scheduler.hpp"


class KIM2Device : public KIM2CommEventListener, public KineisDevice {
private:
    KIM2Comm m_kim2_comm;

public:
    KIM2Device();
    ~KIM2Device();
	void send(const KineisModulation mode, const KineisPacket& packet, const unsigned int size_bits) override;
	void stop_send() override;
	

private:
	//State machine
    enum KIM2ManagerState {
		power_off,
		power_on,
		init,
		idle,
		transmit,
		error
	};

	// Top-level state
	Scheduler::TaskHandle m_task;
	KIM2ManagerState      m_state;
	bool                  m_stopping;
	bool                  m_cmd_is_ok;
	bool                  m_is_error;
	struct Timeout {
		Scheduler::TaskHandle handle;
		bool     running;
		uint64_t end;
	} m_timeout;

	// Argos TX state
	KineisPacket m_tx_buffer;
	KineisPacket m_packet_buffer;
	KineisModulation m_tx_mode;
	bool         m_tx_done;

	//State machine
	void state_machine();
	void run_state_machine(uint16_t delay_ms = 100);
	void state_power_off();
	void state_power_off_enter();
	void state_power_off_exit();
	void state_power_on();
	void state_power_on_enter();
	void state_power_on_exit();
	void state_idle();
	void state_idle_enter();
	void state_idle_exit();
	void state_init();
	void state_init_enter();
	void state_init_exit();
	void state_transmit();
	void state_transmit_enter();
	void state_transmit_exit();
	void state_error();
	void state_error_enter();
	void state_error_exit();

	// Events
	void react(const KIM2CommEventRespOk&) override;
	void react(const KIM2CommEventTxDone&) override;
	void react(const KIM2CommEventRespError&) override;
	void react(const KIM2CommEventUartError&) override;

	// Private functions
	bool send_AT(KIM2::ATCmd cmd, const std::optional<std::string>& params = std::nullopt);
	void start_device();
	void power_off_immediate(void);
	void cancel_timeout();
	void initiate_timeout(unsigned int timeout_ms = 1000);
	void on_timeout();
};