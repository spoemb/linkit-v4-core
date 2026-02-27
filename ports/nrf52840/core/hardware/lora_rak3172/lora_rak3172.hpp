#pragma once

#include "lora_rak3172_comm.hpp"
#include "kineis_device.hpp"
#include "scheduler.hpp"
#include <string>

/**
 * LoRa RAK3172-SiP device driver.
 *
 * Implements the KineisDevice interface to integrate with ArgosTxService.
 * Communicates via UART AT commands (RUI3 firmware) with the RAK3172-SiP module.
 *
 * State machine:
 *   power_off → power_on → configure → joining → idle → transmit → idle
 *                                                         ↓
 *                                                       error → power_off
 */
class LoRaDevice : public LoRaCommEventListener, public KineisDevice {
private:
    LoRaComm m_lora_comm;

public:
    LoRaDevice();
    ~LoRaDevice();

    // KineisDevice interface
    void send(const KineisModulation mode, const KineisPacket& packet, const unsigned int size_bits) override;
    void stop_send() override;
    void start_receive(const KineisModulation mode) override;
    bool stop_receive() override;
    void set_frequency(double freq_mhz) override;
    void set_tcxo_warmup_time(unsigned int ms) override;

    // LoRa-specific public API
    bool is_joined() const { return m_joined; }

    // LoRa configuration (set before power_on or during idle)
    struct LoRaConfig {
        std::string deveui;     // 16 hex chars - empty = read from module
        std::string appeui;     // 16 hex chars
        std::string appkey;     // 32 hex chars
        std::string devaddr;    // 8 hex chars (ABP only)
        std::string appskey;    // 32 hex chars (ABP only)
        std::string nwkskey;    // 32 hex chars (ABP only)
        uint8_t njm   = LoRa::DEFAULT_NJM;     // 0=ABP, 1=OTAA
        uint8_t band  = LoRa::DEFAULT_BAND;     // 4=EU868
        uint8_t dr    = LoRa::DEFAULT_DR;        // Data rate
        uint8_t adr   = LoRa::DEFAULT_ADR;       // ADR enable
        uint8_t txp   = LoRa::DEFAULT_TXP;       // TX power index
        uint8_t cfm   = LoRa::DEFAULT_CFM;       // 0=unconfirmed, 1=confirmed
        uint8_t fport = LoRa::DEFAULT_FPORT;     // Application port
        uint8_t device_class = 'A';              // A, B, or C
    };

    LoRaConfig m_config;

private:
    // State machine
    enum State {
        power_off,
        power_on,
        configure,
        joining,
        idle,
        transmit,
        error
    };

    // Top-level state
    Scheduler::TaskHandle m_task;
    State                 m_state;
    volatile bool         m_cmd_is_ok;
    volatile bool         m_is_error;
    volatile bool         m_joined;
    volatile bool         m_join_failed;
    volatile bool         m_tx_done;

    struct Timeout {
        Scheduler::TaskHandle handle;
    } m_timeout;

    // TX state
    std::string m_packet_buffer;    // Hex-encoded payload waiting to be sent

    // Configuration step tracking
    unsigned int m_config_step;

    // State machine methods
    void state_machine();
    void run_state_machine(uint16_t delay_ms = 100);

    void state_power_off_enter();
    void state_power_off();
    void state_power_off_exit();

    void state_power_on_enter();
    void state_power_on();
    void state_power_on_exit();

    void state_configure_enter();
    void state_configure();
    void state_configure_exit();

    void state_joining_enter();
    void state_joining();
    void state_joining_exit();

    void state_idle_enter();
    void state_idle();
    void state_idle_exit();

    void state_transmit_enter();
    void state_transmit();
    void state_transmit_exit();

    void state_error_enter();
    void state_error();
    void state_error_exit();

    // Event handlers from LoRaComm
    void react(const LoRaCommEventRespOk&) override;
    void react(const LoRaCommEventRespError&) override;
    void react(const LoRaCommEventJoined&) override;
    void react(const LoRaCommEventJoinFailed&) override;
    void react(const LoRaCommEventTxDone&) override;
    void react(const LoRaCommEventRxData&) override;
    void react(const LoRaCommEventUartError&) override;

    // Helpers
    bool send_AT(LoRa::ATCmd cmd, const std::optional<std::string>& params = std::nullopt);
    void start_device();
    void power_off_immediate();
    void cancel_timeout();
    void initiate_timeout(unsigned int timeout_ms = 1000);
    void on_timeout();
    void load_config_from_store();
};
