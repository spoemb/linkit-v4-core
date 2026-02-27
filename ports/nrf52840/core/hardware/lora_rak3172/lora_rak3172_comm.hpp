#pragma once

#include "events.hpp"
#include <stdint.h>
#include <string>
#include <optional>
#include "nrf_libuarte_async.h"

namespace LoRa {

    enum ATCmd {
        AT_TEST = 0,        // AT             - connection test
        AT_RESET,           // ATZ            - MCU reset
        AT_GET_VER,         // AT+VER=?       - firmware version
        AT_SET_NWM,         // AT+NWM=        - network working mode (0=P2P, 1=LoRaWAN)
        AT_GET_NWM,         // AT+NWM=?       - get network working mode
        AT_SET_NJM,         // AT+NJM=        - join mode (0=ABP, 1=OTAA)
        AT_GET_NJM,         // AT+NJM=?       - get join mode
        AT_SET_BAND,        // AT+BAND=       - frequency band
        AT_GET_BAND,        // AT+BAND=?      - get frequency band
        AT_SET_DEVEUI,      // AT+DEVEUI=     - device EUI
        AT_GET_DEVEUI,      // AT+DEVEUI=?    - get device EUI
        AT_SET_APPEUI,      // AT+APPEUI=     - application EUI
        AT_GET_APPEUI,      // AT+APPEUI=?    - get application EUI
        AT_SET_APPKEY,      // AT+APPKEY=     - application key
        AT_GET_APPKEY,      // AT+APPKEY=?    - get application key
        AT_SET_DEVADDR,     // AT+DEVADDR=    - device address (ABP)
        AT_GET_DEVADDR,     // AT+DEVADDR=?   - get device address
        AT_SET_APPSKEY,     // AT+APPSKEY=    - application session key (ABP)
        AT_SET_NWKSKEY,     // AT+NWKSKEY=    - network session key (ABP)
        AT_SET_CLASS,       // AT+CLASS=      - device class (A/B/C)
        AT_GET_CLASS,       // AT+CLASS=?     - get device class
        AT_SET_DR,          // AT+DR=         - data rate
        AT_GET_DR,          // AT+DR=?        - get data rate
        AT_SET_ADR,         // AT+ADR=        - adaptive data rate
        AT_GET_ADR,         // AT+ADR=?       - get ADR status
        AT_SET_TXP,         // AT+TXP=        - transmit power index
        AT_GET_TXP,         // AT+TXP=?       - get TX power
        AT_SET_CFM,         // AT+CFM=        - confirmed/unconfirmed mode
        AT_GET_CFM,         // AT+CFM=?       - get confirm mode
        AT_SET_RETY,        // AT+RETY=       - confirmed uplink retry count
        AT_JOIN,            // AT+JOIN=       - join network
        AT_GET_NJS,         // AT+NJS=?       - get join status
        AT_SEND,            // AT+SEND=       - send data
        AT_GET_RECV,        // AT+RECV=?      - get last received data
        AT_GET_RSSI,        // AT+RSSI=?      - last RSSI
        AT_GET_SNR,         // AT+SNR=?       - last SNR
        AT_SET_DCS,         // AT+DCS=        - duty cycle setting
        AT_SET_MASK,        // AT+MASK=       - channel mask
        AT_SET_CHS,         // AT+CHS=        - single channel mode
        AT_SET_RX1DL,       // AT+RX1DL=      - RX1 delay
        AT_SET_RX2DR,       // AT+RX2DR=      - RX2 data rate
        AT_SET_PNM,         // AT+PNM=        - public network mode
        AT_SET_LPM,         // AT+LPM=        - low power mode
        AT_SET_SLEEP,       // AT+SLEEP=      - enter sleep
        AT_UNKNOWN
    };

    enum RespType {
        RESP_OK = 0,
        RESP_ERROR,
        RESP_PARAM_ERROR,
        RESP_BUSY_ERROR,
        RESP_NO_NETWORK,
        RESP_VALUE,         // Value response (before OK)
        RESP_EVT_JOINED,
        RESP_EVT_JOIN_FAILED,
        RESP_EVT_TX_DONE,
        RESP_EVT_SEND_CONFIRMED_OK,
        RESP_EVT_SEND_CONFIRMED_FAILED,
        RESP_EVT_RX,        // Downlink data received
        RESP_UNKNOWN
    };

    // RAK3172 response strings
    const std::string OK_RESP        = "OK";
    const std::string ERR_RESP       = "AT_ERROR";
    const std::string PARAM_ERR_RESP = "AT_PARAM_ERROR";
    const std::string BUSY_ERR_RESP  = "AT_BUSY_ERROR";
    const std::string NO_NET_RESP    = "AT_NO_NETWORK_JOINED";

    // Async event prefixes
    const std::string EVT_PREFIX           = "+EVT:";
    const std::string EVT_JOINED           = "+EVT:JOINED";
    const std::string EVT_JOIN_FAILED      = "+EVT:JOIN FAILED";
    const std::string EVT_TX_DONE          = "+EVT:TX_DONE";
    const std::string EVT_SEND_CONF_OK     = "+EVT:SEND CONFIRMED OK";
    const std::string EVT_SEND_CONF_FAIL   = "+EVT:SEND CONFIRMED FAILED";
    const std::string EVT_RX_PREFIX        = "+EVT:RX_";

    // Default LoRaWAN configuration
    static constexpr uint8_t DEFAULT_NWM   = 1;     // LoRaWAN mode
    static constexpr uint8_t DEFAULT_NJM   = 1;     // OTAA
    static constexpr uint8_t DEFAULT_BAND  = 4;     // EU868
    static constexpr uint8_t DEFAULT_DR    = 0;     // SF12/125kHz (longest range)
    static constexpr uint8_t DEFAULT_ADR   = 1;     // ADR enabled
    static constexpr uint8_t DEFAULT_TXP   = 0;     // Max TX power
    static constexpr uint8_t DEFAULT_CFM   = 0;     // Unconfirmed messages
    static constexpr uint8_t DEFAULT_FPORT = 2;     // Application port
    static constexpr uint8_t DEFAULT_RETY  = 1;     // 1 retry for confirmed

    static constexpr uint8_t END_CHAR_CR = '\r';
    static constexpr uint8_t END_CHAR_LF = '\n';
}

// Events emitted by LoRaComm
struct LoRaCommEventRespOk {};
struct LoRaCommEventRespError {
    LoRa::RespType error_type;
    LoRaCommEventRespError() : error_type(LoRa::RESP_ERROR) {}
    LoRaCommEventRespError(LoRa::RespType t) : error_type(t) {}
};
struct LoRaCommEventJoined {};
struct LoRaCommEventJoinFailed {};
struct LoRaCommEventTxDone {};
struct LoRaCommEventRxData {
    int port;
    std::string payload;  // hex-encoded
    LoRaCommEventRxData() : port(0) {}
    LoRaCommEventRxData(int p, const std::string& d) : port(p), payload(d) {}
};
struct LoRaCommEventUartError {
    unsigned int error_type;
    LoRaCommEventUartError(unsigned int a) : error_type(a) {}
};

class LoRaCommEventListener {
public:
    virtual ~LoRaCommEventListener() {}
    virtual void react(const LoRaCommEventRespOk&) {}
    virtual void react(const LoRaCommEventRespError&) {}
    virtual void react(const LoRaCommEventJoined&) {}
    virtual void react(const LoRaCommEventJoinFailed&) {}
    virtual void react(const LoRaCommEventTxDone&) {}
    virtual void react(const LoRaCommEventRxData&) {}
    virtual void react(const LoRaCommEventUartError&) {}
};

class LoRaComm : public EventEmitter<LoRaCommEventListener> {
public:
    std::string m_last_value;       // Last value response from a read command
    std::string m_deveui;           // Cached device EUI
    bool m_is_rx_started = false;

    LoRaComm(unsigned int libuarte_async_instance = 1);

    void init();
    void deinit();
    bool send(LoRa::ATCmd cmd, const std::optional<std::string>& params = std::nullopt);

    // Callbacks for async event handler
    void handle_tx_done(void);
    void handle_rx_buffer(uint8_t * buffer, uint8_t length);
    void handle_error(unsigned int error_type);

private:
    unsigned int m_uart_instance;
    bool m_is_init;
    bool m_is_send_busy;
    std::string m_tx_buffer;
    std::string m_rx_buffer;
    nrf_libuarte_async_config_t m_uart_config;  // Local config copy (modified baudrate)

    bool send_at_cmd(LoRa::ATCmd cmd, const std::optional<std::string>& params = std::nullopt);
    LoRa::RespType parse_rx_line(const std::string& line);
};
