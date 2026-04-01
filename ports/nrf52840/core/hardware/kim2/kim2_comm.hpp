#pragma once

#include "events.hpp"
#include <stdint.h>
#include <string>


namespace KIM2 {
    /*Public types*/
    enum ATCmd {
        AT_PING = 0,
        AT_GET_ID,
        AT_GET_ADDR,
        AT_SET_RCONF,
        AT_SAVE_RCONF,
        AT_SET_KMAC_BASIC,
        AT_SET_LPM,
        AT_TX,
        AT_UNKNOWN
    };

    enum RespType {
        RESP_OK = 0,
        RESP_ERROR,
        RESP_CONFIG,
        RESP_TX_STATUS,
        RESP_UNKNOWN
    };

    static constexpr uint8_t SYNC_CHAR = '+';
    static constexpr uint8_t END_CHAR_1 = '\r';
    static constexpr uint8_t END_CHAR_2 = '\n';
    const std::string OK_RESPONSE = "+OK\r\n";
    const std::string ID_RESPONSE = "+ID=";
    const std::string ADDR_RESPONSE = "+ADDR=";
    const std::string TX_RESPONSE = "+TX=";
    const std::string ERR_RESPONSE = "+ERROR=";
    static constexpr uint8_t ID_SIZE = 6;
    static constexpr uint8_t ADDR_SIZE = 8;
}

struct KIM2CommEventTxDone {};
struct KIM2CommEventRespOk {};
struct KIM2CommEventRespError {};
struct KIM2CommEventUartError {
    unsigned int error_type;
    KIM2CommEventUartError(unsigned int a) : error_type(a) {}
};

class KIM2CommEventListener {
    public:
        virtual ~KIM2CommEventListener() {}
        virtual void react(const KIM2CommEventTxDone&) {}
        virtual void react(const KIM2CommEventRespOk&) {}
        virtual void react(const KIM2CommEventRespError&) {}
        virtual void react(const KIM2CommEventUartError&) {}
    };

class KIM2Comm : public EventEmitter<KIM2CommEventListener> {
public:
    unsigned int m_kineis_id = 0;
    unsigned int m_hex_addr = 0;
    uint16_t m_tx_status = 0xFFFF;
    bool m_is_rx_started = false;

    KIM2Comm(unsigned int libuarte_async_instance = 1);
    
    void init();
    void deinit();
    bool send(KIM2::ATCmd cmd, const std::optional<std::string>& params = std::nullopt);

    //Callbacks for async event handler
    void handle_tx_done(void);
    void handle_rx_buffer(uint8_t * buffer, uint8_t length);
    void handle_error(unsigned int error_type);

private: 
    unsigned int m_uart_instance;
    bool m_is_init;
    bool m_is_send_busy;
    std::string m_tx_buffer;
    std::string m_rx_buffer;

    bool send_at_cmd(KIM2::ATCmd cmd, const std::optional<std::string>& params = std::nullopt);
    KIM2::RespType parse_rx_message(const std::string& buffer, uint8_t size, uint8_t * used_length);
    
};
