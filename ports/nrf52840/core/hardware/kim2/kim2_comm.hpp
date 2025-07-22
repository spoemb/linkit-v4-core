#pragma once

#include "events.hpp"
#include <stdint.h>
#include <string>


namespace KIM2 {
    /*Public types*/
    enum ATCmd {
        AT_PING = 0,
        AT_ID,
        AT_ADDR,
        AT_RCONF_SET,
        AT_KMAC_BASIC,
        AT_LPM_SET,
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
struct KIM2CommEventOk {};
struct KIM2CommEventError {};

class KIM2CommEventListener {
    public:
        virtual ~KIM2CommEventListener() {}
        virtual void react(const KIM2CommEventTxDone&) {}
        virtual void react(const KIM2CommEventOk&) {}
        virtual void react(const KIM2CommEventError&) {}
};

class KIM2Comm : public EventEmitter<KIM2CommEventListener> {
public:
    uint8_t m_ascii_id[KIM2::ID_SIZE + 1] = {0};
    uint8_t m_ascii_addr[KIM2::ADDR_SIZE + 1] = {0};
    uint16_t m_tx_status = 0xFFFF;

    KIM2Comm(unsigned int libuarte_async_instance = 1);
    
    void init();
    void deinit();
    bool send(KIM2::ATCmd cmd, const std::optional<std::string>& params = std::nullopt);

    //Callbacks for async event handler
    void handle_tx_done(void);
    void handle_rx_buffer(uint8_t * buffer, uint8_t length);

private: 
    unsigned int m_uart_instance;
    bool m_is_init;
    bool m_is_send_busy;
    std::string m_tx_buffer;
    std::string m_rx_buffer;

    bool send_at_cmd(KIM2::ATCmd cmd, const std::optional<std::string>& params = std::nullopt);
    KIM2::RespType parse_rx_message(const std::string& buffer, uint8_t size, uint8_t * used_length);
    
};
