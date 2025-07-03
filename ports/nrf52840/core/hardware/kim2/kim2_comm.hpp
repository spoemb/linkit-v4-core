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
        AT_UNKNOWN
    };

    enum RespType {
        RX_OK = 0,
        RX_ERROR,
        RX_CONFIG,
        RX_UNKNOWN
    };

    static constexpr uint8_t SYNC_CHAR = '+';
    static constexpr uint8_t END_CHAR_1 = '\r';
    static constexpr uint8_t END_CHAR_2 = '\n';
    const std::string OK_RESPONSE = "+OK\r\n";
    const std::string ID_RESPONSE = "+ID=";
    const std::string ADDR_RESPONSE = "+ADDR=";
    static constexpr uint8_t ID_SIZE = 6;
    static constexpr uint8_t ADDR_SIZE = 8;
}

struct KIM2CommEventTxDone {};
struct KIM2CommEventOk {};

class KIM2CommEventListener {
    public:
        virtual ~KIM2CommEventListener() {}
        virtual void react(const KIM2CommEventTxDone&) {}
        virtual void react(const KIM2CommEventOk&) {}
};

class KIM2Comm : public EventEmitter<KIM2CommEventListener> {
public:
    uint8_t m_ascii_id[KIM2::ID_SIZE + 1];
    uint8_t m_ascii_addr[KIM2::ADDR_SIZE + 1];

    KIM2Comm(unsigned int libuarte_async_instance = 1);
    
    void init();
    void deinit();
    bool send(KIM2::ATCmd cmd);

    //Callbacks for async event handler
    void handle_tx_done(void);
    void handle_rx_buffer(uint8_t * buffer, uint8_t length);

private: 
    unsigned int m_uart_instance;
    bool m_is_init;
    bool m_is_send_busy;
    uint8_t m_tx_buffer[125];
    uint8_t m_rx_buffer[125];

    bool send_at_cmd(KIM2::ATCmd cmd, char * params, uint8_t params_size);
    KIM2::RespType parse_rx_message(unsigned char * buffer, uint8_t size, uint8_t * used_length);
    
};
