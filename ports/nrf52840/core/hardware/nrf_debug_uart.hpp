#pragma once

#include <cstdint>

class NrfDebugUart {
public:
    static void init(uint32_t tx_pin);
    static void write(const char* data, int len);
    static bool is_init();
private:
    static bool m_is_init;
};
