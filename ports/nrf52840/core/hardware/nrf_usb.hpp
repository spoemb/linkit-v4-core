#pragma once

#include <string>

#define NRF_USB_RX_BUFFER_SIZE  4128  // Same as BLE buffer size

class NrfUSB final {
public:
    static void init();
    static void uninit();
    static int write(char *ptr, int len);
    static int read(char *ptr, int max_len);
    static std::string read_line();  // Returns complete line if available (terminated by \r or \n)
    static bool has_data();
    static void process();
    static void set_port_open(bool is_open);
    static bool is_port_open() { return m_port_open; }

    // Called from USB event handler when data is received
    static void on_rx_data(const char *data, size_t len);

private:
    static bool m_port_open;
    static char m_rx_buffer[NRF_USB_RX_BUFFER_SIZE];
    static volatile size_t m_rx_write_idx;
    static volatile size_t m_rx_read_idx;
    static volatile bool m_line_ready;
};
