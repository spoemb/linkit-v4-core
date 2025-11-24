#pragma once

class NrfUSB final {
public:
    static void init();
    static void uninit();
    static int write(char *ptr, int len);
    static void process();
    static void set_port_open(bool is_open);

private:
    static bool m_port_open;
};
