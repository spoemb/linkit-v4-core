#pragma once

class NrfUSB final {
public:

    static bool m_port_open;

    static void init();
    static void uninit();
    static int write(char *ptr, int len);
    static bool process();

private:

};
