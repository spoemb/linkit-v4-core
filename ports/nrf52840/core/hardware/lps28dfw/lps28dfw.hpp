#pragma once

#include "pressure_sensor.hpp"
#include "lps28dfw_reg.h"
#include <string>

class LPS28DFW : public PressureSensorDevice {
public:
    LPS28DFW(unsigned int bus, unsigned char address = 0x5C);
    bool init();
    void read(double& temperature, double& pressure) override;

private:
    unsigned int m_bus;
    unsigned char m_addr;

    stmdev_ctx_t m_ctx;
    lps28dfw_md_t m_mode;

    static int32_t platform_write(void* handle, uint8_t reg, const uint8_t* bufp, uint16_t len);
    static int32_t platform_read(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len);
};
