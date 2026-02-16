#include "lps28dfw.hpp"
#include "nrf_i2c.hpp"
#include "pmu.hpp"
#include "debug.hpp"
#include "gpio.hpp"  // For SensorsPowerGuard (VSENSORS management)

LPS28DFW::LPS28DFW(unsigned int bus, unsigned char address)
    : m_bus(bus), m_addr(address) {
    m_ctx.write_reg = platform_write;
    m_ctx.read_reg = platform_read;
    m_ctx.handle = this;
    init();
}

bool LPS28DFW::init() {
    SensorsPowerGuard power_guard;  // Acquire VSENSORS power for I2C access
    DEBUG_TRACE("LPS28DFW::init - bus=%u, addr=0x%02X", m_bus, m_addr);
    lps28dfw_id_t id;
    if (lps28dfw_id_get(&m_ctx, &id) != 0 || id.whoami != LPS28DFW_ID) {
        DEBUG_ERROR("LPS28DFW::init - WHOAMI mismatch or read error");
        return false;
    }

    if (lps28dfw_init_set(&m_ctx, LPS28DFW_DRV_RDY) != 0) return false;

    m_mode.odr = LPS28DFW_ONE_SHOT;
    m_mode.avg = LPS28DFW_32_AVG;
    m_mode.lpf = LPS28DFW_LPF_DISABLE;
    m_mode.fs  = LPS28DFW_4060hPa;

    if (lps28dfw_mode_set(&m_ctx, &m_mode) != 0) return false;

    return true;
}

void LPS28DFW::read(double& temperature, double& pressure) {
    SensorsPowerGuard power_guard;  // Acquire VSENSORS power for I2C access
    if (lps28dfw_trigger_sw(&m_ctx, &m_mode) != 0) {
        DEBUG_ERROR("LPS28DFW::read - trigger_sw failed");
        throw ErrorCode::I2C_COMMS_ERROR;
    }

    PMU::delay_ms(10); // allow sensor to complete conversion

    lps28dfw_data_t data;
    if (lps28dfw_data_get(&m_ctx, &m_mode, &data) != 0) {
        DEBUG_ERROR("LPS28DFW::read - data_get failed");
        throw ErrorCode::I2C_COMMS_ERROR;
    }

    temperature = static_cast<double>(data.heat.deg_c);
    pressure = static_cast<double>(data.pressure.hpa) / 1000.0; // convert hPa to bar

    DEBUG_TRACE("LPS28DFW::read - %.2f °C, %.5f bar", temperature, pressure);
}

int32_t LPS28DFW::platform_write(void* handle, uint8_t reg, const uint8_t* bufp, uint16_t len) {
    auto* self = static_cast<LPS28DFW*>(handle);

    if (len == 0) {
        return 0; // Nothing to write
    }

    // Allocate temporary buffer: [reg][data...]
    constexpr size_t max_buffer_size = 32; // adjust if needed
    if (static_cast<size_t>(len) + 1 > max_buffer_size) {
        DEBUG_ERROR("LPS28DFW::platform_write - length exceeds max buffer size");
        return -1; // Out of range
    }

    uint8_t buffer[max_buffer_size];
    buffer[0] = reg;
    memcpy(&buffer[1], bufp, len);

    try {
        NrfI2C::write(self->m_bus, self->m_addr, buffer, len + 1, false);
    } catch (...) {
        return -1;
    }
    return 0;

}

int32_t LPS28DFW::platform_read(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len) {
    auto* self = static_cast<LPS28DFW*>(handle);

    uint8_t reg_addr = reg;
    try {
        NrfI2C::write(self->m_bus, self->m_addr, &reg_addr, sizeof(reg_addr), true);  // No stop condition
        NrfI2C::read(self->m_bus, self->m_addr, bufp, len);
    } catch (...) {
        return -1;
    }

    return 0;
}