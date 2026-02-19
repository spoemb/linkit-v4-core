#include "fake_battery_mon.hpp"
#include "debug.hpp"

// Fake battery values - always fully charged
#define FAKE_BATTERY_MV     4100    // 4.1V
#define FAKE_BATTERY_LEVEL  100     // 100%

FakeBatteryMonitor::FakeBatteryMonitor(uint16_t critical_voltage, uint8_t low_level)
    : BatteryMonitor(low_level, critical_voltage)
{
    DEBUG_INFO("FakeBatteryMonitor: Initialized (always returns %.2fV)", FAKE_BATTERY_MV / 1000.0);

    // Set initial values
    m_last_voltage_mv = FAKE_BATTERY_MV;
    m_last_level = FAKE_BATTERY_LEVEL;
    m_is_low_level = false;
    m_is_critical_voltage = false;
}

void FakeBatteryMonitor::internal_update()
{
    // Always return the same fake values
    m_last_voltage_mv = FAKE_BATTERY_MV;
    m_last_level = FAKE_BATTERY_LEVEL;
    m_is_low_level = false;
    m_is_critical_voltage = false;

    DEBUG_TRACE("FakeBatteryMonitor: Update - %umV | %u%%", m_last_voltage_mv, m_last_level);
}