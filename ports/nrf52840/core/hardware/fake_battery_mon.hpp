#pragma once

#include "battery.hpp"

/**
 * FakeBatteryMonitor - Always returns 4.1V and 100% level
 * Useful for testing or when no battery monitoring hardware is present
 */
class FakeBatteryMonitor : public BatteryMonitor {
private:
    void internal_update() override;

public:
    FakeBatteryMonitor(uint16_t critical_voltage = 2800, uint8_t low_level = 10);
};