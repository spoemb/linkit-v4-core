#pragma once

/**
 * @file fake_battery_mon.hpp
 * @brief Stub battery monitor that always reports 4.1 V / 100%.
 *
 * Used for development boards without battery monitoring hardware,
 * or when BATTERY_MONITOR_FAKE is set in CMake.
 */

#include "battery.hpp"

class FakeBatteryMonitor : public BatteryMonitor {
private:
	void internal_update() override;

public:
	FakeBatteryMonitor(uint8_t critical_level = 5, uint8_t low_level = 10);
};
