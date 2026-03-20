#pragma once

#include <cstdint>
#include "events.hpp"

struct BatteryMonitorEventVoltageCritical {};

class BatteryMonitorEventListener {
public:
	virtual ~BatteryMonitorEventListener() {}
	virtual void react(BatteryMonitorEventVoltageCritical const &) {};
};

class BatteryMonitor : public EventEmitter<BatteryMonitorEventListener> {
protected:
	uint16_t m_last_voltage_mv;
	uint8_t  m_last_level;
	uint8_t  m_critical_level;
	uint8_t  m_low_level;
	bool     m_is_low_level;
	bool     m_is_critical_voltage;

private:
	bool     m_is_critical_voltage_last;

	void actuate_events() {
		// Hysteresis: fire critical event on falling edge only,
		// require recovery above critical+hysteresis to re-arm
		if (m_is_critical_voltage && !m_is_critical_voltage_last) {
			notify<BatteryMonitorEventVoltageCritical>({});
			m_is_critical_voltage_last = true;
		} else if (m_is_critical_voltage_last && !m_is_critical_voltage) {
			// Only clear the flag when voltage recovers above hysteresis band
			// Subclasses set m_is_critical_voltage with hysteresis already applied
			m_is_critical_voltage_last = false;
		}
	}
	virtual void internal_update() {}

public:
	BatteryMonitor(uint8_t low_level, uint8_t critical_level) :
		m_last_voltage_mv(0), m_last_level(0),
		m_critical_level(critical_level), m_low_level(low_level),
		m_is_low_level(false),
		m_is_critical_voltage(false),
		m_is_critical_voltage_last(false) {}
	virtual ~BatteryMonitor() {}
	uint16_t get_voltage() { return m_last_voltage_mv; }
	uint8_t get_level() { return m_last_level; }
	bool is_battery_low() { return m_is_low_level; }
	bool is_battery_critical() { return m_is_critical_voltage; }
	void update() {
		internal_update();
		actuate_events();
	}
	virtual int shutdown() { return 0; }  // Optional shutdown for fuel gauges
};
