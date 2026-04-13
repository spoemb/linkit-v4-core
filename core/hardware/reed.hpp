/**
 * @file reed.hpp
 * @brief Reed switch gesture detector — engage, short hold, long hold, release.
 */

#pragma once

#include <functional>
#include "switch.hpp"
#include "timer.hpp"
#include "scheduler.hpp"

enum class ReedSwitchGesture {
	ENGAGE,
	SHORT_HOLD,
	LONG_HOLD,
	RELEASE
};


/// @brief Reed switch with timed gesture detection (short hold 3s, long hold 6s).
class ReedSwitch {
private:
	Switch  &m_switch;
	uint64_t m_last_trigger_time = 0;
	unsigned int m_short_hold_period_ms;
	unsigned int m_long_hold_period_ms;
	Scheduler::TaskHandle m_task = {};

	void switch_state_handler(bool);

protected:
	std::function<void(ReedSwitchGesture)> m_user_callback;

public:
	ReedSwitch(
			Switch &sw,
			unsigned int short_hold_period_ms = 3000,
			unsigned int long_hold_period_ms = 6000
			);
	~ReedSwitch();
	void start(std::function<void(ReedSwitchGesture)> func);
	void stop();
	bool is_engaged() { return m_switch.get_state(); }
};
