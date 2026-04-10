#pragma once

/**
 * @file nrf_switch.hpp
 * @brief nRF52840 debounced switch driver (GPIOTE + timer-based debounce).
 *
 * On edge detection, GPIOTE events are disabled and a timer schedules a
 * readback after the hysteresis period.  This filters all bounce interrupts
 * and produces a single clean state change callback.
 */

#include "switch.hpp"
#include "timer.hpp"

class NrfSwitch : public Switch {
private:
	Timer::TimerHandle m_timer_handle;
	bool m_debouncing = false;  ///< True while GPIOTE is disabled during debounce

	/// @brief Called from GPIOTE ISR — starts debounce timer.
	void process_event(bool state);

	/// @brief Apply new state and call handler if changed.
	void update_state(bool state);

public:
	/// @param pin                BSP GPIO enum index.
	/// @param hysteresis_time_ms Debounce period in ms.
	/// @param active_state       Pin level when switch is "on" (true = high).
	NrfSwitch(int pin, unsigned int hysteresis_time_ms, bool active_state = true);
	~NrfSwitch();

	void start(std::function<void(bool)> func) override;
	void stop() override;
	bool get_state();
	void pause() override;
	void resume() override;

	// GPIOTE ISR needs to call process_event via the pin lookup table
	friend void nrfx_gpiote_switch_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
};
