#pragma once

/**
 * @file nrf_irq.hpp
 * @brief GPIOTE interrupt wrapper — defers pin events to the scheduler.
 *
 * Wraps the nRF GPIOTE driver so that pin change events are forwarded
 * to a user callback via the scheduler (not in ISR context).
 */

#include <functional>
#include "scheduler.hpp"
#include "irq.hpp"

class NrfIRQ : public IRQ {
private:
	int m_pin;  ///< BSP GPIO enum index (not raw nRF pin number)
	Scheduler::TaskHandle m_task;

public:
	/// @param pin  BSP GPIO enum index.
	/// @throws ErrorCode::RESOURCE_NOT_AVAILABLE if GPIOTE channel allocation fails.
	NrfIRQ(int pin);
	~NrfIRQ();

	void enable(std::function<void()> func) override;
	void disable() override;

	/// @brief Read current pin level (polarity-aware).
	bool poll();

	/// @brief Called from GPIOTE ISR — posts callback to scheduler.
	void process_event();
};
