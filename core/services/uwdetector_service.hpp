/**
 * @file uwdetector_service.hpp
 * @brief Underwater detector service — periodic SWS/pressure sampling to detect submersion.
 */

#pragma once

#include <functional>
#include "scheduler.hpp"
#include "service.hpp"

/// @brief Underwater detector — samples a switch/sensor to determine surface vs submerged state.
class UWDetectorService : public Service {

public:
	/// @param name  Service name (default "UWDetector").
	UWDetectorService(const char *name = "UWDetector") :
		Service(ServiceIdentifier::UW_SENSOR, name) {}
	virtual ~UWDetectorService() {}

protected:
	double m_activation_threshold = 0.0;
	unsigned int m_enable_sample_delay = 0;
	bool m_current_state = false;

	/// @brief Read the current detector state (subclasses override for SWS/pressure).
	/// @return true if submerged.
	virtual bool detector_state() { return false; }
	virtual bool service_is_enabled() = 0;
	void service_init() override;

private:
	bool m_is_first_time = true;
	bool m_pending_state = false;
	unsigned int m_period_underwater_ms = 0;
	unsigned int m_period_surface_ms = 0;
	unsigned int m_sample_iteration = 0;
	unsigned int m_sample_gap = 0;
	unsigned int m_min_dry_samples = 0;
	unsigned int m_dry_count = 0;

protected:
	unsigned int m_max_samples = 0;

	void service_term() override;
	unsigned int service_next_schedule_in_ms() override;
	void service_initiate() override;
	bool service_is_usable_underwater() override;
};
