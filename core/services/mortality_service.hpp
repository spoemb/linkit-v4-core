#pragma once

#include "service.hpp"
#include "messages.hpp"
#include "haversine.hpp"
#include "timeutils.hpp"
#include "debug.hpp"

class MortalityService : public Service {
public:
	MortalityService(Logger *logger = nullptr);

	void notify_peer_event(ServiceEvent& event) override;

	// Public accessor for ArgosTxService to read current confidence
	unsigned int get_confidence() const { return m_state.confidence; }
	MortalityStatus get_status() const { return m_state.status; }
	uint8_t get_last_activity() const { return m_session_activity; }

protected:
	void service_init() override;
	void service_term() override;
	bool service_is_enabled() override;
	unsigned int service_next_schedule_in_ms() override;
	void service_initiate() override;
	bool service_cancel() override;

private:
	// Persisted state (restored from FsLog on boot)
	MortalityInfo m_state;

	// Session-local sensor data (collected from peer events)
	bool m_has_activity;
	bool m_has_temperature;
	bool m_has_gps;
	uint8_t  m_session_activity;
	double   m_session_body_temp;
	double   m_session_lat;
	double   m_session_lon;
	int32_t  m_session_gps_speed;  // mm/s

	void reset_session_data();
	void evaluate_mortality();
	void persist_state();
	bool all_inputs_collected() const;
	unsigned int day_of_year(std::time_t epoch) const;
};
