/**
 * @file mortality_service.hpp
 * @brief Bird mortality detection service — confidence scoring from activity, temperature, GPS.
 */

#pragma once

#include "service.hpp"
#include "logger.hpp"
#include "messages.hpp"
#include "haversine.hpp"
#include "timeutils.hpp"
#include "debug.hpp"

/// @brief CSV log formatter for mortality entries.
class MortalityLogFormatter : public LogFormatter {
public:
	const std::string header() override {
		return "log_datetime,confidence,consecutive_days,status,last_activity,last_body_temp,last_lat,last_lon,last_eval_epoch\r\n";
	}
	const std::string log_entry(const LogEntry& e) override {
		char entry[256], d1[25];
		const auto *log = reinterpret_cast<const MortalityLogEntry *>(&e);
		std::time_t t;
		std::tm *tm;

		t = convert_epochtime(log->header.year, log->header.month, log->header.day, log->header.hours, log->header.minutes, log->header.seconds);
		tm = std::gmtime(&t);
		std::strftime(d1, sizeof(d1), "%d/%m/%Y %H:%M:%S", tm);

		const char *status_str = (log->info.status == MortalityStatus::CONFIRMED) ? "CONFIRMED" :
		                         (log->info.status == MortalityStatus::SUSPECTED) ? "SUSPECTED" : "ALIVE";

		snprintf(entry, sizeof(entry), "%s,%u,%u,%s,%u,%u,%.6f,%.6f,%lu\r\n",
				d1,
				log->info.confidence,
				log->info.consecutive_days,
				status_str,
				log->info.last_activity,
				log->info.last_body_temp,
				log->info.last_lat,
				log->info.last_lon,
				log->info.last_eval_epoch);
		return std::string(entry);
	}
};

/// @brief Bird mortality detection — computes confidence (0-100%) from activity, temperature, GPS.
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
	bool m_has_activity = false;
	bool m_has_temperature = false;
	bool m_has_gps = false;
	uint8_t  m_session_activity = 0;
	double   m_session_body_temp = 0.0;
	double   m_session_lat = 0.0;
	double   m_session_lon = 0.0;
	int32_t  m_session_gps_speed = 0;  ///< mm/s

	void reset_session_data();
	void evaluate_mortality();
	void persist_state();
	bool all_inputs_collected() const;
	unsigned int day_of_year(std::time_t epoch) const;
};
