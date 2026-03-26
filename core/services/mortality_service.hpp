#pragma once

#include "service.hpp"
#include "logger.hpp"
#include "messages.hpp"
#include "haversine.hpp"
#include "timeutils.hpp"
#include "debug.hpp"

class MortalityLogFormatter : public LogFormatter {
public:
	const std::string header() override {
		return "log_datetime,confidence,consecutive_days,status,last_activity,last_body_temp,last_lat,last_lon,last_eval_epoch\r\n";
	}
	const std::string log_entry(const LogEntry& e) override {
		char entry[256], d1[25];
		const MortalityLogEntry *log = (const MortalityLogEntry *)&e;
		std::time_t t;
		std::tm *tm;

		t = convert_epochtime(log->header.year, log->header.month, log->header.day, log->header.hours, log->header.minutes, log->header.seconds);
		tm = std::gmtime(&t);
		std::strftime(d1, sizeof(d1), "%d/%m/%Y %H:%M:%S", tm);

		const char *status_str = (log->info.status == MortalityStatus::CONFIRMED) ? "CONFIRMED" :
		                         (log->info.status == MortalityStatus::SUSPECTED) ? "SUSPECTED" : "ALIVE";

		snprintf(entry, sizeof(entry), "%s,%u,%u,%s,%u,%u,%.6f,%.6f,%u\r\n",
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
