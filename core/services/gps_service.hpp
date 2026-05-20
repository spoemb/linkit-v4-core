/**
 * @file gps_service.hpp
 * @brief GNSS acquisition service — periodic fix, cold start, fastloc, CloudLocate.
 */

#pragma once

#include <atomic>
#include <functional>

#include "gps.hpp"
#include "service.hpp"
#include "logger.hpp"
#include "timeutils.hpp"
#include "scheduler.hpp"


/// @brief CSV log formatter for GPS entries (used by DUMPD command).
class GPSLogFormatter : public LogFormatter {
public:
	const std::string header() override {
		return "log_datetime,batt_voltage,iTOW,fix_datetime,valid,onTime,ttff,fixType,flags,flags2,flags3,numSV,lon,lat,height,hMSL,hAcc,vAcc,velN,velE,velD,gSpeed,headMot,sAcc,headAcc,pDOP,vDOP,hDOP,headVeh\r\n";
	}
	const std::string log_entry(const LogEntry& e) override {
		char entry[512], d1[128], d2[128];
		const auto *gps = reinterpret_cast<const GPSLogEntry *>(&e);

		snprintf(d1, sizeof(d1), "%02hhu/%02hhu/%04hu %02hhu:%02hhu:%02hhu",
		        gps->header.day, gps->header.month, gps->header.year,
		        gps->header.hours, gps->header.minutes, gps->header.seconds);
        snprintf(d2, sizeof(d2), "%02hhu/%02hhu/%04hu %02hhu:%02hhu:%02hhu",
                gps->info.day, gps->info.month, gps->info.year,
                gps->info.hour, gps->info.min, gps->info.sec);

		// Convert to CSV
		snprintf(entry, sizeof(entry), "%s,%f,%u,%s,%u,%u,%u,%u,%u,%u,%u,%u,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",
				d1,
				(double)gps->info.batt_voltage/1000,
				(unsigned int)gps->info.iTOW,
				d2,
				(unsigned int)gps->info.valid,
				(unsigned int)gps->info.onTime,
				(unsigned int)gps->info.ttff,
				(unsigned int)gps->info.fixType,
				(unsigned int)gps->info.flags,
				(unsigned int)gps->info.flags2,
				(unsigned int)gps->info.flags3,
				(unsigned int)gps->info.numSV,
				gps->info.lon,
				gps->info.lat,
				(double)gps->info.height / 1000,
				(double)gps->info.hMSL / 1000,
				(double)gps->info.hAcc / 1000,
				(double)gps->info.vAcc / 1000,
				(double)gps->info.velN / 1000,
				(double)gps->info.velE / 1000,
				(double)gps->info.velD / 1000,
				(double)gps->info.gSpeed / 1000,
				(double)gps->info.headMot,
				(double)gps->info.sAcc / 1000,
				(double)gps->info.headAcc,
				(double)gps->info.pDOP,
				(double)gps->info.vDOP,
				(double)gps->info.hDOP,
				(double)gps->info.headVeh);
		return std::string(entry);
	}
};

/// @brief GNSS acquisition service — periodic fixes, cold start, fastloc/CloudLocate fallback.
class GPSService : public Service, public GPSEventListener  {
public:
	GPSService(GPSDevice& device, Logger *logger) : Service(ServiceIdentifier::GNSS_SENSOR, "GNSS", logger), m_device(device) {
	    m_device.subscribe(*this);
	}
	void notify_peer_event(ServiceEvent& e) override;

	/// Manual / DTE entry-point for the V_BCKP coin-cell charge mode.
	/// duration_s > 0  → start (or extend) a charge session for the given seconds.
	/// duration_s == 0 → abort the current charge session immediately.
	/// Returns false if the request was refused (GNSS active or hardware not idle).
	bool request_backup_charge(unsigned int duration_s);

	/// True if charge is active OR pending (M10 powering off before retry).
	/// Used by gentracker's reed-switch handler to intercept events in both phases.
	bool is_backup_charge_active() const { return m_backup_active || m_pending_backup_duration_s > 0; }

	/// Set callbacks invoked on charge start and stop.
	/// on_start fires once when a new session begins (not on timer refresh).
	/// on_stop  fires on every stop (manual, timer, abort) and clears both callbacks.
	void set_backup_charge_callbacks(std::function<void()> on_start, std::function<void()> on_stop);

protected:

	// Service interface methods
	void service_init() override;
	void service_term() override;
	bool service_is_enabled() override;
	unsigned int service_next_schedule_in_ms() override;
	void service_initiate() override;
	bool service_cancel() override;
	unsigned int service_next_timeout() override;
	bool service_is_triggered_on_surfaced(bool &) override;
	bool service_is_usable_underwater() override;
	bool service_is_triggered_on_event(ServiceEvent&, bool&) override;

private:
	GPSDevice&   m_device;
	bool         m_is_first_fix_found = false;
	bool         m_is_first_schedule = true;
	unsigned int m_cold_start_ntry = 0;  ///< Consecutive failed acquisitions (reset on fix or surface)
	uint64_t     m_wakeup_time = 0;
	std::time_t  m_next_schedule = 0;
	struct {
		GNSSData data;
	} m_gnss_data = {};
	unsigned int m_num_gps_fixes = 0;
	bool m_is_active = false;

	// Backup-cell (V_BCKP) charge mode bookkeeping (independent of m_is_active).
	bool m_backup_active = false;
	bool m_underwater = false;
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	/// Gate: after surfacing, defer GNSS power-on until ArgosTxService completes
	/// its first satellite TX. Reduces CPU contention during the burst sequence.
	/// Set true on surface (in notify_peer_event), cleared on first ARGOS_TX SERVICE_INACTIVE.
	bool m_defer_gnss_until_argos_first_tx = false;
#endif
	unsigned int m_pending_backup_duration_s = 0;  ///< Set when waiting for M10 poweroff to retry
	Scheduler::TaskHandle m_backup_exit_task;     ///< Auto-exit timer once backup-charge is active
	Scheduler::TaskHandle m_backup_retry_task;    ///< Retry scheduler used while waiting for M10 poweroff
	Scheduler::TaskHandle m_backup_periodic_task;
	std::function<void()> m_on_backup_charge_start;
	std::function<void()> m_on_backup_charge_stop;

	void backup_charge_schedule_next();
	void backup_charge_periodic_fire();
	void backup_charge_stop_internal();
	void schedule_backup_charge_retry(unsigned int attempt);

    void react(const GPSEventMaxNavSamples&) override;
    void react(const GPSEventMaxSatSamples&) override;
    void react(const GPSEventPVT&) override;
    void react(const GPSEventPVTDegraded&) override;
    void react(const GPSEventRawMeasurement&) override;
    void react(const GPSEventCloudLocateReady&) override;
    void react(const GPSEventError&) override;
    void react(const GPSEventPowerOff&) override;

	// Private methods for GNSS
	void task_process_gnss_data();
	void task_process_degraded_gnss_data();
	void task_process_cloudlocate_data();
	GPSLogEntry invalid_log_entry();
	void gnss_data_callback(GNSSData data);
	void gnss_degraded_callback(GNSSData data);
	void gnss_cloudlocate_callback(GNSSRawMeasurement data);

	GNSSRawMeasurement m_raw_measurement;
};
