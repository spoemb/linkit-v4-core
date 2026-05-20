/**
 * @file argos_tx_service.hpp
 * @brief Argos TX service — orchestrates satellite packet transmission as a Service.
 */

#pragma once

#include <ctime>
#include <optional>
#include "kineis_device.hpp"
#include "service.hpp"
#include "config_store.hpp"
#include "service_scheduler.hpp"
#include "depth_pile.hpp"
#include "argos_packet_builder.hpp"
#include "argos_tx_scheduler.hpp"

/// @brief Argos satellite TX service — builds and transmits packets via KineisDevice.
class ArgosTxService : public Service, KineisEventListener {
public:
	ArgosTxService(KineisDevice& device);
	void notify_peer_event(ServiceEvent& e) override;

protected:
	void service_init() override;
	void service_term() override;
	bool service_is_enabled() override;
	unsigned int service_next_schedule_in_ms() override;
	void service_initiate() override;
	bool service_cancel() override;
	unsigned int service_next_timeout() override;
	bool service_is_triggered_on_surfaced(bool &immediate) override;
	bool service_is_active_on_initiate() override;

private:
	KineisDevice& m_kineis;
	DepthPileManager m_depth_pile_manager;
	ArgosTxScheduler m_sched;
	bool m_is_first_tx = true;
	bool m_is_tx_pending = false;
	bool m_tcxo_skip_on_next_tx = false;
	unsigned int m_session_tx_count = 0;
	std::function<void()> m_scheduled_task;
	KineisModulation m_scheduled_mode = KineisModulation::LDA2;

	// Surfacing burst state
	bool m_is_surfacing_burst = false;
	bool m_awaiting_surfacing = false;  ///< Burst ended, waiting for next surface event
	unsigned int m_doppler_burst_count = 0;
	bool m_has_gnss_fix_since_surfacing = false;
	bool m_first_gnss_tx_sent = false;
	bool m_last_tx_had_gps = false;
	bool m_cooldown_armed = false;

	// First-TX latency metric: PMU timestamp (ms) when surface was detected.
	// Used to log how long it took for the first satellite TX to complete after
	// the SWS state change. Reset on each surface event, consumed in react(KineisEventTxComplete).
	uint64_t m_surface_detected_ms = 0;

	void react(KineisEventTxStarted const &) override;
	void react(KineisEventTxComplete const &) override;
	void react(KineisEventDeviceError const &) override;

	void process_certification_burst();
	void process_time_sync_burst();
	void process_gnss_burst();
	void process_sensor_burst();
	void process_doppler_burst();

	// Adaptive modulation: switch RCONF if needed before TX
	bool ensure_modulation(KineisModulation target);
	std::string get_rconf_for_modulation(KineisModulation mode);
	KineisModulation m_last_preconfig_mod = KineisModulation::LDA2;
	std::optional<KineisModulation> m_modulation_preconfig;

	// Device error backoff
	static constexpr unsigned int DEVICE_ERROR_MAX_CONSECUTIVE = 3;
	static constexpr unsigned int DEVICE_ERROR_BACKOFF_BASE_MS = 60000;
	static constexpr unsigned int DEVICE_ERROR_BACKOFF_MAX_MS  = 600000;
	unsigned int m_consecutive_device_errors = 0;
};
