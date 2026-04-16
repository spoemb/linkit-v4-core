/**
 * @file lora_tx_service.hpp
 * @brief LoRa TX service — orchestrates LoRaWAN packet transmission as a Service.
 */

#pragma once

#include <ctime>
#include <optional>
#include <functional>
#include "kineis_device.hpp"
#include "service.hpp"
#include "config_store.hpp"
#include "service_scheduler.hpp"
#include "depth_pile.hpp"
#include "lora_packet_builder.hpp"
#include "lora_tx_scheduler.hpp"

/// @brief LoRa TX service — builds and transmits packets via KineisDevice (LoRa backend).
class LoRaTxService : public Service, KineisEventListener {
public:
	LoRaTxService(KineisDevice& device);

	/// @brief Handle peer events (GPS fix, sensor data, underwater state).
	/// @param e  Peer service event.
	void notify_peer_event(ServiceEvent& e) override;

protected:
	void service_init() override;
	void service_term() override;
	bool service_is_enabled() override;
	unsigned int service_next_schedule_in_ms() override;
	void service_initiate() override;
	bool service_cancel() override;
	unsigned int service_next_timeout() override;
	bool service_is_triggered_on_surfaced(bool& immediate) override;
	bool service_is_active_on_initiate() override;

private:
	KineisDevice& m_device;
	DepthPileManager m_depth_pile_manager;
	LoRaTxScheduler m_sched;
	bool m_is_first_tx = true;
	bool m_is_tx_pending = false;
	unsigned int m_session_tx_count = 0;
	bool m_last_tx_had_gps = false;
	bool m_is_surfacing_burst = false;
	bool m_awaiting_surfacing = false;  ///< Burst ended, waiting for next surface event
	bool m_has_gnss_fix_since_surfacing = false;
	bool m_first_gnss_tx_sent = false;
	unsigned int m_status_burst_count = 0;
	std::function<void()> m_scheduled_task;

	void react(KineisEventTxStarted const&) override;
	void react(KineisEventTxComplete const&) override;
	void react(KineisEventDeviceError const&) override;

	void process_gps_burst();
	void process_sensor_burst();
	void process_status_burst();

	/// @brief Get max payload bytes for current DR setting.
	/// @return Max bytes from LoRaPayloadLimits.
	unsigned int get_max_payload_bytes();
};
