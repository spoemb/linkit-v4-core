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

	// Pre-warm of the first surfacing-burst Doppler packet. While underwater
	// (only in SURFACING_BURST mode) the battery is sampled and the Doppler
	// payload is built so the first TX at surface skips the ADC read + packet
	// build on its critical path. Refreshed if the prep is older than 1h.
	bool m_is_underwater = false;
	KineisPacket m_prepared_doppler_packet;
	unsigned int m_prepared_doppler_size_bits = 0;
	KineisModulation m_prepared_doppler_mode = KineisModulation::LDA2;
	uint64_t m_prepared_at_ms = 0;
	static constexpr uint64_t PREPARED_DOPPLER_REFRESH_MS = 3600000ULL;  ///< 1 hour

	void react(KineisEventTxStarted const &) override;
	void react(KineisEventTxComplete const &) override;
	void react(KineisEventDeviceError const &) override;

	void process_certification_burst();
	void process_time_sync_burst();
	void process_gnss_burst();
	void process_sensor_burst();
	void process_doppler_burst();
	void prepare_doppler_packet();

	// Adaptive modulation: switch RCONF if needed before TX
	bool ensure_modulation(KineisModulation target);
	std::string get_rconf_for_modulation(KineisModulation mode);

	// @brief Modulation to use when adaptive is OFF. The user's master RCONF
	// (ARGOS_RADIOCONF) is encrypted hex — we can't tell locally which
	// modulation it encodes. The device layer (KIM2) reads back AT+RCONF=? at
	// init and caches the actual modulation, exposed via get_current_modulation().
	// SMD doesn't auto-detect (m_modulation stays at LDA2 default), so SMD
	// users keep today's behavior. Falls back to LDA2 on first cold boot
	// before init has run.
	KineisModulation resolve_non_adaptive_modulation();

	// @brief Bitmask of modulations whose per-mod RCONF is present (32-char hex)
	// in the config store. Used by burst processors to skip a TX cleanly when
	// the would-be fallback modulation can't hold the payload (instead of
	// hitting KIM2's silent payload-too-long drop + 30 s service timeout).
	// Computed at service_init() and on every scheduling cycle so runtime
	// PARMW edits are reflected. Bits: 0=LDK, 1=LDA2, 2=VLDA4.
	uint8_t m_modulation_avail_mask = 0;
	void refresh_modulation_availability();
	bool is_modulation_provisioned(KineisModulation mode) const;
	static bool size_fits_modulation(unsigned int payload_bits, KineisModulation mode);
	KineisModulation m_last_preconfig_mod = KineisModulation::LDA2;
	std::optional<KineisModulation> m_modulation_preconfig;

	// Device error backoff
	static constexpr unsigned int DEVICE_ERROR_MAX_CONSECUTIVE = 3;
	static constexpr unsigned int DEVICE_ERROR_BACKOFF_BASE_MS = 60000;
	static constexpr unsigned int DEVICE_ERROR_BACKOFF_MAX_MS  = 600000;
	unsigned int m_consecutive_device_errors = 0;
};
