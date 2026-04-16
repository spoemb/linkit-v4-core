/**
 * @file lora_tx_service.cpp
 * @brief LoRa TX service — scheduling, burst preparation, TX event handling.
 */

#include <climits>
#include <algorithm>

#include "lora_tx_service.hpp"
#include "gps.hpp"
#include "messages.hpp"
#include "timeutils.hpp"
#include "binascii.hpp"
#include "debug.hpp"

extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern GPSDevice *gps_device;

LoRaTxService::LoRaTxService(KineisDevice& device) :
	Service(ServiceIdentifier::LORA_TX, "LORATX"),
	m_device(device) {
}

void LoRaTxService::service_init() {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);

	m_device.subscribe(*this);

	// Warn if SURFACING_BURST mode is configured without underwater detection
	if (argos_config.mode == BaseArgosMode::SURFACING_BURST && !argos_config.underwater_en) {
		DEBUG_WARN("LoRaTxService: SURFACING_BURST mode requires UNDERWATER_EN=1 — burst will not trigger without SWS");
	}

	// Use argos_id as seed for scheduler jitter (or any unique device identifier)
	m_sched.reset(argos_config.argos_id);
	m_depth_pile_manager.clear();
	m_is_first_tx = true;
	m_is_tx_pending = false;
	m_session_tx_count = 0;
	m_is_surfacing_burst = false;
	m_awaiting_surfacing = false;
	m_has_gnss_fix_since_surfacing = false;
	m_first_gnss_tx_sent = false;
	m_status_burst_count = 0;
	m_last_tx_had_gps = false;

	DEBUG_TRACE("LoRaTxService::service_init: initialized");
}

void LoRaTxService::service_term() {
	m_device.unsubscribe(*this);
}

bool LoRaTxService::service_is_enabled() {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	return (argos_config.mode != BaseArgosMode::OFF);
}

unsigned int LoRaTxService::service_next_schedule_in_ms() {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	std::time_t now = service_current_time();

	DEBUG_TRACE("LoRaTxService::service_next_schedule_in_ms");

	// Critical battery check
	if (argos_config.is_lb) {
		service_update_battery();
		unsigned int critical_level = configuration_store->read_param<unsigned int>(ParamID::LB_CRITICAL_THRESH);
		unsigned int current_soc = service_get_level();
		if (current_soc < critical_level) {
			DEBUG_INFO("LoRaTxService: CRITICAL battery SOC %u%% < %u%% - shutdown",
			           current_soc, critical_level);
			configuration_store->save_params();
			PMU::powerdown();
			return Service::SCHEDULE_DISABLED;
		}
	}

	if (argos_config.mode == BaseArgosMode::OFF) {
		return Service::SCHEDULE_DISABLED;
	}

	// Wait for GNSS time to be known before scheduling
	if (argos_config.gnss_en && !service_is_time_known()) {
		DEBUG_TRACE("LoRaTxService::service_next_schedule_in_ms: waiting for GNSS time");
		return Service::SCHEDULE_DISABLED;
	}

	// If depth pile has eligible entries, schedule GPS or sensor burst
	if (m_depth_pile_manager.eligible()) {
		if (argos_config.sensor_tx_enable) {
			m_scheduled_task = [this]() { process_sensor_burst(); };
		} else {
			m_scheduled_task = [this]() { process_gps_burst(); };
		}
	} else {
		// No GPS data available, send status heartbeat
		m_scheduled_task = [this]() { process_status_burst(); };
	}

	if (argos_config.mode == BaseArgosMode::DUTY_CYCLE) {
		if (m_is_first_tx && m_depth_pile_manager.eligible()) {
			m_sched.schedule_at(now);
			return 0;
		}
		return m_sched.schedule_duty_cycle(argos_config, now);
	}
	if (argos_config.mode == BaseArgosMode::LEGACY) {
		if (m_is_first_tx && m_depth_pile_manager.eligible()) {
			m_sched.schedule_at(now);
			return 0;
		}
		return m_sched.schedule_legacy(argos_config, now);
	}
	if (argos_config.mode == BaseArgosMode::SURFACING_BURST) {

		// Phase 1: Status heartbeat burst (battery level) until GNSS fix
		if (m_is_surfacing_burst && !m_has_gnss_fix_since_surfacing) {

			// Check max message limit (0 = unlimited)
			unsigned int burst_max_msg = configuration_store->read_param<unsigned int>(ParamID::SURFACING_BURST_MAX_MSG);
			if (burst_max_msg > 0 &&
				m_status_burst_count >= burst_max_msg) {
				DEBUG_INFO("LoRaTxService::SURFACING_BURST: max status messages reached (%u/%u)",
				           m_status_burst_count, burst_max_msg);
				m_is_surfacing_burst = false;
				m_awaiting_surfacing = true;
				return Service::SCHEDULE_DISABLED;
			}

			m_scheduled_task = [this]() { process_status_burst(); };

			// First message is immediate
			if (m_status_burst_count == 0) {
				DEBUG_INFO("LoRaTxService::SURFACING_BURST: status #%u (immediate)", m_status_burst_count + 1);
				m_sched.schedule_at(now);
				return 0;
			}

			// Progressive interval: init + (count-1) * step, capped at max
			unsigned int interval_s = argos_config.surfacing_burst_init_s +
				(m_status_burst_count - 1) * argos_config.surfacing_burst_step_s;
			if (interval_s > argos_config.surfacing_burst_max_s)
				interval_s = argos_config.surfacing_burst_max_s;

			DEBUG_INFO("LoRaTxService::SURFACING_BURST: status #%u in %u s", m_status_burst_count + 1, interval_s);
			m_sched.schedule_at(now + interval_s);
			return interval_s * 1000;
		}

		// Phase 2: GNSS fix available — TX position immediately, then TR_NOM
		if (m_has_gnss_fix_since_surfacing) {
			if (!m_depth_pile_manager.eligible()) {
				DEBUG_TRACE("LoRaTxService::SURFACING_BURST: GNSS phase but no eligible entries");
				return Service::SCHEDULE_DISABLED;
			}
			if (argos_config.sensor_tx_enable) {
				m_scheduled_task = [this]() { process_sensor_burst(); };
			} else {
				m_scheduled_task = [this]() { process_gps_burst(); };
			}

			// First GNSS TX is immediate after fix, then use tx_interval_s
			// Note: m_first_gnss_tx_sent is set in service_initiate(), not here.
			if (!m_first_gnss_tx_sent) {
				DEBUG_INFO("LoRaTxService::SURFACING_BURST: GNSS fix — TX immediate");
				m_sched.schedule_at(now);
				return 0;
			}
			return m_sched.schedule_legacy(argos_config, now);
		}

		// Burst ended — wait for next surfacing event
		if (m_awaiting_surfacing) {
			return Service::SCHEDULE_DISABLED;
		}

		return Service::SCHEDULE_DISABLED;
	}

	return Service::SCHEDULE_DISABLED;
}

void LoRaTxService::service_initiate() {
	DEBUG_TRACE("LoRaTxService::service_initiate");
	m_is_first_tx = false;
	m_is_tx_pending = true;

	// Track status burst count in SURFACING_BURST phase 1
	if (m_is_surfacing_burst && !m_has_gnss_fix_since_surfacing) {
		m_status_burst_count++;
	}

	// Mark first GNSS TX as sent only when actually executing (not during scheduling)
	if (m_has_gnss_fix_since_surfacing && !m_first_gnss_tx_sent) {
		m_first_gnss_tx_sent = true;
	}

	m_scheduled_task();
}

bool LoRaTxService::service_is_active_on_initiate() {
	return false;
}

bool LoRaTxService::service_cancel() {
	DEBUG_TRACE("LoRaTxService::service_cancel: pending=%u", m_is_tx_pending);
	bool is_pending = m_is_tx_pending;
	m_is_tx_pending = false;
	m_device.stop_send();
	return is_pending;
}

unsigned int LoRaTxService::service_next_timeout() {
	// LoRa TX can take longer than Argos: join delay + on-air time at low DR
	return 60000;
}

bool LoRaTxService::service_is_triggered_on_surfaced(bool& immediate) {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	// In SURFACING_BURST mode, reschedule immediately on surfacing
	immediate = (argos_config.mode == BaseArgosMode::SURFACING_BURST);
	return true;
}

void LoRaTxService::notify_peer_event(ServiceEvent& e) {

	// During SURFACING_BURST status phase, CloudLocate/Fastloc/NO_FIX entries are already
	// sent directly in process_status_burst() — skip depth pile to avoid double transmission.
	bool skip_depth_pile = false;
	if (m_is_surfacing_burst && !m_has_gnss_fix_since_surfacing &&
	    e.event_source == ServiceIdentifier::GNSS_SENSOR &&
	    e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		GPSLogEntry& gps = std::get<GPSLogEntry>(e.event_data);
		if (gps.info.event_type == GPSEventType::CLOUDLOCATE ||
		    gps.info.event_type == GPSEventType::FASTLOC ||
		    gps.info.event_type == GPSEventType::NO_FIX) {
			skip_depth_pile = true;
		}
	}

	if (!skip_depth_pile) {
		m_depth_pile_manager.notify_peer_event(e);
	}

	if (e.event_source == ServiceIdentifier::GNSS_SENSOR &&
		e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		GPSLogEntry& entry = std::get<GPSLogEntry>(e.event_data);
		ArgosConfig argos_config;
		configuration_store->get_argos_configuration(argos_config);

		// Real GPS fix: purge degraded entries and handle phase transition
		if (entry.info.valid && entry.info.event_type != GPSEventType::FASTLOC) {
			unsigned int purged = m_depth_pile_manager.purge_non_fix_entries();
			if (purged) {
				DEBUG_INFO("LoRaTxService::notify_peer_event: purged %u non-fix entries from depth pile", purged);
			}

			if (argos_config.mode == BaseArgosMode::SURFACING_BURST) {
				if ((m_is_surfacing_burst || m_awaiting_surfacing) && !m_has_gnss_fix_since_surfacing) {
					DEBUG_INFO("LoRaTxService::SURFACING_BURST: GNSS fix after %u status messages — switching to GPS phase",
					           m_status_burst_count);
					m_has_gnss_fix_since_surfacing = true;
					m_awaiting_surfacing = false;
					m_first_gnss_tx_sent = false;
					service_reschedule();
				}
			} else {
				// LEGACY / DUTY_CYCLE: GNSS fix → TX immediately, reset TR_NOM timer
				DEBUG_INFO("LoRaTxService: GNSS fix — reschedule for immediate TX");
				m_is_first_tx = true;
				service_reschedule();
			}
		}
	} else if (e.event_source == ServiceIdentifier::UW_SENSOR &&
			e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		ArgosConfig argos_config;
		configuration_store->get_argos_configuration(argos_config);
		if (std::get<bool>(e.event_data) == true) {
			// Dive: reset surfacing burst state
			if (argos_config.mode == BaseArgosMode::SURFACING_BURST) {
				m_is_surfacing_burst = false;
				m_awaiting_surfacing = false;
				m_status_burst_count = 0;
				m_has_gnss_fix_since_surfacing = false;
				m_first_gnss_tx_sent = false;
			}
		} else {
			// Surface
			std::time_t earliest_schedule = service_current_time() + argos_config.dry_time_before_tx;
			m_sched.set_earliest_schedule(earliest_schedule);
			if (argos_config.mode == BaseArgosMode::SURFACING_BURST) {
				m_is_surfacing_burst = true;
				m_awaiting_surfacing = false;
				m_status_burst_count = 0;
				m_has_gnss_fix_since_surfacing = false;
				m_first_gnss_tx_sent = false;
				m_is_first_tx = true;
				DEBUG_INFO("LoRaTxService::SURFACING_BURST: surface detected — starting status burst");
			}
		}
	}

	Service::notify_peer_event(e);
}

unsigned int LoRaTxService::get_max_payload_bytes() {
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	unsigned int dr = configuration_store->read_param<unsigned int>(ParamID::LORA_DR);
#else
	unsigned int dr = 0;
#endif
	return LoRaPayloadLimits::max_payload_for_dr((uint8_t)dr);
}

void LoRaTxService::process_gps_burst() {
	DEBUG_TRACE("LoRaTxService::process_gps_burst");

	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);

	unsigned int max_payload = get_max_payload_bytes();
	unsigned int max_entries = LoRaPacketBuilder::max_gps_entries(max_payload);

	// Retrieve GPS entries from depth pile
	std::vector<GPSLogEntry*> v = m_depth_pile_manager.retrieve_gps((unsigned int)argos_config.depth_pile);

	if (v.size()) {
		KineisPacket packet;
		unsigned int size_bits;

		// CloudLocate entries: send as dedicated CloudLocate packet
		if (v.back()->info.event_type == GPSEventType::CLOUDLOCATE) {
			const uint8_t* overlay = reinterpret_cast<const uint8_t*>(&v.back()->info.lon);
			uint8_t format_id = overlay[0];
			const uint8_t* blob = &overlay[1];
			unsigned int blob_size = (format_id == (uint8_t)BaseCloudLocateFormat::MEASC12) ? 12 : 20;
			packet = LoRaPacketBuilder::build_cloudlocate_packet(blob, blob_size, format_id,
					argos_config.is_lb, v.back()->info.batt_voltage, size_bits);
		// Fastloc entries: send as unified sensor packet (GPS + fastloc quality metadata)
		} else if (v.back()->info.event_type == GPSEventType::FASTLOC) {
			packet = LoRaPacketBuilder::build_sensor_packet(v.back(),
					nullptr, nullptr, nullptr, nullptr, nullptr,
					argos_config.is_out_of_zone, argos_config.is_lb,
					size_bits);
		} else {
			// Filter out any CloudLocate/fastloc entries that may be mixed in
			v.erase(std::remove_if(v.begin(), v.end(), [](const GPSLogEntry* e) {
				return e->info.event_type == GPSEventType::CLOUDLOCATE ||
				       e->info.event_type == GPSEventType::FASTLOC;
			}), v.end());
			if (v.empty()) {
				DEBUG_WARN("LoRaTxService::process_gps_burst: all entries filtered (mixed types)");
				m_is_tx_pending = false;
				service_complete();
				return;
			}
			// Trim to max entries that fit in payload
			if (v.size() > max_entries)
				v.resize(max_entries);

			packet = LoRaPacketBuilder::build_gps_packet(v,
					argos_config.is_out_of_zone, argos_config.is_lb,
					argos_config.delta_time_loc,
					max_payload,
					size_bits);
		}

		DEBUG_INFO("LoRaTxService::process_gps_burst: data=%s sz=%u bits",
				Binascii::hexlify(packet).c_str(), size_bits);
		m_last_tx_had_gps = true;
		m_device.send(KineisModulation::LDA2, packet, size_bits);
	} else {
		DEBUG_WARN("LoRaTxService::process_gps_burst: no eligible entries in depth pile");
		if (m_has_gnss_fix_since_surfacing) {
			DEBUG_INFO("LoRaTxService::process_gps_burst: ending GNSS phase (depth pile exhausted)");
			m_has_gnss_fix_since_surfacing = false;
			m_first_gnss_tx_sent = false;
			m_awaiting_surfacing = true;
		}
		m_is_tx_pending = false;
		service_complete();
	}
}

void LoRaTxService::process_sensor_burst() {
	DEBUG_TRACE("LoRaTxService::process_sensor_burst");

	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);

	GPSLogEntry* gps = m_depth_pile_manager.retrieve_gps_single((unsigned int)argos_config.depth_pile);

	if (gps != nullptr) {
		// CloudLocate entries: send as dedicated CloudLocate packet
		if (gps->info.event_type == GPSEventType::CLOUDLOCATE) {
			const uint8_t* overlay = reinterpret_cast<const uint8_t*>(&gps->info.lon);
			uint8_t format_id = overlay[0];
			const uint8_t* blob = &overlay[1];
			unsigned int blob_size = (format_id == (uint8_t)BaseCloudLocateFormat::MEASC12) ? 12 : 20;
			unsigned int size_bits;
			KineisPacket packet = LoRaPacketBuilder::build_cloudlocate_packet(blob, blob_size, format_id,
					service_is_battery_level_low(), gps->info.batt_voltage, size_bits);
			DEBUG_INFO("LoRaTxService::process_sensor_burst: CloudLocate fmt=%u data=%s",
					format_id, Binascii::hexlify(packet).c_str());
			m_last_tx_had_gps = true;
			m_device.send(KineisModulation::LDA2, packet, size_bits);
			return;
		}

		// Fastloc entries are handled transparently by build_sensor_packet
		// (the fastloc flag bit is set automatically when event_type == FASTLOC)
		unsigned int size_bits;
		KineisPacket packet = LoRaPacketBuilder::build_sensor_packet(gps,
				m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile, ServiceIdentifier::ALS_SENSOR),
				m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile, ServiceIdentifier::PH_SENSOR),
				m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile, ServiceIdentifier::PRESSURE_SENSOR),
#ifdef BOARD_RSPB
				m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile, ServiceIdentifier::THERMISTOR_SENSOR),
#else
				m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile, ServiceIdentifier::SEA_TEMP_SENSOR),
#endif
#if ENABLE_AXL_SENSOR
				m_depth_pile_manager.retrieve_sensor_single((unsigned int)argos_config.depth_pile, ServiceIdentifier::AXL_SENSOR),
#else
				nullptr,
#endif
				argos_config.is_out_of_zone,
				argos_config.is_lb,
				size_bits);

		DEBUG_INFO("LoRaTxService::process_sensor_burst: data=%s sz=%u bits",
				Binascii::hexlify(packet).c_str(), size_bits);
		m_last_tx_had_gps = true;
		m_device.send(KineisModulation::LDA2, packet, size_bits);
	} else {
		DEBUG_WARN("LoRaTxService::process_sensor_burst: no eligible entries in depth pile");
		if (m_has_gnss_fix_since_surfacing) {
			DEBUG_INFO("LoRaTxService::process_sensor_burst: ending GNSS phase (depth pile exhausted)");
			m_has_gnss_fix_since_surfacing = false;
			m_first_gnss_tx_sent = false;
			m_awaiting_surfacing = true;
		}
		m_is_tx_pending = false;
		service_complete();
	}
}

void LoRaTxService::process_status_burst() {
	DEBUG_TRACE("LoRaTxService::process_status_burst");

	service_update_battery();
	unsigned int size_bits;

	// Progressive CloudLocate: after the first status ping, if CloudLocate is enabled
	// and raw measurements are available, send a CloudLocate packet.
	unsigned int fastloc_mode = configuration_store->read_param<unsigned int>(ParamID::GNSS_FASTLOC_MODE);
	if (fastloc_mode == (unsigned int)BaseFastlocMode::CLOUDLOCATE &&
	    m_status_burst_count > 0 && gps_device && gps_device->has_raw_measurement()) {
		GNSSRawMeasurement raw = gps_device->get_raw_measurement();
		unsigned int cl_format = configuration_store->read_param<unsigned int>(ParamID::GNSS_CLOUDLOCATE_FORMAT);

		const uint8_t* blob = nullptr;
		unsigned int blob_size = 0;
		uint8_t format_id = 0;

		// Select blob based on configured format (MEAS50 available live for LoRa)
		if (cl_format == (unsigned int)BaseCloudLocateFormat::MEAS50 && raw.has_meas50) {
			blob = raw.meas50; blob_size = 50; format_id = (uint8_t)BaseCloudLocateFormat::MEAS50;
		} else if (cl_format == (unsigned int)BaseCloudLocateFormat::MEAS20 && raw.has_meas20) {
			blob = raw.meas20; blob_size = 20; format_id = (uint8_t)BaseCloudLocateFormat::MEAS20;
		} else if (raw.has_measc12) {
			blob = raw.measc12; blob_size = 12; format_id = (uint8_t)BaseCloudLocateFormat::MEASC12;
		} else if (raw.has_meas20) {
			blob = raw.meas20; blob_size = 20; format_id = (uint8_t)BaseCloudLocateFormat::MEAS20;
		}

		if (blob) {
			unsigned int packet_bytes = (LoRaPacketBuilder::BITS_PKT_TYPE + LoRaPacketBuilder::BITS_CL_FORMAT +
			                             LoRaPacketBuilder::BITS_FLAGS + LoRaPacketBuilder::BITS_VOLTAGE +
			                             blob_size * 8 + 7) / 8;
			unsigned int max_payload = get_max_payload_bytes();
			if (packet_bytes > max_payload) {
				DEBUG_WARN("LoRaTxService::process_status_burst: CloudLocate MEAS%u (%u bytes) exceeds DR payload limit (%u bytes), skipping",
				           blob_size, packet_bytes, max_payload);
				// Don't send — fall through to normal status burst
			} else {
				KineisPacket packet = LoRaPacketBuilder::build_cloudlocate_packet(
					blob, blob_size, format_id,
					service_is_battery_level_low(), service_get_voltage(), size_bits);
				DEBUG_INFO("LoRaTxService::process_status_burst: CLOUDLOCATE #%u fmt=%u sz=%u data=%s",
				           m_status_burst_count + 1, format_id, blob_size,
				           Binascii::hexlify(packet).c_str());
				m_last_tx_had_gps = true;
				m_device.send(KineisModulation::LDA2, packet, size_bits);
				return;
			}
		}
	}

	// Progressive fastloc: after the first status ping, if fastloc is enabled
	// and the GPS has a degraded PVT available, send a fastloc sensor packet
	// instead of status-only. Same logic as Argos process_doppler_burst().
	if (fastloc_mode >= (unsigned int)BaseFastlocMode::DEGRADED_PVT &&
	    m_status_burst_count > 0 && gps_device && gps_device->has_degraded_pvt()) {
		GNSSData degraded = gps_device->get_degraded_pvt();

		// Build a temporary GPSLogEntry from the degraded PVT
		GPSLogEntry fastloc_entry{};
		fastloc_entry.header.log_type = LOG_GPS;
		fastloc_entry.info.lat = degraded.lat;
		fastloc_entry.info.lon = degraded.lon;
		fastloc_entry.info.height = degraded.height;
		fastloc_entry.info.hMSL = degraded.hMSL;
		fastloc_entry.info.hAcc = degraded.hAcc;
		fastloc_entry.info.vAcc = degraded.vAcc;
		fastloc_entry.info.velN = degraded.velN;
		fastloc_entry.info.velE = degraded.velE;
		fastloc_entry.info.velD = degraded.velD;
		fastloc_entry.info.gSpeed = degraded.gSpeed;
		fastloc_entry.info.headMot = degraded.headMot;
		fastloc_entry.info.sAcc = degraded.sAcc;
		fastloc_entry.info.headAcc = degraded.headAcc;
		fastloc_entry.info.pDOP = degraded.pDOP;
		fastloc_entry.info.vDOP = degraded.vDOP;
		fastloc_entry.info.hDOP = degraded.hDOP;
		fastloc_entry.info.headVeh = degraded.headVeh;
		fastloc_entry.info.fixType = degraded.fixType;
		fastloc_entry.info.numSV = degraded.numSV;
		fastloc_entry.info.ttff = degraded.ttff;
		fastloc_entry.info.onTime = degraded.ttff;
		fastloc_entry.info.batt_voltage = service_get_voltage();
		fastloc_entry.info.schedTime = service_current_time();
		fastloc_entry.info.valid = true;
		fastloc_entry.info.event_type = GPSEventType::FASTLOC;

		KineisPacket packet = LoRaPacketBuilder::build_sensor_packet(
			&fastloc_entry, nullptr, nullptr, nullptr, nullptr, nullptr,
			false, service_is_battery_level_low(), size_bits);

		DEBUG_INFO("LoRaTxService::process_status_burst: FASTLOC #%u hAcc=%um numSV=%u data=%s",
		           m_status_burst_count + 1, degraded.hAcc, degraded.numSV,
		           Binascii::hexlify(packet).c_str());
		m_last_tx_had_gps = true;
		m_device.send(KineisModulation::LDA2, packet, size_bits);
		return;
	}

	// Standard status packet (first ping, or fastloc not available)
	KineisPacket packet = LoRaPacketBuilder::build_status_packet(
			service_get_voltage(),
			service_is_battery_level_low(),
			size_bits);

	DEBUG_INFO("LoRaTxService::process_status_burst: data=%s sz=%u bits",
			Binascii::hexlify(packet).c_str(), size_bits);
	m_last_tx_had_gps = false;
	m_device.send(KineisModulation::LDA2, packet, size_bits);
}

void LoRaTxService::react(KineisEventTxStarted const&) {
	DEBUG_TRACE("LoRaTxService::react: KineisEventTxStarted");
	service_active();
}

void LoRaTxService::react(KineisEventTxComplete const&) {
	DEBUG_TRACE("LoRaTxService::react: KineisEventTxComplete");
	m_is_tx_pending = false;

	// Increment TX counter
	configuration_store->increment_tx_counter();
	m_session_tx_count++;

	// Update last TX date time
	std::time_t t = service_current_time();
	configuration_store->write_param(ParamID::LAST_TX, t);

	// Counters updated in RAM — flash persistence deferred to periodic flush / powerdown

	// Check session TX limit
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	if (argos_config.shutdown_ntime_sat > 0 && m_session_tx_count >= argos_config.shutdown_ntime_sat) {
		DEBUG_INFO("LoRaTxService: Session TX limit reached (%u/%u) | shutdown",
		           m_session_tx_count, argos_config.shutdown_ntime_sat);
		configuration_store->save_params();  // Flush before shutdown
		PMU::powerdown();
		return;
	}

	// Activate cooldown on any TX during a surfacing cycle
	{
		std::time_t now = service_current_time();
		if (now > 0)
			ServiceManager::set_cycle_complete(now);
	}

	m_sched.notify_tx_complete();
	service_complete();
}

void LoRaTxService::react(KineisEventDeviceError const&) {
	DEBUG_TRACE("LoRaTxService::react: KineisEventDeviceError");
	if (service_cancel())
		service_complete();
}
