#include <climits>
#include <algorithm>

#include "lora_tx_service.hpp"
#include "messages.hpp"
#include "timeutils.hpp"
#include "bitpack.hpp"
#include "binascii.hpp"
#include "debug.hpp"


extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;


// ============================================================================
// LoRaPacketBuilder
// ============================================================================

unsigned int LoRaPacketBuilder::convert_speed(double x) {
	return (SECONDS_PER_HOUR * x) / (2 * MM_PER_KM);
}

unsigned int LoRaPacketBuilder::convert_battery_voltage(unsigned int battery_voltage) {
	return std::min((unsigned int)127, (unsigned int)std::max((int)battery_voltage - (int)REF_BATT_MV, (int)0) / MV_PER_UNIT);
}

unsigned int LoRaPacketBuilder::convert_latitude(double x) {
	if (x >= 0)
		return x * LON_LAT_RESOLUTION;
	else
		return ((unsigned int)((x - 0.00005) * NEG_LON_LAT_RESOLUTION)) | 1 << 20;
}

unsigned int LoRaPacketBuilder::convert_longitude(double x) {
	if (x >= 0)
		return x * LON_LAT_RESOLUTION;
	else
		return ((unsigned int)((x - 0.00005) * NEG_LON_LAT_RESOLUTION)) | 1 << 21;
}

unsigned int LoRaPacketBuilder::convert_heading(double x) {
	return (unsigned int)(x / 1.42);
}

unsigned int LoRaPacketBuilder::convert_altitude(double x) {
	return std::min((double)MAX_ALTITUDE, std::max((double)MIN_ALTITUDE, x / (MM_PER_METER * METRES_PER_UNIT)));
}

unsigned int LoRaPacketBuilder::max_gps_entries(unsigned int max_payload_bytes) {
	unsigned int max_bits = max_payload_bytes * BITS_PER_BYTE;
	// Header(12) + count(4) + delta(4) + first_gps(86) = 106 bits minimum
	unsigned int overhead_bits = BITS_HEADER + BITS_GPS_COUNT + BITS_DELTA_TIME + BITS_GPS_FULL;
	if (max_bits < overhead_bits)
		return 0;
	// Each additional entry is 50 bits
	unsigned int remaining = max_bits - overhead_bits;
	return 1 + remaining / BITS_GPS_DELTA;
}

KineisPacket LoRaPacketBuilder::build_gps_packet(std::vector<GPSLogEntry*>& v,
		bool is_out_of_zone, bool is_low_battery,
		BaseDeltaTimeLoc delta_time_loc,
		unsigned int max_payload_bytes,
		unsigned int& size_bits) {

	DEBUG_TRACE("LoRaPacketBuilder::build_gps_packet: %u entries, max=%u bytes", v.size(), max_payload_bytes);

	unsigned int max_entries = max_gps_entries(max_payload_bytes);
	unsigned int num_entries = std::min((unsigned int)v.size(), std::min(max_entries, 15U));

	// Compute packet size
	size_bits = BITS_HEADER + BITS_GPS_COUNT + BITS_DELTA_TIME + BITS_GPS_FULL;
	if (num_entries > 1)
		size_bits += (num_entries - 1) * BITS_GPS_DELTA;

	unsigned int packet_bytes = (size_bits + 7) / 8;
	KineisPacket packet;
	packet.assign(packet_bytes, 0);

	unsigned int base_pos = 0;

	// Header
	uint8_t pkt_type = (num_entries <= 1) ? PKT_TYPE_GPS_SINGLE : PKT_TYPE_GPS_MULTI;
	PACK_BITS((unsigned int)pkt_type, packet, base_pos, BITS_PKT_TYPE);

	// Flags: out_of_zone(1) + low_battery(1) + valid(1)
	bool valid = (num_entries > 0 && v[0]->info.valid);
	unsigned int flags = ((is_out_of_zone ? 1U : 0U) << 2) |
	                     ((is_low_battery ? 1U : 0U) << 1) |
	                     (valid ? 1U : 0U);
	PACK_BITS(flags, packet, base_pos, BITS_FLAGS);

	// Voltage from first entry (or 0)
	unsigned int batt = 0;
	if (num_entries > 0)
		batt = convert_battery_voltage((unsigned int)v[0]->info.batt_voltage);
	PACK_BITS(batt, packet, base_pos, BITS_VOLTAGE);

	// GPS count
	PACK_BITS(num_entries, packet, base_pos, BITS_GPS_COUNT);

	// Delta time loc
	PACK_BITS((unsigned int)delta_time_loc, packet, base_pos, BITS_DELTA_TIME);

	// First GPS entry (full with timestamp)
	if (num_entries > 0) {
		GPSLogEntry* gps = v[0];
		uint16_t year;
		uint8_t month, day, hour, min, sec;
		convert_datetime_to_epoch(gps->info.schedTime, year, month, day, hour, min, sec);

		PACK_BITS((unsigned int)day, packet, base_pos, BITS_DAY);
		PACK_BITS((unsigned int)hour, packet, base_pos, BITS_HOUR);
		PACK_BITS((unsigned int)min, packet, base_pos, BITS_MIN);

		if (gps->info.valid) {
			unsigned int lat = convert_latitude(gps->info.lat);
			PACK_BITS(lat, packet, base_pos, BITS_LATITUDE);
			unsigned int lon = convert_longitude(gps->info.lon);
			PACK_BITS(lon, packet, base_pos, BITS_LONGITUDE);
			unsigned int speed = convert_speed((double)gps->info.gSpeed);
			PACK_BITS(speed, packet, base_pos, BITS_SPEED);
			unsigned int heading = convert_heading(gps->info.headMot);
			PACK_BITS(heading, packet, base_pos, BITS_HEADING);
			if (gps->info.fixType == FIXTYPE_3D) {
				unsigned int alt = convert_altitude((double)gps->info.hMSL);
				PACK_BITS(alt, packet, base_pos, BITS_ALTITUDE);
			} else {
				PACK_BITS((unsigned int)INVALID_ALTITUDE, packet, base_pos, BITS_ALTITUDE);
			}
			PACK_BITS(std::min((unsigned int)gps->info.numSV, 15U), packet, base_pos, BITS_NUMSV);
		} else {
			// Invalid fix: fill with max values
			PACK_BITS(0x1FFFFF, packet, base_pos, BITS_LATITUDE);
			PACK_BITS(0x3FFFFF, packet, base_pos, BITS_LONGITUDE);
			PACK_BITS(0x7F, packet, base_pos, BITS_SPEED);
			PACK_BITS(0xFF, packet, base_pos, BITS_HEADING);
			PACK_BITS((unsigned int)INVALID_ALTITUDE, packet, base_pos, BITS_ALTITUDE);
			PACK_BITS(0U, packet, base_pos, BITS_NUMSV);
		}
	}

	// Subsequent GPS entries (delta: lat + lon + speed only)
	for (unsigned int i = 1; i < num_entries; i++) {
		GPSLogEntry* gps = v[i];
		if (gps->info.valid) {
			unsigned int lat = convert_latitude(gps->info.lat);
			PACK_BITS(lat, packet, base_pos, BITS_LATITUDE);
			unsigned int lon = convert_longitude(gps->info.lon);
			PACK_BITS(lon, packet, base_pos, BITS_LONGITUDE);
			unsigned int speed = convert_speed((double)gps->info.gSpeed);
			PACK_BITS(speed, packet, base_pos, BITS_SPEED);
		} else {
			PACK_BITS(0x1FFFFF, packet, base_pos, BITS_LATITUDE);
			PACK_BITS(0x3FFFFF, packet, base_pos, BITS_LONGITUDE);
			PACK_BITS(0x7F, packet, base_pos, BITS_SPEED);
		}
	}

	DEBUG_INFO("LoRaPacketBuilder::build_gps_packet: entries=%u data=%s sz=%u bits",
			num_entries, Binascii::hexlify(packet).c_str(), size_bits);

	return packet;
}

KineisPacket LoRaPacketBuilder::build_sensor_packet(GPSLogEntry* gps,
		ServiceSensorData* als, ServiceSensorData* ph,
		ServiceSensorData* pressure, ServiceSensorData* sea_temp,
		ServiceSensorData* axl,
		bool is_out_of_zone, bool is_low_battery,
		unsigned int& size_bits) {

	DEBUG_TRACE("LoRaPacketBuilder::build_sensor_packet");

	// Compute presence bitmask
	uint8_t mask = 0;
	if (gps != nullptr)       mask |= 0x20;  // bit 5
	if (als != nullptr)       mask |= 0x10;  // bit 4
	if (ph != nullptr)        mask |= 0x08;  // bit 3
	if (pressure != nullptr)  mask |= 0x04;  // bit 2
	if (sea_temp != nullptr)  mask |= 0x02;  // bit 1
	if (axl != nullptr)       mask |= 0x01;  // bit 0

	// Compute total size
	size_bits = BITS_HEADER + BITS_SENSOR_MASK;
	if (gps)      size_bits += BITS_GPS_FULL;
	if (als)      size_bits += BITS_ALS;
	if (ph)       size_bits += BITS_PH;
	if (pressure) size_bits += BITS_PRESSURE_FULL;
	if (sea_temp) size_bits += BITS_SEA_TEMP;
	if (axl)      size_bits += BITS_AXL_FULL;

	unsigned int packet_bytes = (size_bits + 7) / 8;
	KineisPacket packet;
	packet.assign(packet_bytes, 0);

	unsigned int base_pos = 0;

	// Header
	PACK_BITS((unsigned int)PKT_TYPE_SENSOR, packet, base_pos, BITS_PKT_TYPE);

	// Flags
	bool valid = (gps != nullptr && gps->info.valid);
	unsigned int flags = ((is_out_of_zone ? 1U : 0U) << 2) |
	                     ((is_low_battery ? 1U : 0U) << 1) |
	                     (valid ? 1U : 0U);
	PACK_BITS(flags, packet, base_pos, BITS_FLAGS);

	// Voltage
	unsigned int batt = 0;
	if (gps) batt = convert_battery_voltage((unsigned int)gps->info.batt_voltage);
	PACK_BITS(batt, packet, base_pos, BITS_VOLTAGE);

	// Sensor presence bitmask
	PACK_BITS((unsigned int)mask, packet, base_pos, BITS_SENSOR_MASK);

	// GPS data (full entry with timestamp)
	if (gps) {
		uint16_t year;
		uint8_t month, day, hour, min, sec;
		convert_datetime_to_epoch(gps->info.schedTime, year, month, day, hour, min, sec);

		PACK_BITS((unsigned int)day, packet, base_pos, BITS_DAY);
		PACK_BITS((unsigned int)hour, packet, base_pos, BITS_HOUR);
		PACK_BITS((unsigned int)min, packet, base_pos, BITS_MIN);

		if (gps->info.valid) {
			unsigned int lat = convert_latitude(gps->info.lat);
			PACK_BITS(lat, packet, base_pos, BITS_LATITUDE);
			unsigned int lon = convert_longitude(gps->info.lon);
			PACK_BITS(lon, packet, base_pos, BITS_LONGITUDE);
			unsigned int speed = convert_speed((double)gps->info.gSpeed);
			PACK_BITS(speed, packet, base_pos, BITS_SPEED);
			unsigned int heading = convert_heading(gps->info.headMot);
			PACK_BITS(heading, packet, base_pos, BITS_HEADING);
			if (gps->info.fixType == FIXTYPE_3D) {
				unsigned int alt = convert_altitude((double)gps->info.hMSL);
				PACK_BITS(alt, packet, base_pos, BITS_ALTITUDE);
			} else {
				PACK_BITS((unsigned int)INVALID_ALTITUDE, packet, base_pos, BITS_ALTITUDE);
			}
			PACK_BITS(std::min((unsigned int)gps->info.numSV, 15U), packet, base_pos, BITS_NUMSV);
		} else {
			PACK_BITS(0x1FFFFF, packet, base_pos, BITS_LATITUDE);
			PACK_BITS(0x3FFFFF, packet, base_pos, BITS_LONGITUDE);
			PACK_BITS(0x7F, packet, base_pos, BITS_SPEED);
			PACK_BITS(0xFF, packet, base_pos, BITS_HEADING);
			PACK_BITS((unsigned int)INVALID_ALTITUDE, packet, base_pos, BITS_ALTITUDE);
			PACK_BITS(0U, packet, base_pos, BITS_NUMSV);
		}
	}

	// ALS sensor
	if (als) {
		PACK_BITS((unsigned int)als->port[0], packet, base_pos, BITS_ALS);
	}

	// PH sensor
	if (ph) {
		PACK_BITS((unsigned int)ph->port[0], packet, base_pos, BITS_PH);
	}

	// Pressure sensor (pressure + temperature)
	if (pressure) {
		PACK_BITS((unsigned int)pressure->port[0], packet, base_pos, BITS_PRESSURE);
		PACK_BITS((unsigned int)pressure->port[1], packet, base_pos, BITS_PRESS_TEMP);
	}

	// Sea temperature / Thermistor
	if (sea_temp) {
		PACK_BITS((unsigned int)sea_temp->port[0], packet, base_pos, BITS_SEA_TEMP);
	}

	// Accelerometer
	if (axl) {
		PACK_BITS((unsigned int)axl->port[0], packet, base_pos, BITS_AXL_TEMP);
		PACK_BITS((unsigned int)axl->port[1], packet, base_pos, BITS_AXL_AXIS);
		PACK_BITS((unsigned int)axl->port[2], packet, base_pos, BITS_AXL_AXIS);
		PACK_BITS((unsigned int)axl->port[3], packet, base_pos, BITS_AXL_AXIS);
		PACK_BITS((unsigned int)axl->port[4], packet, base_pos, BITS_AXL_ACT);
	}

	DEBUG_INFO("LoRaPacketBuilder::build_sensor_packet: mask=0x%02x data=%s sz=%u bits",
			mask, Binascii::hexlify(packet).c_str(), size_bits);

	return packet;
}

KineisPacket LoRaPacketBuilder::build_status_packet(unsigned int battery_voltage,
		bool is_low_battery, unsigned int& size_bits) {

	DEBUG_TRACE("LoRaPacketBuilder::build_status_packet");

	size_bits = BITS_HEADER;  // 12 bits
	unsigned int packet_bytes = (size_bits + 7) / 8;  // 2 bytes
	KineisPacket packet;
	packet.assign(packet_bytes, 0);

	unsigned int base_pos = 0;

	PACK_BITS((unsigned int)PKT_TYPE_STATUS, packet, base_pos, BITS_PKT_TYPE);

	// Flags: out_of_zone=0, low_battery, valid=0
	unsigned int flags = (is_low_battery ? 1U : 0U) << 1;
	PACK_BITS(flags, packet, base_pos, BITS_FLAGS);

	unsigned int batt = convert_battery_voltage(battery_voltage);
	PACK_BITS(batt, packet, base_pos, BITS_VOLTAGE);

	DEBUG_INFO("LoRaPacketBuilder::build_status_packet: data=%s sz=%u bits",
			Binascii::hexlify(packet).c_str(), size_bits);

	return packet;
}


// ============================================================================
// LoRaTxScheduler
// ============================================================================

LoRaTxScheduler::LoRaTxScheduler() :
		m_rand(std::mt19937()) {
	m_last_schedule_abs.reset();
	m_curr_schedule_abs.reset();
	m_earliest_schedule.reset();
}

void LoRaTxScheduler::reset(unsigned int seed) {
	m_earliest_schedule.reset();
	m_last_schedule_abs.reset();
	m_rand.seed(seed);
}

bool LoRaTxScheduler::is_in_duty_cycle(uint64_t time_ms, unsigned int duty_cycle) {
	// Duty cycle is a 24-bit field, one bit per hour of the day
	uint64_t msec_of_day = (time_ms % (SECONDS_PER_DAY * MSECS_PER_SECOND));
	unsigned int hour_of_day = msec_of_day / (SECONDS_PER_HOUR * MSECS_PER_SECOND);
	return (duty_cycle & (0x800000 >> hour_of_day));
}

int LoRaTxScheduler::compute_random_jitter(bool jitter_en, int min, int max) {
	if (jitter_en) {
		std::uniform_int_distribution<int> dist(min, max);
		int jitter = dist(m_rand);
		DEBUG_TRACE("LoRaTxScheduler::compute_random_jitter: jitter=%d", jitter);
		return jitter;
	} else {
		return 0;
	}
}

void LoRaTxScheduler::schedule_periodic(unsigned int period_ms, bool jitter_en, unsigned int duty_cycle, uint64_t now_ms) {
	uint64_t start_time;

	DEBUG_TRACE("LoRaTxScheduler::schedule_periodic: now=%llu last=%llu tr=%u jitter=%u", now_ms,
			m_last_schedule_abs.has_value() ? m_last_schedule_abs.value() : 0,
			period_ms, jitter_en);

	// Handle earliest TX time
	while (m_earliest_schedule.has_value()) {
		DEBUG_TRACE("LoRaTxScheduler::schedule_periodic: earliest TX is in %llu", m_earliest_schedule.value() - now_ms);

		if (m_earliest_schedule.value() >= now_ms) {
			start_time = m_earliest_schedule.value();
			if (m_last_schedule_abs.has_value() &&
				start_time < m_last_schedule_abs.value())
				start_time = m_last_schedule_abs.value();

			if (is_in_duty_cycle(start_time, duty_cycle)) {
				m_curr_schedule_abs = start_time;
				return;
			} else {
				break;
			}
		} else {
			m_earliest_schedule.reset();
		}
		break;
	}

	// Compute new schedule
	if (!m_last_schedule_abs.has_value()) {
		start_time = now_ms + compute_random_jitter(jitter_en, 0);
	} else {
		start_time = m_last_schedule_abs.value() + period_ms + compute_random_jitter(jitter_en);
		if ((start_time + (MSECS_PER_SECOND * SECONDS_PER_DAY)) < now_ms)
			start_time = now_ms;
	}

	DEBUG_TRACE("LoRaTxScheduler::schedule_periodic: starting @ %llu", start_time);

	// Iterate forward to find a schedule within duty cycle (max 24h search)
	uint64_t elapsed_time = 0;
	while (elapsed_time <= (MSECS_PER_SECOND * SECONDS_PER_DAY)) {
		if (is_in_duty_cycle(start_time, duty_cycle) && start_time >= now_ms) {
			DEBUG_TRACE("LoRaTxScheduler::schedule_periodic: found schedule @ %llu", start_time);
			m_curr_schedule_abs = start_time;
			return;
		} else {
			start_time += period_ms;
			elapsed_time += period_ms;
		}
	}

	DEBUG_ERROR("LoRaTxScheduler::schedule_periodic: no schedule found!");
	m_curr_schedule_abs.reset();
	throw ErrorCode::RESOURCE_NOT_AVAILABLE;
}

unsigned int LoRaTxScheduler::schedule_duty_cycle(ArgosConfig& config, std::time_t now) {
	try {
		schedule_periodic((config.tr_nom * MSECS_PER_SECOND), config.argos_tx_jitter_en, config.duty_cycle, ((uint64_t)now * MSECS_PER_SECOND));
		return m_curr_schedule_abs.value() - (now * MSECS_PER_SECOND);
	} catch (...) {
		return INVALID_SCHEDULE;
	}
}

unsigned int LoRaTxScheduler::schedule_legacy(ArgosConfig& config, std::time_t now) {
	try {
		schedule_periodic((config.tr_nom * MSECS_PER_SECOND), config.argos_tx_jitter_en, DUTYCYCLE_24HRS, (now * MSECS_PER_SECOND));
		return m_curr_schedule_abs.value() - (now * MSECS_PER_SECOND);
	} catch (...) {
		return INVALID_SCHEDULE;
	}
}

void LoRaTxScheduler::set_earliest_schedule(std::time_t earliest) {
	DEBUG_TRACE("LoRaTxScheduler::set_earliest_schedule: t=%llu", earliest);
	m_earliest_schedule = (uint64_t)earliest * MSECS_PER_SECOND;
}

void LoRaTxScheduler::schedule_at(std::time_t t) {
	DEBUG_TRACE("LoRaTxScheduler::schedule_at: t=%llu", t);
	m_curr_schedule_abs = (uint64_t)t * MSECS_PER_SECOND;
}

void LoRaTxScheduler::notify_tx_complete() {
	m_last_schedule_abs = m_curr_schedule_abs;
}


// ============================================================================
// LoRaTxService
// ============================================================================

LoRaTxService::LoRaTxService(KineisDevice& device) :
	Service(ServiceIdentifier::LORA_TX, "LORATX"),
	m_device(device) {
}

void LoRaTxService::service_init() {
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);

	m_device.subscribe(*this);

	// Use argos_id as seed for scheduler jitter (or any unique device identifier)
	m_sched.reset(argos_config.argos_id);
	m_depth_pile_manager.clear();
	m_is_first_tx = true;
	m_is_tx_pending = false;
	m_session_tx_count = 0;

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
			DEBUG_INFO("LoRaTxService: CRITICAL battery SOC %u%% < %u%% - immediate powerdown",
			           current_soc, critical_level);
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
		return m_sched.schedule_duty_cycle(argos_config, now);
	}
	if (argos_config.mode == BaseArgosMode::LEGACY) {
		return m_sched.schedule_legacy(argos_config, now);
	}

	return Service::SCHEDULE_DISABLED;
}

void LoRaTxService::service_initiate() {
	DEBUG_TRACE("LoRaTxService::service_initiate");
	m_is_first_tx = false;
	m_is_tx_pending = true;
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
	immediate = false;
	return true;
}

void LoRaTxService::notify_peer_event(ServiceEvent& e) {
	m_depth_pile_manager.notify_peer_event(e);

	if (e.event_source == ServiceIdentifier::GNSS_SENSOR &&
		e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		// Reschedule if no existing schedule
		if (!service_is_scheduled()) {
			DEBUG_TRACE("LoRaTxService::notify_peer_event: rescheduling as no existing schedule");
			service_reschedule();
		}
	} else if (e.event_source == ServiceIdentifier::UW_SENSOR &&
			e.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
		if (std::get<bool>(e.event_data) == false) {
			ArgosConfig argos_config;
			configuration_store->get_argos_configuration(argos_config);
			std::time_t earliest_schedule = service_current_time() + argos_config.dry_time_before_tx;
			m_sched.set_earliest_schedule(earliest_schedule);
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
		// Trim to max entries that fit in payload
		if (v.size() > max_entries)
			v.resize(max_entries);

		unsigned int size_bits;
		KineisPacket packet = LoRaPacketBuilder::build_gps_packet(v,
				argos_config.is_out_of_zone, argos_config.is_lb,
				argos_config.delta_time_loc,
				max_payload,
				size_bits);

		DEBUG_INFO("LoRaTxService::process_gps_burst: data=%s sz=%u bits",
				Binascii::hexlify(packet).c_str(), size_bits);
		m_device.send(KineisModulation::LDA2, packet, size_bits);
	} else {
		DEBUG_WARN("LoRaTxService::process_gps_burst: no eligible entries in depth pile");
		service_complete();
	}
}

void LoRaTxService::process_sensor_burst() {
	DEBUG_TRACE("LoRaTxService::process_sensor_burst");

	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);

	GPSLogEntry* gps = m_depth_pile_manager.retrieve_gps_single((unsigned int)argos_config.depth_pile);

	if (gps != nullptr) {
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
		m_device.send(KineisModulation::LDA2, packet, size_bits);
	} else {
		DEBUG_WARN("LoRaTxService::process_sensor_burst: no eligible entries in depth pile");
		service_complete();
	}
}

void LoRaTxService::process_status_burst() {
	DEBUG_TRACE("LoRaTxService::process_status_burst");

	service_update_battery();
	unsigned int size_bits;
	KineisPacket packet = LoRaPacketBuilder::build_status_packet(
			service_get_voltage(),
			service_is_battery_level_low(),
			size_bits);

	DEBUG_INFO("LoRaTxService::process_status_burst: data=%s sz=%u bits",
			Binascii::hexlify(packet).c_str(), size_bits);
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

	// Save configuration params
	configuration_store->save_params();

	// Check session TX limit
	ArgosConfig argos_config;
	configuration_store->get_argos_configuration(argos_config);
	if (argos_config.shutdown_ntime_sat > 0 && m_session_tx_count >= argos_config.shutdown_ntime_sat) {
		DEBUG_INFO("LoRaTxService: Session TX limit reached (%u/%u) | powering down",
		           m_session_tx_count, argos_config.shutdown_ntime_sat);
		PMU::powerdown();
		return;
	}

	m_sched.notify_tx_complete();
	service_complete();
}

void LoRaTxService::react(KineisEventDeviceError const&) {
	DEBUG_TRACE("LoRaTxService::react: KineisEventDeviceError");
	if (service_cancel())
		service_complete();
}
