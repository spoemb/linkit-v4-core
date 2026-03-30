#pragma once

#include <ctime>
#include <optional>
#include <random>
#include <functional>
#include "kineis_device.hpp"
#include "service.hpp"
#include "config_store.hpp"
#include "service_scheduler.hpp"
#include "depth_pile.hpp"


// LoRa payload limits by data rate
// NOTE: These values are for EU868. US915/AS923/other bands have different limits
// (e.g., US915 DR0 = 11 bytes). If multi-band support is needed, make this band-aware.
// With ADR enabled, the network server may change DR dynamically — use DR0 as safe fallback.
struct LoRaPayloadLimits {
	static constexpr unsigned int DR0_MAX_BYTES = 51;   // SF12/125kHz
	static constexpr unsigned int DR1_MAX_BYTES = 51;   // SF11/125kHz
	static constexpr unsigned int DR2_MAX_BYTES = 51;   // SF10/125kHz
	static constexpr unsigned int DR3_MAX_BYTES = 115;  // SF9/125kHz
	static constexpr unsigned int DR4_MAX_BYTES = 222;  // SF8/125kHz
	static constexpr unsigned int DR5_MAX_BYTES = 222;  // SF7/125kHz

	static unsigned int max_payload_for_dr(uint8_t dr) {
		switch (dr) {
			case 0: return DR0_MAX_BYTES;
			case 1: return DR1_MAX_BYTES;
			case 2: return DR2_MAX_BYTES;
			case 3: return DR3_MAX_BYTES;
			case 4: return DR4_MAX_BYTES;
			case 5: return DR5_MAX_BYTES;
			default: return DR0_MAX_BYTES;  // Safe fallback for unknown DR
		}
	}
};


class LoRaPacketBuilder {
public:
	// Packet type identifiers (2 bits)
	static constexpr uint8_t PKT_TYPE_GPS_SINGLE  = 0b00;
	static constexpr uint8_t PKT_TYPE_GPS_MULTI   = 0b01;
	static constexpr uint8_t PKT_TYPE_SENSOR      = 0b10;
	static constexpr uint8_t PKT_TYPE_STATUS       = 0b11;

	// Field bit widths
	static constexpr unsigned int BITS_PKT_TYPE    = 2;
	static constexpr unsigned int BITS_FLAGS       = 3;   // out_of_zone, low_battery, valid
	static constexpr unsigned int BITS_VOLTAGE     = 7;
	static constexpr unsigned int BITS_GPS_COUNT   = 4;   // 0-15 entries
	static constexpr unsigned int BITS_DELTA_TIME  = 4;
	static constexpr unsigned int BITS_DAY         = 5;
	static constexpr unsigned int BITS_HOUR        = 5;
	static constexpr unsigned int BITS_MIN         = 6;
	static constexpr unsigned int BITS_LATITUDE    = 21;
	static constexpr unsigned int BITS_LONGITUDE   = 22;
	static constexpr unsigned int BITS_SPEED       = 7;
	static constexpr unsigned int BITS_HEADING     = 8;
	static constexpr unsigned int BITS_ALTITUDE    = 8;
	static constexpr unsigned int BITS_NUMSV       = 4;
	static constexpr unsigned int BITS_SENSOR_MASK = 6;

	// Sensor field bit widths
	static constexpr unsigned int BITS_ALS         = 17;
	static constexpr unsigned int BITS_PH          = 14;
	static constexpr unsigned int BITS_PRESSURE    = 15;
	static constexpr unsigned int BITS_PRESS_TEMP  = 14;
	static constexpr unsigned int BITS_SEA_TEMP    = 14;
	static constexpr unsigned int BITS_AXL_TEMP    = 14;
	static constexpr unsigned int BITS_AXL_AXIS    = 15;
	static constexpr unsigned int BITS_AXL_ACT     = 8;

	// Composite sizes
	static constexpr unsigned int BITS_HEADER      = BITS_PKT_TYPE + BITS_FLAGS + BITS_VOLTAGE;  // 12
	static constexpr unsigned int BITS_GPS_FULL    = BITS_DAY + BITS_HOUR + BITS_MIN +
	                                                  BITS_LATITUDE + BITS_LONGITUDE +
	                                                  BITS_SPEED + BITS_HEADING +
	                                                  BITS_ALTITUDE + BITS_NUMSV;  // 86
	static constexpr unsigned int BITS_GPS_DELTA   = BITS_LATITUDE + BITS_LONGITUDE + BITS_SPEED;  // 50
	static constexpr unsigned int BITS_AXL_FULL    = BITS_AXL_TEMP + (3 * BITS_AXL_AXIS) + BITS_AXL_ACT;  // 67
	static constexpr unsigned int BITS_PRESSURE_FULL = BITS_PRESSURE + BITS_PRESS_TEMP;  // 29

	// Conversion constants (same as ArgosPacketBuilder)
	static constexpr unsigned int FIXTYPE_3D        = 3;
	static constexpr unsigned int MM_PER_METER      = 1000;
	static constexpr unsigned int MM_PER_KM         = 1000000;
	static constexpr unsigned int MV_PER_UNIT       = 20;
	static constexpr unsigned int METRES_PER_UNIT   = 40;
	static constexpr unsigned int SECONDS_PER_HOUR  = 3600;
	static constexpr unsigned int MIN_ALTITUDE      = 0;
	static constexpr unsigned int MAX_ALTITUDE      = 254;
	static constexpr unsigned int INVALID_ALTITUDE  = 255;
	static constexpr unsigned int REF_BATT_MV       = 2700;
	static constexpr unsigned int LON_LAT_RESOLUTION = 10000;
	static constexpr int NEG_LON_LAT_RESOLUTION     = -10000;
	static constexpr unsigned int BITS_PER_BYTE     = 8;

	// Conversion functions (same formulas as ArgosPacketBuilder)
	static unsigned int convert_altitude(double x);
	static unsigned int convert_heading(double x);
	static unsigned int convert_speed(double x);
	static unsigned int convert_latitude(double x);
	static unsigned int convert_longitude(double x);
	static unsigned int convert_battery_voltage(unsigned int battery_voltage);

	// Compute how many GPS entries fit in a given payload size (bytes)
	static unsigned int max_gps_entries(unsigned int max_payload_bytes);

	// Build methods
	static KineisPacket build_gps_packet(std::vector<GPSLogEntry*>& v,
			bool is_out_of_zone, bool is_low_battery,
			BaseDeltaTimeLoc delta_time_loc,
			unsigned int max_payload_bytes,
			unsigned int& size_bits);

	static KineisPacket build_sensor_packet(GPSLogEntry* gps,
			ServiceSensorData* als, ServiceSensorData* ph,
			ServiceSensorData* pressure, ServiceSensorData* sea_temp,
			ServiceSensorData* axl,
			bool is_out_of_zone, bool is_low_battery,
			unsigned int& size_bits);

	static KineisPacket build_status_packet(unsigned int battery_voltage,
			bool is_low_battery, unsigned int& size_bits);
};


class LoRaTxScheduler {
private:
	std::optional<uint64_t> m_last_schedule_abs;
	std::optional<uint64_t> m_curr_schedule_abs;
	std::optional<uint64_t> m_earliest_schedule;
	std::mt19937 m_rand;

	static constexpr unsigned int MSECS_PER_SECOND = 1000;
	static constexpr unsigned int SECONDS_PER_HOUR = 3600;
	static constexpr unsigned int SECONDS_PER_DAY  = 86400;
	static constexpr unsigned int DUTYCYCLE_24HRS  = 0xFFFFFFU;

	int compute_random_jitter(bool jitter_en, int min = -5000, int max = 5000);
	void schedule_periodic(unsigned int period_ms, bool jitter_en, unsigned int duty_cycle, uint64_t now_ms);

public:
	static constexpr unsigned int INVALID_SCHEDULE = (unsigned int)-1;
	static bool is_in_duty_cycle(uint64_t time_ms, unsigned int duty_cycle);

	LoRaTxScheduler();
	unsigned int schedule_duty_cycle(ArgosConfig& config, std::time_t now);
	unsigned int schedule_legacy(ArgosConfig& config, std::time_t now);
	void set_earliest_schedule(std::time_t t);
	void reset(unsigned int seed);
	void schedule_at(std::time_t t);
	void notify_tx_complete();
};


class LoRaTxService : public Service, KineisEventListener {
public:
	LoRaTxService(KineisDevice& device);
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
	bool m_is_first_tx;
	bool m_is_tx_pending;
	unsigned int m_session_tx_count;
	bool m_last_tx_had_gps;
	std::function<void()> m_scheduled_task;

	void react(KineisEventTxStarted const&) override;
	void react(KineisEventTxComplete const&) override;
	void react(KineisEventDeviceError const&) override;

	void process_gps_burst();
	void process_sensor_burst();
	void process_status_burst();

	unsigned int get_max_payload_bytes();
};
