#pragma once

#include <ctime>
#include <optional>
#include <random>
#include "kineis_device.hpp"
#include "service.hpp"
#include "config_store.hpp"
#include "service_scheduler.hpp"
#include "depth_pile.hpp"


class ArgosPacketBuilder {
public:
	static inline const unsigned int SHORT_PACKET_HEADER		 = 0b000;
	static inline const unsigned int SHORT_PACKET_BITS   		 = 96;
	static inline const unsigned int SHORT_PACKET_PAYLOAD_BITS   = 94;
	static inline const unsigned int SHORT_PACKET_BYTES			 = 12;

	// CRC8 and BCH are handled by the satellite module (SMD/KIM2)
	static inline const unsigned int LONG_PACKET_BITS   		 = 224;
	static inline const unsigned int LONG_PACKET_PAYLOAD_BITS    = 192;
	static inline const unsigned int LONG_PACKET_BYTES			 = 26;  // Buffer for max 4 GPS entries (208 bits)

	static inline const unsigned int SENSOR_PACKET_HEADER 		 = 0b001;
	// Internal buffer allocation: must cover worst-case packing before truncation
	// GPS(75) + ALS(17) + PH(14) + pressure(29) + thermistor(14) + AXL(67) = 216 bits = 27 bytes
	// Non-SMD adds 8-bit CRC field = 224 bits = 28 bytes
	static inline const unsigned int SENSOR_PACKET_BYTES		 = 28;
	// Max transmittable sensor packet size (LDA2 constraint)
	static inline const unsigned int SENSOR_PACKET_MAX_TX_BYTES  = 24;
	static inline const unsigned int SENSOR_PACKET_MAX_TX_BITS   = SENSOR_PACKET_MAX_TX_BYTES * 8;

	static inline const unsigned int DOPPLER_PACKET_BITS   		 = 24;
	static inline const unsigned int DOPPLER_PACKET_PAYLOAD_BITS = 24;
	static inline const unsigned int DOPPLER_PACKET_BYTES		 = 3;

	// RSPB Doppler (VLDA4): header(3) + battery_soc(7) + activity(7) + mortality(7) = 24 bits
	static inline const unsigned int RSPB_DOPPLER_HEADER         = 0b110;  // Type 6
	static inline const unsigned int RSPB_DOPPLER_PACKET_BITS    = 24;
	static inline const unsigned int RSPB_DOPPLER_PACKET_BYTES   = 3;

	static inline const unsigned int FIXTYPE_3D			 = 3;
	static inline const unsigned int HOURS_PER_DAY       = 24;
	static inline const unsigned int SECONDS_PER_MINUTE	 = 60;
	static inline const unsigned int SECONDS_PER_HOUR    = 3600;
	static inline const unsigned int SECONDS_PER_DAY     = (SECONDS_PER_HOUR * HOURS_PER_DAY);
	static inline const unsigned int MM_PER_METER		 = 1000;
	static inline const unsigned int MM_PER_KM   	  	 = 1000000;
	static inline const unsigned int MV_PER_UNIT		 = 20;
	static inline const unsigned int MS_PER_SEC			 = 1000;
	static inline const unsigned int METRES_PER_UNIT     = 40;
	static inline const unsigned int DEGREES_PER_UNIT	 = (1.0f/1.42f);
	static inline const unsigned int BITS_PER_BYTE		 = 8;
	static inline const unsigned int MIN_ALTITUDE		 = 0;
	static inline const unsigned int MAX_ALTITUDE		 = 254;
	static inline const unsigned int INVALID_ALTITUDE	 = 255;
	static inline const unsigned int REF_BATT_MV	 	 = 2700;
	static inline const unsigned int LON_LAT_RESOLUTION  = 10000;
	static inline const int NEG_LON_LAT_RESOLUTION  = -10000;
	static inline const unsigned int MAX_GPS_ENTRIES_IN_PACKET = 4;

	static unsigned int convert_altitude(double x);
	static unsigned int convert_heading(double x);
	static unsigned int convert_speed(double x);
	static unsigned int convert_latitude(double x);
	static unsigned int convert_longitude(double x);
	static unsigned int convert_battery_voltage(unsigned int battery_voltage);
	static KineisPacket build_short_packet(GPSLogEntry* v,
			bool is_out_of_zone,
			bool is_low_battery);
	static KineisPacket build_long_packet(std::vector<GPSLogEntry*> &v,
			bool is_out_of_zone,
			bool is_low_battery,
			BaseDeltaTimeLoc delta_time_loc);
	static KineisPacket build_gnss_packet(std::vector<GPSLogEntry*> &v,
			bool is_out_of_zone,
			bool is_low_battery,
			BaseDeltaTimeLoc delta_time_loc,
			unsigned int &size_bits);
	static KineisPacket build_certification_packet(std::string cert_tx_payload, unsigned int &size_bits);
	static KineisPacket build_doppler_packet(unsigned int battery, bool is_low_battery, unsigned int &size_bits);
	static KineisPacket build_sensor_packet(GPSLogEntry* v,
			ServiceSensorData *als_sensor,
			ServiceSensorData *ph_sensor,
			ServiceSensorData *pressure_sensor,
			ServiceSensorData *sea_temp_sensor,
			ServiceSensorData *axl_sensor,
			bool is_out_of_zone, bool is_low_battery,
			unsigned int& size_bits
			);

	// RSPB dedicated packet formats (bird tracker with mortality detection)
	// RSPB Long (LDA2): header(3) + time(16) + GPS(51) + battery(8) + pressure(29) + thermistor(14) + AXL_XYZ(45) + activity(8) + mortality(7) = 181 bits
	// RSPB Short (LDK): header(3) + time(16) + GPS(51) + battery(8) + pressure(15) + thermistor(14) + activity(8) + mortality(7) = 122 bits
	static inline const unsigned int RSPB_LONG_HEADER            = 0b100;  // Type 4
	static inline const unsigned int RSPB_SHORT_HEADER           = 0b101;  // Type 5
	static inline const unsigned int RSPB_LONG_PACKET_BITS       = 181;
	static inline const unsigned int RSPB_LONG_PACKET_BYTES      = 23;
	static inline const unsigned int RSPB_SHORT_PACKET_BITS      = 122;
	static inline const unsigned int RSPB_SHORT_PACKET_BYTES     = 16;

	static KineisPacket build_rspb_long_packet(GPSLogEntry* gps,
			ServiceSensorData *pressure_sensor,
			ServiceSensorData *thermistor_sensor,
			ServiceSensorData *axl_sensor,
			bool is_out_of_zone, bool is_low_battery,
			unsigned int mortality_confidence,
			unsigned int &size_bits);

	static KineisPacket build_rspb_short_packet(GPSLogEntry* gps,
			ServiceSensorData *pressure_sensor,
			ServiceSensorData *thermistor_sensor,
			ServiceSensorData *axl_sensor,
			bool is_out_of_zone, bool is_low_battery,
			unsigned int mortality_confidence,
			unsigned int &size_bits);

	// RSPB Doppler: battery SOC + activity + mortality in 24 bits (VLDA4)
	static KineisPacket build_rspb_doppler_packet(
			unsigned int battery_soc,
			unsigned int activity,
			unsigned int mortality_confidence,
			unsigned int &size_bits);
};

class ArgosTxScheduler {
private:
	struct Location {
		double longitude;
		double latitude;
		Location(double x, double y) {
			longitude = x;
			latitude = y;
		}
	};
	std::optional<uint64_t> m_last_schedule_abs;
	std::optional<uint64_t> m_curr_schedule_abs;
	std::optional<uint64_t> m_earliest_schedule;
	std::mt19937 m_rand;
	std::optional<Location> m_location;

	static inline const unsigned int SECONDS_PER_MINUTE = 60;
	static inline const unsigned int MINUTES_PER_HOUR   = 60;
	static inline const unsigned int HOURS_PER_DAY      = 24;
	static inline const unsigned int MSECS_PER_SECOND   = 1000;
	static inline const unsigned int SECONDS_PER_HOUR   = MINUTES_PER_HOUR * SECONDS_PER_MINUTE;
	static inline const unsigned int SECONDS_PER_DAY    = HOURS_PER_DAY * SECONDS_PER_HOUR;
	static inline const unsigned int DUTYCYCLE_24HRS    = 0xFFFFFFU;
	static inline const unsigned int ARGOS_TX_MARGIN_MSECS = 0;

	int compute_random_jitter(bool jitter_en, int min = -5000, int max = 5000);
	void schedule_periodic(unsigned int period_ms, bool jitter_en, unsigned int duty_cycle, uint64_t now_ms);

public:
	static inline const unsigned int INVALID_SCHEDULE   = (unsigned int)-1;
	static bool is_in_duty_cycle(uint64_t time_ms, unsigned int duty_cycle);

	ArgosTxScheduler();
	unsigned int schedule_duty_cycle(ArgosConfig& config, std::time_t now);
	unsigned int schedule_legacy(ArgosConfig& config, std::time_t now);
	void set_earliest_schedule(std::time_t t);
	void set_last_location(double lon, double lat);
	unsigned int get_last_schedule();
	void reset(unsigned int seed);
	void schedule_at(std::time_t t);
	void notify_tx_complete();
};


class ArgosTxService : public Service, KineisEventListener {
public:
	ArgosTxService(KineisDevice& device);
	void notify_peer_event(ServiceEvent& e) override;

protected:
	// Service interface methods
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
	bool m_is_first_tx;
	bool m_is_tx_pending;
	bool m_tcxo_skip_on_next_tx;
	unsigned int m_session_tx_count;
	std::function<void()> m_scheduled_task;
	KineisModulation m_scheduled_mode;

	// Surfacing burst state
	bool m_is_surfacing_burst;
	unsigned int m_doppler_burst_count;
	unsigned int m_gnss_burst_count;
	bool m_has_gnss_fix_since_surfacing;
	bool m_last_tx_had_gps;

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
	KineisModulation m_last_preconfig_mod;
	std::optional<KineisModulation> m_modulation_preconfig;  // Cached target for deferred switch
};
