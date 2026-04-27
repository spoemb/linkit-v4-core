/**
 * @file argos_packet_builder.hpp
 * @brief Argos packet builder — constructs short, long, Doppler, sensor, RSPB, CloudLocate packets.
 *
 * All methods are static — no state, no dependencies on Service framework.
 * Used by ArgosTxService to build packets before sending via KineisDevice.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "messages.hpp"
#include "kineis_device.hpp"
#include "config_store.hpp"
#include "service.hpp"

/// @brief Argos packet encoding constants and static builders.
class ArgosPacketBuilder {
public:
	// === Packet format constants ===

	// LDA2 frame layout (applies to long, sensor, fastloc, RSPB long, CloudLocate MEAS20):
	// 24 bytes total = 23 bytes data (184 bits) + 1 byte CRC8 at byte 23.
	// LDK and VLDA4 frames have CRC integrated by the SMD/KIM2 module — only LDA2
	// requires the firmware to compute and embed CRC8 inside the payload.
	static constexpr unsigned int LDA2_FRAME_BITS              = 192;
	static constexpr unsigned int LDA2_FRAME_BYTES             = 24;
	static constexpr unsigned int LDA2_DATA_BITS               = 184;  ///< Bits before CRC8

	// Short packet (LDK, modem-handled CRC)
	static constexpr unsigned int SHORT_PACKET_HEADER      = 0b000;
	static constexpr unsigned int SHORT_PACKET_BITS        = 96;
	static constexpr unsigned int SHORT_PACKET_PAYLOAD_BITS = 94;
	static constexpr unsigned int SHORT_PACKET_BYTES       = 12;

	// Long packet (LDA2, firmware-embedded CRC8 at byte 23)
	static constexpr unsigned int LONG_PACKET_BITS         = LDA2_FRAME_BITS;
	static constexpr unsigned int LONG_PACKET_PAYLOAD_BITS = LDA2_DATA_BITS;
	static constexpr unsigned int LONG_PACKET_BYTES        = LDA2_FRAME_BYTES;

	// Fastloc degraded PVT packet (LDA2, firmware-embedded CRC8 at byte 23)
	static constexpr unsigned int FASTLOC_PACKET_HEADER    = 0b010;
	static constexpr unsigned int FASTLOC_PACKET_BITS      = LDA2_FRAME_BITS;
	static constexpr unsigned int FASTLOC_PACKET_BYTES     = LDA2_FRAME_BYTES;

	// CloudLocate packet (Type 7): header(3) + format(2) + blob(variable) + battery(8)
	// MEAS20 variant goes on LDA2 with firmware CRC8; MEASC12 stays on LDK (modem CRC).
	static constexpr unsigned int CLOUDLOCATE_PACKET_HEADER    = 0b111;
	static constexpr unsigned int CLOUDLOCATE_MEASC12_BITS     = 128;   // LDK
	static constexpr unsigned int CLOUDLOCATE_MEASC12_BYTES    = 16;
	static constexpr unsigned int CLOUDLOCATE_MEAS20_BITS      = LDA2_FRAME_BITS;
	static constexpr unsigned int CLOUDLOCATE_MEAS20_BYTES     = LDA2_FRAME_BYTES;

	// Sensor packet (Type 1, LDA2). Always emitted as a full 24-byte LDA2 frame so the
	// CRC sits at byte 23. Adaptive LDK fallback for tiny sensor packets is therefore
	// disabled — sensor packets always go on LDA2 now.
	static constexpr unsigned int SENSOR_PACKET_HEADER         = 0b001;
	static constexpr unsigned int SENSOR_PACKET_BYTES          = LDA2_FRAME_BYTES;
	static constexpr unsigned int SENSOR_PACKET_MAX_TX_BYTES   = LDA2_FRAME_BYTES;
	static constexpr unsigned int SENSOR_PACKET_MAX_TX_BITS    = LDA2_DATA_BITS;

	// Doppler packet (24 bits, VLDA4)
	static constexpr unsigned int DOPPLER_PACKET_BITS          = 24;
	static constexpr unsigned int DOPPLER_PACKET_PAYLOAD_BITS  = 24;
	static constexpr unsigned int DOPPLER_PACKET_BYTES         = 3;

	// RSPB Doppler (Type 6, VLDA4)
	static constexpr unsigned int RSPB_DOPPLER_HEADER          = 0b110;
	static constexpr unsigned int RSPB_DOPPLER_PACKET_BITS     = 24;
	static constexpr unsigned int RSPB_DOPPLER_PACKET_BYTES    = 3;

	// RSPB dedicated packet formats (bird tracker with mortality)
	static constexpr unsigned int RSPB_LONG_HEADER             = 0b100;
	static constexpr unsigned int RSPB_SHORT_HEADER            = 0b101;
	static constexpr unsigned int RSPB_LONG_PACKET_DATA_BITS   = 181;             ///< Useful bits before CRC
	static constexpr unsigned int RSPB_LONG_PACKET_BITS        = LDA2_FRAME_BITS; ///< Full LDA2 frame (data + CRC)
	static constexpr unsigned int RSPB_LONG_PACKET_BYTES       = LDA2_FRAME_BYTES;
	static constexpr unsigned int RSPB_SHORT_PACKET_BITS       = 122;
	static constexpr unsigned int RSPB_SHORT_PACKET_BYTES      = 16;

	// === Encoding constants ===

	static constexpr unsigned int FIXTYPE_3D           = 3;
	static constexpr unsigned int MM_PER_METER         = 1000;
	static constexpr unsigned int MM_PER_KM            = 1000000;
	static constexpr unsigned int MV_PER_UNIT          = 20;
	static constexpr unsigned int METRES_PER_UNIT      = 40;
	static constexpr double       DEGREES_PER_UNIT     = 1.0 / 1.42;  ///< ~0.704 deg/unit
	static constexpr unsigned int BITS_PER_BYTE        = 8;
	static constexpr unsigned int MIN_ALTITUDE         = 0;
	static constexpr unsigned int MAX_ALTITUDE         = 254;
	static constexpr unsigned int INVALID_ALTITUDE     = 255;
	static constexpr unsigned int REF_BATT_MV          = 2700;
	static constexpr unsigned int LON_LAT_RESOLUTION   = 10000;
	static constexpr int          NEG_LON_LAT_RESOLUTION = -10000;
	static constexpr unsigned int MAX_GPS_ENTRIES_IN_PACKET = 3;  ///< Reduced from 4 to fit CRC8 in 24-byte LDA2 frame
	static constexpr unsigned int SECONDS_PER_HOUR     = 3600;

	// === Conversion helpers ===

	static unsigned int convert_altitude(double x);
	static unsigned int convert_heading(double x);
	static unsigned int convert_speed(double x);
	static unsigned int convert_latitude(double x);
	static unsigned int convert_longitude(double x);
	static unsigned int convert_battery_voltage(unsigned int battery_voltage);

	// === Packet builders ===

	/// @brief Build short GPS packet (1 fix, 96 bits).
	static KineisPacket build_short_packet(GPSLogEntry* v,
			bool is_out_of_zone, bool is_low_battery);

	/// @brief Build long GPS packet (up to 4 fixes, 224 bits).
	static KineisPacket build_long_packet(std::vector<GPSLogEntry*> &v,
			bool is_out_of_zone, bool is_low_battery,
			BaseDeltaTimeLoc delta_time_loc);

	/// @brief Build fastloc degraded PVT packet (192 bits).
	static KineisPacket build_fastloc_packet(GPSLogEntry* v, bool is_low_battery);

	/// @brief Build CloudLocate raw measurement packet.
	static KineisPacket build_cloudlocate_packet(const uint8_t* blob, unsigned int blob_size,
			uint8_t format_id, unsigned int battery_voltage, bool is_low_battery);

	/// @brief Get CloudLocate packet size in bits for a given format.
	static unsigned int cloudlocate_packet_bits(uint8_t format_id);

	/// @brief Build GNSS packet (auto-selects short or long based on entry count).
	static KineisPacket build_gnss_packet(std::vector<GPSLogEntry*> &v,
			bool is_out_of_zone, bool is_low_battery,
			BaseDeltaTimeLoc delta_time_loc, unsigned int &size_bits);

	/// @brief Build certification TX packet from hex payload string.
	static KineisPacket build_certification_packet(std::string cert_tx_payload, unsigned int &size_bits);

	/// @brief Build Doppler packet (24 bits, no GPS).
	static KineisPacket build_doppler_packet(unsigned int battery, bool is_low_battery, unsigned int &size_bits);

	/// @brief Build sensor packet (GPS + optional ALS/PH/pressure/temp/AXL).
	static KineisPacket build_sensor_packet(GPSLogEntry* v,
			ServiceSensorData *als_sensor,
			ServiceSensorData *ph_sensor,
			ServiceSensorData *pressure_sensor,
			ServiceSensorData *sea_temp_sensor,
			ServiceSensorData *axl_sensor,
			bool is_out_of_zone, bool is_low_battery,
			unsigned int& size_bits);

	/// @brief Build RSPB long packet (LDA2, bird tracker with mortality).
	static KineisPacket build_rspb_long_packet(GPSLogEntry* gps,
			ServiceSensorData *pressure_sensor,
			ServiceSensorData *thermistor_sensor,
			ServiceSensorData *axl_sensor,
			bool is_out_of_zone, bool is_low_battery,
			unsigned int mortality_confidence,
			unsigned int &size_bits);

	/// @brief Build RSPB short packet (LDK, bird tracker with mortality).
	static KineisPacket build_rspb_short_packet(GPSLogEntry* gps,
			ServiceSensorData *pressure_sensor,
			ServiceSensorData *thermistor_sensor,
			ServiceSensorData *axl_sensor,
			bool is_out_of_zone, bool is_low_battery,
			unsigned int mortality_confidence,
			unsigned int &size_bits);

	/// @brief Build RSPB Doppler packet (VLDA4, mortality + activity).
	static KineisPacket build_rspb_doppler_packet(
			unsigned int battery_soc,
			unsigned int activity,
			unsigned int mortality_confidence,
			unsigned int &size_bits);
};
