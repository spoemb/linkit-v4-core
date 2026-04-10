#pragma once

#include <map>
#include <cstdint>
#include <cstring>
#include <functional>
#include "base_types.hpp"
#include "events.hpp"

struct GPSNavSettings {
    BaseGNSSFixMode  fix_mode;
	BaseGNSSDynModel dyn_model;
	bool			 assistnow_autonomous_enable;
	bool             assistnow_offline_enable;
	bool             hdop_filter_en;
	unsigned int     hdop_filter_threshold;
	bool             hacc_filter_en;
    unsigned int     hacc_filter_threshold;
    bool             debug_enable = false;
    bool             sat_tracking = false;
    unsigned int     num_consecutive_fixes = 1;
    unsigned int     max_nav_samples = 0;
    unsigned int     max_sat_samples = 0;
    unsigned int     constellation_mask = 0x0F;  // bit0=GPS, bit1=GAL, bit2=GLO, bit3=BDS, bit4=QZSS, bit5=SBAS
    unsigned int     orbmaxerr = 300;
    unsigned int     min_cno = 10;
    unsigned int     min_elev = 10;
    unsigned int     ano_stale_threshold_s = 25 * 24 * 3600;  // ANO staleness threshold in seconds (default: 25 days)
    bool             cloudlocate_enable = false;  // Enable MEAS message capture for CloudLocate
};

struct GNSSData {
	uint32_t   iTOW;
	uint16_t   year;
	uint8_t    month;
	uint8_t    day;
	uint8_t    hour;
	uint8_t    min;
	uint8_t    sec;
	uint8_t    valid;
	uint32_t   tAcc;
	int32_t    nano;
	uint8_t    fixType;
	uint8_t    flags;
	uint8_t    flags2;
	uint8_t    flags3;
	uint8_t    numSV;
	double     lon;       // Degrees
	double     lat;       // Degrees
	int32_t    height;    // mm
	int32_t    hMSL;      // mm
	uint32_t   hAcc;      // mm
	uint32_t   vAcc;      // mm
	int32_t    velN;      // mm
	int32_t    velE;      // mm
	int32_t    velD;      // mm
	int32_t    gSpeed;    // mm/s
	float      headMot;   // Degrees
	uint32_t   sAcc;      // mm/s
	float      headAcc;   // Degrees
	float      pDOP;
	float      vDOP;
	float      hDOP;
	float      headVeh;   // Degrees
	uint32_t   ttff;      // ms
};

struct GNSSDeviceInfo {
    char swVersion[30];
    char hwVersion[10];
    uint8_t uniqueId[5];
    bool valid;
};

struct GNSSAlmanacStatus {
    bool file_present;
    unsigned int file_size;
    unsigned int total_records;
    unsigned int valid_records;
    bool stale;
};

struct GNSSRawMeasurement {
    uint8_t measc12[12];
    uint8_t meas20[20];
    uint8_t meas50[50];
    bool has_measc12;
    bool has_meas20;
    bool has_meas50;
    GNSSRawMeasurement() : has_measc12(false), has_meas20(false), has_meas50(false) {
        std::memset(measc12, 0, sizeof(measc12));
        std::memset(meas20, 0, sizeof(meas20));
        std::memset(meas50, 0, sizeof(meas50));
    }
};

struct GPSEventMaxNavSamples {};
struct GPSEventMaxSatSamples {};

struct GPSEventSatReport {
	unsigned int numSvs;
	unsigned int bestSignalQuality;
	GPSEventSatReport(unsigned int a, unsigned int b) : numSvs(a), bestSignalQuality(b) {}
};

struct GPSEventError {};
struct GPSEventPowerOn {};
struct GPSEventDeviceInfoReady {};
struct GPSEventPowerOff {
    bool fix_found;
    GPSEventPowerOff(bool a) : fix_found(a) {}
};
struct GPSEventPVT {
    GNSSData& data;
    GPSEventPVT(GNSSData& a) : data(a) {}
};
struct GPSEventPVTDegraded {
    GNSSData& data;
    GPSEventPVTDegraded(GNSSData& a) : data(a) {}
};
struct GPSEventRawMeasurement {
    GNSSRawMeasurement& data;
    GPSEventRawMeasurement(GNSSRawMeasurement& a) : data(a) {}
};

class GPSEventListener {
public:
    virtual ~GPSEventListener() {}
    virtual void react(const GPSEventPowerOn&) {}
    virtual void react(const GPSEventPowerOff&) {}
    virtual void react(const GPSEventError&) {}
    virtual void react(const GPSEventPVT&) {}
    virtual void react(const GPSEventSatReport&) {}
    virtual void react(const GPSEventMaxNavSamples&) {}
    virtual void react(const GPSEventPVTDegraded&) {}
    virtual void react(const GPSEventRawMeasurement&) {}
    virtual void react(const GPSEventMaxSatSamples&) {}
    virtual void react(const GPSEventDeviceInfoReady&) {}
};

class GPSDevice : public EventEmitter<GPSEventListener> {
public:
    using PassthroughCallback = std::function<void(const uint8_t*, size_t)>;

    virtual ~GPSDevice() {}
    // These methods are specific to the chipset and should be implemented by device-specific subclass
    virtual void power_off() = 0;
    virtual void power_on(const GPSNavSettings& nav_settings) = 0;
    virtual GNSSDeviceInfo get_device_info() const { return {}; }
    virtual GNSSAlmanacStatus get_almanac_status(unsigned int ano_stale_threshold_s = 25 * 24 * 3600) const { (void)ano_stale_threshold_s; return {}; }

    // Fastloc: query best degraded PVT accumulated during current acquisition
    virtual bool has_degraded_pvt() const { return false; }
    virtual GNSSData get_degraded_pvt() const { return {}; }

    // CloudLocate: query latest raw GNSS measurement snapshot
    virtual bool has_raw_measurement() const { return false; }
    virtual GNSSRawMeasurement get_raw_measurement() const { return {}; }

    // Bridge/passthrough mode (default: not supported)
    virtual bool start_bridge(PassthroughCallback) { return false; }
    virtual void stop_bridge() {}
    virtual bool is_bridge_active() const { return false; }
    virtual bool bridge_send(const uint8_t*, size_t) { return false; }
    virtual void bridge_process_rx() {}
};
