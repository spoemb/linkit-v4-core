#pragma once

#include <string>
#include <vector>
#include <variant>
#include <ctime>

extern "C" {
	#include "previpass.h"
}

// Default sensor enable macros if not defined by build system
#ifndef ENABLE_CAM_SENSOR
#define ENABLE_CAM_SENSOR 0
#endif

#ifndef ENABLE_PRESSURE_SENSOR
#define ENABLE_PRESSURE_SENSOR 0
#endif

#ifndef ENABLE_ALS_SENSOR
#define ENABLE_ALS_SENSOR 0
#endif

#ifndef ENABLE_PH_SENSOR
#define ENABLE_PH_SENSOR 0
#endif

#ifndef ENABLE_SEA_TEMP_SENSOR
#define ENABLE_SEA_TEMP_SENSOR 0
#endif

#ifndef ENABLE_CDT_SENSOR
#define ENABLE_CDT_SENSOR 0
#endif

#ifndef ENABLE_THERMISTOR_SENSOR
#define ENABLE_THERMISTOR_SENSOR 0
#endif

#ifndef ENABLE_AXL_SENSOR
#define ENABLE_AXL_SENSOR 1
#endif

#ifndef ENABLE_SWS_LOG
#define ENABLE_SWS_LOG 0
#endif

#ifndef LORA_RAK3172
#define LORA_RAK3172 0
#endif

#define BASE_TEXT_MAX_LENGTH  128
#define BASE_MAX_PAYLOAD_LENGTH 0xFFF
#define KEY_LENGTH            5

// Offset applied to Argos frequency parameter supplied over DTE interface
#define ARGOS_FREQUENCY_OFFSET	4016200U
#define ARGOS_FREQUENCY_MULT	10000U

enum class ParamID {
	// === Core params (always present) ===
	ARGOS_DECID                              = 0,
	ARGOS_HEXID                              = 1,
	DEVICE_MODEL                             = 2,
	FW_APP_VERSION                           = 3,
	LAST_TX                                  = 4,
	TX_COUNTER                               = 5,
	BATT_SOC                                 = 6,
	LAST_FULL_CHARGE_DATE                    = 7,
	PROFILE_NAME                             = 8,
	_RESERVED_9                              = 9,  // Was AOP_STATUS — status is in PASPW allcast data
	ARGOS_AOP_DATE                           = 10,
	ARGOS_FREQ                               = 11,
	ARGOS_POWER                              = 12,
	TR_NOM                                   = 13,
	ARGOS_MODE                               = 14,
	NTRY_PER_MESSAGE                         = 15,
	DUTY_CYCLE                               = 16,
	GNSS_EN                                  = 17,
	DLOC_ARG_NOM                             = 18,
	ARGOS_DEPTH_PILE                         = 19,
	_RESERVED_20                             = 20, // Was GPS_CONST_SELECT — never implemented
	GLONASS_CONST_SELECT                     = 21,
	GNSS_HDOPFILT_EN                         = 22,
	GNSS_HDOPFILT_THR                        = 23,
	GNSS_ACQ_TIMEOUT                         = 24,
	GNSS_NTRY                                = 25,
	UNDERWATER_EN                            = 26,
	DRY_TIME_BEFORE_TX                       = 27,
	SAMPLING_UNDER_FREQ                      = 28,
	LB_EN                                    = 29,
	LB_THRESHOLD                              = 30,
	LB_ARGOS_POWER                           = 31,
	TR_LB                                    = 32,
	LB_ARGOS_MODE                            = 33,
	LB_ARGOS_DUTY_CYCLE                      = 34,
	LB_GNSS_EN                               = 35,
	DLOC_ARG_LB                              = 36,
	LB_GNSS_HDOPFILT_THR                     = 37,
	LB_ARGOS_DEPTH_PILE                      = 38,
	LB_GNSS_ACQ_TIMEOUT                      = 39,
	SAMPLING_SURF_FREQ                       = 40,
	PP_MIN_ELEVATION                         = 41,
	PP_MAX_ELEVATION                         = 42,
	PP_MIN_DURATION                          = 43,
	PP_MAX_PASSES                            = 44,
	PP_LINEAR_MARGIN                         = 45,
	PP_COMP_STEP                             = 46,
	GNSS_COLD_ACQ_TIMEOUT                    = 47,
	GNSS_FIX_MODE                            = 48,
	GNSS_DYN_MODEL                           = 49,
	GNSS_HACCFILT_EN                         = 50,
	GNSS_HACCFILT_THR                        = 51,
	GNSS_MIN_NUM_FIXES                       = 52,
	GNSS_COLD_START_RETRY_PERIOD             = 53,
	ARGOS_TIME_SYNC_BURST_EN                 = 54,
	LED_MODE                                 = 55,
	ARGOS_TX_JITTER_EN                       = 56,
	ARGOS_RX_EN                              = 57,
	ARGOS_RX_MAX_WINDOW                      = 58,
	ARGOS_RX_AOP_UPDATE_PERIOD               = 59,
	ARGOS_RX_COUNTER                         = 60,
	ARGOS_RX_TIME                            = 61,
	GNSS_ASSISTNOW_EN                        = 62,
	LB_GNSS_HACCFILT_THR                     = 63,
	LB_NTRY_PER_MESSAGE                      = 64,
	ZONE_TYPE                                = 65,
	ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE   = 66,
	ZONE_ENABLE_ACTIVATION_DATE              = 67,
	ZONE_ACTIVATION_DATE                     = 68,
	ZONE_ARGOS_DEPTH_PILE                    = 69,
	_RESERVED_70                             = 70,
	ZONE_ARGOS_REPETITION_SECONDS            = 71,
	ZONE_ARGOS_MODE                          = 72,
	ZONE_ARGOS_DUTY_CYCLE                    = 73,
	ZONE_ARGOS_NTRY_PER_MESSAGE              = 74,
	ZONE_GNSS_DELTA_ARG_LOC_ARGOS_SECONDS    = 75,
	ZONE_GNSS_HDOPFILT_THR                   = 76,
	ZONE_GNSS_HACCFILT_THR                   = 77,
	ZONE_GNSS_ACQ_TIMEOUT                    = 78,
	ZONE_CENTER_LONGITUDE                    = 79,
	ZONE_CENTER_LATITUDE                     = 80,
	ZONE_RADIUS                              = 81,
	CERT_TX_ENABLE                           = 82,
	CERT_TX_PAYLOAD                          = 83,
	CERT_TX_MODULATION                       = 84,
	CERT_TX_REPETITION                       = 85,
	HW_VERSION                               = 86,
	BATT_VOLTAGE                             = 87,
	// === External wakeup / TPL5111 (slots 88-91 always reserved) ===
#ifdef EXTERNAL_WAKEUP
	SHUTDOWN_TIMER                           = 88,
	BOOT_COUNTER                             = 89,
	BOOT_COUNTER_MODULO                      = 90,
	WAKEUP_PERIOD                            = 91,
#endif
	// === Post-wakeup params ===
	ARGOS_TCXO_WARMUP_TIME                   = 92,
	DEVICE_DECID                             = 93,
	GNSS_TRIGGER_ON_SURFACED                 = 94,
	GNSS_TRIGGER_ON_AXL_WAKEUP              = 95,
	UNDERWATER_DETECT_SOURCE                 = 96,
	UNDERWATER_DETECT_THRESH                 = 97,
	// === pH sensor (slots 98-100 always reserved) ===
#if ENABLE_PH_SENSOR
	PH_SENSOR_ENABLE                         = 98,
	PH_SENSOR_PERIODIC                       = 99,
	PH_SENSOR_VALUE                          = 100,
#endif
	// === Sea temperature sensor (slots 101-103 always reserved) ===
#if ENABLE_SEA_TEMP_SENSOR
	SEA_TEMP_SENSOR_ENABLE                   = 101,
	SEA_TEMP_SENSOR_PERIODIC                 = 102,
	SEA_TEMP_SENSOR_VALUE                    = 103,
#endif
	// === ALS sensor (slots 104-106 always reserved) ===
#if ENABLE_ALS_SENSOR
	ALS_SENSOR_ENABLE                        = 104,
	ALS_SENSOR_PERIODIC                      = 105,
	ALS_SENSOR_VALUE                         = 106,
#endif
	// === CDT sensor (slots 107-111 always reserved) ===
#if ENABLE_CDT_SENSOR
	CDT_SENSOR_ENABLE                        = 107,
	CDT_SENSOR_PERIODIC                      = 108,
	CDT_SENSOR_CONDUCTIVITY_VALUE            = 109,
	CDT_SENSOR_DEPTH_VALUE                   = 110,
	CDT_SENSOR_TEMPERATURE_VALUE             = 111,
#endif
	// === Thermistor sensor (slots 112-116 always reserved) ===
#if ENABLE_THERMISTOR_SENSOR
	THERMISTOR_SENSOR_ENABLE                 = 112,
	THERMISTOR_SENSOR_PERIODIC               = 113,
	THERMISTOR_SENSOR_VALUE                  = 114,
	THERMISTOR_SENSOR_WAKEUP_THRESH          = 115,
	THERMISTOR_SENSOR_WAKEUP_SAMPLES         = 116,
#endif
	EXT_LED_MODE                             = 117,
	// === Accelerometer sensor (slots 118-123 always reserved) ===
#if ENABLE_AXL_SENSOR
	AXL_SENSOR_ENABLE                        = 118,
	AXL_SENSOR_PERIODIC                      = 119,
	AXL_SENSOR_WAKEUP_THRESH                = 120,
	AXL_SENSOR_WAKEUP_SAMPLES               = 121,
	AXL_SENSOR_MEASUREMENT_RANGE             = 122,
	AXL_SENSOR_POWER_MODE                    = 123,
#endif
	// === Pressure sensor (slots 124-125 always reserved) ===
#if ENABLE_PRESSURE_SENSOR
	PRESSURE_SENSOR_ENABLE                   = 124,
	PRESSURE_SENSOR_PERIODIC                 = 125,
#endif
	// === Misc params ===
	DEBUG_OUTPUT_MODE                        = 126,
	GNSS_ASSISTNOW_OFFLINE_EN               = 127,
	UW_MAX_SAMPLES                           = 128,
	UW_MIN_DRY_SAMPLES                      = 129,
	UW_SAMPLE_GAP                            = 130,
	UW_PIN_SAMPLE_DELAY                      = 131,
	SWS_ANALOG_THRESHOLD_MIN                 = 132,
	SWS_ANALOG_THRESHOLD_MAX                 = 133,
	SWS_ANALOG_HYSTERESIS                    = 134,
	SWS_ANALOG_CALIB_INTERVAL                = 135,
	UW_MAX_DIVE_TIME                         = 136,
	UW_MIN_SURFACE_TIME                      = 137,
	UW_DIVE_MODE_ENABLE                      = 138,
	UW_DIVE_MODE_START_TIME                  = 139,
	UW_GNSS_DRY_SAMPLING                     = 140,
	UW_GNSS_WET_SAMPLING                     = 141,
	UW_GNSS_MAX_SAMPLES                      = 142,
	UW_GNSS_MIN_DRY_SAMPLES                 = 143,
	UW_GNSS_DETECT_THRESH                    = 144,
	LB_CRITICAL_THRESH                       = 145,
	// === Pressure sensor logging (slot 146 always reserved) ===
#if ENABLE_PRESSURE_SENSOR
	PRESSURE_SENSOR_LOGGING_MODE             = 146,
#endif
	GNSS_TRIGGER_COLD_START_ON_SURFACED      = 147,
	// === Sensor TX mode params (slots always reserved) ===
#if ENABLE_SEA_TEMP_SENSOR
	SEA_TEMP_SENSOR_ENABLE_TX_MODE           = 148,
	SEA_TEMP_SENSOR_ENABLE_TX_MAX_SAMPLES    = 149,
	SEA_TEMP_SENSOR_ENABLE_TX_SAMPLE_PERIOD  = 150,
#endif
#if ENABLE_PH_SENSOR
	PH_SENSOR_ENABLE_TX_MODE                 = 151,
	PH_SENSOR_ENABLE_TX_MAX_SAMPLES          = 152,
	PH_SENSOR_ENABLE_TX_SAMPLE_PERIOD        = 153,
#endif
#if ENABLE_ALS_SENSOR
	ALS_SENSOR_ENABLE_TX_MODE                = 154,
	ALS_SENSOR_ENABLE_TX_MAX_SAMPLES         = 155,
	ALS_SENSOR_ENABLE_TX_SAMPLE_PERIOD       = 156,
#endif
#if ENABLE_PRESSURE_SENSOR
	PRESSURE_SENSOR_ENABLE_TX_MODE           = 157,
	PRESSURE_SENSOR_ENABLE_TX_MAX_SAMPLES    = 158,
	PRESSURE_SENSOR_ENABLE_TX_SAMPLE_PERIOD  = 159,
#endif
#if ENABLE_AXL_SENSOR
	AXL_SENSOR_ENABLE_TX_MODE                = 160,
	AXL_SENSOR_ENABLE_TX_MAX_SAMPLES         = 161,
	AXL_SENSOR_ENABLE_TX_SAMPLE_PERIOD       = 162,
#endif
#if ENABLE_THERMISTOR_SENSOR
	THERMISTOR_SENSOR_ENABLE_TX_MODE         = 163,
	THERMISTOR_SENSOR_ENABLE_TX_MAX_SAMPLES  = 164,
	THERMISTOR_SENSOR_ENABLE_TX_SAMPLE_PERIOD = 165,
#endif
	// === Camera sensor (slots 166-171 always reserved) ===
#if ENABLE_CAM_SENSOR
	CAM_ENABLE                               = 166,
	CAM_TRIGGER_ON_SURFACED                  = 167,
	CAM_TRIGGER_ON_AXL_WAKEUP               = 168,
	CAM_PERIOD_ON                            = 169,
	CAM_PERIOD_OFF                           = 170,
	LB_CAM_EN                               = 171,
#endif
	// === Satellite credentials (slots 172-173 always reserved) ===
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	ARGOS_SECKEY                             = 172,
#endif
	ARGOS_RADIOCONF                          = 173,
	// === Session shutdown control (slots 174-176 always reserved) ===
	SHUTDOWN_NTIME_SAT                       = 174,
	LB_SHUTDOWN_NTIME_SAT                    = 175,
	GNSS_SESSION_SINGLE_FIX                  = 176,
#if ENABLE_PRESSURE_SENSOR
	PRESSURE_SENSOR_FULL_SCALE               = 177,
#endif
	GNSS_TOKEN                               = 178,
	// === Pseudo RTC (slot 179 always reserved) ===
#ifdef EXTERNAL_WAKEUP
	LAST_KNOWN_RTC                           = 179,
#endif
	// === System status (slot 180 always reserved) ===
	RTC_CURRENT_TIME                         = 180,
	// === LoRa RAK3172 parameters (slots 181-194 always reserved) ===
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	LORA_DEVEUI                              = 181,
	LORA_APPEUI                              = 182,
	LORA_APPKEY                              = 183,
	LORA_DEVADDR                             = 184,
	LORA_APPSKEY                             = 185,
	LORA_NWKSKEY                             = 186,
	LORA_NJM                                 = 187,
	LORA_BAND                                = 188,
	LORA_CLASS                               = 189,
	LORA_DR                                  = 190,
	LORA_ADR                                 = 191,
	LORA_TXP                                 = 192,
	LORA_CFM                                 = 193,
	LORA_FPORT                               = 194,
	LORA_LP_MODE                             = 195,  // 0=shutdown (0µA), 1=standby (~1.7µA, fast wake)
#endif
	// === Surfacing burst parameters (slots 196-198 always reserved) ===
	SURFACING_BURST_INIT_S                   = 196,
	SURFACING_BURST_STEP_S                   = 197,
	SURFACING_BURST_MAX_S                    = 198,
	// === Sentinel (fixed regardless of #ifdef combinations) ===
	__PARAM_SIZE                             = 199,
	__NULL_PARAM                             = 0xFFFF
};

enum class BaseEncoding {
	DECIMAL,
	HEXADECIMAL,
	TEXT,
	DATESTRING,
	BASE64,
	BOOLEAN,
	UINT,
	FLOAT,
	DEPTHPILE,
	ARGOSMODE,
	ARGOSPOWER,
	AQPERIOD,
	ARGOSFREQ,
	GNSSFIXMODE,
	GNSSDYNMODEL,
	LEDMODE,
	ZONETYPE,
	MODULATION,
	UWDETECTSOURCE,
	DEBUGMODE,
	PRESSURESENSORLOGGINGMODE,
	PRESSURESENSORFULLSCALE,
	SENSORENABLETXMODE,
	KEY_LIST,
	KEY_VALUE_LIST
};

enum class BasePressureSensorLoggingMode {
	ALWAYS = 0,
	UW_THRESHOLD,
};

enum class BasePressureSensorFullScale {
	FS_1260 = 0,
	FS_4060,
};

enum class BaseUnderwaterDetectSource {
	SWS = 0,
	PRESSURE_SENSOR,
	GNSS,
	SWS_GNSS
};

enum class BaseLogDType {
	INTERNAL          = 0,
	GNSS_SENSOR       = 1,
	ALS_SENSOR        = 2,
	PH_SENSOR         = 3,
	RTD_SENSOR        = 4,
	CDT_SENSOR        = 5,
	CAM_SENSOR        = 6,
	AXL_SENSOR        = 7,
	PRESSURE_SENSOR   = 8,
	THERMISTOR_SENSOR = 9,
	TSYS01_SENSOR     = 10,
	SWS_LOG           = 11
};

enum class BaseEraseType {
	GNSS_SENSOR       = 1,
	SYSTEM            = 2,
	ALL               = 3,
	ALS_SENSOR        = 4,
	PH_SENSOR         = 5,
	RTD_SENSOR        = 6,
	CDT_SENSOR        = 7,
	CAM_SENSOR        = 8,
	AXL_SENSOR        = 9,
	PRESSURE_SENSOR   = 10,
	THERMISTOR_SENSOR = 11,
	TSYS01_SENSOR     = 12,
	SWS_LOG           = 13
};

enum class BaseSensorCalType {
	AXL        = 0,
	PRESSURE   = 1,
	ALS        = 2,
	PH         = 3,
	RTD        = 4,
	CDT        = 5,
	MCP47X6    = 6,
	THERMISTOR = 7
};

enum class BaseArgosMode {
	OFF,
	PASS_PREDICTION,
	LEGACY,
	DUTY_CYCLE,
	DOPPLER,
	SURFACING_BURST
};

enum class BaseArgosPower {
	POWER_3_MW = 1,
	POWER_40_MW,
	POWER_200_MW,
	POWER_500_MW,
	POWER_5_MW,
	POWER_50_MW,
	POWER_350_MW,
	POWER_750_MW,
	POWER_1000_MW,
	POWER_1500_MW
};

inline const char *argos_power_to_string(BaseArgosPower power) {
	if (power == BaseArgosPower::POWER_3_MW)
		return "3 mW";
	if (power == BaseArgosPower::POWER_40_MW)
		return "40 mW";
	if (power == BaseArgosPower::POWER_200_MW)
		return "200 mW";
	if (power == BaseArgosPower::POWER_500_MW)
		return "500 mW";
	if (power == BaseArgosPower::POWER_5_MW)
		return "5 mW";
	if (power == BaseArgosPower::POWER_50_MW)
		return "50 mW";
	if (power == BaseArgosPower::POWER_350_MW)
		return "350 mW";
	if (power == BaseArgosPower::POWER_750_MW)
		return "750 mW";
	if (power == BaseArgosPower::POWER_1000_MW)
		return "1000 mW";
	if (power == BaseArgosPower::POWER_1500_MW)
		return "1500 mW";
	return "UNKNOWN";
}


inline BaseArgosPower argos_integer_to_power(unsigned int power) {
	if (power == 3)
		return  BaseArgosPower::POWER_3_MW;
	if (power == 40)
		return BaseArgosPower::POWER_40_MW;
	if (power == 200)
		return BaseArgosPower::POWER_200_MW;
	if (power == 500)
		return BaseArgosPower::POWER_500_MW;
	if (power == 5)
		return BaseArgosPower::POWER_5_MW;
	if (power == 50)
		return BaseArgosPower::POWER_50_MW;
	if (power == 350)
		return BaseArgosPower::POWER_350_MW;
	if (power == 750)
		return BaseArgosPower::POWER_750_MW;
	if (power == 1000)
		return BaseArgosPower::POWER_1000_MW;
	if (power == 1500)
		return BaseArgosPower::POWER_1500_MW;
	return BaseArgosPower::POWER_3_MW;
}

inline unsigned int argos_power_to_integer(BaseArgosPower power) {
	switch (power) {
	case BaseArgosPower::POWER_40_MW:
		return 40;
	case BaseArgosPower::POWER_500_MW:
		return 500;
	case BaseArgosPower::POWER_200_MW:
		return 200;
	case BaseArgosPower::POWER_3_MW:
		return 3;
	case BaseArgosPower::POWER_5_MW:
		return 5;
	case BaseArgosPower::POWER_50_MW:
		return 50;
	case BaseArgosPower::POWER_350_MW:
		return 350;
	case BaseArgosPower::POWER_750_MW:
		return 750;
	case BaseArgosPower::POWER_1000_MW:
		return 1000;
	case BaseArgosPower::POWER_1500_MW:
		return 1500;
	default:
		return 0;
	}
}

enum class BaseDepthPile {
	DEPTH_PILE_1 = 1,
	DEPTH_PILE_2,
	DEPTH_PILE_3,
	DEPTH_PILE_4,
	DEPTH_PILE_8 = 8,
	DEPTH_PILE_12 = 12,
	DEPTH_PILE_16 = 16,
	DEPTH_PILE_20 = 20,
	DEPTH_PILE_24 = 24
};
using BaseArgosDepthPile = BaseDepthPile;

enum class BaseDeltaTimeLoc {
	DELTA_T_10MIN = 1,
	DELTA_T_15MIN,
	DELTA_T_30MIN,
	DELTA_T_1HR,
	DELTA_T_2HR,
	DELTA_T_3HR,
	DELTA_T_4HR,
	DELTA_T_6HR,
	DELTA_T_12HR,
	DELTA_T_24HR
};

enum class BaseGNSSFixMode {
	FIX_2D = 1,
	FIX_3D = 2,
	AUTO = 3
};

enum class BaseGNSSDynModel {
	PORTABLE = 0,
	STATIONARY = 2,
	PEDESTRIAN = 3,
	AUTOMOTIVE = 4,
	SEA = 5,
	AIRBORNE_1G = 6,
	AIRBORNE_2G = 7,
	AIRBORNE_4G = 8,
	WRIST_WORN_WATCH = 9,
	BIKE = 10
};

enum class BaseLEDMode {
	OFF,
	HRS_24,
	ALWAYS = 3
};

enum class BaseZoneType {
	CIRCLE = 1
};

enum class BaseDebugMode {
	UART,     // UART debug output on SWO pin (P0.11 for RSPB)
	USB_CDC,  // USB CDC debug output (default for Linkit V4)
	BLE_NUS   // Bluetooth Low Energy Nordic UART Service
};

enum class BaseSensorEnableTxMode {
	OFF,
	ONESHOT,
	MEAN,
	MEDIAN,
};


enum class BaseArgosModulation {
	LDK,
	A2,
	A4
};

inline const char *argos_modulation_to_string(BaseArgosModulation m) {
	switch (m) {
	case BaseArgosModulation::A2:
		return "LDA2";
	case BaseArgosModulation::LDK:
		return "LDK";
	case BaseArgosModulation::A4:
		return "VLDA4";
	default:
		return "UNKNOWN";
	}
}

#define MAX_AOP_SATELLITE_ENTRIES		40

inline bool operator==(const AopSatelliteEntry_t& lhs, const AopSatelliteEntry_t& rhs)
{
	return lhs.ascNodeDriftDeg == rhs.ascNodeDriftDeg &&
			lhs.ascNodeLongitudeDeg == rhs.ascNodeLongitudeDeg &&
			lhs.bulletin.day == rhs.bulletin.day &&
			lhs.bulletin.hour == rhs.bulletin.hour &&
			lhs.bulletin.minute == rhs.bulletin.minute &&
			lhs.bulletin.second == rhs.bulletin.second &&
			lhs.bulletin.month == rhs.bulletin.month &&
			lhs.bulletin.year == rhs.bulletin.year &&
			lhs.downlinkStatus == rhs.downlinkStatus &&
			lhs.inclinationDeg == rhs.inclinationDeg &&
			lhs.orbitPeriodMin == rhs.orbitPeriodMin &&
			lhs.satHexId == rhs.satHexId &&
			lhs.semiMajorAxisDriftMeterPerDay == rhs.semiMajorAxisDriftMeterPerDay &&
			lhs.semiMajorAxisKm == rhs.semiMajorAxisKm &&
			lhs.uplinkStatus == rhs.uplinkStatus;
}

inline bool operator!=(const AopSatelliteEntry_t& lhs, const AopSatelliteEntry_t& rhs)
{
    return !(lhs == rhs);
}

struct BasePassPredict {
	unsigned int	   version_code;
	uint8_t num_records;
	AopSatelliteEntry_t records[MAX_AOP_SATELLITE_ENTRIES];
};

inline bool operator==(const BasePassPredict& lhs, const BasePassPredict& rhs)
{
    if (lhs.version_code != rhs.version_code || lhs.num_records != rhs.num_records)
        return false;
    for (uint8_t i = 0; i < lhs.num_records && i < MAX_AOP_SATELLITE_ENTRIES; i++) {
        if (!(lhs.records[i] == rhs.records[i]))
            return false;
    }
    return true;
}

struct BaseRawData {
	// Use of a pointer and length field is permitted for encoding base64
	void *ptr;
	unsigned int length;  // Set to 0 if using string as raw data source

	// Passing string instead is generally preferred wherever possible
	std::string  str;
};

using BaseKey = std::string;
using BaseName = std::string;
using BaseConstraint = std::variant<unsigned int, int, double, std::string>;

// !!! Do not change the ordering of variants and also make sure std::string is the first entry !!!
using BaseType = std::variant<std::string, unsigned int, int, double, std::time_t, BaseRawData, BaseArgosMode, BaseArgosPower, BaseDepthPile, bool, BaseGNSSFixMode, BaseGNSSDynModel, BaseLEDMode, BaseZoneType, BaseArgosModulation, BaseUnderwaterDetectSource, BaseDebugMode, BasePressureSensorLoggingMode, BasePressureSensorFullScale, BaseSensorEnableTxMode>;

struct BaseMap {
	BaseName 	   name;
	BaseKey  	   key;
	BaseEncoding   encoding;
	BaseConstraint min_value;
	BaseConstraint max_value;
	std::vector<BaseConstraint> permitted_values;
	bool           is_implemented;
	bool           is_writable;
};
