#include "dte_params.hpp"

// Helper for EXTERNAL_WAKEUP which is defined as a flag (no value)
#ifdef EXTERNAL_WAKEUP
#define HAS_EXTERNAL_WAKEUP true
#else
#define HAS_EXTERNAL_WAKEUP false
#endif

// ARGOS_SMD is only defined on targets with SMD satellite module
#ifndef ARGOS_SMD
#define ARGOS_SMD 0
#endif

const BaseMap param_map[] = {
	{ "ARGOS_DECID", "IDP12", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, true, true },
	{ "ARGOS_HEXID", "IDT06", BaseEncoding::HEXADECIMAL, 0U, 0xFFFFFFFFU, {}, true, true },
	{ "DEVICE_MODEL", "IDT02", BaseEncoding::TEXT, "", "", {}, true, true },
	{ "FW_APP_VERSION", "IDT03", BaseEncoding::TEXT, "", "", {}, true, false },
	{ "LAST_TX", "ART01", BaseEncoding::DATESTRING, 0, 0, {}, true, false },
	{ "TX_COUNTER", "ART02", BaseEncoding::UINT, 0U, 0U, {}, true, false },
	{ "BATT_SOC", "POT03", BaseEncoding::UINT, 0U, 100U, {}, true, false },
	{ "LAST_FULL_CHARGE_DATE", "POT05", BaseEncoding::DATESTRING, 0, 0, {}, true, false },
	{ "PROFILE_NAME", "IDP11", BaseEncoding::TEXT, "", "", {}, true, true },
	{ "_RESERVED_9", "", BaseEncoding::UINT, 0, 0, {}, false, false },  // Reserved: AOP status is part of PASPW allcast data, not a DTE param
	{ "ARGOS_AOP_DATE", "ART03", BaseEncoding::DATESTRING, 0, 0, {}, true, false },
	{ "_RESERVED_ARGOS_FREQ", "", BaseEncoding::ARGOSFREQ, 401.6200, 401.6800, {}, false, false },  // Obsolete: RADIOCONF controls frequency
	{ "_RESERVED_ARGOS_POWER", "", BaseEncoding::ARGOSPOWER, 0, 0, { 0, 1, 2, 3 }, false, false },  // Obsolete: RADIOCONF controls power
	{ "TR_NOM", "ARP05", BaseEncoding::UINT, 30U, 1200U, {}, true, true },
	{ "ARGOS_MODE", "ARP01", BaseEncoding::ARGOSMODE, 0, 0, { 0, 1, 2, 3, 4 }, true, true },
	{ "NTRY_PER_MESSAGE", "ARP19", BaseEncoding::UINT, 0U, 86400U, {}, true, true },
	{ "DUTY_CYCLE", "ARP18", BaseEncoding::UINT, 0U, 0xFFFFFFU, {}, true, true },
	{ "GNSS_EN", "GNP01", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "DLOC_ARG_NOM", "ARP11", BaseEncoding::AQPERIOD, 0, 0, { 0, 1, 2, 3, 4, 5, 6, 7, 8 }, true, true },
	{ "ARGOS_DEPTH_PILE", "ARP16", BaseEncoding::DEPTHPILE, 0U, 0U, {1U, 2U, 3U, 4U, 8U, 9U, 10U, 11U, 12U}, true, true },
	{ "_RESERVED_20", "", BaseEncoding::UINT, 0, 0, {}, false, false },  // Reserved: constellation select never implemented
	{ "GLONASS_CONST_SELECT", "GNP08", BaseEncoding::UINT, 0, 0, {}, false, true },
	{ "GNSS_HDOPFILT_EN", "GNP02", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "GNSS_HDOPFILT_THR", "GNP03", BaseEncoding::UINT, 2U, 15U, {}, true, true },
	{ "GNSS_ACQ_TIMEOUT", "GNP05", BaseEncoding::UINT, 10U, 600U, {}, true, true },
	{ "GNSS_NTRY", "GNP04", BaseEncoding::UINT, 0U, 0U, {}, false, true },
	{ "UNDERWATER_EN", "UNP01", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "DRY_TIME_BEFORE_TX", "UNP02", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, true, true },
	{ "SAMPLING_UNDER_FREQ", "UNP03", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },
	{ "LB_EN", "LBP01", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "LB_THRESHOLD", "LBP02", BaseEncoding::UINT, 0U, 100U, {}, true, true },
	{ "_RESERVED_LB_ARGOS_POWER", "", BaseEncoding::ARGOSPOWER, 0, 0, { 0, 1, 2, 3 }, false, false },  // Obsolete: RADIOCONF controls power
	{ "TR_LB", "ARP06", BaseEncoding::UINT, 30U, 1200U, {}, true, true },
	{ "LB_ARGOS_MODE", "LBP04", BaseEncoding::ARGOSMODE, 0U, 0U, { 0U, 1U, 2U, 3U, 4U }, true, true },
	{ "LB_ARGOS_DUTY_CYCLE", "LBP05", BaseEncoding::UINT, 0U, 0xFFFFFFU, {}, true, true },
	{ "LB_GNSS_EN", "LBP06", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "DLOC_ARG_LB", "ARP12", BaseEncoding::AQPERIOD, 0, 0, { 0, 1, 2, 3, 4, 5, 6, 7, 8 }, true, true },
	{ "LB_GNSS_HDOPFILT_THR", "LBP07", BaseEncoding::UINT, 2U, 15U, {}, true, true },
	{ "LB_ARGOS_DEPTH_PILE", "LBP08", BaseEncoding::DEPTHPILE, 0U, 0U, {1U, 2U, 3U, 4U, 8U, 9U, 10U, 11U, 12U}, true, true },
	{ "LB_GNSS_ACQ_TIMEOUT", "LBP09", BaseEncoding::UINT, 10U, 600U, {}, true, true },
	{ "SAMPLING_SURF_FREQ", "UNP04", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },
    { "PP_MIN_ELEVATION", "PPP01", BaseEncoding::FLOAT, 0.0, 90.0, {}, true, true },
	{ "PP_MAX_ELEVATION", "PPP02", BaseEncoding::FLOAT, 0.0, 90.0, {}, true, true },
    { "PP_MIN_DURATION", "PPP03", BaseEncoding::UINT, 20U, 3600U, {}, true, true },
	{ "PP_MAX_PASSES", "PPP04", BaseEncoding::UINT, 1U, 10000U, {}, true, true },
	{ "PP_LINEAR_MARGIN", "PPP05", BaseEncoding::UINT, 1U, 3600U, {}, true, true },
	{ "PP_COMP_STEP", "PPP06", BaseEncoding::UINT, 1U, 1000U, {}, true, true },
	{ "GNSS_COLD_ACQ_TIMEOUT", "GNP09", BaseEncoding::UINT, 10U, 600U, {}, true, true },
	{ "GNSS_FIX_MODE", "GNP10", BaseEncoding::GNSSFIXMODE, 0U, 0U, {1U, 2U, 3U}, true, true },
	{ "GNSS_DYN_MODEL", "GNP11", BaseEncoding::GNSSDYNMODEL, 0U, 0U, {0U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U}, true, true },
	{ "GNSS_HACCFILT_EN", "GNP20", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "GNSS_HACCFILT_THR", "GNP21", BaseEncoding::UINT, 0U, 0U, {}, true, true },
	{ "GNSS_MIN_NUM_FIXES", "GNP22", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },
	{ "GNSS_COLD_START_RETRY_PERIOD", "GNP23", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },
	{ "ARGOS_TIME_SYNC_BURST_EN", "ARP30", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "LED_MODE", "LDP01", BaseEncoding::LEDMODE, 0U, 0U, {}, true, true },
	{ "ARGOS_TX_JITTER_EN", "ARP31", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "ARGOS_RX_EN", "ARP32", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "ARGOS_RX_MAX_WINDOW", "ARP33", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },
	{ "ARGOS_RX_AOP_UPDATE_PERIOD", "ARP34", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, true, true },
	{ "ARGOS_RX_COUNTER", "ART10", BaseEncoding::UINT, 0U, 0U, {}, true, false },
	{ "ARGOS_RX_TIME", "ART11", BaseEncoding::UINT, 0U, 0U, {}, true, false },
	{ "GNSS_ASSISTNOW_EN", "GNP24", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "LB_GNSS_HACCFILT_THR", "LBP10", BaseEncoding::UINT, 0U, 0U, {}, true, true },
	{ "LB_NTRY_PER_MESSAGE", "LBP11", BaseEncoding::UINT, 0U, 86400U, {}, true, true },

	//////////////////////////
	// ZONE FILE
	//////////////////////////
	{ "ZONE_TYPE", "ZOP01", BaseEncoding::ZONETYPE, 0, 0, { 0U }, true, true },
	{ "ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE", "ZOP04", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "ZONE_ENABLE_ACTIVATION_DATE", "ZOP05", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "ZONE_ACTIVATION_DATE", "ZOP06", BaseEncoding::DATESTRING, 0, 0, {}, true, true },
	{ "ZONE_ARGOS_DEPTH_PILE", "ZOP08", BaseEncoding::DEPTHPILE, 0U, 0U, {1U, 2U, 3U, 4U, 8U, 9U, 10U, 11U, 12U}, true, true },
	{ "_RESERVED_70", "", BaseEncoding::ARGOSPOWER, 0U, 0U, { 0, 1, 2, 3 }, false, false },
	{ "ZONE_ARGOS_REPETITION_SECONDS", "ZOP10", BaseEncoding::UINT, 30U, 1200U, {}, true, true },
	{ "ZONE_ARGOS_MODE", "ZOP11", BaseEncoding::ARGOSMODE, 0U, 0U, { 0U, 1U, 2U, 3U, 4U }, true, true },
	{ "ZONE_ARGOS_DUTY_CYCLE", "ZOP12", BaseEncoding::UINT, 0U, 0xFFFFFFU, {}, true, true },
	{ "ZONE_ARGOS_NTRY_PER_MESSAGE", "ZOP13", BaseEncoding::UINT, 0U, 86400U, {}, true, true },
	{ "ZONE_GNSS_DELTA_ARG_LOC_ARGOS_SECONDS", "ZOP14", BaseEncoding::AQPERIOD, 0, 0, { 0, 1, 2, 3, 4, 5, 6, 7, 8 }, true, true },
	{ "ZONE_GNSS_HDOPFILT_THR", "ZOP15", BaseEncoding::UINT, 2U, 15U, {}, true, true },
	{ "ZONE_GNSS_HACCFILT_THR", "ZOP16", BaseEncoding::UINT, 0U, 0U, {}, true, true },
	{ "ZONE_GNSS_ACQ_TIMEOUT", "ZOP17", BaseEncoding::UINT, 10U, 600U, {}, true, true },
	{ "ZONE_CENTER_LONGITUDE", "ZOP18", BaseEncoding::FLOAT, -180.0, 180.0, {}, true, true },
	{ "ZONE_CENTER_LATITUDE", "ZOP19", BaseEncoding::FLOAT, -90.0, 90.0, {}, true, true },
	{ "ZONE_RADIUS", "ZOP20", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },

	//////////////////////////
	// CERTIFICATION
	//////////////////////////
	{ "CERT_TX_ENABLE", "CTP01", BaseEncoding::BOOLEAN, 0, 0, { }, true, true },
	{ "CERT_TX_PAYLOAD", "CTP02", BaseEncoding::TEXT, "", "", {}, true, true },
	{ "CERT_TX_MODULATION", "CTP03", BaseEncoding::MODULATION, 0, 0, { 0U, 1U, 2U }, true, true },
	{ "CERT_TX_REPETITION", "CTP04", BaseEncoding::UINT, 2U, 0xFFFFFFFFU, { }, true, true },

	// HW version identification
	{ "HW_VERSION", "IDT04", BaseEncoding::TEXT, "", "", {}, true, false },

	// Battery voltage
	{ "BATT_VOLTAGE", "POT06", BaseEncoding::FLOAT, 0.0, 12.0, {}, true, false },

	// [88-91] TPL5111 power management parameters (slots always reserved)
	{ "SHUTDOWN_TIMER", "PWP01", BaseEncoding::UINT, 0U, 86400U, {}, HAS_EXTERNAL_WAKEUP, true },
	{ "BOOT_COUNTER", "PWP02", BaseEncoding::UINT, 0U, 0U, {}, HAS_EXTERNAL_WAKEUP, false },
	{ "BOOT_COUNTER_MODULO", "PWP03", BaseEncoding::UINT, 2U, 1000U, {}, HAS_EXTERNAL_WAKEUP, true },
	{ "WAKEUP_PERIOD", "PWP04", BaseEncoding::UINT, 0U, 86400U, {}, HAS_EXTERNAL_WAKEUP, true },

	// TCXO warmup period
	{ "ARGOS_TCXO_WARMUP_TIME", "ARP35", BaseEncoding::UINT, 0U, 30U, {}, true, true },

	{ "DEVICE_DECID", "IDT10", BaseEncoding::UINT, 0U, 0U, {}, true, false },
	{ "GNSS_TRIGGER_ON_SURFACED", "GNP25", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "GNSS_TRIGGER_ON_AXL_WAKEUP", "GNP26", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "UNDERWATER_DETECT_SOURCE", "UNP10", BaseEncoding::UWDETECTSOURCE, 0, 0, {}, true, true },
	{ "UNDERWATER_DETECT_THRESH", "UNP11", BaseEncoding::FLOAT, (double)0.0, (double)0.0, {}, true, true },

	// [98-100] pH sensor (slots always reserved)
	{ "PH_SENSOR_ENABLE", "PHP01", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_PH_SENSOR, true },
	{ "PH_SENSOR_PERIODIC", "PHP02", BaseEncoding::UINT, 0U, 0U, {}, ENABLE_PH_SENSOR, true },
	{ "PH_SENSOR_VALUE", "PHP03", BaseEncoding::FLOAT, (double)0.0, (double)0.0, {}, ENABLE_PH_SENSOR, true },
	// [101-103] Sea temperature sensor (slots always reserved)
	{ "SEA_TEMP_SENSOR_ENABLE", "STP01", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_SEA_TEMP_SENSOR, true },
	{ "SEA_TEMP_SENSOR_PERIODIC", "STP02", BaseEncoding::UINT, 0U, 0U, {}, ENABLE_SEA_TEMP_SENSOR, true },
	{ "SEA_TEMP_SENSOR_VALUE", "STP03", BaseEncoding::FLOAT, (double)0.0, (double)0.0, {}, ENABLE_SEA_TEMP_SENSOR, true },
	// [104-106] ALS sensor (slots always reserved)
	{ "ALS_SENSOR_ENABLE", "LTP01", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_ALS_SENSOR, true },
	{ "ALS_SENSOR_PERIODIC", "LTP02", BaseEncoding::UINT, 0U, 0U, {}, ENABLE_ALS_SENSOR, true },
	{ "ALS_SENSOR_VALUE", "LTP03", BaseEncoding::FLOAT, (double)0.0, (double)0.0, {}, ENABLE_ALS_SENSOR, true },
	// [107-111] CDT sensor (slots always reserved)
	{ "CDT_SENSOR_ENABLE", "CDP01", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_CDT_SENSOR, true },
	{ "CDT_SENSOR_PERIODIC", "CDP02", BaseEncoding::UINT, 0U, 0U, {}, ENABLE_CDT_SENSOR, true },
	{ "CDT_SENSOR_CONDUCTIVITY", "CDP03", BaseEncoding::FLOAT, (double)0.0, (double)0.0, {}, ENABLE_CDT_SENSOR, true },
	{ "CDT_SENSOR_DEPTH", "CDP04", BaseEncoding::FLOAT, (double)0.0, (double)0.0, {}, ENABLE_CDT_SENSOR, true },
	{ "CDT_SENSOR_TEMPERATURE", "CDP05", BaseEncoding::FLOAT, (double)0.0, (double)0.0, {}, ENABLE_CDT_SENSOR, true },
	// [112-116] Thermistor sensor (slots always reserved)
	{ "THERMISTOR_SENSOR_ENABLE", "THP01", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_THERMISTOR_SENSOR, true },
	{ "THERMISTOR_SENSOR_PERIODIC", "THP02", BaseEncoding::UINT, 0U, 0U, {}, ENABLE_THERMISTOR_SENSOR, true },
	{ "THERMISTOR_SENSOR_VALUE", "THP03", BaseEncoding::FLOAT, (double)0.0, (double)0.0, {}, ENABLE_THERMISTOR_SENSOR, false },
	{ "THERMISTOR_SENSOR_WAKEUP_THRESH", "THP04", BaseEncoding::FLOAT, (double)0.0, (double)0.0, {}, ENABLE_THERMISTOR_SENSOR, true },
	{ "THERMISTOR_SENSOR_WAKEUP_SAMPLES", "THP05", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, ENABLE_THERMISTOR_SENSOR, true },
	// [117] External LED
	{ "EXT_LED_MODE", "LDP02", BaseEncoding::LEDMODE, 0U, 0U, {}, true, true },
	// [118-123] Accelerometer sensor (slots always reserved)
	{ "AXL_SENSOR_ENABLE", "AXP01", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_AXL_SENSOR, true },
	{ "AXL_SENSOR_PERIODIC", "AXP02", BaseEncoding::UINT, 0U, 0U, {}, ENABLE_AXL_SENSOR, true },
	{ "AXL_SENSOR_WAKEUP_THRESH", "AXP03", BaseEncoding::FLOAT, (double)0.0, (double)8.0, {}, ENABLE_AXL_SENSOR, true },
	{ "AXL_SENSOR_WAKEUP_SAMPLES", "AXP04", BaseEncoding::UINT, 0U, 50U, {}, ENABLE_AXL_SENSOR, true },
	{ "AXL_SENSOR_MEASUREMENT_RANGE", "AXP08", BaseEncoding::UINT, 0U, 4U, {}, ENABLE_AXL_SENSOR, true },
	{ "AXL_SENSOR_POWER_MODE", "AXP09", BaseEncoding::UINT, 0U, 2U, {}, ENABLE_AXL_SENSOR, true },
	// [124-125] Pressure sensor (slots always reserved)
	{ "PRESSURE_SENSOR_ENABLE", "PRP01", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_PRESSURE_SENSOR, true },
	{ "PRESSURE_SENSOR_PERIODIC", "PRP02", BaseEncoding::UINT, 0U, 0U, {}, ENABLE_PRESSURE_SENSOR, true },

	// DEBUG
	{ "DEBUG_OUTPUT_MODE", "DBP01", BaseEncoding::DEBUGMODE, 0, 0, {}, true, true },

	// GNSS ANO
	{ "GNSS_ASSISTNOW_OFFLINE_EN", "GNP27", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },

	// Extended UW parameters
	{ "UW_MAX_SAMPLES", "UNP05", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },
	{ "UW_MIN_DRY_SAMPLES", "UNP06", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },
	{ "UW_SAMPLE_GAP", "UNP07", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },
	{ "UW_PIN_SAMPLE_DELAY", "UNP08", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },

	// SWS Analog parameters
	{ "SWS_ANALOG_THRESHOLD_MIN", "UNP20", BaseEncoding::UINT, 0U, 16383U, {}, true, true },
	{ "SWS_ANALOG_THRESHOLD_MAX", "UNP21", BaseEncoding::UINT, 50U, 16383U, {}, true, true },
	{ "SWS_ANALOG_HYSTERESIS", "UNP22", BaseEncoding::UINT, 0U, 50U, {}, true, true },
	{ "SWS_ANALOG_CALIB_INTERVAL", "UNP23", BaseEncoding::UINT, 60U, 0xFFFFFFFFU, {}, true, true },
	{ "UW_MAX_DIVE_TIME", "UNP24", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, true, true },
	{ "UW_MIN_SURFACE_TIME", "UNP25", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, true, true },

	// Dive mode
	{ "UW_DIVE_MODE_ENABLE", "UNP12", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	{ "UW_DIVE_MODE_START_TIME", "UNP13", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, true, true },

	// GNSS UW parameters
	{ "UW_GNSS_DRY_SAMPLING", "UNP14", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },
	{ "UW_GNSS_WET_SAMPLING", "UNP15", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },
	{ "UW_GNSS_MAX_SAMPLES", "UNP16", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },
	{ "UW_GNSS_MIN_DRY_SAMPLES", "UNP17", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, true, true },
	{ "UW_GNSS_DETECT_THRESH", "UNP18", BaseEncoding::UINT, 1U, 7U, {}, true, true },

	// Critical battery threshold
	{ "LB_CRITICAL_THRESH", "LBP12", BaseEncoding::UINT, 0U, 100U, {}, true, true },

	// [146] Pressure sensor logging mode (slot always reserved)
	{ "PRESSURE_SENSOR_LOGGING_MODE", "PRP03", BaseEncoding::PRESSURESENSORLOGGINGMODE, 0, 0, {}, ENABLE_PRESSURE_SENSOR, true },
	// [147] GNSS trigger cold start on surfaced
	{ "GNSS_TRIGGER_COLD_START_ON_SURFACED", "GNP28", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	// [148-150] Sea temp TX options (slots always reserved)
	{ "SEA_TEMP_SENSOR_ENABLE_TX_MODE", "STP04", BaseEncoding::SENSORENABLETXMODE, 0, 0, {}, ENABLE_SEA_TEMP_SENSOR, true },
	{ "SEA_TEMP_SENSOR_ENABLE_TX_MAX_SAMPLES", "STP05", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_SEA_TEMP_SENSOR, true },
	{ "SEA_TEMP_SENSOR_ENABLE_TX_SAMPLE_PERIOD", "STP06", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_SEA_TEMP_SENSOR, true },
	// [151-153] pH TX options (slots always reserved)
	{ "PH_SENSOR_ENABLE_TX_MODE", "PHP04", BaseEncoding::SENSORENABLETXMODE, 0, 0, {}, ENABLE_PH_SENSOR, true },
	{ "PH_SENSOR_ENABLE_TX_MAX_SAMPLES", "PHP05", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_PH_SENSOR, true },
	{ "PH_SENSOR_ENABLE_TX_SAMPLE_PERIOD", "PHP06", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_PH_SENSOR, true },
	// [154-156] ALS TX options (slots always reserved)
	{ "ALS_SENSOR_ENABLE_TX_MODE", "LTP04", BaseEncoding::SENSORENABLETXMODE, 0, 0, {}, ENABLE_ALS_SENSOR, true },
	{ "ALS_SENSOR_ENABLE_TX_MAX_SAMPLES", "LTP05", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_ALS_SENSOR, true },
	{ "ALS_SENSOR_ENABLE_TX_SAMPLE_PERIOD", "LTP06", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_ALS_SENSOR, true },
	// [157-159] Pressure TX options (slots always reserved)
	{ "PRESSURE_SENSOR_ENABLE_TX_MODE", "PRP04", BaseEncoding::SENSORENABLETXMODE, 0, 0, {}, ENABLE_PRESSURE_SENSOR, true },
	{ "PRESSURE_SENSOR_ENABLE_TX_MAX_SAMPLES", "PRP05", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_PRESSURE_SENSOR, true },
	{ "PRESSURE_SENSOR_ENABLE_TX_SAMPLE_PERIOD", "PRP06", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_PRESSURE_SENSOR, true },
	// [160-162] AXL TX options (slots always reserved)
	{ "AXL_SENSOR_ENABLE_TX_MODE", "AXP05", BaseEncoding::SENSORENABLETXMODE, 0, 0, {}, ENABLE_AXL_SENSOR, true },
	{ "AXL_SENSOR_ENABLE_TX_MAX_SAMPLES", "AXP06", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_AXL_SENSOR, true },
	{ "AXL_SENSOR_ENABLE_TX_SAMPLE_PERIOD", "AXP07", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_AXL_SENSOR, true },
	// [163-165] Thermistor TX options (slots always reserved)
	{ "THERMISTOR_SENSOR_ENABLE_TX_MODE", "THP06", BaseEncoding::SENSORENABLETXMODE, 0, 0, {}, ENABLE_THERMISTOR_SENSOR, true },
	{ "THERMISTOR_SENSOR_ENABLE_TX_MAX_SAMPLES", "THP07", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_THERMISTOR_SENSOR, true },
	{ "THERMISTOR_SENSOR_ENABLE_TX_SAMPLE_PERIOD", "THP08", BaseEncoding::UINT, 1U, 0xFFFFFFFFU, {}, ENABLE_THERMISTOR_SENSOR, true },
	// [166-171] Camera sensor (slots always reserved)
	{ "CAM_ENABLE", "CAP01", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_CAM_SENSOR, true },
	{ "CAM_TRIGGER_ON_SURFACED", "CAP02", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_CAM_SENSOR, true },
	{ "CAM_TRIGGER_ON_AXL_WAKEUP", "CAP03", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_CAM_SENSOR, true },
	{ "CAM_PERIOD_ON", "CAP04", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, ENABLE_CAM_SENSOR, true },
	{ "CAM_PERIOD_OFF", "CAP05", BaseEncoding::UINT, 0U, 0xFFFFFFFFU, {}, ENABLE_CAM_SENSOR, true },
	{ "LB_CAM_EN", "LBP13", BaseEncoding::BOOLEAN, 0, 0, {}, ENABLE_CAM_SENSOR, true },
	// [172-173] SMD credentials (slots always reserved)
	{ "ARGOS_SECKEY", "IDP13", BaseEncoding::TEXT, "", "", {}, (ARGOS_SMD == 1), true },
	{ "ARGOS_RADIOCONF", "IDP14", BaseEncoding::TEXT, "", "", {}, (ARGOS_SMD == 1), true },
	// [174-176] Session shutdown control (slots always reserved)
	{ "SHUTDOWN_NTIME_SAT", "PWP05", BaseEncoding::UINT, 0U, 65535U, {}, true, true },
	{ "LB_SHUTDOWN_NTIME_SAT", "LBP14", BaseEncoding::UINT, 0U, 65535U, {}, true, true },
	{ "GNSS_SESSION_SINGLE_FIX", "GNP30", BaseEncoding::BOOLEAN, 0, 0, {}, true, true },
	// [177] Pressure sensor full scale (slots always reserved)
	{ "PRESSURE_SENSOR_FULL_SCALE", "PRP07", BaseEncoding::PRESSURESENSORFULLSCALE, 0, 0, {}, ENABLE_PRESSURE_SENSOR, true },
	// [178] GNSS token (e.g. u-blox AssistNow/CloudLocate token)
	{ "GNSS_TOKEN", "GNP31", BaseEncoding::TEXT, "", "", {}, true, true },
	// [179] Last known RTC timestamp for pseudo RTC at boot (TPL5111)
	{ "LAST_KNOWN_RTC", "PWP06", BaseEncoding::UINT, 0U, 0U, {}, HAS_EXTERNAL_WAKEUP, false },
	// [180] Current RTC time (live, refreshed on STATR read)
	{ "RTC_CURRENT_TIME", "SYT01", BaseEncoding::UINT, 0U, 0U, {}, true, false },
};

const size_t param_map_size = sizeof(param_map) / sizeof(param_map[0]);
static_assert(sizeof(param_map) / sizeof(param_map[0]) == (size_t)ParamID::__PARAM_SIZE,
              "param_map size must match ParamID::__PARAM_SIZE");
