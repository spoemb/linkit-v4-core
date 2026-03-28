#include "dte_commands.hpp"

const DTECommandMap command_map[] = {
	{
		.name = "PARML",
		.command = DTECommand::PARML_REQ,
		.prototype = 
		{
		}
	},
	{
		.name = "PARMR",
		.command = DTECommand::PARMR_REQ,
		.prototype = 
		{
			{
				.name = "keys",
				.key = "",
				.encoding = BaseEncoding::KEY_LIST,
				.min_value = 0,
				.max_value = 0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	{
		.name = "PARMW",
		.command = DTECommand::PARMW_REQ,
		.prototype = 
		{
			{
				.name = "key_values",
				.key = "",
				.encoding = BaseEncoding::KEY_VALUE_LIST,
				.min_value = 0,
				.max_value = 0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	{
		.name = "PROFR",
		.command = DTECommand::PROFR_REQ,
		.prototype = 
		{
		}
	},
	{
		.name = "PROFW",
		.command = DTECommand::PROFW_REQ,
		.prototype = 
		{
			{
				.name = "profile_name",
				.key = "",
				.encoding = BaseEncoding::TEXT,
				.min_value = 1,
				.max_value = 128,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	{
		.name = "PASPW",
		.command = DTECommand::PASPW_REQ,
		.prototype = 
		{
			{
				.name = "prepass_file",
				.key = "",
				.encoding = BaseEncoding::BASE64,
				.min_value = 0,
				.max_value = 0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	{
		.name = "SECUR",
		.command = DTECommand::SECUR_REQ,
		.prototype = 
		{
				{
					.name = "accesscode",
					.key = "",
					.encoding = BaseEncoding::HEXADECIMAL,
					.min_value = 0U,
					.max_value = 0U,
					.permitted_values = {},
					.is_implemented = false,
					.is_writable = false
				},
		}
	},
	{
		.name = "DUMPM",
		.command = DTECommand::DUMPM_REQ,
		.prototype = 
		{
			{
				.name = "start_address",
				.key = "",
				.encoding = BaseEncoding::HEXADECIMAL,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "length",
				.key = "",
				.encoding = BaseEncoding::HEXADECIMAL,
				.min_value = 0U,
				.max_value = 0x500U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	{
		.name = "DUMPD",
		.command = DTECommand::DUMPD_REQ,
		.prototype =
		{
			{
				.name = "d_type",
				.key = "",
				.encoding = BaseEncoding::HEXADECIMAL,
				.min_value = 0U,
				.max_value = 12U,  // Updated: includes MORTALITY(12)
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
		}
	},
	{
		.name = "RSTVW",
		.command = DTECommand::RSTVW_REQ,
		.prototype =
		{
				{
					.name = "index",
					.key = "",
					.encoding = BaseEncoding::HEXADECIMAL,
					.min_value = 0U,
					.max_value = 0U,
					.permitted_values = { 1U, 2U, 3U, 4U },
					.is_implemented = false,
					.is_writable = false
				},
		}
	},
	{
		.name = "RSTBW",
		.command = DTECommand::RSTBW_REQ,
		.prototype = 
		{
		}
	},
	{
		.name = "FACTW",
		.command = DTECommand::FACTW_REQ,
		.prototype = 
		{
		}
	},
	{
		.name = "STATR",
		.command = DTECommand::STATR_REQ,
		.prototype =
		{
			{
				.name = "keys",
				.key = "",
				.encoding = BaseEncoding::KEY_LIST,
				.min_value = 0,
				.max_value = 0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	{
		.name = "ERASE",
		.command = DTECommand::ERASE_REQ,
		.prototype =
		{
			{
				.name = "log_type",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 1U,
				.max_value = 14U,  // Updated: includes MORTALITY(14)
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	{
		.name = "SCALW",
		.command = DTECommand::SCALW_REQ,
		.prototype =
		{
			{
				.name = "sensor",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 7U,  // Updated: includes THERMISTOR (BaseSensorCalType)
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "offset",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "value",
				.key = "",
				.encoding = BaseEncoding::FLOAT,
				.min_value = 0.0,
				.max_value = 0.0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	{
		.name = "SATTX",
		.command = DTECommand::ARGOSTX_REQ,
		.prototype =
		{
			{
				.name = "modulation",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 2U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				// Stored mode: size (decimal string, e.g. "24")
				// Custom mode: radioconf (32-char hex string)
				.name = "radioconf_or_size",
				.key = "",
				.encoding = BaseEncoding::TEXT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				// Stored mode: tcxo (optional)
				// Custom mode: size
				.name = "size_or_tcxo",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				// Custom mode only: tcxo (optional)
				.name = "tcxo",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		},
		.min_args = 2  // modulation + radioconf_or_size required; rest optional
	},
	{
		.name = "SCALR",
		.command = DTECommand::SCALR_REQ,
		.prototype =
		{
			{
				.name = "sensor",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 7U,  // Updated: includes THERMISTOR (BaseSensorCalType)
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "offset",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	// SENSR - Sensor/GNSS read command
	// Usage: $SENSR,<sensors_bitmask>,<gnss_timeout_s>*<checksum>\r\n
	// sensors_bitmask: 1=battery, 2=pressure, 4=GNSS, 8=accel, 15=all
	// Response: $SENSR,<status>,<batt_mv>,<batt_soc>,<pressure_bar>,<temp_c>,<altitude_m>,<lat>,<lon>,<hdop>,<num_sv>,<accel_x>,<accel_y>,<accel_z>,<accel_temp>*<checksum>\r\n
	{
		.name = "SENSR",
		.command = DTECommand::SENSR_REQ,
		.prototype =
		{
			{
				.name = "sensors",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 1U,
				.max_value = 255U,  // Bitmask: 1=battery, 2=pressure, 4=GNSS, 8=accel, 16=thermistor, 32=sea_temp, 64=ALS, 128=pH
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "timeout",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 5U,
				.max_value = 300U,  // GNSS timeout in seconds (5-300s)
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	// PWRON - Power on/off components command
	// Usage: $PWRON#001;<component>\r
	// Components: 0=all, 1=gnss, 2=sensors, 3=satellite, 4=off
	// Response: $O;PWRON#000;\r or $N;PWRON#00x;<error>\r
	{
		.name = "PWRON",
		.command = DTECommand::PWRON_REQ,
		.prototype =
		{
			{
				.name = "component",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 4U,  // 0=all, 1=gnss, 2=sensors, 3=satellite, 4=off
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	// SWSST - SWS analog calibration status read (no arguments)
	{
		.name = "SWSST",
		.command = DTECommand::SWSST_REQ,
		.prototype = {}
	},
	// SATDP - Satellite Doppler calibration (no arguments)
	// Usage: $SATDP#000;\r
	// Starts periodic Doppler TX at TR_NOM interval until device reset
	{
		.name = "SATDP",
		.command = DTECommand::SATDP_REQ,
		.prototype = {}
	},
	// GNSSI - GNSS device info (no arguments)
	// Usage: $GNSSI#000;\r
	// Returns unique ID, SW version, HW version from GNSS module
	{
		.name = "GNSSI",
		.command = DTECommand::GNSSI_REQ,
		.prototype = {}
	},
	// GNSSA - GNSS almanac file validation (no arguments)
	// Usage: $GNSSA#000;\r
	// Returns almanac file presence, size, record counts, staleness
	{
		.name = "GNSSA",
		.command = DTECommand::GNSSA_REQ,
		.prototype = {}
	},
	// RTCW - RTC manual write (set time before GNSS fix)
	// Usage: $RTCW#00A;<unix_timestamp>\r
	// Sets the RTC to the given unix timestamp value
	{
		.name = "RTCW",
		.command = DTECommand::RTCW_REQ,
		.prototype =
		{
			{
				.name = "timestamp",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,  // No max limit for unix timestamp
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	// SWSTST - SWS test mode start/stop
	// Usage: $SWSTST#001;1\r (start) or $SWSTST#001;0\r (stop)
	// Response: $O;SWSTST#001;<running>\r
	{
		.name = "SWSTST",
		.command = DTECommand::SWSTST_REQ,
		.prototype =
		{
			{
				.name = "action",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 1U,  // 0=stop, 1=start
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	// SWSCAL - SWS guided calibration (LED-assisted air/water measurement)
	// Usage: $SWSCAL#001;1\r (start) or $SWSCAL#001;0\r (cancel)
	// Response: $O;SWSCAL#003;<status>,<air>,<water>\r
	{
		.name = "SWSCAL",
		.command = DTECommand::SWSCAL_REQ,
		.prototype =
		{
			{
				.name = "action",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 1U,  // 0=cancel, 1=start
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	// GNSSBR - GNSS UART bridge/passthrough mode (direct u-blox access via USB)
	// Usage: $GNSSBR#001;1\r (start) — exit by typing +++
	{
		.name = "GNSSBR",
		.command = DTECommand::GNSSBR_REQ,
		.prototype =
		{
			{
				.name = "action",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 1U,  // 0=stop, 1=start
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	// SMD DFU command - allows firmware update of SMD satellite module
	// Usage: $SMDDFU#001;<action>\r
	// Actions: 0=enter, 1=exit, 2=status, 3=update, 4=info, 5=version
	// Note: Firmware data is sent via OTA file transfer, not in this command
	{
		.name = "SMDDFU",
		.command = DTECommand::SMDDFU_REQ,
		.prototype =
		{
			{
				.name = "action",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 5U,  // 0=enter, 1=exit, 2=status, 3=update, 4=info, 5=version
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	// SMD SPI applicative test - tests all A+ protocol read commands
	// Usage: $SMDTST#000;\r
	{
		.name = "SMDTST",
		.command = DTECommand::SMDTST_REQ,
		.prototype = {}
	},
	{
		.name = "SMDCD",
		.command = DTECommand::SMDCD_REQ,
		.prototype =
		{
			{
				.name = "id",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 999999U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "addr",
				.key = "",
				.encoding = BaseEncoding::HEXADECIMAL,
				.min_value = 0U,
				.max_value = 0xFFFFFFFFU,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "seckey",
				.key = "",
				.encoding = BaseEncoding::TEXT,
				.min_value = "",
				.max_value = "",
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "radioconf",
				.key = "",
				.encoding = BaseEncoding::TEXT,
				.min_value = "",
				.max_value = "",
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
#endif
	// Satellite credentials verify — read-back from hardware and compare with config store
	// Works on both SMD and KIM2
	// Usage: $SATVF#000;\r
	{
		.name = "SATVF",
		.command = DTECommand::SATVF_REQ,
		.prototype = {}
	},
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	// LoRa test transmission
	// Usage: $LORATX#001;<size>\r
	{
		.name = "LORATX",
		.command = DTECommand::LORATX_REQ,
		.prototype =
		{
			{
				.name = "size",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 1U,
				.max_value = 222U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	// LoRa UART bridge/passthrough mode (direct RUI3 AT access via USB)
	// Usage: $LORABR#001;1\r (start) — exit by typing +++
	{
		.name = "LORABR",
		.command = DTECommand::LORABR_REQ,
		.prototype =
		{
			{
				.name = "action",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 1U,  // 0=stop, 1=start
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
#endif
	{
		.name = "PARML",
		.command = DTECommand::PARML_RESP,
		.prototype =
		{
			{
				.name = "keys",
				.key = "",
				.encoding = BaseEncoding::KEY_LIST,
				.min_value = 0,
				.max_value = 0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	{
		.name = "PARMR",
		.command = DTECommand::PARMR_RESP,
		.prototype = 
		{
			{
				.name = "key_values",
				.key = "",
				.encoding = BaseEncoding::KEY_VALUE_LIST,
				.min_value = 0,
				.max_value = 0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	{
		.name = "PARMW",
		.command = DTECommand::PARMW_RESP,
		.prototype = 
		{
		}
	},
	{
		.name = "PROFR",
		.command = DTECommand::PROFR_RESP,
		.prototype = 
		{
			{
				.name = "profile_name",
				.key = "",
				.encoding = BaseEncoding::TEXT,
				.min_value = "",
				.max_value = "",
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	{
		.name = "PROFW",
		.command = DTECommand::PROFW_RESP,
		.prototype = 
		{
		}
	},
	{
		.name = "PASPW",
		.command = DTECommand::PASPW_RESP,
		.prototype = 
		{
		}
	},
	{
		.name = "SECUR",
		.command = DTECommand::SECUR_RESP,
		.prototype = 
		{
		}
	},
	{
		.name = "DUMPM",
		.command = DTECommand::DUMPM_RESP,
		.prototype = 
		{
			{
				.name = "data",
				.key = "",
				.encoding = BaseEncoding::BASE64,
				.min_value = 0,
				.max_value = 0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	{
		.name = "DUMPD",
		.command = DTECommand::DUMPD_RESP,
		.prototype = 
		{
			{
				.name = "mmm",
				.key = "",
				.encoding = BaseEncoding::HEXADECIMAL,
				.min_value = 0U,
				.max_value = 0xFFFU,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "MMM",
				.key = "",
				.encoding = BaseEncoding::HEXADECIMAL,
				.min_value = 0U,
				.max_value = 0xFFFU,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "data",
				.key = "",
				.encoding = BaseEncoding::BASE64,
				.min_value = 0,
				.max_value = 0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	{
		.name = "RSTVW",
		.command = DTECommand::RSTVW_RESP,
		.prototype =
		{
		}
	},
	{
		.name = "RSTBW",
		.command = DTECommand::RSTBW_RESP,
		.prototype = 
		{
		}
	},
	{
		.name = "FACTW",
		.command = DTECommand::FACTW_RESP,
		.prototype = 
		{
		}
	},
	{
		.name = "STATR",
		.command = DTECommand::STATR_RESP,
		.prototype =
		{
			{
				.name = "key_values",
				.key = "",
				.encoding = BaseEncoding::KEY_VALUE_LIST,
				.min_value = 0,
				.max_value = 0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	{
		.name = "ERASE",
		.command = DTECommand::ERASE_RESP,
		.prototype =
		{
		}
	},
	{
		.name = "SCALW",
		.command = DTECommand::SCALW_RESP,
		.prototype =
		{
		}
	},
	{
		.name = "SATTX",
		.command = DTECommand::ARGOSTX_RESP,
		.prototype =
		{
		}
	},
	{
		.name = "SCALR",
		.command = DTECommand::SCALR_RESP,
		.prototype =
		{
			{
				.name = "value",
				.key = "",
				.encoding = BaseEncoding::FLOAT,
				.min_value = 0.0,
				.max_value = 0.0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	// SENSR response - sensor readings
	{
		.name = "SENSR",
		.command = DTECommand::SENSR_RESP,
		.prototype =
		{
			{
				.name = "batt_mv",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "batt_soc",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 100U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "pressure",
				.key = "",
				.encoding = BaseEncoding::FLOAT,
				.min_value = 0.0,
				.max_value = 0.0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "temperature",
				.key = "",
				.encoding = BaseEncoding::FLOAT,
				.min_value = 0.0,
				.max_value = 0.0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "altitude",
				.key = "",
				.encoding = BaseEncoding::FLOAT,
				.min_value = 0.0,
				.max_value = 0.0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "lat",
				.key = "",
				.encoding = BaseEncoding::FLOAT,
				.min_value = 0.0,
				.max_value = 0.0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "lon",
				.key = "",
				.encoding = BaseEncoding::FLOAT,
				.min_value = 0.0,
				.max_value = 0.0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "hdop",
				.key = "",
				.encoding = BaseEncoding::FLOAT,
				.min_value = 0.0,
				.max_value = 0.0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "num_sv",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "accel_x",
				.key = "",
				.encoding = BaseEncoding::FLOAT,
				.min_value = 0.0,
				.max_value = 0.0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "accel_y",
				.key = "",
				.encoding = BaseEncoding::FLOAT,
				.min_value = 0.0,
				.max_value = 0.0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "accel_z",
				.key = "",
				.encoding = BaseEncoding::FLOAT,
				.min_value = 0.0,
				.max_value = 0.0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "accel_temp",
				.key = "",
				.encoding = BaseEncoding::FLOAT,
				.min_value = 0.0,
				.max_value = 0.0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "activity",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 255U,  // Activity level 0-255
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "thermistor_temp",
				.key = "",
				.encoding = BaseEncoding::FLOAT,
				.min_value = 0.0,
				.max_value = 0.0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
			.name = "sea_temp",
			.key = "",
			.encoding = BaseEncoding::FLOAT,
			.min_value = 0.0,
			.max_value = 0.0,
			.permitted_values = {},
			.is_implemented = false,
			.is_writable = false
		},
		{
			.name = "als_lux",
			.key = "",
			.encoding = BaseEncoding::FLOAT,
			.min_value = 0.0,
			.max_value = 0.0,
			.permitted_values = {},
			.is_implemented = false,
			.is_writable = false
		},
		{
			.name = "ph",
			.key = "",
			.encoding = BaseEncoding::FLOAT,
			.min_value = 0.0,
			.max_value = 0.0,
			.permitted_values = {},
			.is_implemented = false,
			.is_writable = false
		},
		{
			.name = "sensor_status",
			.key = "",
			.encoding = BaseEncoding::UINT,
			.min_value = 0U,
			.max_value = 0xFFU,
			.permitted_values = {},
			.is_implemented = false,
			.is_writable = false
		}
	}
	},
	// PWRON response - simple acknowledgement
	{
		.name = "PWRON",
		.command = DTECommand::PWRON_RESP,
		.prototype = {}
	},
	// SWSST response - SWS calibration status values
	{
		.name = "SWSST",
		.command = DTECommand::SWSST_RESP,
		.prototype =
		{
			{
				.name = "air",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "water",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "threshold",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "hysteresis",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "raw_adc",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "filtered_adc",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "calibrated",
				.key = "",
				.encoding = BaseEncoding::BOOLEAN,
				.min_value = 0U,
				.max_value = 1U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "underwater",
				.key = "",
				.encoding = BaseEncoding::BOOLEAN,
				.min_value = 0U,
				.max_value = 1U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "time_in_state",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "surface_level",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 5U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "contrast_x10",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "observed_peak",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "sample_delay_us",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	// SATDP response - simple OK/error acknowledgement
	{
		.name = "SATDP",
		.command = DTECommand::SATDP_RESP,
		.prototype = {}
	},
	// GNSSI response - unique ID, SW version, HW version
	{
		.name = "GNSSI",
		.command = DTECommand::GNSSI_RESP,
		.prototype =
		{
			{
				.name = "unique_id",
				.key = "",
				.encoding = BaseEncoding::TEXT,
				.min_value = "",
				.max_value = "",
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "sw_version",
				.key = "",
				.encoding = BaseEncoding::TEXT,
				.min_value = "",
				.max_value = "",
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "hw_version",
				.key = "",
				.encoding = BaseEncoding::TEXT,
				.min_value = "",
				.max_value = "",
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	// GNSSA response - almanac file status
	{
		.name = "GNSSA",
		.command = DTECommand::GNSSA_RESP,
		.prototype =
		{
			{
				.name = "present",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 1U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "file_size",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "total_records",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "valid_records",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "stale",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 1U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	// RTCW response - simple acknowledgement
	{
		.name = "RTCW",
		.command = DTECommand::RTCW_RESP,
		.prototype = {}
	},
	// SWSTST response - SWS test mode running state
	{
		.name = "SWSTST",
		.command = DTECommand::SWSTST_RESP,
		.prototype =
		{
			{
				.name = "running",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 1U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	// SWSCAL response - guided calibration result
	{
		.name = "SWSCAL",
		.command = DTECommand::SWSCAL_RESP,
		.prototype =
		{
			{
				.name = "status",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 3U,  // 0=in progress, 1=success, 2=failed, 3=cancelled
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "air",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 16383U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "water",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 16383U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	// GNSSBR response - simple acknowledgement
	{
		.name = "GNSSBR",
		.command = DTECommand::GNSSBR_RESP,
		.prototype = {}
	},
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	// SMD DFU response
	// Response: $SMDDFU,<status>,<dfu_mode>,<progress>[,<info>]*<checksum>\r\n
	{
		.name = "SMDDFU",
		.command = DTECommand::SMDDFU_RESP,
		.prototype =
		{
			{
				.name = "status",
				.key = "",
				.encoding = BaseEncoding::UINT,  // DFU response status code
				.min_value = 0U,
				.max_value = 0xFFU,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "dfu_mode",
				.key = "",
				.encoding = BaseEncoding::BOOLEAN,  // True if in DFU mode
				.min_value = 0U,
				.max_value = 1U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "progress",
				.key = "",
				.encoding = BaseEncoding::UINT,  // Progress percentage 0-100
				.min_value = 0U,
				.max_value = 100U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "info",
				.key = "",
				.encoding = BaseEncoding::TEXT,  // Additional info (bootloader version, etc.)
				.min_value = "",
				.max_value = "",
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	// SMDTST response - applicative SPI test results
	{
		.name = "SMDTST",
		.command = DTECommand::SMDTST_RESP,
		.prototype =
		{
			{
				.name = "info",
				.key = "",
				.encoding = BaseEncoding::TEXT,
				.min_value = "",
				.max_value = "",
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			}
		}
	},
	{
		.name = "SMDCD",
		.command = DTECommand::SMDCD_RESP,
		.prototype =
		{
		}
	},
#endif
	{
		.name = "SATVF",
		.command = DTECommand::SATVF_RESP,
		.prototype =
		{
		}
	},
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	{
		.name = "LORATX",
		.command = DTECommand::LORATX_RESP,
		.prototype = {}
	},
	{
		.name = "LORABR",
		.command = DTECommand::LORABR_RESP,
		.prototype = {}
	},
#endif
};

const size_t command_map_size = sizeof(command_map) / sizeof(command_map[0]);
