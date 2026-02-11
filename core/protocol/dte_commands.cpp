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
				.max_value = 10U,  // Updated: includes THERMISTOR_SENSOR(9), TSYS01_SENSOR(10)
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
					.permitted_values = { 1U, 3U, 4U },
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
				.max_value = 12U,  // Updated: includes THERMISTOR_SENSOR(11), TSYS01_SENSOR(12)
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
				.name = "power",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "freq",
				.key = "",
				.encoding = BaseEncoding::FLOAT,
				.min_value = 0.0,
				.max_value = 0.0,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "size",
				.key = "",
				.encoding = BaseEncoding::UINT,
				.min_value = 0U,
				.max_value = 0U,
				.permitted_values = {},
				.is_implemented = false,
				.is_writable = false
			},
			{
				.name = "tcxo_warmup",
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
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	// SMD DFU command - allows firmware update of SMD satellite module
	// Usage: $SMDDFU,<action>[,<firmware_data_base64>]*<checksum>\r\n
	// Actions: 0=enter, 1=exit, 2=status, 3=update, 4=info, 5=version
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
			},
			{
				.name = "firmware",
				.key = "",
				.encoding = BaseEncoding::BASE64,  // Optional: firmware binary in base64
				.min_value = 0,
				.max_value = 0,
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
#endif
};

const size_t command_map_size = sizeof(command_map) / sizeof(command_map[0]);
