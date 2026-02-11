#pragma once

#include "base_types.hpp"


#define RESP_CMD_BASE   0x80

enum class DTECommand {
	PARML_REQ,
	PARMR_REQ,
	PARMW_REQ,
	PROFR_REQ,
	PROFW_REQ,
	PASPW_REQ,
	SECUR_REQ,
	DUMPM_REQ,
	DUMPD_REQ,
	RSTVW_REQ,
	RSTBW_REQ,
	FACTW_REQ,
	STATR_REQ,
	ERASE_REQ,
	SCALW_REQ,
	ARGOSTX_REQ,
	SCALR_REQ,
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	SMDDFU_REQ,    // SMD satellite module DFU command
#endif
	__NUM_REQ,
	PARML_RESP = RESP_CMD_BASE,
	PARMR_RESP,
	PARMW_RESP,
	PROFR_RESP,
	PROFW_RESP,
	PASPW_RESP,
	SECUR_RESP,
	DUMPM_RESP,
	DUMPD_RESP,
	RSTVW_RESP,
	RSTBW_RESP,
	FACTW_RESP,
	STATR_RESP,
	ERASE_RESP,
	SCALW_RESP,
	ARGOSTX_RESP,
	SCALR_RESP,
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	SMDDFU_RESP,   // SMD satellite module DFU response
#endif
	__NUM_RESP
};

// SMD DFU action types (for SMDDFU command)
enum class SmdDfuAction {
	ENTER = 0,     // Enter DFU mode
	EXIT = 1,      // Exit DFU mode (jump to application)
	STATUS = 2,    // Get DFU status
	UPDATE = 3,    // Perform firmware update (requires firmware data)
	INFO = 4,      // Get bootloader info
	VERSION = 5    // Get application firmware version
};

struct DTECommandMap {
	std::string name;
	DTECommand  command;
	std::vector<BaseMap> prototype;
};

extern const DTECommandMap command_map[];
extern const size_t command_map_size;
