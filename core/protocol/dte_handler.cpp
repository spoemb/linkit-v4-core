#include "dte_handler.hpp"
#include "argos_tx_service.hpp"
#include "scheduler.hpp"
#include "pmu.hpp"
#include "rgb_led.hpp"
#include <cmath>

extern RGBLed *status_led;

extern Scheduler *system_scheduler;

DTEHandler::DTEHandler() {
	m_dumpd_NNN = 0;
	m_dumpd_mmm = 0;
	m_dumpd_d_type = 0xFFFFFFFF;  // Invalid initial value
	m_sat_device_active = false;
	m_lora_tx_active = false;
	m_doppler_cal_active = false;
	m_doppler_cal_first_tx = false;
	m_gnssi_pending = false;
	m_gps_subscribed = false;
}

DTEHandler::~DTEHandler() {}

void DTEHandler::set_async_write(std::function<void(const std::string&)> fn) { m_async_write = fn; }

void DTEHandler::reset_state() {
	m_dumpd_NNN = 0;
	m_dumpd_mmm = 0;
	m_dumpd_d_type = 0xFFFFFFFF;
}

// Fix #37: Shared helper for PARMR_REQ and STATR_REQ
std::string DTEHandler::read_params_by_filter(int error_code, std::vector<ParamID>& params, char filter_char, DTECommand resp_cmd) {

	if (error_code) {
		return DTEEncoder::encode(resp_cmd, error_code);
	}

	// Check special case where params is zero length => retrieve all matching key types
	if (params.size() == 0) {
		for (unsigned int i = 0; i < param_map_size; i++) {
			if (param_map[i].is_implemented &&
				param_map[i].key[2] == filter_char)
				params.push_back((ParamID)i);
		}
	}

	std::vector<ParamValue> param_values;
	for (unsigned int i = 0; i < params.size(); i++) {
		BaseType x = configuration_store->read_param<BaseType>(params[i]);
		ParamValue p = {
			params[i],
			x
		};
		param_values.push_back(p);
	}

	return DTEEncoder::encode(resp_cmd, param_values);
}

#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
// Fix #38: Shared helper for SMD null-check pattern
std::string DTEHandler::smd_not_available_error(const char* func_name, DTECommand resp_cmd) {
	DEBUG_ERROR("DTEHandler::%s: SMD satellite instance not available", func_name);
	return DTEEncoder::encode(resp_cmd, (int)DTEError::INCORRECT_DATA);
}
#endif

std::string DTEHandler::PARML_REQ(int error_code) {

	if (error_code) {
		return DTEEncoder::encode(DTECommand::PARML_RESP, error_code);
	}

	// Build up a list of all implemented parameters
	std::vector<ParamID> params;
	for (unsigned int i = 0; i < param_map_size; i++) {
		if (param_map[i].is_implemented) {
			params.push_back(static_cast<ParamID>(i));
		}
	}

	return DTEEncoder::encode(DTECommand::PARML_RESP, params);
}

std::string DTEHandler::PARMW_REQ(int error_code, std::vector<ParamValue>& param_values, DTEAction& action) {

	if (error_code) {
		return DTEEncoder::encode(DTECommand::PARMW_RESP, error_code);
	}

	for (unsigned int i = 0; i < param_values.size(); i++) {
		if (param_map[(int)param_values[i].param].is_writable)
			configuration_store->write_param(param_values[i].param, param_values[i].value);
		else
			DEBUG_WARN("DTEHandler::PARMW_REQ: not writing read-only attribute %s", param_map[(int)param_values[i].param].name.c_str());
	}

	// Save all the parameters
	configuration_store->save_params();

	// Notify configuration updated action
	action = DTEAction::CONFIG_UPDATED;

	return DTEEncoder::encode(DTECommand::PARMW_RESP, DTEError::OK);
}

std::string DTEHandler::PARMR_REQ(int error_code, std::vector<ParamID>& params) {
	return read_params_by_filter(error_code, params, 'P', DTECommand::PARMR_RESP);
}

std::string DTEHandler::STATR_REQ(int error_code, std::vector<ParamID>& params) {
	// Refresh live RTC value before reading status params
	if (rtc) {
		configuration_store->write_param(ParamID::RTC_CURRENT_TIME, static_cast<unsigned int>(rtc->gettime()));
	}
	return read_params_by_filter(error_code, params, 'T', DTECommand::STATR_RESP);
}

std::string DTEHandler::PROFW_REQ(int error_code, std::vector<BaseType>& arg_list) {

	if (!error_code) {
		if (arg_list.size() < 1)
			return DTEEncoder::encode(DTECommand::PROFW_RESP, (int)DTEError::MISSING_ARGUMENT);
		configuration_store->write_param(ParamID::PROFILE_NAME, arg_list[0]);
	}

	return DTEEncoder::encode(DTECommand::PROFW_RESP, error_code);
}

std::string DTEHandler::PROFR_REQ(int error_code) {

	if (error_code) {
		return DTEEncoder::encode(DTECommand::PROFR_RESP, error_code);
	}

	return DTEEncoder::encode(DTECommand::PROFR_RESP, error_code, configuration_store->read_param<std::string>(ParamID::PROFILE_NAME));
}

std::string DTEHandler::SECUR_REQ(int error_code, std::vector<BaseType>& arg_list) {
	// Default access code - can be changed or made configurable
	// Using ARGOS ID as access code for device-specific security
	static constexpr unsigned int SECUR_ACCESS_CODE = 0x12345678;

	if (error_code) {
		return DTEEncoder::encode(DTECommand::SECUR_RESP, error_code);
	}

	// Validate access code
	if (arg_list.empty()) {
		return DTEEncoder::encode(DTECommand::SECUR_RESP, (int)DTEError::MISSING_ARGUMENT);
	}

	unsigned int provided_code = std::get<unsigned int>(arg_list[0]);

	// Read device ARGOS ID to use as dynamic access code
	unsigned int device_argos_id = configuration_store->read_param<unsigned int>(ParamID::ARGOS_DECID);

	// Accept either the static code or the device's ARGOS ID as valid access codes
	if (provided_code != SECUR_ACCESS_CODE && provided_code != device_argos_id) {
		DEBUG_WARN("DTEHandler::SECUR_REQ: Invalid access code provided (0x%08X)", provided_code);
		return DTEEncoder::encode(DTECommand::SECUR_RESP, (int)DTEError::INVALID_ACCESS_CODE);
	}

	DEBUG_INFO("DTEHandler::SECUR_REQ: Access granted");
	return DTEEncoder::encode(DTECommand::SECUR_RESP, (int)DTEError::OK);
}

std::string DTEHandler::RSTVW_REQ(int error_code, std::vector<BaseType>& arg_list) {

	if (error_code) {
		return DTEEncoder::encode(DTECommand::RSTVW_RESP, error_code);
	}

	if (arg_list.size() < 1)
		return DTEEncoder::encode(DTECommand::RSTVW_RESP, (int)DTEError::MISSING_ARGUMENT);

	unsigned int variable_id = std::get<unsigned int>(arg_list[0]);
	unsigned int zero = 0;

	if (variable_id == 1) {
		// TX_COUNTER
		configuration_store->write_param(ParamID::TX_COUNTER, zero);
	} else if (variable_id == 2) {
		// BOOT_COUNTER
#ifdef EXTERNAL_WAKEUP
		configuration_store->write_param(ParamID::BOOT_COUNTER, zero);
#else
		error_code = (int)DTEError::INCORRECT_DATA;
#endif
	} else if (variable_id == 3) {
		// RX_COUNTER
		configuration_store->write_param(ParamID::ARGOS_RX_COUNTER, zero);
	} else if (variable_id == 4) {
		// RX_TIME
		configuration_store->write_param(ParamID::ARGOS_RX_TIME, zero);
	} else {
		// Invalid variable ID
		error_code = (int)DTEError::INCORRECT_DATA;
	}

	if (!error_code) {
		configuration_store->save_params();
	}

	return DTEEncoder::encode(DTECommand::RSTVW_RESP, error_code);
}

std::string DTEHandler::RSTBW_REQ(int error_code) {

	return DTEEncoder::encode(DTECommand::RSTBW_RESP, error_code);

}

std::string DTEHandler::FACTW_REQ(int error_code) {

	return DTEEncoder::encode(DTECommand::FACTW_RESP, error_code);

}

std::string DTEHandler::DUMPM_REQ(int error_code, std::vector<BaseType>& arg_list) {

	if (error_code) {
		return DTEEncoder::encode(DTECommand::DUMPM_RESP, error_code);
	}

	if (!memory_access) {
		return DTEEncoder::encode(DTECommand::DUMPM_RESP, (int)DTEError::INCORRECT_DATA);
	}

	if (arg_list.size() < 2)
		return DTEEncoder::encode(DTECommand::DUMPM_RESP, (int)DTEError::MISSING_ARGUMENT);

	unsigned int address = std::get<unsigned int>(arg_list[0]);
	unsigned int length = std::get<unsigned int>(arg_list[1]);
	BaseRawData raw = {
		.ptr = memory_access->get_physical_address(address, length),
		.length = length,
		.str = ""
	};

	return DTEEncoder::encode(DTECommand::DUMPM_RESP, error_code, raw);
}

std::string DTEHandler::PASPW_REQ(int error_code, std::vector<BaseType>& arg_list) {

	while (!error_code) {
		BasePassPredict pass_predict;
		std::string paspw_bits = std::get<std::string>(arg_list[0]);
		try {
			PassPredictCodec::decode(paspw_bits, pass_predict);
		} catch (ErrorCode e) {
			error_code = (int)DTEError::INCORRECT_DATA;
			break;  // Do not write configuration store
		}

		// If the number of records is zero then return an incorrect data error and flag this
		// back to the user without updating the configuration store
		if (pass_predict.num_records == 0) {
			DEBUG_ERROR("DTEHandler::PASPW_REQ: no AOP records | not updating the config store");
			error_code = (int)DTEError::INCORRECT_DATA;
			break;
		}

		// Scan through all the records and find the entry whose bulletin date
		// is the most recent, skipping any non-operational satellites
		std::time_t argos_aop_date = 0;
		for (unsigned int i = 0; i < pass_predict.num_records; i++)
		{
			if (pass_predict.records[i].bulletin.year) {
				std::time_t t = convert_epochtime(pass_predict.records[i].bulletin.year, pass_predict.records[i].bulletin.month, pass_predict.records[i].bulletin.day, pass_predict.records[i].bulletin.hour, pass_predict.records[i].bulletin.minute, pass_predict.records[i].bulletin.second);
				if (t > argos_aop_date)
					argos_aop_date = t;
			}
		}

		if (argos_aop_date)
		{
			// Update configuration store
			configuration_store->write_pass_predict(pass_predict);

			// Set to most recent AOP bulletin record
			configuration_store->write_param(ParamID::ARGOS_AOP_DATE, argos_aop_date);

			// Save configuration to commit AOPDATE
			configuration_store->save_params();

			// Log that the PASPW has been updated
			struct tm tm_buf;
			auto time = gmtime_r(&argos_aop_date, &tm_buf);
			char buff[256];
			if (time)
				std::strftime(buff, sizeof(buff), "%d/%m/%Y %H:%M:%S", time);
			else
				std::snprintf(buff, sizeof(buff), "(gmtime failed)");
			DEBUG_INFO("DTEHandler:PASPW_REQ: saving PASPW with #AOPs=%u ARGOS_AOP_DATE=%s", (unsigned int)pass_predict.num_records, buff);
			break;
		} else {
			DEBUG_ERROR("DTEHandler::PASPW_REQ: no valid AOP records | not updating the config store");
			error_code = (int)DTEError::INCORRECT_DATA;
			break;
		}
	}

	return DTEEncoder::encode(DTECommand::PASPW_RESP, error_code);
}

std::string DTEHandler::DUMPD_REQ(int error_code, std::vector<BaseType>& arg_list, DTEAction& action) {

	action = DTEAction::NONE;

	// A bit of explanation here.  The protocol for DUMPD sends back a payload format of
	// mmm,MMM,<payload>
	// where mmm is a packet index and MMM is the maximum packet index whose value
	// is NNN-1, where NNN is the total number of packets to send.
	if (error_code) {
		// Reset state variables back to zero if an error arises
		m_dumpd_NNN = 0;
		m_dumpd_mmm = 0;
		m_dumpd_d_type = 0xFFFFFFFF;
		return DTEEncoder::encode(DTECommand::DUMPD_RESP, error_code);
	}

	Logger *logger = nullptr;

	// Extract the d_type parameter from arg_list to determine which log file to use
	unsigned int d_type = std::get<unsigned int>(arg_list[0]);

	// Reset state if dump type changed mid-stream (prevents state leak)
	if (d_type != m_dumpd_d_type) {
		m_dumpd_NNN = 0;
		m_dumpd_mmm = 0;
		m_dumpd_d_type = d_type;
	}

	try {
		logger = LoggerManager::find_by_name(m_logger_dump.at(d_type).c_str());
	} catch (...) {
		// Ignore any exceptions -- the logger will be nullptr
	}

	// Either invalid log file or the logger doesn't exist
	if (logger == nullptr) {
		error_code = (int)DTEError::INCORRECT_DATA;
		return DTEEncoder::encode(DTECommand::DUMPD_RESP, error_code);
	}

	// Get the log formatter
	LogFormatter *formatter;
	formatter = logger->get_log_formatter();

	// Check to see if this is the first item
	unsigned int total_entries = logger->num_entries();
	if (0 == m_dumpd_NNN) {
		m_dumpd_NNN = (total_entries + (DTE_HANDLER_MAX_LOG_DUMP_ENTRIES-1)) / DTE_HANDLER_MAX_LOG_DUMP_ENTRIES;
		// Special case where log file is empty we set NNN to 1 and will send an empty payload
		m_dumpd_NNN = m_dumpd_NNN == 0 ? 1 : m_dumpd_NNN;
		m_dumpd_mmm = 0;
	}

	LogEntry log_entry;
	BaseRawData raw_data;
	unsigned int start_index = m_dumpd_mmm * DTE_HANDLER_MAX_LOG_DUMP_ENTRIES;
	unsigned int num_entries = (start_index < total_entries) ? std::min(total_entries - start_index, DTE_HANDLER_MAX_LOG_DUMP_ENTRIES) : 0;
	raw_data.ptr = nullptr;
	raw_data.length = 0;

	// Set CSV header line if this is the first packet output
	if (0 == m_dumpd_mmm)
		raw_data.str.append(formatter->header());

	for (unsigned int i = 0; i < num_entries; i++) {
		logger->read(&log_entry, i + start_index);
		// Concatenate formatted log entry
		raw_data.str.append(formatter->log_entry(log_entry));
	}

	// Note that MMM=NNN-1
	std::string msg = DTEEncoder::encode(DTECommand::DUMPD_RESP, error_code, m_dumpd_mmm, m_dumpd_NNN - 1, raw_data);

	m_dumpd_mmm++; // Increment in readiness for next iteration
	if (m_dumpd_mmm == m_dumpd_NNN) {
		// Sequence complete - reset all state
		m_dumpd_NNN = 0;
		m_dumpd_mmm = 0;
		m_dumpd_d_type = 0xFFFFFFFF;
	} else {
		action = DTEAction::AGAIN;  // Inform caller that we need another iteration
	}

	return msg;
}

std::string DTEHandler::ERASE_REQ(int error_code, std::vector<BaseType>& arg_list) {

	if (error_code) {
		return DTEEncoder::encode(DTECommand::ERASE_RESP, error_code);
	}

	DEBUG_TRACE("Processing ERASE");

	// Extract the d_type parameter from arg_list to determine which log file(s) to erase
	unsigned int d_type = std::get<unsigned int>(arg_list[0]);

	if (d_type == (unsigned int)BaseEraseType::ALL) {
		// Truncate all loggers
		LoggerManager::truncate();
	} else {
		Logger *logger = nullptr;

		try {
			logger = LoggerManager::find_by_name(m_logger_erase.at(d_type).c_str());
		} catch (...) {
			// Ignore any exceptions -- the logger will be nullptr
		}

		if (logger)
		{
			DEBUG_TRACE("Truncating log %s", logger->get_name());
			logger->truncate();
		}
		else
		{
			error_code = (int)DTEError::INCORRECT_DATA;
		}
	}
	return DTEEncoder::encode(DTECommand::ERASE_RESP, error_code);
}

std::string DTEHandler::SCALW_REQ(int error_code, std::vector<BaseType>& arg_list) {

	if (error_code) {
		return DTEEncoder::encode(DTECommand::SCALW_RESP, error_code);
	}

	if (arg_list.size() < 3)
		return DTEEncoder::encode(DTECommand::SCALW_RESP, (int)DTEError::MISSING_ARGUMENT);

	// Extract the device_id parameter from arg_list to determine which device to calibrate
	unsigned int device_id = std::get<unsigned int>(arg_list[0]);

	// Extract the calibration offset parameter from arg_list to determine which sensor offset to calibrate
	unsigned int offset = std::get<unsigned int>(arg_list[1]);

	// Extract the calibration value parameter from arg_list to use
	double value = std::get<double>(arg_list[2]);

	try {
		const char *name = m_scalx.at(device_id).c_str();
		DEBUG_TRACE("Calibrating device %s...", name);
		Calibratable& s = CalibratableManager::find_by_name(name);
		s.calibration_write(value, offset);
	} catch (...) {
		DEBUG_TRACE("Device calibration failed");
		error_code = (int)DTEError::INCORRECT_DATA;
	}

	return DTEEncoder::encode(DTECommand::SCALW_RESP, error_code);
}

std::string DTEHandler::SCALR_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::SCALR_RESP, error_code);
	}

	if (arg_list.size() < 2)
		return DTEEncoder::encode(DTECommand::SCALR_RESP, (int)DTEError::MISSING_ARGUMENT);

	// Extract the device_id parameter from arg_list to determine which device to calibrate
	unsigned int device_id = std::get<unsigned int>(arg_list[0]);

	// Extract the calibration offset parameter from arg_list to determine which sensor offset to calibrate
	unsigned int offset = std::get<unsigned int>(arg_list[1]);

	double value;

	try {
		const char *name = m_scalx.at(device_id).c_str();
		DEBUG_TRACE("Read device %s calibration setting %u...", name, offset);
		Calibratable& s = CalibratableManager::find_by_name(name);
		s.calibration_read(value, offset);
	} catch (...) {
		DEBUG_TRACE("Device calibration read failed");
		error_code = (int)DTEError::INCORRECT_DATA;
	}

	if (error_code) {
		return DTEEncoder::encode(DTECommand::SCALR_RESP, error_code);
	}

	return DTEEncoder::encode(DTECommand::SCALR_RESP, error_code, value);
}

#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
std::string DTEHandler::SMDDFU_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::SMDDFU_RESP, error_code);
	}

	unsigned int action = std::get<unsigned int>(arg_list[0]);
	unsigned int status = 0;
	bool dfu_mode = false;
	unsigned int progress = 0;
	std::string info = "";

	if (!smd_sat_instance) {
		return smd_not_available_error("SMDDFU_REQ", DTECommand::SMDDFU_RESP);
	}

	switch ((SmdDfuAction)action) {
	case SmdDfuAction::VERSION:
		// Get application firmware version (SMD must NOT be in DFU/bootloader mode)
		{
			// Check if SMD is in DFU mode - cannot read app version in bootloader
			if (smd_sat_instance->is_dfu_mode()) {
				status = 1;
				info = "SMD in DFU mode - exit first";
				break;
			}

			std::string version = smd_sat_instance->get_firmware_version();
			if (version.empty()) {
				status = 1;  // Error
				info = "Failed to read version";
			} else {
				status = 0;  // OK
				info = version;
			}
		}
		break;

	case SmdDfuAction::ENTER:
		if (smd_sat_instance->dfu_enter()) {
			status = 0;
			dfu_mode = true;
		} else {
			status = 1;
		}
		break;

	case SmdDfuAction::EXIT:
		if (smd_sat_instance->dfu_exit()) {
			status = 0;
			dfu_mode = false;
		} else {
			status = 1;
		}
		break;

	case SmdDfuAction::STATUS:
		status = 0;
		dfu_mode = smd_sat_instance->is_dfu_mode();
		break;

	case SmdDfuAction::INFO:
		dfu_mode = smd_sat_instance->is_dfu_mode();
		if (dfu_mode) {
			SmdDfuInfo dfu_info;
			if (smd_sat_instance->dfu_get_bootloader_info(&dfu_info)) {
				status = 0;
				info = "BL v" + std::to_string(dfu_info.version_major) + "." +
				       std::to_string(dfu_info.version_minor);
			} else {
				status = 1;
			}
		} else {
			status = 1;
			info = "Not in DFU mode";
		}
		break;

	case SmdDfuAction::UPDATE:
		// Firmware update is handled via OTA file transfer, not this command
		// This action just triggers the update process using previously uploaded firmware
		status = 1;
		info = "Use OTA to upload firmware first";
		break;

	default:
		return DTEEncoder::encode(DTECommand::SMDDFU_RESP, (int)DTEError::INCORRECT_DATA);
	}

	return DTEEncoder::encode(DTECommand::SMDDFU_RESP, (unsigned int)0, status, dfu_mode, progress, info);
}

std::string DTEHandler::SMDTST_REQ(int error_code, std::vector<BaseType>& arg_list) {
	(void)arg_list;
	if (error_code) {
		return DTEEncoder::encode(DTECommand::SMDTST_RESP, error_code);
	}

	if (!smd_sat_instance) {
		return smd_not_available_error("SMDTST_REQ", DTECommand::SMDTST_RESP);
	}

	std::string test_result = smd_sat_instance->smd_spi_test();
	if (test_result.empty()) {
		return DTEEncoder::encode(DTECommand::SMDTST_RESP, (int)DTEError::INCORRECT_DATA);
	}

	return DTEEncoder::encode(DTECommand::SMDTST_RESP, (unsigned int)0, test_result);
}

std::string DTEHandler::SMDCD_REQ(int error_code, std::vector<BaseType>& arg_list) {

	if (error_code) {
		return DTEEncoder::encode(DTECommand::SMDCD_RESP, error_code);
	}

	// Extract the Argos SMD ID
	unsigned int dec_id = std::get<unsigned int>(arg_list[0]);
	// Extract the Argos SMD ADDR
	unsigned int address = std::get<unsigned int>(arg_list[1]);
	// Extract the Argos SMD Sec key
	std::string seckey = std::get<std::string>(arg_list[2]);
	// Extract the Argos SMD radio conf
	std::string radioconf = std::get<std::string>(arg_list[3]);

	try {
		DEBUG_TRACE("SMDCD_REQ...");

		if (!smd_sat_instance) {
			return smd_not_available_error("SMDCD_REQ", DTECommand::SMDCD_RESP);
		}

		// If not already active then subscribe to events and setup a sufficiently
		// long idle period before the driver shuts off argos power
		if (!m_sat_device_active) {
			smd_sat_instance->subscribe(*this);
			smd_sat_instance->set_idle_timeout(30000);
			m_sat_device_active = true;
		}

		// Write credentials to SMD
		smd_sat_instance->set_credentials(dec_id, address, seckey, radioconf);

		// Read back credentials from SMD to confirm
		smd_sat_instance->read_credentials(&dec_id, &address, &seckey, &radioconf);

		// Update configuration store with confirmed values
		configuration_store->write_param(ParamID::ARGOS_DECID, dec_id);
		configuration_store->write_param(ParamID::ARGOS_HEXID, address);
		configuration_store->write_param(ParamID::ARGOS_SECKEY, seckey);
		configuration_store->write_param(ParamID::ARGOS_RADIOCONF, radioconf);
		configuration_store->save_params();
	} catch (...) {
		error_code = (int)DTEError::INCORRECT_DATA;
	}

	return DTEEncoder::encode(DTECommand::SMDCD_RESP, error_code);
}
#endif

std::string DTEHandler::ARGOSTX_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::ARGOSTX_RESP, error_code);
	}

	if (arg_list.size() < 4) {
		return DTEEncoder::encode(DTECommand::ARGOSTX_RESP, (int)DTEError::MISSING_ARGUMENT);
	}

	// Extract the argos modulation (0=>LDK, 1=>LDA2, 2=>VLDA4)
	KineisModulation modulation = (KineisModulation)std::get<unsigned int>(arg_list[0]);

	// Extract the argos power level in mW
	unsigned int power = std::get<unsigned int>(arg_list[1]);

	// Extract the argos frequency (double)
	double freq = std::get<double>(arg_list[2]);

	// Extract the payload size in bytes
	unsigned int num_bytes = std::get<unsigned int>(arg_list[3]);

	// Extract the TCXO warmup time in seconds (optional)
	unsigned int tcxo_time = arg_list.size() > 4 ? std::get<unsigned int>(arg_list[4]) : 0;

	DEBUG_INFO("DTEHandler::ARGOSTX_REQ: modulation=%u power=%u freq=%.6f size=%u tcxo=%u",
		(unsigned int)modulation, power, freq, num_bytes, tcxo_time);

	try {
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
		if (!smd_sat_instance) {
			return smd_not_available_error("ARGOSTX_REQ", DTECommand::ARGOSTX_RESP);
		}

		// If not already active then subscribe to events and setup a sufficiently
		// long idle period before the driver shuts off argos power
		if (!m_sat_device_active) {
			smd_sat_instance->subscribe(*this);
			smd_sat_instance->set_idle_timeout(30000);
			m_sat_device_active = true;
		}

		// Configure and transmit
		smd_sat_instance->set_tx_power(power);
		smd_sat_instance->set_tcxo_warmup_time(tcxo_time);
		smd_sat_instance->set_frequency(freq);
		KineisPacket packet(num_bytes, 0xFF);
		smd_sat_instance->send(modulation, packet, 8 * num_bytes);

		// No immediate response — wait for KineisEventTxComplete/DeviceError
		// which will send the async response via react() callbacks
		return {};
#else
		DEBUG_WARN("DTEHandler::ARGOSTX_REQ: No satellite device available");
		return DTEEncoder::encode(DTECommand::ARGOSTX_RESP, (int)DTEError::INCORRECT_DATA);
#endif
	} catch (...) {
		error_code = (int)DTEError::INCORRECT_DATA;
	}

	return DTEEncoder::encode(DTECommand::ARGOSTX_RESP, error_code);
}

#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
std::string DTEHandler::LORATX_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::LORATX_RESP, error_code);
	}

	if (arg_list.size() < 1) {
		return DTEEncoder::encode(DTECommand::LORATX_RESP, (int)DTEError::MISSING_ARGUMENT);
	}

	unsigned int num_bytes = std::get<unsigned int>(arg_list[0]);

	DEBUG_INFO("DTEHandler::LORATX_REQ: size=%u bytes", num_bytes);

	if (!lora_device_instance) {
		DEBUG_WARN("DTEHandler::LORATX_REQ: LoRa device not available");
		return DTEEncoder::encode(DTECommand::LORATX_RESP, (int)DTEError::INCORRECT_DATA);
	}

	try {
		if (!m_sat_device_active) {
			lora_device_instance->subscribe(*this);
			m_sat_device_active = true;
		}
		m_lora_tx_active = true;

		// Build test payload (all 0xFF bytes)
		KineisPacket packet(num_bytes, 0xFF);
		lora_device_instance->send(KineisModulation::LDA2, packet, 8 * num_bytes);

		// Async response — wait for KineisEventTxComplete/DeviceError
		return {};
	} catch (...) {
		m_lora_tx_active = false;
		error_code = (int)DTEError::INCORRECT_DATA;
	}

	return DTEEncoder::encode(DTECommand::LORATX_RESP, error_code);
}

std::string DTEHandler::LORABR_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::LORABR_RESP, error_code);
	}

	if (arg_list.size() < 1) {
		return DTEEncoder::encode(DTECommand::LORABR_RESP, (int)DTEError::MISSING_ARGUMENT);
	}

	unsigned int action = std::get<unsigned int>(arg_list[0]);

	if (!lora_device_instance) {
		DEBUG_WARN("DTEHandler::LORABR_REQ: LoRa device not available");
		return DTEEncoder::encode(DTECommand::LORABR_RESP, (int)DTEError::INCORRECT_DATA);
	}

	if (action == 1) {
		// Start bridge: raw UART RX → USB CDC via async_write
		auto write_fn = m_async_write;
		lora_device_instance->start_bridge([write_fn](const uint8_t* data, size_t len) {
			if (write_fn) {
				write_fn(std::string(reinterpret_cast<const char*>(data), len));
			}
		});
		DEBUG_INFO("DTEHandler::LORABR_REQ: bridge STARTED — type +++ to exit");
	} else {
		lora_device_instance->stop_bridge();
		DEBUG_INFO("DTEHandler::LORABR_REQ: bridge STOPPED");
	}

	return DTEEncoder::encode(DTECommand::LORABR_RESP, (int)DTEError::OK);
}
#endif


std::string DTEHandler::SENSR_REQ(int error_code, std::vector<BaseType>& arg_list) {
	// Response values (initialized to defaults/invalid)
	unsigned int batt_mv = 0;
	unsigned int batt_soc = 0;
	double pressure = 0.0;
	double temperature = 0.0;
	double altitude = 0.0;
	double lat = 0.0;
	double lon = 0.0;
	double hdop = 99.9;
	unsigned int num_sv = 0;
	double accel_x = 0.0;
	double accel_y = 0.0;
	double accel_z = 0.0;
	double accel_temp = 0.0;
	unsigned int activity = 0;
	double thermistor_temp = 0.0;

	if (error_code) {
		return DTEEncoder::encode(DTECommand::SENSR_RESP, error_code,
			batt_mv, batt_soc, pressure, temperature, altitude, lat, lon, hdop, num_sv,
			accel_x, accel_y, accel_z, accel_temp, activity, thermistor_temp);
	}

	// Parse parameters
	if (arg_list.size() < 2) {
		return DTEEncoder::encode(DTECommand::SENSR_RESP, (int)DTEError::MISSING_ARGUMENT,
			batt_mv, batt_soc, pressure, temperature, altitude, lat, lon, hdop, num_sv,
			accel_x, accel_y, accel_z, accel_temp, activity, thermistor_temp);
	}

	unsigned int sensors_mask = std::get<unsigned int>(arg_list[0]);
	unsigned int timeout_s = std::get<unsigned int>(arg_list[1]);
	(void)timeout_s;  // Currently unused - for future GNSS acquisition

	DEBUG_INFO("DTEHandler::SENSR_REQ: sensors_mask=0x%02X timeout=%us", sensors_mask, timeout_s);

	// Read battery (SensrType::BATTERY = 0x01)
	if (sensors_mask & 0x01) {
		if (battery_monitor) {
			battery_monitor->update();  // Refresh readings
			batt_mv = battery_monitor->get_voltage();
			batt_soc = battery_monitor->get_level();
			DEBUG_TRACE("SENSR: Battery %umV | %u%%", batt_mv, batt_soc);
		} else {
			DEBUG_WARN("SENSR: Battery monitor not available");
		}
	}

	// Read pressure sensor (SensrType::PRESSURE = 0x02)
	if (sensors_mask & 0x02) {
		try {
			Sensor& prs = SensorManager::find_by_name("PRS");
			pressure = prs.read(0);      // Channel 0 = pressure (bar)
			temperature = prs.read(1);   // Channel 1 = temperature (C)
			// Compute barometric altitude: altitude = 44330 * (1 - (P/P0)^(1/5.255))
			double sea_level_hpa = 1013.25;
			prs.calibration_read(sea_level_hpa, 0);
			double pressure_hpa = pressure * 1000.0;
			if (sea_level_hpa > 0.0 && pressure_hpa > 0.0) {
				altitude = 44330.0 * (1.0 - std::pow(pressure_hpa / sea_level_hpa, 1.0 / 5.255));
			}
			DEBUG_TRACE("SENSR: Pressure %.4f bar | Temp %.2f C | Altitude %.2f m", pressure, temperature, altitude);
		} catch (...) {
			DEBUG_WARN("SENSR: Pressure sensor not available");
			// Leave pressure/temperature/altitude at defaults (0.0)
		}
	}

	// Read GNSS (SensrType::GNSS = 0x04)
	// Note: Currently returns cached last known position, not live acquisition
	if (sensors_mask & 0x04) {
		const GPSLogEntry& gps_entry = configuration_store->get_last_gps_entry();
		if (gps_entry.info.valid) {
			lat = gps_entry.info.lat;
			lon = gps_entry.info.lon;
			hdop = gps_entry.info.hDOP;
			num_sv = gps_entry.info.numSV;
			DEBUG_TRACE("SENSR: GNSS lat=%.6f lon=%.6f hdop=%.1f numSV=%u",
				lat, lon, hdop, num_sv);
		} else {
			DEBUG_WARN("SENSR: No valid GNSS fix available (returning cached/invalid)");
			// Leave at defaults - hdop=99.9 indicates no fix
		}
	}

	// Read accelerometer (SensrType::ACCEL = 0x08)
#if ENABLE_AXL_SENSOR
	if (sensors_mask & 0x08) {
		try {
			Sensor& axl = SensorManager::find_by_name("AXL");
			// Channel 1 triggers read_xyz and returns X
			accel_x = axl.read(1);  // X axis (g-force)
			accel_y = axl.read(2);  // Y axis (g-force, cached)
			accel_z = axl.read(3);  // Z axis (g-force, cached)
			accel_temp = axl.read(0);  // Temperature (°C, already converted by BMA400)
			activity = (unsigned int)axl.read(4);  // Activity level (0-255)
			DEBUG_TRACE("SENSR: Accel X=%.3f Y=%.3f Z=%.3f T=%.1f C activity=%u",
				accel_x, accel_y, accel_z, accel_temp, activity);
		} catch (...) {
			DEBUG_WARN("SENSR: Accelerometer not available");
			// Leave accel values at defaults (0.0)
		}
	}
#else
	(void)(sensors_mask & 0x08);  // Suppress unused warning
#endif

	// Read thermistor (SensrType::THERMISTOR = 0x10)
#if ENABLE_THERMISTOR_SENSOR
	if (sensors_mask & 0x10) {
		try {
			Sensor& therm = SensorManager::find_by_name("THERMISTOR");
			thermistor_temp = therm.read(0);  // Channel 0 = temperature (°C)
			DEBUG_TRACE("SENSR: Thermistor %.2f C", thermistor_temp);
		} catch (...) {
			DEBUG_WARN("SENSR: Thermistor not available");
		}
	}
#else
	(void)(sensors_mask & 0x10);  // Suppress unused warning
#endif

	return DTEEncoder::encode(DTECommand::SENSR_RESP, (int)DTEError::OK,
		batt_mv, batt_soc, pressure, temperature, altitude, lat, lon, hdop, num_sv,
		accel_x, accel_y, accel_z, accel_temp, activity, thermistor_temp);
}

// PWRON handler - Power on/off components
std::string DTEHandler::PWRON_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::PWRON_RESP, error_code);
	}

	// Parse component parameter
	if (arg_list.size() < 1) {
		return DTEEncoder::encode(DTECommand::PWRON_RESP, (int)DTEError::MISSING_ARGUMENT);
	}

	unsigned int component = std::get<unsigned int>(arg_list[0]);
	DEBUG_INFO("DTEHandler::PWRON_REQ: component=%u", component);

	switch ((ComponentPower)component) {
	case ComponentPower::ALL:
		// Power on all components via drivers
		DEBUG_TRACE("PWRON: Powering ON all components");
		if (gps_device) {
			GPSNavSettings nav = {};
			gps_device->power_on(nav);
		}
		GPIOPins::acquire_sensors_pwr();
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
		GPIOPins::set(SAT_PWR_EN);
#elif defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
		GPIOPins::set(SAT_PWR_EN);
#endif
		break;

	case ComponentPower::GNSS:
		// Power on GNSS via M10Q driver (triggers full state machine: configure, etc.)
		DEBUG_TRACE("PWRON: Powering ON GNSS");
		if (gps_device) {
			GPSNavSettings nav = {};
			gps_device->power_on(nav);
		} else {
			return DTEEncoder::encode(DTECommand::PWRON_RESP, (int)DTEError::INCORRECT_DATA);
		}
		break;

	case ComponentPower::SENSORS:
		// Power on sensors
		DEBUG_TRACE("PWRON: Powering ON sensors");
		GPIOPins::acquire_sensors_pwr();
		break;

	case ComponentPower::SATELLITE:
		// Power on satellite module (requires VSENSORS for stable power rail)
		DEBUG_TRACE("PWRON: Powering ON satellite module");
		GPIOPins::acquire_sensors_pwr();
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
		GPIOPins::set(SAT_PWR_EN);
#elif defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
		GPIOPins::set(SAT_PWR_EN);
#endif
		break;

	case ComponentPower::OFF:
		// Power off all components via drivers
		DEBUG_TRACE("PWRON: Powering OFF all components");
		if (gps_device) {
			gps_device->power_off();
		}
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
		GPIOPins::clear(SAT_PWR_EN);
#elif defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
		GPIOPins::clear(SAT_PWR_EN);
#endif
		GPIOPins::release_sensors_pwr();
		break;

	default:
		return DTEEncoder::encode(DTECommand::PWRON_RESP, (int)DTEError::VALUE_OUT_OF_RANGE);
	}

	return DTEEncoder::encode(DTECommand::PWRON_RESP, (int)DTEError::OK);
}

std::string DTEHandler::SWSST_REQ(int error_code) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::SWSST_RESP, error_code);
	}

#ifdef SWS_ADC
	auto st = SWSAnalogService::get_status();

	return DTEEncoder::encode(DTECommand::SWSST_RESP, (int)DTEError::OK,
		(unsigned int)st.threshold_air,
		(unsigned int)st.threshold_water,
		(unsigned int)st.threshold_current,
		(unsigned int)st.hysteresis,
		(unsigned int)st.last_raw_adc,
		(unsigned int)st.last_filtered_adc,
		(unsigned int)st.is_calibrated,
		(unsigned int)st.is_underwater,
		(unsigned int)st.time_in_state_sec,
		(unsigned int)st.surface_level,
		(unsigned int)st.contrast_x10,
		(unsigned int)st.observed_peak);
#else
	return DTEEncoder::encode(DTECommand::SWSST_RESP, (int)DTEError::PARAM_KEY_UNRECOGNISED);
#endif
}

std::string DTEHandler::SWSTST_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::SWSTST_RESP, error_code);
	}

	if (arg_list.size() < 1) {
		return DTEEncoder::encode(DTECommand::SWSTST_RESP, (int)DTEError::MISSING_ARGUMENT);
	}

	unsigned int action = std::get<unsigned int>(arg_list[0]);

#ifdef SWS_ADC
	if (action == 1) {
		// Start test mode and register async push callback
		auto write_fn = m_async_write;
		SWSAnalogService::set_status_notify([write_fn](const SWSAnalogService::Status& st) {
			if (write_fn) {
				write_fn(DTEEncoder::encode(DTECommand::SWSST_RESP, (int)DTEError::OK,
					(unsigned int)st.threshold_air,
					(unsigned int)st.threshold_water,
					(unsigned int)st.threshold_current,
					(unsigned int)st.hysteresis,
					(unsigned int)st.last_raw_adc,
					(unsigned int)st.last_filtered_adc,
					(unsigned int)st.is_calibrated,
					(unsigned int)st.is_underwater,
					(unsigned int)st.time_in_state_sec,
					(unsigned int)st.surface_level,
					(unsigned int)st.contrast_x10,
					(unsigned int)st.observed_peak));
			}
		});
		SWSAnalogService::set_on_test_stop([]() {
			// Restore normal LED: flash blue (ConfigNotConnected default)
			if (status_led)
				status_led->flash(RGBLedColor::BLUE);
		});
		SWSAnalogService::start_test_mode();
	} else {
		SWSAnalogService::stop_test_mode();
		SWSAnalogService::clear_status_notify();
		SWSAnalogService::clear_on_test_stop();
	}

	unsigned int running = SWSAnalogService::is_test_running() ? 1U : 0U;
	return DTEEncoder::encode(DTECommand::SWSTST_RESP, (int)DTEError::OK, running);
#else
	(void)action;
	return DTEEncoder::encode(DTECommand::SWSTST_RESP, (int)DTEError::PARAM_KEY_UNRECOGNISED);
#endif
}

std::string DTEHandler::GNSSBR_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::GNSSBR_RESP, error_code);
	}

	if (arg_list.size() < 1) {
		return DTEEncoder::encode(DTECommand::GNSSBR_RESP, (int)DTEError::MISSING_ARGUMENT);
	}

	unsigned int action = std::get<unsigned int>(arg_list[0]);

	if (!gps_device) {
		DEBUG_WARN("DTEHandler::GNSSBR_REQ: GNSS device not available");
		return DTEEncoder::encode(DTECommand::GNSSBR_RESP, (int)DTEError::INCORRECT_DATA);
	}

	if (action == 1) {
		// Start bridge: raw UART RX → USB CDC via async_write
		auto write_fn = m_async_write;
		gps_device->start_bridge([write_fn](const uint8_t* data, size_t len) {
			if (write_fn) {
				write_fn(std::string(reinterpret_cast<const char*>(data), len));
			}
		});
		DEBUG_INFO("DTEHandler::GNSSBR_REQ: bridge STARTED — type +++ to exit");
	} else {
		gps_device->stop_bridge();
		DEBUG_INFO("DTEHandler::GNSSBR_REQ: bridge STOPPED");
	}

	return DTEEncoder::encode(DTECommand::GNSSBR_RESP, (int)DTEError::OK);
}

// Helper to format and return GNSSI response from cached info
static std::string format_gnssi_response(const GNSSDeviceInfo& info) {
	char uid_hex[11];
	snprintf(uid_hex, sizeof(uid_hex), "%02X%02X%02X%02X%02X",
		info.uniqueId[0], info.uniqueId[1], info.uniqueId[2],
		info.uniqueId[3], info.uniqueId[4]);

	return DTEEncoder::encode(DTECommand::GNSSI_RESP, (int)DTEError::OK,
		std::string(uid_hex),
		std::string(info.swVersion),
		std::string(info.hwVersion));
}

std::string DTEHandler::GNSSI_REQ(int error_code) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::GNSSI_RESP, error_code);
	}

	if (!gps_device) {
		return DTEEncoder::encode(DTECommand::GNSSI_RESP, (int)DTEError::INCORRECT_DATA);
	}

	// If info is already cached, return immediately
	auto info = gps_device->get_device_info();
	if (info.valid) {
		return format_gnssi_response(info);
	}

	// Autonomous mode: power on GNSS, wait for device info via async callback
	DEBUG_INFO("DTEHandler::GNSSI_REQ: info not cached, powering on GNSS autonomously");

	if (!m_gps_subscribed) {
		gps_device->subscribe(*this);
		m_gps_subscribed = true;
	}

	m_gnssi_pending = true;
	GNSSConfig gnss_config;
	configuration_store->get_gnss_configuration(gnss_config);
	GPSNavSettings nav = {
		gnss_config.fix_mode,
		gnss_config.dyn_model,
		gnss_config.assistnow_enable,
		gnss_config.assistnow_offline_enable,
		gnss_config.hdop_filter_enable,
		gnss_config.hdop_filter_threshold,
		gnss_config.hacc_filter_enable,
		gnss_config.hacc_filter_threshold,
	};
	gps_device->power_on(nav);

	// No immediate response — GPSEventDeviceInfoReady will trigger async response
	return {};
}

void DTEHandler::react(const GPSEventDeviceInfoReady&) {
	if (!m_gnssi_pending) return;
	m_gnssi_pending = false;

	DEBUG_INFO("DTEHandler::react: GPSEventDeviceInfoReady — sending GNSSI response");

	auto info = gps_device->get_device_info();
	std::string resp;
	if (info.valid) {
		resp = format_gnssi_response(info);
	} else {
		resp = DTEEncoder::encode(DTECommand::GNSSI_RESP, (int)DTEError::INCORRECT_DATA);
	}

	if (m_async_write) {
		m_async_write(resp);
	}

	// Power off GNSS now that we have the info
	gps_device->power_off();
}

void DTEHandler::react(const GPSEventError&) {
	if (!m_gnssi_pending) return;
	m_gnssi_pending = false;

	DEBUG_WARN("DTEHandler::react: GPSEventError — GNSSI failed");

	if (m_async_write) {
		m_async_write(DTEEncoder::encode(DTECommand::GNSSI_RESP, (int)DTEError::INCORRECT_DATA));
	}

	// Power off GNSS to reset state machine back to idle
	gps_device->power_off();
}

std::string DTEHandler::GNSSA_REQ(int error_code) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::GNSSA_RESP, error_code);
	}

	if (!gps_device) {
		return DTEEncoder::encode(DTECommand::GNSSA_RESP, (int)DTEError::INCORRECT_DATA);
	}

	auto status = gps_device->get_almanac_status();

	return DTEEncoder::encode(DTECommand::GNSSA_RESP, (int)DTEError::OK,
		(unsigned int)(status.file_present ? 1 : 0),
		(unsigned int)status.file_size,
		(unsigned int)status.total_records,
		(unsigned int)status.valid_records,
		(unsigned int)(status.stale ? 1 : 0));
}

std::string DTEHandler::RTCW_REQ(int error_code, std::vector<BaseType>& arg_list) {
	if (error_code) {
		return DTEEncoder::encode(DTECommand::RTCW_RESP, error_code);
	}

	if (arg_list.size() < 1) {
		return DTEEncoder::encode(DTECommand::RTCW_RESP, (int)DTEError::MISSING_ARGUMENT);
	}

	unsigned int timestamp = std::get<unsigned int>(arg_list[0]);

	if (timestamp == 0) {
		return DTEEncoder::encode(DTECommand::RTCW_RESP, (int)DTEError::VALUE_OUT_OF_RANGE);
	}

	if (!rtc) {
		return DTEEncoder::encode(DTECommand::RTCW_RESP, (int)DTEError::INCORRECT_DATA);
	}

	// Set the RTC to the provided unix timestamp
	rtc->settime(static_cast<std::time_t>(timestamp));
	DEBUG_INFO("DTEHandler::RTCW_REQ: RTC set to %u", timestamp);

#ifdef EXTERNAL_WAKEUP
	// Also update LAST_KNOWN_RTC so the pseudo RTC chain continues from this value
	configuration_store->write_param(ParamID::LAST_KNOWN_RTC, timestamp);
	configuration_store->save_params();
	DEBUG_TRACE("DTEHandler::RTCW_REQ: LAST_KNOWN_RTC updated to %u", timestamp);
#endif

	return DTEEncoder::encode(DTECommand::RTCW_RESP, (int)DTEError::OK);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-enum"

DTEAction DTEHandler::handle_dte_message(const std::string& req, std::string& resp) {
	DTECommand command;
	std::vector<ParamID> params;
	std::vector<ParamValue> param_values;
	std::vector<BaseType> arg_list;
	unsigned int error_code = (unsigned int)DTEError::OK;
	DTEAction action = DTEAction::NONE;

	if (!configuration_store) {
		DEBUG_ERROR("DTEHandler: configuration_store is null");
		return action;
	}

	try {
		if (!DTEDecoder::decode(req, command, error_code, arg_list, params, param_values))
			return action;
	} catch (ErrorCode e) {

		switch (e) {
		case ErrorCode::DTE_PROTOCOL_MESSAGE_TOO_LARGE:
			error_code = (unsigned int)DTEError::MESSAGE_TOO_LARGE;
			break;
		case ErrorCode::DTE_PROTOCOL_PARAM_KEY_UNRECOGNISED:
			error_code = (unsigned int)DTEError::PARAM_KEY_UNRECOGNISED;
			break;
		case ErrorCode::DTE_PROTOCOL_UNEXPECTED_ARG:
			error_code = (unsigned int)DTEError::UNEXPECTED_ARGUMENT;
			break;
		case ErrorCode::DTE_PROTOCOL_VALUE_OUT_OF_RANGE:
			error_code = (unsigned int)DTEError::VALUE_OUT_OF_RANGE;
			break;
		case ErrorCode::DTE_PROTOCOL_MISSING_ARG:
			error_code = (unsigned int)DTEError::MISSING_ARGUMENT;
			break;
		case ErrorCode::DTE_PROTOCOL_BAD_FORMAT:
			error_code = (unsigned int)DTEError::BAD_FORMAT;
			break;
		case ErrorCode::DTE_PROTOCOL_PAYLOAD_LENGTH_MISMATCH:
			error_code = (unsigned int)DTEError::DATA_LENGTH_MISMATCH;
			break;
		case ErrorCode::DTE_PROTOCOL_UNKNOWN_COMMAND:
			error_code = (unsigned int)DTEError::INCORRECT_COMMAND;
			break;
		default:
			DEBUG_ERROR("DTEHandler: unhandled ErrorCode %u from decoder", (unsigned int)e);
			error_code = (unsigned int)DTEError::BAD_FORMAT;
			break;
		}
	}

	try {
	switch(command) {
	case DTECommand::PARML_REQ:
		resp = PARML_REQ(error_code);
		break;
	case DTECommand::PARMW_REQ:
		resp = PARMW_REQ(error_code, param_values, action);
		break;
	case DTECommand::PARMR_REQ:
		resp = PARMR_REQ(error_code, params);
		break;
	case DTECommand::STATR_REQ:
		resp = STATR_REQ(error_code, params);
		break;
	case DTECommand::PROFW_REQ:
		resp = PROFW_REQ(error_code, arg_list);
		break;
	case DTECommand::PROFR_REQ:
		resp = PROFR_REQ(error_code);
		break;
	case DTECommand::SECUR_REQ:
		resp = SECUR_REQ(error_code, arg_list);
		if (!error_code) action = DTEAction::SECUR;
		break;
	case DTECommand::RSTVW_REQ:
		resp = RSTVW_REQ(error_code, arg_list);
		break;
	case DTECommand::RSTBW_REQ:
		resp = RSTBW_REQ(error_code);
		if (!error_code) action = DTEAction::RESET;
		break;
	case DTECommand::FACTW_REQ:
		resp = FACTW_REQ(error_code);
		if (!error_code) action = DTEAction::FACTR;
		break;
	case DTECommand::DUMPM_REQ:
		resp = DUMPM_REQ(error_code, arg_list);
		break;
	case DTECommand::PASPW_REQ:
		resp = PASPW_REQ(error_code, arg_list);
		break;
	case DTECommand::DUMPD_REQ:
		resp = DUMPD_REQ(error_code, arg_list, action);
		break;
	case DTECommand::ERASE_REQ:
		resp = ERASE_REQ(error_code, arg_list);
		break;
	case DTECommand::SCALW_REQ:
		resp = SCALW_REQ(error_code, arg_list);
		break;
	case DTECommand::SCALR_REQ:
		resp = SCALR_REQ(error_code, arg_list);
		break;
	case DTECommand::ARGOSTX_REQ:
		resp = ARGOSTX_REQ(error_code, arg_list);
		break;
	case DTECommand::SENSR_REQ:
		resp = SENSR_REQ(error_code, arg_list);
		break;
	case DTECommand::PWRON_REQ:
		resp = PWRON_REQ(error_code, arg_list);
		break;
	case DTECommand::SWSST_REQ:
		resp = SWSST_REQ(error_code);
		break;
	case DTECommand::SWSTST_REQ:
		resp = SWSTST_REQ(error_code, arg_list);
		break;
	case DTECommand::GNSSBR_REQ:
		resp = GNSSBR_REQ(error_code, arg_list);
		break;
	case DTECommand::GNSSI_REQ:
		resp = GNSSI_REQ(error_code);
		break;
	case DTECommand::GNSSA_REQ:
		resp = GNSSA_REQ(error_code);
		break;
	case DTECommand::RTCW_REQ:
		resp = RTCW_REQ(error_code, arg_list);
		break;
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	case DTECommand::SMDDFU_REQ:
		resp = SMDDFU_REQ(error_code, arg_list);
		break;
	case DTECommand::SMDTST_REQ:
		resp = SMDTST_REQ(error_code, arg_list);
		break;
	case DTECommand::SMDCD_REQ:
		resp = SMDCD_REQ(error_code, arg_list);
		break;
#endif
	case DTECommand::SATDP_REQ:
		resp = SATDP_REQ(error_code, arg_list);
		break;
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
	case DTECommand::LORATX_REQ:
		resp = LORATX_REQ(error_code, arg_list);
		break;
	case DTECommand::LORABR_REQ:
		resp = LORABR_REQ(error_code, arg_list);
		break;
#endif
	default:
		break;
	}
	} catch (...) {
		DEBUG_ERROR("DTEHandler: unexpected exception in command handler");
		resp.clear();
	}

	return action;
}

#pragma GCC diagnostic pop

// KineisEventListener: async TX result notification
void DTEHandler::react(KineisEventTxComplete const& ) {
	DEBUG_INFO("DTEHandler::react: KineisEventTxComplete");
	if (m_doppler_cal_first_tx) {
		// First SATDP TX succeeded — respond OK, then schedule periodic TX
		m_doppler_cal_first_tx = false;
		if (m_async_write)
			m_async_write(DTEEncoder::encode(DTECommand::SATDP_RESP, (int)DTEError::OK));
		schedule_doppler_cal_tx();
	} else if (m_doppler_cal_active) {
		// Periodic Doppler TX completed — schedule next one
		schedule_doppler_cal_tx();
	} else {
		// Regular ARGOSTX/LORATX async response
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
		DTECommand resp_cmd = m_lora_tx_active ? DTECommand::LORATX_RESP : DTECommand::ARGOSTX_RESP;
		m_lora_tx_active = false;
#else
		DTECommand resp_cmd = DTECommand::ARGOSTX_RESP;
#endif
		if (m_async_write) {
			std::string resp = DTEEncoder::encode(resp_cmd, (int)DTEError::OK);
			DEBUG_TRACE("DTEHandler::react: async responding: %s", resp.c_str());
			m_async_write(resp);
		}
	}
}

void DTEHandler::react(KineisEventDeviceError const& ) {
	DEBUG_WARN("DTEHandler::react: KineisEventDeviceError");
	if (m_doppler_cal_first_tx) {
		// First SATDP TX failed — respond error, abort calibration
		m_doppler_cal_active = false;
		m_doppler_cal_first_tx = false;
		if (m_async_write)
			m_async_write(DTEEncoder::encode(DTECommand::SATDP_RESP, (int)DTEError::INCORRECT_DATA));
	} else if (m_doppler_cal_active) {
		// Periodic Doppler TX failed — retry on next cycle
		DEBUG_WARN("DTEHandler: SATDP periodic TX failed | will retry");
		schedule_doppler_cal_tx();
	} else {
		// Regular ARGOSTX/LORATX async error response
#if defined(LORA_RAK3172) && (LORA_RAK3172 == 1)
		DTECommand resp_cmd = m_lora_tx_active ? DTECommand::LORATX_RESP : DTECommand::ARGOSTX_RESP;
		m_lora_tx_active = false;
#else
		DTECommand resp_cmd = DTECommand::ARGOSTX_RESP;
#endif
		if (m_async_write) {
			std::string resp = DTEEncoder::encode(resp_cmd, (int)DTEError::INCORRECT_DATA);
			DEBUG_TRACE("DTEHandler::react: async responding: %s", resp.c_str());
			m_async_write(resp);
		}
	}
}

// KineisEventListener: handle power off to release subscription
void DTEHandler::react(KineisEventPowerOff const& ) {
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	if (m_sat_device_active && !m_doppler_cal_active) {
		m_sat_device_active = false;
		smd_sat_instance->set_idle_timeout(3000);
		smd_sat_instance->unsubscribe(*this);
	}
#endif
}

std::string DTEHandler::SATDP_REQ(int error_code, std::vector<BaseType>& arg_list) {
	(void)arg_list;

	if (error_code) {
		return DTEEncoder::encode(DTECommand::SATDP_RESP, error_code);
	}

	if (m_doppler_cal_active) {
		return DTEEncoder::encode(DTECommand::SATDP_RESP, (int)DTEError::INCORRECT_DATA);
	}

#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	if (!smd_sat_instance) {
		return smd_not_available_error("SATDP_REQ", DTECommand::SATDP_RESP);
	}

	// Subscribe to satellite events if not already active
	if (!m_sat_device_active) {
		smd_sat_instance->subscribe(*this);
		m_sat_device_active = true;
	}
	smd_sat_instance->set_idle_timeout(0);  // Keep satellite powered on indefinitely

	// Build and send the first Doppler packet
	unsigned int size_bits;
	unsigned int batt_mv = battery_monitor ? battery_monitor->get_voltage() : 3700;
	bool is_lb = battery_monitor ? battery_monitor->is_battery_low() : false;
	KineisPacket packet = ArgosPacketBuilder::build_doppler_packet(batt_mv, is_lb, size_bits);

	DEBUG_INFO("DTEHandler::SATDP_REQ: sending first Doppler packet (%u bits)", size_bits);
	smd_sat_instance->send(KineisModulation::LDA2, packet, size_bits);

	m_doppler_cal_active = true;
	m_doppler_cal_first_tx = true;

	// No immediate response — wait for KineisEventTxComplete/DeviceError
	return {};
#else
	DEBUG_WARN("DTEHandler::SATDP_REQ: No satellite device available");
	return DTEEncoder::encode(DTECommand::SATDP_RESP, (int)DTEError::INCORRECT_DATA);
#endif
}

void DTEHandler::schedule_doppler_cal_tx() {
#if defined(ARGOS_SMD) && (ARGOS_SMD == 1)
	ArgosConfig config;
	configuration_store->get_argos_configuration(config);
	unsigned int delay_ms = config.tr_nom * 1000;

	DEBUG_TRACE("DTEHandler::schedule_doppler_cal_tx: next TX in %u ms", delay_ms);

	system_scheduler->post_task_prio(
		[this]() {
			if (!m_doppler_cal_active || !smd_sat_instance) return;
			unsigned int size_bits;
			unsigned int batt_mv = battery_monitor ? battery_monitor->get_voltage() : 3700;
			bool is_lb = battery_monitor ? battery_monitor->is_battery_low() : false;
			KineisPacket packet = ArgosPacketBuilder::build_doppler_packet(batt_mv, is_lb, size_bits);
			DEBUG_TRACE("DTEHandler: SATDP periodic TX (%u bits)", size_bits);
			smd_sat_instance->send(KineisModulation::LDA2, packet, size_bits);
		},
		"SATDPPeriodicTx",
		Scheduler::DEFAULT_PRIORITY,
		delay_ms
	);
#endif
}
