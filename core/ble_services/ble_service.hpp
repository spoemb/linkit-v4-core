/**
 * @file ble_service.hpp
 * @brief Abstract BLE service interface — DTE data, OTA events, connection lifecycle.
 */

#pragma once

#include <functional>
#include <string>

enum class BLEServiceEventType {
	CONNECTED,
	DISCONNECTED,
	DTE_DATA_RECEIVED,
	OTA_START,
	OTA_FILE_DATA,
	OTA_ABORT,
	OTA_END
};

struct BLEServiceEvent {
	BLEServiceEventType event_type;
	union {
		struct {
			unsigned int file_id;
			unsigned int file_size;
			unsigned int crc32;
		};
		struct {
			void *data;
			unsigned int length;
		};
	};
};

class BLEService {
public:
	virtual ~BLEService() {}
	virtual void init() {}
	virtual void start(std::function<int(BLEServiceEvent& event)> on_event) = 0;
	virtual void stop() = 0;
	virtual bool write(std::string str) = 0;
	virtual bool write_best_effort(std::string str) { return write(str); }
	virtual std::string read_line() = 0;
	virtual void set_device_name(const std::string&) = 0;
};
