#pragma once

#include "logger.hpp"
#include "messages.hpp"
#include "sensor_service.hpp"
#include "timeutils.hpp"
#include <cmath>      // for std::sqrt

template <typename E>
constexpr typename std::underlying_type<E>::type to_underlying(E e) noexcept
{
    return static_cast<typename std::underlying_type<E>::type>(e);
};

struct __attribute__((packed)) AXLLogEntry {
	LogHeader header;
	union {
		struct {
			double x;
			double y;
			double z;
			uint8_t activity;
			double temperature;
			bool   wakeup_triggered;
		};
		uint8_t data[MAX_LOG_PAYLOAD];
	};
};

class AXLLogFormatter : public LogFormatter {
public:
	const std::string header() override {
		return "log_datetime,x,y,z,activity,wakeup_triggered,temperature\r\n";
	}
	const std::string log_entry(const LogEntry& e) override {
		char entry[512], d1[128];
		const AXLLogEntry *log = (const AXLLogEntry *)&e;
		std::time_t t;
		std::tm *tm;

		t = convert_epochtime(log->header.year, log->header.month, log->header.day, log->header.hours, log->header.minutes, log->header.seconds);
		tm = std::gmtime(&t);
		std::strftime(d1, sizeof(d1), "%d/%m/%Y %H:%M:%S", tm);

		// Convert to CSV
		snprintf(entry, sizeof(entry), "%s,%f,%f,%f,%u,%u,%f\r\n",
				d1,
				log->x, log->y, log->z, log->activity, log->wakeup_triggered, log->temperature);
		return std::string(entry);
	}
};

enum AXLSensorPort : unsigned int {
	TEMPERATURE,
	X,
	Y,
	Z,
	ACTIVITY,
	WAKEUP_TRIGGERED
};

template<typename T, std::size_t N>
constexpr bool has_duplicates_bitmask(const std::array<T, N>& values)
{
    using underlying = typename std::underlying_type<T>::type;
    underlying mask = 0;

    for (std::size_t i = 0; i < N; ++i)
	{
        underlying val = static_cast<underlying>(values[i]);
        if (mask & (underlying(1) << val))
		{
            return true;
        }
        mask |= (underlying(1) << val);
    }
    return false;
}

enum class AXLCalibration : unsigned int {
	WAKEUP_THRESH		= 0,
	WAKEUP_DURATION		= 1,
	G_FORCE				= 2,
	POWER_MODE			= 3,
	X					= 4,
	Y					= 5,
	Z					= 6
};

constexpr std::array<AXLCalibration, 7> axlValues = {
    AXLCalibration::WAKEUP_THRESH,
    AXLCalibration::WAKEUP_DURATION,
    AXLCalibration::G_FORCE,
    AXLCalibration::POWER_MODE,
    AXLCalibration::X,
    AXLCalibration::Y,
    AXLCalibration::Z
};

enum AXLEvent : unsigned int {
	WAKEUP
};

class AXLSensorService : public SensorService {
public:
	AXLSensorService(Sensor& sensor, Logger *logger = nullptr) : SensorService(sensor, ServiceIdentifier::AXL_SENSOR, "AXL", logger) {
		static_assert(!has_duplicates_bitmask(axlValues), "Error: duplicate values exist in enum class");
	}

private:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
	void sensor_populate_log_entry(LogEntry *e, ServiceSensorData& data) override {
		AXLLogEntry *log = (AXLLogEntry *)e;
		log->header.log_type = LOG_AXL;
		log->x = data.port[(unsigned int)AXLSensorPort::X];
		log->y = data.port[(unsigned int)AXLSensorPort::Y];
		log->z = data.port[(unsigned int)AXLSensorPort::Z];
		log->activity = data.port[(unsigned int)AXLSensorPort::ACTIVITY];
		log->wakeup_triggered = data.port[(unsigned int)AXLSensorPort::WAKEUP_TRIGGERED];
		log->temperature = data.port[(unsigned int)AXLSensorPort::TEMPERATURE];
		service_set_log_header_time(log->header, service_current_time());
	}
#pragma GCC diagnostic pop

	void sensor_init() override {
		// Read all calibration parameters from config
		double g_thresh 			= service_read_param<double>(ParamID::AXL_SENSOR_WAKEUP_THRESH);
		unsigned int duration 		= service_read_param<unsigned int>(ParamID::AXL_SENSOR_WAKEUP_SAMPLES);
		unsigned int g_force 		= service_read_param<unsigned int>(ParamID::AXL_SENSOR_MEASUREMENT_RANGE);
		unsigned int power_mode 	= service_read_param<unsigned int>(ParamID::AXL_SENSOR_POWER_MODE);
		double x_calibration 		= service_read_param<double>(ParamID::AXL_SENSOR_X_CALIBRATION);
		double y_calibration 		= service_read_param<double>(ParamID::AXL_SENSOR_Y_CALIBRATION);
		double z_calibration 		= service_read_param<double>(ParamID::AXL_SENSOR_Z_CALIBRATION);

		// Write calibration parameters to sensor using enum offsets
		m_sensor.calibration_write(g_force, to_underlying(AXLCalibration::G_FORCE));
		m_sensor.calibration_write(power_mode, to_underlying(AXLCalibration::POWER_MODE));
		m_sensor.calibration_write(x_calibration, to_underlying(AXLCalibration::X));
		m_sensor.calibration_write(y_calibration, to_underlying(AXLCalibration::Y));
		m_sensor.calibration_write(z_calibration, to_underlying(AXLCalibration::Z));

		// Enable wakeup interrupt if threshold is configured and sensor is enabled
		if (g_thresh && sensor_is_enabled()) {
			m_sensor.calibration_write(g_thresh, to_underlying(AXLCalibration::WAKEUP_THRESH));
			m_sensor.calibration_write(duration, to_underlying(AXLCalibration::WAKEUP_DURATION));
			m_sensor.install_event_handler(AXLEvent::WAKEUP, [this]() {
				DEBUG_TRACE("AXLSensorService::event");
				sensor_handler(false);
			});
		}
	};

	void sensor_term() override {
		m_sensor.remove_event_handler(AXLEvent::WAKEUP);
	};

	bool sensor_is_enabled() override {
		return service_read_param<bool>(ParamID::AXL_SENSOR_ENABLE);
	}

	unsigned int sensor_max_samples() override {
		return service_read_param<unsigned int>(ParamID::AXL_SENSOR_ENABLE_TX_MAX_SAMPLES);
	}

	unsigned int sensor_num_channels() override { return 6U; }

	unsigned int sensor_periodic() override {
		unsigned int schedule =
				1000 * service_read_param<unsigned int>(ParamID::AXL_SENSOR_PERIODIC);
		return schedule == 0 ? Service::SCHEDULE_DISABLED : schedule;
	}

	unsigned int sensor_tx_periodic() override {
		return service_read_param<unsigned int>(ParamID::AXL_SENSOR_ENABLE_TX_SAMPLE_PERIOD);
	}

	bool sensor_is_usable_underwater() override { return true; }

	BaseSensorEnableTxMode sensor_enable_tx_mode() override {
		return service_read_param<BaseSensorEnableTxMode>(ParamID::AXL_SENSOR_ENABLE_TX_MODE);
	}
};
