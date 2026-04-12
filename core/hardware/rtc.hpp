#pragma once

/**
 * @file rtc.hpp
 * @brief Abstract real-time clock interface (epoch seconds).
 */

#include <ctime>

class RTC {
public:
	virtual ~RTC() = default;
	virtual std::time_t gettime() = 0;          ///< Current epoch time (seconds since 1970)
	virtual void settime(std::time_t time) = 0;  ///< Set wall-clock time (typically from GNSS fix)
	virtual bool is_set() = 0;                   ///< True if settime() has been called at least once
};
