#pragma once

/**
 * @file pmu.hpp
 * @brief Power Management Unit — watchdog, reset, power-down, deep idle, crash trace.
 */

#include <cstdint>

/// @brief Crash type identifier stored in .noinit RAM for post-reset diagnostics.
enum PMULogType {
	WDT,
	HARDFAULT,
	ETL,
	MMAN,
	STACK,
	MALLOC
};

/// @brief Reset cause determined from NRF_POWER->RESETREAS + GPREGRET.
enum class ResetCause {
	POWER_ON,
	HARD_RESET,
	WDT_RESET,
	SOFT_RESET,
	PSEUDO_POWER_ON,
};

class PMU {
private:
	static void watchdog_handler(void);

public:
	static void initialise();
	/// @param dfu_mode  Reserved (not currently used — always does a normal reset).
	static void reset(bool dfu_mode);

	/// @brief Shut down the device — saves RTC, cuts power rails, enters System OFF or infinite sleep.
	static void powerdown();

	/// @brief Enter WFE (Wait For Event) — CPU sleeps until next interrupt/event.
	static void run();

	static void delay_ms(unsigned ms);
	static void delay_us(unsigned us);
	static void start_watchdog();
	static void kick_watchdog();

	static ResetCause reset_cause();
	static const char *reset_cause_str();
	static const char *hardware_version();
	static uint32_t device_identifier();

	/// @brief Capture callstack into .noinit RAM (called from fault handlers).
	/// @param type  Crash type identifier (WDT, HARDFAULT, ETL, etc.).
	static void save_stack(PMULogType type);

	/// @brief Print saved callstack if CRC is valid, then invalidate.
	static void print_stack();

	static uint64_t get_timestamp_ms();
	static bool was_firmware_updated();

	/// @brief Cut peripheral power rails when no task is due soon (> 5 s).
	static void reduce_power_rails();

	/// @brief Restore peripheral power rails before next scheduled task.
	static void restore_power_rails();
};
