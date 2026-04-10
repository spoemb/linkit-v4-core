#pragma once

#include <string>
#include <cstdint>

enum PMULogType {
	WDT,
	HARDFAULT,
	ETL,
	MMAN,
	STACK,
	MALLOC
};

enum class ResetCause {
	POWER_ON,
	HARD_RESET,
	WDT_RESET,
	SOFT_RESET,
	PSEUDO_POWER_ON,
};


class PMU {
public:
private:
	static void watchdog_handler(void);

public:
	static void initialise();
	static void reset(bool dfu_mode);
	static void powerdown();
	static void run();
	static void delay_ms(unsigned ms);
	static void delay_us(unsigned us);
	static void start_watchdog();
	static void kick_watchdog();
	static ResetCause reset_cause();
	static const char* reset_cause_str();
	static const std::string hardware_version();
	static uint32_t device_identifier();
	static void save_stack(PMULogType type);
	static void print_stack();
	static uint64_t get_timestamp_ms();
	static bool was_firmware_updated();
	static void enter_deep_idle();
	static void exit_deep_idle();
};
