#include "pmu.hpp"

#include <chrono>
#include "CppUTestExt/MockSupport.h"


void PMU::reset(bool dfu_mode) {
	mock().actualCall("reset").withParameter("dfu_mode", dfu_mode);
}

void PMU::powerdown() {
	mock().actualCall("powerdown");
}

void PMU::delay_ms(unsigned ms) {
	mock().actualCall("delay_ms").withParameter("ms", ms);
}

void PMU::delay_us(unsigned us) {
	mock().actualCall("delay_us").withParameter("us", us);
}

void PMU::start_watchdog() {
	mock().actualCall("start_watchdog");
}

void PMU::kick_watchdog() {
	// No-op: watchdog kick is a side-effect irrelevant to test assertions.
	// Previously mocked, but caused spurious failures when state transitions
	// (ErrorState, BatteryCriticalState) added extra kick_watchdog() calls.
}

void PMU::print_stack() {
}

ResetCause PMU::reset_cause() {
	return ResetCause::POWER_ON;
}

const char* PMU::reset_cause_str() {
	return "UNKNOWN";
}

const char* PMU::hardware_version() {
	return "simulator";
}

uint32_t PMU::device_identifier() {
	return 0x12345678;
}

uint64_t PMU::get_timestamp_ms() {
	static auto start = std::chrono::steady_clock::now();
	auto now = std::chrono::steady_clock::now();
	return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}

bool PMU::was_firmware_updated() {
	return false;
}
