/**
 * @file hauled_mode_service.cpp
 * @brief HauledModeService — see hauled_mode_service.hpp.
 */

#include <cstddef>
#include <cstring>

#include "hauled_mode_service.hpp"
#include "config_store.hpp"
#include "interrupt_lock.hpp"
#include "rtc.hpp"
#include "debug.hpp"

#ifndef CPPUTEST
#include "crc16.h"
#else
#include <cstdint>
static inline uint16_t crc16_compute(const uint8_t *data, uint16_t length, const uint16_t *) {
	uint16_t crc = 0xFFFF;
	for (uint16_t i = 0; i < length; i++) {
		crc ^= static_cast<uint16_t>(data[i]) << 8;
		for (uint8_t j = 0; j < 8; j++)
			crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
	}
	return crc;
}
#endif

extern ConfigurationStore *configuration_store;
extern RTC *rtc;

namespace {

struct HauledNoinit {
	std::time_t last_uw_event_rtc;
	uint8_t     in_hauled;          // 0 = AT_SEA, 1 = HAULED
	uint8_t     uw_events_since_hauled;
	uint16_t    pad;                // keep `crc` deterministically aligned
	uint16_t    crc;
};

#ifndef CPPUTEST
HauledNoinit s_noinit __attribute__((section(".noinit")));
#else
HauledNoinit s_noinit;
#endif

uint16_t noinit_crc() {
	return crc16_compute(
		reinterpret_cast<const uint8_t *>(&s_noinit),
		offsetof(decltype(s_noinit), crc),
		nullptr);
}

void clear_state() {
	std::memset(&s_noinit, 0, sizeof(s_noinit));
	s_noinit.crc = noinit_crc();
}

} // namespace

void HauledModeService::restore_state() {
	if (s_noinit.crc == noinit_crc() && s_noinit.in_hauled <= 1) {
		DEBUG_INFO("HauledModeService: restored from noinit (in_hauled=%u, last_uw=%u, returns=%u)",
		           s_noinit.in_hauled,
		           (unsigned int)s_noinit.last_uw_event_rtc,
		           s_noinit.uw_events_since_hauled);
		return;
	}
	DEBUG_TRACE("HauledModeService: noinit invalid, starting fresh (AT_SEA)");
	clear_state();
}

void HauledModeService::on_underwater_event(bool submerged, std::time_t now) {
	if (!configuration_store) return;
	// Only act on dive events; surfacing transitions are not informative
	// for hauled detection (the dry counter just keeps running).
	if (!submerged) return;

	InterruptLock lock;
	s_noinit.last_uw_event_rtc = now;

	if (s_noinit.in_hauled) {
		// Hauled → AT_SEA hysteresis: count consecutive dive events.
		unsigned int needed = configuration_store->read_param<unsigned int>(ParamID::HAULED_RETURN_EVENTS);
		if (needed == 0) needed = 1;
		if (s_noinit.uw_events_since_hauled < 0xFF)
			s_noinit.uw_events_since_hauled++;
		if (s_noinit.uw_events_since_hauled >= needed) {
			DEBUG_INFO("HauledModeService: HAULED → AT_SEA after %u dive events", s_noinit.uw_events_since_hauled);
			s_noinit.in_hauled = 0;
			s_noinit.uw_events_since_hauled = 0;
		}
	}

	s_noinit.crc = noinit_crc();
}

void HauledModeService::evaluate() {
	if (!rtc || !rtc->is_set()) return;  // No RTC → can't compute age, stay in current state
	evaluate(rtc->gettime());
}

void HauledModeService::evaluate(std::time_t now) {
	if (!configuration_store) return;
	if (!configuration_store->read_param<bool>(ParamID::HAULED_DETECT_EN)) {
		// Detection disabled: ensure we never stay stuck in HAULED.
		if (s_noinit.in_hauled) {
			InterruptLock lock;
			s_noinit.in_hauled = 0;
			s_noinit.uw_events_since_hauled = 0;
			s_noinit.crc = noinit_crc();
		}
		return;
	}
	if (s_noinit.in_hauled) return;  // hysteresis: only on_underwater_event can clear
	if (s_noinit.last_uw_event_rtc == 0) return;  // never seen UW yet — stay AT_SEA
	if (now < s_noinit.last_uw_event_rtc) return; // RTC rollback — wait it out

	unsigned int threshold_h = configuration_store->read_param<unsigned int>(ParamID::HAULED_IDLE_THRESHOLD_H);
	if (threshold_h == 0) return;
	std::time_t threshold_s = (std::time_t)threshold_h * 3600;
	if ((now - s_noinit.last_uw_event_rtc) > threshold_s) {
		InterruptLock lock;
		s_noinit.in_hauled = 1;
		s_noinit.uw_events_since_hauled = 0;
		s_noinit.crc = noinit_crc();
		DEBUG_INFO("HauledModeService: AT_SEA → HAULED (dry for %u s, threshold %u h)",
		           (unsigned int)(now - s_noinit.last_uw_event_rtc), threshold_h);
	}
}

bool HauledModeService::is_hauled() {
	return s_noinit.in_hauled != 0;
}

std::time_t HauledModeService::last_uw_event_rtc() {
	return s_noinit.last_uw_event_rtc;
}

unsigned int HauledModeService::uw_events_since_hauled() {
	return s_noinit.uw_events_since_hauled;
}

void HauledModeService::reset_for_tests() {
	clear_state();
}
