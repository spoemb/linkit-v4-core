/**
 * @file ledsm.cpp
 * @brief LED state machine — entry handlers for each tracker state → RGB color/flash pattern.
 */

#include "ledsm.hpp"
#include "debug.hpp"
#include "config_store.hpp"
#include "rgb_led.hpp"
#include "led.hpp"
#include "rtc.hpp"

extern Led *ext_status_led;
extern RGBLed *status_led;
extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern RTC *rtc;

namespace {

constexpr unsigned long long LED_HRS_24_MS = 24ULL * 3600ULL * 1000ULL;

/// @brief Returns true while we are still inside the 24-hour LED window.
///
/// On regular boards (single boot, no hard power-cycle), `system_timer` uptime
/// is monotonic and accurately tracks the time since deployment.
///
/// On EXTERNAL_WAKEUP boards (TPL5111-driven hard shutdowns), `system_timer`
/// resets to 0 at every wake-up — so the uptime check would never expire and
/// LEDs would stay on for the entire mission, draining the battery.
/// We use the RTC cutoff stored in `LED_HRS24_RTC_CUTOFF` — set once by
/// GPSService to (first valid GNSS RTC + 24h). RTC survives TPL hard shutdowns
/// via `LAST_KNOWN_RTC` (PWP06) restored at every boot, so this gives a real
/// 24-hours-since-deployment window.
///
/// Fallback (EXTERNAL_WAKEUP, no GNSS fix yet OR RTC unknown): keep LEDs on.
/// Better visible-on-bench during early provisioning than silent-on-deployment.
static bool led_24h_window_active() {
#ifdef EXTERNAL_WAKEUP
	if (rtc && rtc->is_set()) {
		std::time_t led_cutoff = configuration_store->read_param<std::time_t>(ParamID::LED_HRS24_RTC_CUTOFF);
		if (led_cutoff != 0) {
			return rtc->gettime() < led_cutoff;
		}
	}
	return true;  // No anchor yet — keep LEDs visible during provisioning
#else
	return system_timer->get_counter() < LED_HRS_24_MS;
#endif
}

}  // namespace


// Macro for determining LED mode state
#define LED_MODE_GUARD \
	if (configuration_store->read_param<BaseLEDMode>(ParamID::LED_MODE) == BaseLEDMode::ALWAYS || \
		(configuration_store->read_param<BaseLEDMode>(ParamID::LED_MODE) == BaseLEDMode::HRS_24 && \
		 led_24h_window_active()))

#define EXT_LED_MODE_GUARD \
	if (configuration_store->read_param<BaseLEDMode>(ParamID::EXT_LED_MODE) == BaseLEDMode::ALWAYS || \
		(configuration_store->read_param<BaseLEDMode>(ParamID::EXT_LED_MODE) == BaseLEDMode::HRS_24 && \
		 led_24h_window_active()))


void LEDOff::entry() {
	DEBUG_TRACE("LEDOff: entry");
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else if (m_is_battery_critical)
		transit<LEDBatteryCritical>();
	else
		status_led->off();
	if (ext_status_led)
		ext_status_led->off();
}

void LEDBoot::entry() {
	DEBUG_TRACE("LEDBoot: entry");
	status_led->flash(RGBLedColor::WHITE, 125);
	if (ext_status_led)
		ext_status_led->flash(125);
}

void LEDPowerDown::entry() {
	DEBUG_TRACE("LEDPowerDown: entry");
	m_is_battery_critical = false;
	// Always flash white during powerdown countdown, regardless of magnet
	status_led->flash(RGBLedColor::WHITE, 50);
	if (ext_status_led)
		ext_status_led->flash(50);
}

void LEDError::entry() {
	DEBUG_TRACE("LEDError: entry");
	m_is_battery_critical = false;
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else
		status_led->flash(RGBLedColor::RED);
	if (ext_status_led)
		ext_status_led->off();
}

void LEDPreOperationalPending::entry() {
	DEBUG_TRACE("LEDPreOperationalPending: entry");
	m_is_battery_critical = false;
	status_led->set(RGBLedColor::GREEN);
	if (ext_status_led)
		ext_status_led->off();
}

void LEDPreOperationalError::entry() {
	DEBUG_TRACE("LEDPreOperationalError: entry");
	m_is_battery_critical = false;
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else
		status_led->flash(RGBLedColor::RED);
	if (ext_status_led)
		ext_status_led->off();
}

void LEDPreOperationalBatteryNominal::entry() {
	DEBUG_TRACE("LEDPreOperationalBatteryNominal: entry");
	m_is_battery_critical = false;
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else
		status_led->flash(RGBLedColor::GREEN);
	if (ext_status_led)
		ext_status_led->off();
}

void LEDPreOperationalBatteryLow::entry() {
	DEBUG_TRACE("LEDPreOperationalBatteryLow: entry");
	m_is_battery_critical = false;
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else
		status_led->flash(RGBLedColor::YELLOW);
	if (ext_status_led)
		ext_status_led->off();
}

void LEDConfigPending::entry() {
	DEBUG_TRACE("LEDConfigPending: entry");
	m_is_battery_critical = false;
	status_led->set(RGBLedColor::BLUE);
	if (ext_status_led)
		ext_status_led->off();
}

void LEDConfigNotConnected::entry() {
	DEBUG_TRACE("LEDConfigNotConnected: entry");
	m_is_battery_critical = false;
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else
		status_led->flash(RGBLedColor::BLUE);
	if (ext_status_led)
		ext_status_led->off();
}

void LEDConfigConnected::entry() {
	DEBUG_TRACE("LEDConfigConnected: entry");
	m_is_battery_critical = false;
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else
		status_led->set(RGBLedColor::BLUE);
	if (ext_status_led)
		ext_status_led->off();
}

void LEDGNSSOn::entry() {
	DEBUG_TRACE("LEDGNSSOn: entry");
	m_is_gnss_on = true;
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else {
		LED_MODE_GUARD {
			status_led->flash(RGBLedColor::CYAN, 1000);
		} else {
			status_led->off();
		}
		EXT_LED_MODE_GUARD {
			if (ext_status_led)
				ext_status_led->on();
		} else {
			if (ext_status_led)
				ext_status_led->off();
		}
	}
}

void LEDGNSSOffWithFix::entry() {
	DEBUG_TRACE("LEDGNSSOffWithFix: entry");
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else {
		if (m_is_gnss_on) {
			LED_MODE_GUARD {
				status_led->set(RGBLedColor::GREEN);
			} else {
				status_led->off();
			}
			if (ext_status_led)
				ext_status_led->off();
		}
		system_timer->add_schedule([this]() {
			if (is_in_state<LEDConfigNotConnected>())
				transit<LEDConfigNotConnected>();
			else
				transit<LEDOff>();
		}, system_timer->get_counter() + 3000);
	}
	m_is_gnss_on = false;
}

void LEDGNSSOffWithoutFix::entry() {
	DEBUG_TRACE("LEDGNSSOffWithoutFix: entry");
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else {
		if (m_is_gnss_on) {
			LED_MODE_GUARD {
				status_led->set(RGBLedColor::RED);
			} else {
				status_led->off();
			}
			if (ext_status_led)
				ext_status_led->off();
		}
		system_timer->add_schedule([this]() {
			if (is_in_state<LEDConfigNotConnected>())
				transit<LEDConfigNotConnected>();
			else
				transit<LEDOff>();
		}, system_timer->get_counter() + 3000);
	}
	m_is_gnss_on = false;
}

void LEDArgosTX::entry() {
	DEBUG_TRACE("LEDArgosTX: entry");
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else {
		LED_MODE_GUARD {
			status_led->set(RGBLedColor::MAGENTA);
		} else {
			status_led->off();
		}
	}
}

void LEDArgosTXComplete::entry() {
	DEBUG_TRACE("LEDArgosTXComplete: entry");
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else {
		status_led->off();
		system_timer->add_schedule([this]() {
			if (m_is_gnss_on)
				transit<LEDGNSSOn>();
			else {
				if (is_in_state<LEDConfigNotConnected>())
					transit<LEDConfigNotConnected>();
				else
					transit<LEDOff>();
			}
		}, system_timer->get_counter() + 50); // Add 50ms to avoid race condition on timer tick
	}
}

void LEDBatteryCritical::entry() {
	DEBUG_TRACE("LEDBatteryCritical: entry");
	m_is_battery_critical = true;
	if (m_is_magnet_engaged)
		status_led->set(RGBLedColor::WHITE);
	else {
		status_led->set(RGBLedColor::YELLOW);
	}
}

void LEDDFUUpdate::entry() {
	DEBUG_TRACE("LEDDFUUpdate: entry");
	status_led->flash_alternate(RGBLedColor::BLUE, RGBLedColor::WHITE, 250);
	if (ext_status_led)
		ext_status_led->off();
}

void LEDOTASuccess::entry() {
	DEBUG_TRACE("LEDOTASuccess: entry");
	status_led->set(RGBLedColor::GREEN);
	if (ext_status_led)
		ext_status_led->on();
}

void LEDOTAFailed::entry() {
	DEBUG_TRACE("LEDOTAFailed: entry");
	status_led->flash(RGBLedColor::RED, 200);
	if (ext_status_led)
		ext_status_led->flash(200);
}

void LEDFirmwareApplied::entry() {
	DEBUG_TRACE("LEDFirmwareApplied: entry");
	status_led->flash(RGBLedColor::GREEN, 150);
	if (ext_status_led)
		ext_status_led->flash(150);
}

void LEDConfirmConfig::entry() {
	DEBUG_TRACE("LEDConfirmConfig: entry");
	status_led->flash(RGBLedColor::BLUE, 50);
	if (ext_status_led)
		ext_status_led->off();
}

void LEDConfirmExitConfig::entry() {
	DEBUG_TRACE("LEDConfirmExitConfig: entry");
	status_led->flash(RGBLedColor::GREEN, 50);
	if (ext_status_led)
		ext_status_led->off();
}

void LEDConfirmPowerOff::entry() {
	DEBUG_TRACE("LEDConfirmPowerOff: entry");
	status_led->flash(RGBLedColor::RED, 50);
	if (ext_status_led)
		ext_status_led->off();
}

void LEDSurfaceDetected::entry() {
	DEBUG_TRACE("LEDSurfaceDetected: entry");
	LED_MODE_GUARD {
		status_led->set(RGBLedColor::GREEN);
	} else {
		status_led->off();
	}
	system_timer->add_schedule([this]() {
		transit<LEDOff>();
	}, system_timer->get_counter() + 100);
}

void LEDDiveDetected::entry() {
	DEBUG_TRACE("LEDDiveDetected: entry");
	LED_MODE_GUARD {
		status_led->set(RGBLedColor::BLUE);
	} else {
		status_led->off();
	}
	system_timer->add_schedule([this]() {
		transit<LEDOff>();
	}, system_timer->get_counter() + 100);
}
