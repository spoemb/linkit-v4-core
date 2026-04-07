#pragma once

#include <stdint.h>

#include "timer.hpp"
#include "rgb_led.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include "interrupt_lock.hpp"

extern Timer *system_timer;


class NrfRGBLed : public RGBLed {
private:
	int m_pin_red;
	int m_pin_green;
	int m_pin_blue;
	RGBLedColor m_color;
	RGBLedColor m_flash_color;
	RGBLedColor m_flash_color_alt;
	bool m_is_flashing;
	bool m_flash_state;
	unsigned int m_flash_interval;
	const char *m_name;
	Timer::TimerHandle m_led_timer;

	void toggle_led(void) {
		InterruptLock lock;
		if (!m_is_flashing)
			return;
		if (m_flash_state)
			set_color(m_flash_color);
		else
			set_color(m_flash_color_alt);
		m_flash_state = !m_flash_state;
		m_led_timer = system_timer->add_schedule([this]() {
			if (m_is_flashing)
				toggle_led();
		}, system_timer->get_counter() + m_flash_interval);
	}

private:
	void set_color(RGBLedColor color) {
		switch (color) {
		case RGBLedColor::BLACK:
			GPIOPins::set(m_pin_red);
			GPIOPins::set(m_pin_green);
			GPIOPins::set(m_pin_blue);
			break;
		case RGBLedColor::RED:
			GPIOPins::clear(m_pin_red);
			GPIOPins::set(m_pin_green);
			GPIOPins::set(m_pin_blue);
			break;
		case RGBLedColor::GREEN:
			GPIOPins::set(m_pin_red);
			GPIOPins::clear(m_pin_green);
			GPIOPins::set(m_pin_blue);
			break;
		case RGBLedColor::BLUE:
			GPIOPins::set(m_pin_red);
			GPIOPins::set(m_pin_green);
			GPIOPins::clear(m_pin_blue);
			break;
		case RGBLedColor::CYAN:
			GPIOPins::set(m_pin_red);
			GPIOPins::clear(m_pin_green);
			GPIOPins::clear(m_pin_blue);
			break;
		case RGBLedColor::MAGENTA:
			GPIOPins::clear(m_pin_red);
			GPIOPins::set(m_pin_green);
			GPIOPins::clear(m_pin_blue);
			break;
		case RGBLedColor::YELLOW:
			GPIOPins::clear(m_pin_red);
			GPIOPins::clear(m_pin_green);
			GPIOPins::set(m_pin_blue);
			break;
		case RGBLedColor::WHITE:
			GPIOPins::clear(m_pin_red);
			GPIOPins::clear(m_pin_green);
			GPIOPins::clear(m_pin_blue);
			break;
		default:
			break;
		}
	}
public:
	// Bare-metal LED control — no timer/interrupt dependency, safe for fault handlers and ISR context
	static void set_color_raw(int pin_red, int pin_green, int pin_blue, RGBLedColor color) {
		bool r = false, g = false, b = false;
		switch (color) {
		case RGBLedColor::BLACK:   break;
		case RGBLedColor::RED:     r = true; break;
		case RGBLedColor::GREEN:   g = true; break;
		case RGBLedColor::BLUE:    b = true; break;
		case RGBLedColor::CYAN:    g = true; b = true; break;
		case RGBLedColor::MAGENTA: r = true; b = true; break;
		case RGBLedColor::YELLOW:  r = true; g = true; break;
		case RGBLedColor::WHITE:   r = true; g = true; b = true; break;
		default: break;
		}
		r ? GPIOPins::clear(pin_red)   : GPIOPins::set(pin_red);
		g ? GPIOPins::clear(pin_green) : GPIOPins::set(pin_green);
		b ? GPIOPins::clear(pin_blue)  : GPIOPins::set(pin_blue);
	}

	NrfRGBLed(const char *name, int red, int green, int blue, RGBLedColor color = RGBLedColor::BLACK) {
		m_name = name;
		m_pin_red = red;
		m_pin_green = green;
		m_pin_blue = blue;
		m_color = color;
		set(color);
	}
	void set(RGBLedColor color) override {
		InterruptLock lock;
		system_timer->cancel_schedule(m_led_timer);
		m_is_flashing = false;
		m_color = color;
		set_color(color);
		//DEBUG_TRACE("LED[%s]=%s", m_name, color_to_string(color).c_str());
	}
	void off() override {
		set(RGBLedColor::BLACK);
	}
	void flash(RGBLedColor color, unsigned int interval_ms = 500) override {
		InterruptLock lock;
		system_timer->cancel_schedule(m_led_timer);
		m_flash_interval = interval_ms;
		m_flash_color = color;
		m_flash_color_alt = RGBLedColor::BLACK;
		m_is_flashing = true;
		m_flash_state = true;
		toggle_led();
		//DEBUG_TRACE("LED[%s]=flashing %s", m_name, color_to_string(color).c_str());
	}
	void flash_alternate(RGBLedColor color1, RGBLedColor color2, unsigned int interval_ms = 250) override {
		InterruptLock lock;
		system_timer->cancel_schedule(m_led_timer);
		m_flash_interval = interval_ms;
		m_flash_color = color1;
		m_flash_color_alt = color2;
		m_is_flashing = true;
		m_flash_state = true;
		toggle_led();
	}
	bool is_flashing() override {
		return m_is_flashing;
	}
	RGBLedColor get_state() override {
		return m_color;
	}
};
