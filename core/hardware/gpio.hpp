#ifndef __GPIO_HPP_
#define __GPIO_HPP_

#include <stdint.h>

class GPIOPins {
public:
	static void initialise();
	static void init_pin(uint32_t pin);
	static void set(uint32_t pin);
	static void clear(uint32_t pin);
	static void toggle(uint32_t pin);
	static uint32_t value(uint32_t pin);
	static void disable(uint32_t pin);
	static void enable(uint32_t pin);

	// VSENSORS power management with reference counting
	// Multiple sensors can acquire power; it stays on until ALL release
	static void acquire_sensors_pwr();  // Increment counter, power on if was 0
	static void release_sensors_pwr();  // Decrement counter, power off if reaches 0
	static bool get_sensors_pwr_state();
	static uint8_t get_sensors_pwr_refcount();

	// Pulse pin LOW for duration_ms, then release to high-impedance (INPUT)
	// Useful for SAT_RESET: allows nRF to reset SMD while keeping pin free for probe flashing
	static void pulse_low_then_release(uint32_t pin, uint32_t duration_ms);

	// Drive pin LOW (configure as output and drive low)
	// Use release_to_highz() to release control back to high-impedance
	static void drive_low(uint32_t pin);

	// Release pin to high-impedance (INPUT/disconnected)
	// Allows external probe to control the pin
	static void release_to_highz(uint32_t pin);
private:
	static uint8_t m_sensors_pwr_refcount;
	// Disconnect/reconnect sensor pins when VSENSORS is powered off/on
	// Prevents backfeed through ESD diodes and floating interrupt pins
	static void disconnect_sensor_pins();
	static void reconnect_sensor_pins();
};

// RAII guard for VSENSORS power - automatically releases on scope exit
class SensorsPowerGuard {
public:
	SensorsPowerGuard() { GPIOPins::acquire_sensors_pwr(); }
	~SensorsPowerGuard() { GPIOPins::release_sensors_pwr(); }
	// Non-copyable
	SensorsPowerGuard(const SensorsPowerGuard&) = delete;
	SensorsPowerGuard& operator=(const SensorsPowerGuard&) = delete;
};

#endif // __GPIO_HPP_
