#ifndef __GPIO_HPP_
#define __GPIO_HPP_

#include <stdint.h>

class GPIOPins {
public:
	static void initialise();
	static void set(uint32_t pin);

	static void set_sensors_pwr();
	static void clear_sensors_pwr();
 	static bool get_sensors_pwr_state();
	static void clear(uint32_t pin);
	static void toggle(uint32_t pin);
	static uint32_t value(uint32_t pin);
	static void disable(uint32_t pin);
	static void enable(uint32_t pin);
private:
	static bool m_sensors_pwr_state; // Track the state of the sensors power pin
};

#endif // __GPIO_HPP_
