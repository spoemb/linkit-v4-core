#include <stdint.h>

#include "gpio.hpp"
#include "bsp.hpp"
#include "nrf_i2c.hpp"
#include "debug.hpp"
#include "nrf_delay.h"
bool GPIOPins::m_sensors_pwr_state = false;

void GPIOPins::initialise()
{
	for (uint32_t i = 0; i < (uint32_t)BSP::GPIO::GPIO_TOTAL_NUMBER; i++)
	{
		nrf_gpio_cfg(BSP::GPIO_Inits[i].pin_number,
					 BSP::GPIO_Inits[i].dir, BSP::GPIO_Inits[i].input, BSP::GPIO_Inits[i].pull,
				     BSP::GPIO_Inits[i].drive, BSP::GPIO_Inits[i].sense);
	}
	// 3. Put unused pins into lowest power mode
    for (uint32_t pin = 0; pin < 48; pin++) // P0.00–P0.31 + P1.00–P1.15 = 48 pins total
    {
        // Skip pins already configured
        bool pin_used = false;

        for (uint32_t j = 0; j < (uint32_t)BSP::GPIO::GPIO_TOTAL_NUMBER; j++)
        {
            if (BSP::GPIO_Inits[j].pin_number == pin)
            {
                pin_used = true;
                break;
            }
        }

        if (pin == BSP::UARTAsync_Inits[0].config.rx_pin || pin == BSP::UARTAsync_Inits[0].config.tx_pin)
            pin_used = true;

        // Skip SWD (typically P0.18 and P0.19)
        if (pin == 18 || pin == 19)
            pin_used = true;
        

        if (!pin_used)
        {
            nrf_gpio_cfg(
                pin,
                NRF_GPIO_PIN_DIR_INPUT,
                NRF_GPIO_PIN_INPUT_DISCONNECT,
                NRF_GPIO_PIN_NOPULL,
                NRF_GPIO_PIN_S0S1,
                NRF_GPIO_PIN_NOSENSE
            );
        }
    }

	// Tri-state UART pins on GPS as these are controlled from driver level
	nrf_gpio_cfg_default(BSP::UARTAsync_Inits[0].config.rx_pin);
	nrf_gpio_cfg_default(BSP::UARTAsync_Inits[0].config.tx_pin);

	// Ensure power off state for everything controlling power
	//set(GPS_POWER);
	clear(GPS_POWER);
	#ifdef GPS_RST
	set(GPS_RST);
	#endif
	#ifdef SAT_PWR_EN
	set(SAT_RESET);
	clear(SAT_PWR_EN);
	//set(SAT_PWR_EN);
	#endif
	#ifdef SENSORS_PWR_PIN
	//clear_sensors_pwr();
	set(SENSORS_PWR_PIN);
	#endif
}


#define TWIM_POWER_REG(instance) (*((volatile uint32_t *)((uint32_t)(instance) + 0xFFC)))


void GPIOPins::set_sensors_pwr()
{
	if (!m_sensors_pwr_state)
	{
		DEBUG_TRACE("GPIOPins::set_sensors_pwr: Setting sensors power pin");
		set(SENSORS_PWR_PIN);
		nrf_delay_ms(50);
		//NrfI2C::init(); // Ensure I2C is initialised after setting sensors power
		m_sensors_pwr_state = true;
	}
}
void GPIOPins::clear_sensors_pwr()
{
	if (m_sensors_pwr_state)
	{
		DEBUG_TRACE("GPIOPins::clear_sensors_pwr: Clearing sensors power pin");
    
		//NrfI2C::uninit();
		//clear(SENSORS_PWR_PIN);
		//m_sensors_pwr_state = false;
	}
}	
bool GPIOPins::get_sensors_pwr_state()
{
	return GPIOPins::m_sensors_pwr_state;
}
void GPIOPins::set(uint32_t pin)
{
	nrf_gpio_pin_set(BSP::GPIO_Inits[pin].pin_number);
}

void GPIOPins::clear(uint32_t pin)
{
	nrf_gpio_pin_clear(BSP::GPIO_Inits[pin].pin_number);
}

void GPIOPins::toggle(uint32_t pin)
{
	nrf_gpio_pin_toggle(BSP::GPIO_Inits[pin].pin_number);
}

uint32_t GPIOPins::value(uint32_t pin)
{
	return nrf_gpio_pin_read(BSP::GPIO_Inits[pin].pin_number);
}

void GPIOPins::enable(uint32_t pin)
{
	nrf_gpio_cfg(BSP::GPIO_Inits[pin].pin_number,
				 BSP::GPIO_Inits[pin].dir, BSP::GPIO_Inits[pin].input, BSP::GPIO_Inits[pin].pull,
			     BSP::GPIO_Inits[pin].drive, BSP::GPIO_Inits[pin].sense);
}

void GPIOPins::disable(uint32_t pin)
{
	nrf_gpio_cfg_default(BSP::GPIO_Inits[pin].pin_number);
}
