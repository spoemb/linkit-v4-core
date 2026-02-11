#include <stdint.h>

#include "gpio.hpp"
#include "bsp.hpp"
#include "debug.hpp"
#include "nrf_delay.h"

// Static member initialization
bool GPIOPins::m_sensors_pwr_state = false;

void GPIOPins::initialise()
{
	for (uint32_t i = 0; i < (uint32_t)BSP::GPIO::GPIO_TOTAL_NUMBER; i++)
	{
		nrf_gpio_cfg(BSP::GPIO_Inits[i].pin_number,
					 BSP::GPIO_Inits[i].dir, BSP::GPIO_Inits[i].input, BSP::GPIO_Inits[i].pull,
				     BSP::GPIO_Inits[i].drive, BSP::GPIO_Inits[i].sense);
	}

	// Tri-state UART pins as these are controlled from driver level
	nrf_gpio_cfg_default(BSP::UARTAsync_Inits[0].config.rx_pin);
	nrf_gpio_cfg_default(BSP::UARTAsync_Inits[0].config.tx_pin);
#if !defined(BOARD_RSPB)
	nrf_gpio_cfg_default(BSP::UARTAsync_Inits[1].config.rx_pin);
	nrf_gpio_cfg_default(BSP::UARTAsync_Inits[1].config.tx_pin);
#endif

	// Ensure power off state for everything controlling power
	set(GPS_RST);
	clear(GPS_POWER);

	// SMD satellite module has I2C pins routed to the I2C bus.
	// When SMD is not powered, its internal circuitry pulls the bus LOW.
	// Power ON the SMD (but keep in reset) so its I2C pins don't interfere.
	clear(SAT_RESET);    // Assert reset first (hold SMD in reset)
	nrf_delay_ms(1);
	set(SAT_PWR_EN);     // Power ON the SMD (I2C bus now free)
	nrf_delay_ms(10);    // Wait for SMD power to stabilize

	clear(SWS_ENABLE_PIN);
#ifdef GPIO_AG_PWR_PIN
	set(GPIO_AG_PWR_PIN);	// BMA400 must be ON to avoid leakage current. BMA 400 sleep current is 200nA
#endif
#ifdef SENSORS_PWR_PIN
	set(SENSORS_PWR_PIN);	// Enable sensors power (BMA400, etc.) for RSPB board
	// Note: Full 50ms delay happens in set_sensors_pwr() before I2C init
#endif
#ifdef ADC_ENABLE
	set(ADC_ENABLE);		// Enable ADC to avoid leakage current (I2C pull up)
#endif

#ifdef CAM_PWR_EN
	// Initialize CAM power pins to OFF state
	DEBUG_TRACE("Initializing CAM pins: CAM_PWR_EN=%u (P%u.%u), CAM_PWR_BUTT=%u (P%u.%u)",
		CAM_PWR_EN, (BSP::GPIO_Inits[CAM_PWR_EN].pin_number >> 5), (BSP::GPIO_Inits[CAM_PWR_EN].pin_number & 0x1F),
		CAM_PWR_BUTT, (BSP::GPIO_Inits[CAM_PWR_BUTT].pin_number >> 5), (BSP::GPIO_Inits[CAM_PWR_BUTT].pin_number & 0x1F));
	clear(CAM_PWR_EN);
	clear(CAM_PWR_BUTT);
	// Force re-enable GPIO configuration to ensure pins are properly configured
	enable(CAM_PWR_EN);
	enable(CAM_PWR_BUTT);
	// Set to low again to ensure off state
	clear(CAM_PWR_EN);
	clear(CAM_PWR_BUTT);
	DEBUG_TRACE("CAM pins cleared and configured");
#endif

#ifdef VSYS_SEL
	// Set V_SYS to 3.3V
	set(VSYS_SEL);
#endif
}

void GPIOPins::set(uint32_t pin)
{
	nrf_gpio_pin_set(BSP::GPIO_Inits[pin].pin_number);
}

void GPIOPins::set_sensors_pwr()
{
#ifdef SENSORS_PWR_PIN
	if (!m_sensors_pwr_state)
	{
		DEBUG_TRACE("GPIOPins::set_sensors_pwr: Setting sensors power pin");
		set(SENSORS_PWR_PIN);
		nrf_delay_ms(50);  // 50ms delay for sensors to power up (BMA400 needs ~2ms, LPS28DFW needs ~10ms, extra margin for I2C stability)
		m_sensors_pwr_state = true;
	}
#endif
}

void GPIOPins::clear_sensors_pwr()
{
#ifdef SENSORS_PWR_PIN
	if (m_sensors_pwr_state)
	{
		DEBUG_TRACE("GPIOPins::clear_sensors_pwr: Clearing sensors power pin");
		clear(SENSORS_PWR_PIN);
		m_sensors_pwr_state = false;
	}
#endif
}

bool GPIOPins::get_sensors_pwr_state()
{
	return m_sensors_pwr_state;
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
