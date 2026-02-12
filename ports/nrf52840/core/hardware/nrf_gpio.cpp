#include <stdint.h>

#include "gpio.hpp"
#include "bsp.hpp"
#include "debug.hpp"
#include "nrf_delay.h"

// Static member initialization
uint8_t GPIOPins::m_sensors_pwr_refcount = 0;

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

	// SMD satellite module - keep powered OFF at init
	// SMD driver (smd_sat.cpp) will power it on when needed
	clear(SAT_PWR_EN);
	release_to_highz(SAT_RESET);

	clear(SWS_ENABLE_PIN);
#ifdef GPIO_AG_PWR_PIN
	set(GPIO_AG_PWR_PIN);	// BMA400 must be ON to avoid leakage current. BMA 400 sleep current is 200nA
#endif
#ifdef SENSORS_PWR_PIN
	// VSENSORS starts OFF - sensors will acquire/release power as needed
	clear(SENSORS_PWR_PIN);
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

void GPIOPins::acquire_sensors_pwr()
{
#ifdef SENSORS_PWR_PIN
	if (m_sensors_pwr_refcount == 0)
	{
		DEBUG_TRACE("GPIOPins::acquire_sensors_pwr: Powering ON VSENSORS (refcount 0->1)");
		set(SENSORS_PWR_PIN);
		nrf_delay_ms(50);  // 50ms delay for sensors to power up (BMA400 needs ~2ms, LPS28DFW needs ~10ms, extra margin for I2C stability)
	}
	else
	{
		DEBUG_TRACE("GPIOPins::acquire_sensors_pwr: refcount %u->%u (already on)", m_sensors_pwr_refcount, m_sensors_pwr_refcount + 1);
	}
	m_sensors_pwr_refcount++;
#endif
}

void GPIOPins::release_sensors_pwr()
{
#ifdef SENSORS_PWR_PIN
	if (m_sensors_pwr_refcount == 0)
	{
		DEBUG_WARN("GPIOPins::release_sensors_pwr: refcount already 0, ignoring release");
		return;
	}
	m_sensors_pwr_refcount--;
	if (m_sensors_pwr_refcount == 0)
	{
		DEBUG_TRACE("GPIOPins::release_sensors_pwr: Powering OFF VSENSORS (refcount 1->0)");
		clear(SENSORS_PWR_PIN);
		nrf_delay_ms(50);  // Power supply stabilization delay to avoid reboot when other rails power up
	}
	else
	{
		DEBUG_TRACE("GPIOPins::release_sensors_pwr: refcount %u->%u (still in use)", m_sensors_pwr_refcount + 1, m_sensors_pwr_refcount);
	}
#endif
}

bool GPIOPins::get_sensors_pwr_state()
{
	return m_sensors_pwr_refcount > 0;
}

uint8_t GPIOPins::get_sensors_pwr_refcount()
{
	return m_sensors_pwr_refcount;
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

void GPIOPins::pulse_low_then_release(uint32_t pin, uint32_t duration_ms)
{
	uint32_t pin_number = BSP::GPIO_Inits[pin].pin_number;

	// Configure as output and drive LOW
	nrf_gpio_cfg_output(pin_number);
	nrf_gpio_pin_clear(pin_number);

	// Hold LOW for specified duration
	nrf_delay_ms(duration_ms);

	// Release to high-impedance (input/disconnected) - allows probe to control
	nrf_gpio_cfg_default(pin_number);
}

void GPIOPins::drive_low(uint32_t pin)
{
	uint32_t pin_number = BSP::GPIO_Inits[pin].pin_number;
	nrf_gpio_cfg_output(pin_number);
	nrf_gpio_pin_clear(pin_number);
}

void GPIOPins::release_to_highz(uint32_t pin)
{
	nrf_gpio_cfg_default(BSP::GPIO_Inits[pin].pin_number);
}
