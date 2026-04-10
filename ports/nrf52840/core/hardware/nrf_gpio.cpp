/**
 * @file nrf_gpio.cpp
 * @brief nRF52840 GPIO implementation with VSENSORS power management.
 */

#include <cstdint>

#include "gpio.hpp"
#include "bsp.hpp"
#include "debug.hpp"
#include "nrf_delay.h"
#include "nrf_i2c.hpp"

uint8_t GPIOPins::m_sensors_pwr_refcount = 0;

/// @brief Configure all BSP GPIO pins and set safe initial state for all power rails.
void GPIOPins::initialise()
{
	for (unsigned int i = 0; i < static_cast<unsigned int>(BSP::GPIO::GPIO_TOTAL_NUMBER); i++)
	{
		nrf_gpio_cfg(BSP::GPIO_Inits[i].pin_number,
					BSP::GPIO_Inits[i].dir, BSP::GPIO_Inits[i].input, BSP::GPIO_Inits[i].pull,
					BSP::GPIO_Inits[i].drive, BSP::GPIO_Inits[i].sense);
	}

	// Tri-state UART pins — controlled at driver level, not GPIO init
	nrf_gpio_cfg_default(BSP::UARTAsync_Inits[0].config.rx_pin);
	nrf_gpio_cfg_default(BSP::UARTAsync_Inits[0].config.tx_pin);
#if !defined(BOARD_RSPB)
	nrf_gpio_cfg_default(BSP::UARTAsync_Inits[1].config.rx_pin);
	nrf_gpio_cfg_default(BSP::UARTAsync_Inits[1].config.tx_pin);
#endif

	// Ensure power-off state for peripherals controlling power rails
	clear(GPS_POWER);
	release_to_highz(GPS_RST);  // Disconnect (ext pull-up, GPS off)

	// SMD satellite module — keep powered OFF at init
	clear(SAT_PWR_EN);
	release_to_highz(SAT_RESET);  // Disconnect (ext pull-up, SMD off)
#ifdef SMD_VPA_PIN
	// Drive VPA LOW at boot to prevent floating regulator enable
	drive_low(SMD_VPA_PIN);
#endif

	// RSPB: SMD UART pins (P0.26/P0.14) not used in SPI mode — pulldown to prevent float
#if defined(BOARD_RSPB) && !defined(SMD_UART)
	nrf_gpio_cfg_input(NRF_GPIO_PIN_MAP(0, 26), NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(NRF_GPIO_PIN_MAP(0, 14), NRF_GPIO_PIN_PULLDOWN);
#endif

	clear(SWS_ENABLE_PIN);
#ifdef GPIO_AG_PWR_PIN
	set(GPIO_AG_PWR_PIN);  // BMA400 must be ON to avoid leakage current (sleep = 200 nA)
#endif
#ifdef SENSORS_PWR_PIN
	// FIXME (RSPB): I2C pull-ups R21/R24 are on DCDC_3V3, not VSENSORS.
	// When VSENSORS is OFF, ~1.3 mA backfeeds through sensor ESD diodes.
	// Workaround: power ON VSENSORS immediately at boot and keep it ON.
	// Fix in next PCB revision: move R21/R24 to VSENSORS rail.
#if defined(BOARD_RSPB)
	set(SENSORS_PWR_PIN);
	m_sensors_pwr_refcount = 1;  // Prevent release_sensors_pwr() from cutting it
	reconnect_sensor_pins();
#else
	clear(SENSORS_PWR_PIN);
	disconnect_sensor_pins();
#endif
#endif
#ifdef ADC_ENABLE
	set(ADC_ENABLE);  // Enable ADC to avoid leakage current (I2C pull-up)
#endif

#ifdef CAM_PWR_EN
	DEBUG_TRACE("Initializing CAM pins: CAM_PWR_EN=%u (P%u.%u) | CAM_PWR_BUTT=%u (P%u.%u)",
		CAM_PWR_EN, (BSP::GPIO_Inits[CAM_PWR_EN].pin_number >> 5), (BSP::GPIO_Inits[CAM_PWR_EN].pin_number & 0x1F),
		CAM_PWR_BUTT, (BSP::GPIO_Inits[CAM_PWR_BUTT].pin_number >> 5), (BSP::GPIO_Inits[CAM_PWR_BUTT].pin_number & 0x1F));
	clear(CAM_PWR_EN);
	clear(CAM_PWR_BUTT);
	// Force re-enable GPIO configuration to ensure pins are properly configured
	enable(CAM_PWR_EN);
	enable(CAM_PWR_BUTT);
	clear(CAM_PWR_EN);
	clear(CAM_PWR_BUTT);
	DEBUG_TRACE("CAM pins cleared and configured");
#endif

#ifdef VSYS_SEL
	set(VSYS_SEL);
#endif
}

void GPIOPins::init_pin(uint32_t pin)
{
	nrf_gpio_cfg(BSP::GPIO_Inits[pin].pin_number,
				BSP::GPIO_Inits[pin].dir, BSP::GPIO_Inits[pin].input, BSP::GPIO_Inits[pin].pull,
				BSP::GPIO_Inits[pin].drive, BSP::GPIO_Inits[pin].sense);
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

// ═══════════════════════════════════════════════════════
//  VSENSORS power management (reference-counted)
// ═══════════════════════════════════════════════════════

/**
 * @brief Disconnect sensor I2C and interrupt pins when VSENSORS is off.
 *
 * Prevents backfeed current through sensor ESD diodes and floating
 * interrupt lines triggering spurious GPIOTE events.
 */
void GPIOPins::disconnect_sensor_pins()
{
#ifdef SENSORS_PWR_PIN
	// I2C pins — explicit disconnect (workaround for nRF52840 TWIM pin release errata)
#ifdef ONBOARD_I2C_BUS
	nrf_gpio_cfg_default(BSP::I2C_Inits[ONBOARD_I2C_BUS].twim_config.scl);
	nrf_gpio_cfg_default(BSP::I2C_Inits[ONBOARD_I2C_BUS].twim_config.sda);
#endif
	// Sensor interrupt pins — disconnect input to prevent floating
#ifdef BMA400_WAKEUP_PIN
	disable(BMA400_WAKEUP_PIN);
#endif
#ifdef PRESSURE_WAKEUP_PIN
	disable(PRESSURE_WAKEUP_PIN);
#endif
#endif
}

/// @brief Reconnect sensor interrupt pins after VSENSORS powers on.
/// @note I2C bus pins are reconnected by NrfI2C::init(), not here.
void GPIOPins::reconnect_sensor_pins()
{
#ifdef SENSORS_PWR_PIN
#ifdef BMA400_WAKEUP_PIN
	enable(BMA400_WAKEUP_PIN);
#endif
#ifdef PRESSURE_WAKEUP_PIN
	enable(PRESSURE_WAKEUP_PIN);
#endif
#endif
}

void GPIOPins::acquire_sensors_pwr()
{
#ifdef SENSORS_PWR_PIN
	if (m_sensors_pwr_refcount == UINT8_MAX) {
		DEBUG_ERROR("GPIOPins::acquire_sensors_pwr: refcount overflow — capped at %u", m_sensors_pwr_refcount);
		return;
	}
	if (m_sensors_pwr_refcount == 0)
	{
		DEBUG_TRACE("GPIOPins::acquire_sensors_pwr: Powering ON VSENSORS (refcount 0->1)");
		set(SENSORS_PWR_PIN);
		nrf_delay_ms(50);  // 50 ms for sensor power-up stabilization
		reconnect_sensor_pins();
		// Reinit I2C bus (was disabled when VSENSORS went off)
		if (!NrfI2C::is_enabled(0)) {
			NrfI2C::init();
		}
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
		DEBUG_WARN("GPIOPins::release_sensors_pwr: refcount already 0 | ignoring release");
		return;
	}
	m_sensors_pwr_refcount--;
	if (m_sensors_pwr_refcount == 0)
	{
		DEBUG_TRACE("GPIOPins::release_sensors_pwr: Powering OFF VSENSORS (refcount 1->0)");
		// 1. Uninit I2C bus BEFORE cutting power
		NrfI2C::uninit();
		// 2. Disconnect ALL sensor-related pins to prevent backfeed current
		disconnect_sensor_pins();
		// 3. Cut VSENSORS power
		clear(SENSORS_PWR_PIN);
		nrf_delay_ms(50);  // Power supply discharge stabilization
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

// ═══════════════════════════════════════════════════════
//  Pin control helpers
// ═══════════════════════════════════════════════════════

void GPIOPins::pulse_low_then_release(uint32_t pin, uint32_t duration_ms)
{
	uint32_t pin_number = BSP::GPIO_Inits[pin].pin_number;

	nrf_gpio_cfg_output(pin_number);
	nrf_gpio_pin_clear(pin_number);

	nrf_delay_ms(duration_ms);

	// Release to high-impedance — allows external probe to control pin
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
