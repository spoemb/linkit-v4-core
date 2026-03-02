# Hardware Abstraction Layer

## Overview

The firmware uses abstract C++ interfaces in `core/hardware/` that are implemented in `ports/nrf52840/core/hardware/`. This allows the core firmware to be portable across different MCUs.

## Sensor Abstraction

### Sensor Base Class
Location: `core/hardware/sensor.hpp`

```cpp
class Sensor : public Calibratable {
public:
    Sensor(const char *name = "Sensor");
    virtual double read(unsigned int port = 0) = 0;
    virtual void install_event_handler(unsigned int, std::function<void()>) {}
    virtual void remove_event_handler(unsigned int) {}
    virtual void set_full_scale(unsigned int mode) {}
};
```

### SensorManager
Static registry for named sensors:
- add(Sensor&, name) - Register sensor
- find_by_name(name) - Lookup sensor by name
- clear() - Remove all sensors

### Sensor Implementations

| Name | Driver | Bus | File |
|------|--------|-----|------|
| "PRS" | LPS28DFW (pressure) | I2C | ports/nrf52840/core/hardware/lps28dfw/ |
| "AXL" | BMA400 (accelerometer) | I2C | ports/nrf52840/core/hardware/bma400/ |
| "RTD" | OEM RTD (temperature) | I2C | ports/nrf52840/core/hardware/ |
| "TSYS01" | TSYS01 (temperature) | I2C | ports/nrf52840/core/hardware/ |
| "ALS" | LTR-303 (ambient light) | I2C | ports/nrf52840/core/hardware/ |
| "PH" | OEM pH | I2C | ports/nrf52840/core/hardware/ |
| "CDT" | AD5933 + ADS1115 | I2C | ports/nrf52840/core/hardware/ |
| "THERM" | NTC Thermistor | ADC | ports/nrf52840/core/hardware/ |

### PressureSensor Hierarchy

```
Sensor (abstract)
  └── PressureSensorDevice (abstract, adds read(temp,pressure))
        └── LPS28DFW (hardware driver)

PressureSensor (concrete wrapper)
  - Owns PressureSensorDevice&
  - Implements Sensor::read(port): 0=pressure(bar), 1=temperature(C)
  - Delegates set_full_scale() to device
```

## Power Management

### SensorsPowerGuard (RAII)
Acquires VSENSORS power rail on construction, releases on destruction. Critical because sensor I2C registers are volatile and lost when VSENSORS is powered off.

```cpp
void LPS28DFW::read(double& temperature, double& pressure) {
    SensorsPowerGuard power_guard;  // VSENSORS ON
    // Re-apply configuration (registers are volatile after power cycle)
    lps28dfw_init_set(&m_ctx, LPS28DFW_DRV_RDY);
    lps28dfw_mode_set(&m_ctx, &m_mode);
    // ... read sensor ...
}  // VSENSORS OFF (guard destroyed)
```

### PMU (Power Management Unit)
Abstract interface for power operations:
- delay_ms() - Blocking delay
- hardware_version() - Get board HW version string
- device_identifier() - Get unique device ID

## Communication Interfaces

### I2C
- NrfI2C::write(bus, addr, data, len, no_stop)
- NrfI2C::read(bus, addr, data, len)
- Two buses: ONBOARD_I2C_BUS, EXTERNAL_I2C_BUS (GenTracker only)

### SPI
- Used for satellite module communication (SMD A+ protocol)
- SPI_SATELLITE bus

### UART
- UART_GPS for u-blox M8 GPS communication
- Debug output via UART or USB CDC

### GPIO
- GPIOPins::set() / clear() / read()
- acquire_sensors_pwr() / release_sensors_pwr() - VSENSORS management
- Named pins defined in BSP (GPS_POWER, GPS_RST, SAT_PWR_EN, etc.)

## LED Abstraction

### Single-Color LED
Location: `core/hardware/led.hpp`, `core/hardware/gpio_led.hpp`

```cpp
class Led {
    virtual void on() = 0;
    virtual void off() = 0;
    virtual bool get_state() = 0;
    virtual void flash(unsigned int period_ms) = 0;
    virtual bool is_flashing() = 0;
};
```

Implementation: `GPIOLed` (GPIO-based) and `NrfLed` (nRF52840 with scheduler-based flash).

### RGB LED
Location: `core/hardware/rgb_led.hpp`

```cpp
enum class RGBLedColor { BLACK, RED, GREEN, BLUE, CYAN, MAGENTA, YELLOW, WHITE };

class RGBLed {
    virtual void set(RGBLedColor color) = 0;
    virtual void off() = 0;
    virtual void flash(RGBLedColor color, unsigned int period_ms = 500) = 0;
    virtual void flash_alternate(RGBLedColor color1, RGBLedColor color2, unsigned int period_ms = 250) = 0;
    virtual bool is_flashing() = 0;
    virtual RGBLedColor get_state() = 0;
};
```

Implementation: `NrfRGBLed` (`ports/nrf52840/core/hardware/nrf_rgb_led.hpp`) -- active-low, timer-based flash.

### LED Pins (all board variants)

| Pin | GPIO | Function |
|-----|------|----------|
| GPIO_LED_RED | P1.07 | Red channel (active low) |
| GPIO_LED_GREEN | P1.10 | Green channel (active low) |
| GPIO_LED_BLUE | P1.04 | Blue channel (active low) |

### Global LED Instances

```cpp
extern RGBLed *status_led;       // RGB status LED (main)
extern Led *ext_status_led;      // Optional external single-color LED
```

The LED state machine (`core/sm/ledsm.hpp`) controls `status_led` during normal operation. Services (e.g., SWS test mode) can temporarily override LED state by writing to `status_led` directly.
