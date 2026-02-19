# Adding Sensors

Guide for integrating a new hardware sensor into the firmware.

## Sensor Hierarchy

```
Sensor (core/hardware/sensor.hpp)          ← Abstract interface
  └── MyDeviceDriver (ports/nrf52840/...)  ← Hardware-specific driver

MySensorWrapper (ports/nrf52840/...)       ← Wraps device, implements Sensor
  └── Registers with SensorManager("NAME")
```

## Step 1: Create the Device Driver

In `ports/nrf52840/core/hardware/my_sensor/`:

**my_device.hpp:**
```cpp
#pragma once

class MyDevice {
public:
    MyDevice(unsigned int bus, unsigned char address);
    bool init();
    bool is_initialized() const { return m_initialized; }
    void read(double& value1, double& value2);

private:
    unsigned int m_bus;
    unsigned char m_addr;
    bool m_initialized = false;
};
```

**my_device.cpp:**
```cpp
#include "my_device.hpp"
#include "nrf_i2c.hpp"
#include "gpio.hpp"  // For SensorsPowerGuard
#include "debug.hpp"

MyDevice::MyDevice(unsigned int bus, unsigned char address)
    : m_bus(bus), m_addr(address) {
    m_initialized = init();
}

bool MyDevice::init() {
    SensorsPowerGuard power_guard;  // VSENSORS ON for I2C access
    // Read WHOAMI register, configure sensor
    return true;
}

void MyDevice::read(double& value1, double& value2) {
    SensorsPowerGuard power_guard;  // VSENSORS ON

    // IMPORTANT: Re-apply configuration after power cycle!
    // Sensor registers are volatile and lost when VSENSORS is off.
    // Always re-configure before reading.

    // Read sensor via I2C
    NrfI2C::write(m_bus, m_addr, &reg, 1, true);
    NrfI2C::read(m_bus, m_addr, buf, len);

    // Convert raw data
    value1 = /* ... */;
    value2 = /* ... */;
}
```

**Key pattern**: Always use `SensorsPowerGuard` for I2C access and re-apply sensor configuration in `read()` because registers are lost when VSENSORS powers off between calls.

## Step 2: Create the Sensor Wrapper

In `ports/nrf52840/core/hardware/my_sensor/my_sensor.hpp`:

```cpp
#pragma once
#include "sensor.hpp"
#include "my_device.hpp"

class MySensor : public Sensor {
public:
    MySensor(MyDevice& device)
        : Sensor("MY_SENSOR"), m_device(device) {}

    // Sensor interface: read by port number
    double read(unsigned int port = 0) override {
        double v1, v2;
        m_device.read(v1, v2);
        switch (port) {
            case 0: return v1;
            case 1: return v2;
            default: return 0.0;
        }
    }

private:
    MyDevice& m_device;
};
```

The Sensor constructor registers with `SensorManager` using the provided name. Services look up sensors by name (e.g., `SensorManager::find_by_name("MY_SENSOR")`).

## Step 3: Add Calibration Support (Optional)

The `Sensor` class inherits from `Calibratable`. Override calibration methods if needed:

```cpp
class MySensor : public Sensor {
    // ...
    void calibration_write(const double value, const unsigned int offset) override {
        m_calibration.write(offset, value);
    }
    void calibration_read(double& value, const unsigned int offset) override {
        value = m_calibration.read(offset);
    }
    void calibration_save(bool force) override {
        m_calibration.save(force);
    }
private:
    Calibration m_calibration{"my_sensor"};
};
```

Add a SCALW/SCALR mapping in DTEHandler for the new sensor type in `BaseSensorCalType`.

## Step 4: Instantiate in Board Application

In the board-specific main/application code:

```cpp
// Create device driver
MyDevice my_device(ONBOARD_I2C_BUS, MY_DEVICE_I2C_ADDR);

// Create sensor wrapper (auto-registers with SensorManager)
MySensor my_sensor(my_device);

// Create service (see Adding-Services.md)
MySensorService my_service(my_sensor, &my_logger);
```

## Step 5: Add BSP Definitions

In the board's `bsp.hpp`, add I2C address and pin definitions:

```cpp
#define MY_DEVICE_I2C_ADDR    0x5C
#define MY_DEVICE_I2C_BUS     ONBOARD_I2C_BUS
```

## Step 6: Add Build Flag

In `ports/nrf52840/CMakeLists.txt`, add the enable flag:

```cmake
option(ENABLE_MY_SENSOR "Enable my sensor" OFF)
add_definitions(-DENABLE_MY_SENSOR=${ENABLE_MY_SENSOR})
```

## Checklist

- [ ] Device driver with SensorsPowerGuard and config re-apply in read()
- [ ] Sensor wrapper implementing `Sensor::read(port)`
- [ ] Registered with SensorManager via constructor name
- [ ] BSP I2C address and bus definitions
- [ ] CMake ENABLE_* flag
- [ ] Service created (see [Adding Services](Adding-Services.md))
- [ ] Parameters added (see [Adding Parameters](Adding-Parameters.md))
- [ ] Calibration support (if needed)
- [ ] SCALW/SCALR mapping (if calibration needed)
