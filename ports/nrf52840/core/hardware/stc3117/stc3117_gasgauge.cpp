
#include <stdint.h>

#include "stc3117_gasgauge.hpp"
extern "C" {
	#include "Generic_I2C.h"
 	#include "stc311x_BatteryInfo.h"
	#include "stc311x_gasgauge.h"
}
#include "bsp.hpp"
#include "error.hpp"
#include "debug.hpp"
#include "nrf_delay.h"
#include "gpio.hpp"  // For SensorsPowerGuard (VSENSORS management - needed for I2C bus stability)
#include "nrf_gpio.h"
#include "nrf_i2c.hpp"

#ifndef CPPUTEST
#include "crc16.h"
#else
#define crc16_compute(x, y, z)  0xFFFF
#endif

// Thresholds for low/critical battery filtering
#define CRITICIAL_V_THRESHOLD_MV	250
#define LOW_BATT_THRESHOLD			5

// Power optimization: minimal startup wait time
#define STC3117_STARTUP_WAIT_MS     50
#define STC3117_COUNTER_CHECK_MS    50
#define STC3117_MAX_STARTUP_CHECKS  10

static void GasGauge_DefaultInit(GasGauge_DataTypeDef * GG_struct);

// These filtered values should be retained in noinit RAM so they can survive a reset
static __attribute__((section(".noinit"))) volatile uint16_t m_filtered_values[2];
static __attribute__((section(".noinit"))) volatile uint16_t m_crc;

// Function to register the I2C functions
static void setup_i2c() {
    RegisterI2C_Write(GaugeBatteryMonitor::i2c_write);
    RegisterI2C_Read(GaugeBatteryMonitor::i2c_read);
}


GaugeBatteryMonitor::GaugeBatteryMonitor(uint16_t critical_voltage,
		uint8_t low_level
		) :
		BatteryMonitor(low_level, critical_voltage)
{
    DEBUG_INFO("STC3117: Low-power battery monitor initialized");

    // Setup I2C functions but DON'T start the gauge yet (power saving)
    setup_i2c();

    // Set default values until first read
    m_last_voltage_mv = 4100;
    m_last_level = 100;
    m_is_init = false;  // Will init on first update()
}

int GaugeBatteryMonitor::init() {
    SensorsPowerGuard power_guard;  // Acquire VSENSORS for I2C bus stability (other sensors on same bus)
    DEBUG_TRACE("STC3117: Starting gauge for measurement");

    if (m_is_init) {
        return STC3117_OK;
    }

    // Quick device check
    int status = STC31xx_CheckI2cDeviceId();
    if (status != 0) {
        DEBUG_ERROR("STC3117: Device not found (status=%d)", status);
        return status;
    }

    // Config init
    GasGauge_DefaultInit(&STC3117_GG_struct);

    // Start the gas gauge
    status = GasGauge_Start(&STC3117_GG_struct);
    if (status != 0 && status != -2) {
        DEBUG_ERROR("STC3117: Start failed (status=%d)", status);
        m_is_init = false;
        return status;
    }

    // Wait for gauge to be ready (reduced from 500ms + 2s check loop)
    nrf_delay_ms(STC3117_STARTUP_WAIT_MS);

    // Quick ready check
    for (int i = 0; i < STC3117_MAX_STARTUP_CHECKS; i++) {
        int counter = STC31xx_GetRunningCounter();
        if (counter >= 3) {
            break;  // Ready
        }
        if (counter < 0) {
            DEBUG_ERROR("STC3117: Communication error during startup");
            return -1;
        }
        nrf_delay_ms(STC3117_COUNTER_CHECK_MS);
    }

    m_is_init = true;
    DEBUG_TRACE("STC3117: Gauge started successfully");
    return STC3117_OK;
}


int GaugeBatteryMonitor::shutdown() {
    SensorsPowerGuard power_guard;  // Acquire VSENSORS for I2C bus stability (other sensors on same bus)
    if (!m_is_init) {
        return STC3117_OK;
    }

    DEBUG_TRACE("STC3117: Shutting down gauge for power saving");

    // Put gauge in standby/powerdown mode
    int status = GasGauge_Stop();
    if (status != 0) {
        DEBUG_ERROR("STC3117: Shutdown failed (status=%d)", status);
        return status;
    }

    m_is_init = false;
    return STC3117_OK;
}

// Forwarding C++ I2C functions to the C driver
int GaugeBatteryMonitor::i2c_write(int I2cSlaveAddr, int RegAddress, unsigned char* TxBuffer, int NumberOfBytes)
{
    uint8_t reg_addr = static_cast<uint8_t>(RegAddress);

    if (!NumberOfBytes) {
        return STC3117_OK;
    }

    if (NumberOfBytes > static_cast<int>(STC3117_MAX_BUFFER_LEN + sizeof(reg_addr))) {
        return STC3117_E_OUT_OF_RANGE;
    }

    uint8_t buffer[STC3117_MAX_BUFFER_LEN];
    buffer[0] = reg_addr;
    memcpy(&buffer[1], TxBuffer, NumberOfBytes);

    NrfI2C::write(STC3117_DEVICE, I2cSlaveAddr, buffer, NumberOfBytes + sizeof(reg_addr), false);
    return STC3117_OK;
}

int GaugeBatteryMonitor::i2c_read(int I2cSlaveAddr, int RegAddress, unsigned char* RxBuffer, int NumberOfBytes)
{
    uint8_t reg_addr = static_cast<uint8_t>(RegAddress);
    NrfI2C::write(STC3117_DEVICE, I2cSlaveAddr, &reg_addr, sizeof(reg_addr), true);
    NrfI2C::read(STC3117_DEVICE, I2cSlaveAddr, RxBuffer, NumberOfBytes);
    return STC3117_OK;
}

int GaugeBatteryMonitor::check_i2c_device() {
    SensorsPowerGuard power_guard;  // Acquire VSENSORS for I2C bus stability (other sensors on same bus)
    int status = STC31xx_CheckI2cDeviceId();
    if (status != 0) {
        DEBUG_ERROR("STC3117: I2C device check failed (status=%d)", status);
    }
    return status;
}

void GaugeBatteryMonitor::internal_update() {
    // Acquire VSENSORS for I2C bus stability (other sensors on same bus)
    SensorsPowerGuard power_guard;

    // === POWER OPTIMIZED: Wake up → Read → Shutdown ===

    // 1. Initialize gauge (wake from standby)
    int status = this->init();
    if (status != STC3117_OK) {
        DEBUG_ERROR("STC3117: Failed to init for reading");
        // Keep previous values on error
        return;
    }

    // 2. Read battery data
    uint16_t mv = 0;
    uint8_t level = 0;

    status = GasGauge_Task(&STC3117_GG_struct);

    if (status > 0) {
        // New data available
        mv = (uint16_t)STC3117_GG_struct.Voltage;
        level = (uint8_t)(STC3117_GG_struct.SOC / 10);

        DEBUG_INFO("STC3117: V=%umV SOC=%u%% I=%imA",
            mv, level, STC3117_GG_struct.Current);
    }
    else if (status == 0) {
        // Only previous values available (use them)
        mv = (uint16_t)STC3117_GG_struct.Voltage;
        level = (uint8_t)(STC3117_GG_struct.SOC / 10);

        DEBUG_TRACE("STC3117: Using previous values V=%umV SOC=%u%%", mv, level);
    }
    else {
        // Error - keep previous values
        DEBUG_ERROR("STC3117: Read error (status=%d)", status);
        this->shutdown();  // Still shutdown to save power
        return;
    }

    // 3. Shutdown immediately for power saving
    this->shutdown();

    // 4. Apply filtering to prevent value bouncing
    uint16_t crc = crc16_compute((const uint8_t *)m_filtered_values, sizeof(m_filtered_values), nullptr);
    if (crc == m_crc) {
        // Previously filtered values are valid
        if (m_filtered_values[0] < m_critical_voltage_mv) {
            if (mv >= (m_critical_voltage_mv + CRITICIAL_V_THRESHOLD_MV))
                m_filtered_values[0] = mv;
        } else {
            m_filtered_values[0] = mv;
        }
        if (m_filtered_values[1] < m_low_level) {
            if (level >= (m_low_level + LOW_BATT_THRESHOLD))
                m_filtered_values[1] = level;
        } else {
            m_filtered_values[1] = level;
        }
    } else {
        // No previous values - set new values
        m_filtered_values[0] = mv;
        m_filtered_values[1] = level;
    }

    // Update CRC in noinit RAM
    m_crc = crc16_compute((const uint8_t *)m_filtered_values, sizeof(m_filtered_values), nullptr);

    // Apply new values
    m_last_voltage_mv = mv;
    m_last_level = level;

    // Set flags
    m_is_critical_voltage = m_filtered_values[0] < m_critical_voltage_mv;
    m_is_low_level = m_filtered_values[1] < m_low_level;
}

static void GasGauge_DefaultInit(GasGauge_DataTypeDef * GG_struct)
{
    int Rint;

    // Structure initialization
    GG_struct->Cnom = BATT_CAPACITY;        // Nominal battery capacity in mAh
    GG_struct->Vmode = MONITORING_MODE;     // 1=Voltage mode, 0=mixed mode

    Rint = BATT_RINT;
    if (Rint == 0) Rint = 200;  // Force default

    GG_struct->VM_cnf = (int)((Rint * BATT_CAPACITY) / 977.78);

    if (MONITORING_MODE == MIXED_MODE) {
        GG_struct->CC_cnf = (int)((RSENSE * BATT_CAPACITY) / 49.556);
    }

    // SOC curve adjustment
    GG_struct->SoctabValue[0] = 0;
    GG_struct->SoctabValue[1] = 3 * 2;
    GG_struct->SoctabValue[2] = 6 * 2;
    GG_struct->SoctabValue[3] = 10 * 2;
    GG_struct->SoctabValue[4] = 15 * 2;
    GG_struct->SoctabValue[5] = 20 * 2;
    GG_struct->SoctabValue[6] = 25 * 2;
    GG_struct->SoctabValue[7] = 30 * 2;
    GG_struct->SoctabValue[8] = 40 * 2;
    GG_struct->SoctabValue[9] = 50 * 2;
    GG_struct->SoctabValue[10] = 60 * 2;
    GG_struct->SoctabValue[11] = 65 * 2;
    GG_struct->SoctabValue[12] = 70 * 2;
    GG_struct->SoctabValue[13] = 80 * 2;
    GG_struct->SoctabValue[14] = 90 * 2;
    GG_struct->SoctabValue[15] = 100 * 2;

#ifdef DEFAULT_BATTERY_4V20_MAX
    GG_struct->OcvValue[0] = 0x1770;
    GG_struct->OcvValue[1] = 0x1926;
    GG_struct->OcvValue[2] = 0x19B2;
    GG_struct->OcvValue[3] = 0x19FB;
    GG_struct->OcvValue[4] = 0x1A3E;
    GG_struct->OcvValue[5] = 0x1A6D;
    GG_struct->OcvValue[6] = 0x1A9D;
    GG_struct->OcvValue[7] = 0x1AB6;
    GG_struct->OcvValue[8] = 0x1AD5;
    GG_struct->OcvValue[9] = 0x1B01;
    GG_struct->OcvValue[10] = 0x1B70;
    GG_struct->OcvValue[11] = 0x1BB1;
    GG_struct->OcvValue[12] = 0x1BE8;
    GG_struct->OcvValue[13] = 0x1C58;
    GG_struct->OcvValue[14] = 0x1CF3;
    GG_struct->OcvValue[15] = 0x1DA9;
#endif

#ifdef DEFAULT_BATTERY_4V35_MAX
    GG_struct->OcvValue[0] = 0x1770;
    GG_struct->OcvValue[1] = 0x195D;
    GG_struct->OcvValue[2] = 0x19EE;
    GG_struct->OcvValue[3] = 0x1A1A;
    GG_struct->OcvValue[4] = 0x1A59;
    GG_struct->OcvValue[5] = 0x1A95;
    GG_struct->OcvValue[6] = 0x1AB6;
    GG_struct->OcvValue[7] = 0x1AC7;
    GG_struct->OcvValue[8] = 0x1AEB;
    GG_struct->OcvValue[9] = 0x1B2B;
    GG_struct->OcvValue[10] = 0x1BCC;
    GG_struct->OcvValue[11] = 0x1C13;
    GG_struct->OcvValue[12] = 0x1C57;
    GG_struct->OcvValue[13] = 0x1D09;
    GG_struct->OcvValue[14] = 0x1DCF;
    GG_struct->OcvValue[15] = 0x1EA2;
#endif

    // Capacity derating by temperature
    GG_struct->CapDerating[6] = 0;  // -20°C
    GG_struct->CapDerating[5] = 0;  // -10°C
    GG_struct->CapDerating[4] = 0;  // 0°C
    GG_struct->CapDerating[3] = 0;  // 10°C
    GG_struct->CapDerating[2] = 0;  // 25°C
    GG_struct->CapDerating[1] = 0;  // 40°C
    GG_struct->CapDerating[0] = 0;  // 60°C

    // Alarm thresholds
    GG_struct->Alm_SOC = 1;         // SOC alarm level %
    GG_struct->Alm_Vbat = 3300;     // Vbat alarm level mV

    // Hardware configuration
    GG_struct->Rsense = RSENSE;
    GG_struct->RelaxCurrent = GG_struct->Cnom / 20;  // Relaxation current < C/20 mA
    GG_struct->ForceExternalTemperature = 0;         // Use internal temperature
}
