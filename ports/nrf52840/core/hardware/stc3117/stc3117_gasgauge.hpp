#ifndef __stc3117_gasgauge_H
#define __stc3117_gasgauge_H



#include "battery.hpp"
extern "C" {
	#include "stc311x_BatteryInfo.h"
	#include "stc311x_gasgauge.h"
}

class GaugeBatteryMonitor : public BatteryMonitor {
private:
	bool m_is_init = false;
	GasGauge_DataTypeDef STC3117_GG_struct;

    int check_i2c_device();
	int init();

	void internal_update() override;

public:
	GaugeBatteryMonitor(
			uint16_t critical_voltage = 2800,
			uint8_t low_level = 10);
    bool IsInit() { return m_is_init; };

	int shutdown() override;
	
	// Static C++ I2C functions to forward to the C driver
    static int i2c_write(int I2cSlaveAddr, int RegAddress, unsigned char* TxBuffer, int NumberOfBytes);
    static int i2c_read(int I2cSlaveAddr, int RegAddress, unsigned char* RxBuffer, int NumberOfBytes);

};
#endif