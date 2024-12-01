#include "virtual_uart.hpp"
#include <iostream>

VirtualUART::VirtualUART(uint32_t tx_pin, uint32_t rx_pin, uint32_t baud_rate)
    : tx_pin_(tx_pin), rx_pin_(rx_pin), baud_rate_(baud_rate) {
    // Calculate the bit period in microseconds based on the baud rate
    bit_period_us_ = 1000000 / baud_rate_;
}

void VirtualUART::init() {
    // Configure TX pin as output and set it high (UART idle state)
    nrf_gpio_cfg_output(tx_pin_);
    nrf_gpio_pin_set(tx_pin_);

    // Configure RX pin as input with pull-up resistor
    nrf_gpio_cfg_input(rx_pin_, NRF_GPIO_PIN_PULLUP);
}

void VirtualUART::delayBitPeriod() const {
    nrf_delay_us(bit_period_us_);
}

void VirtualUART::transmit(uint8_t byte) {
    // Start bit (low for 1 bit period)
    nrf_gpio_pin_clear(tx_pin_);
    delayBitPeriod();

    // Send each data bit (LSB first)
    for (int i = 0; i < 8; ++i) {
        if (byte & (1 << i)) {
            nrf_gpio_pin_set(tx_pin_); // Send 1
        } else {
            nrf_gpio_pin_clear(tx_pin_); // Send 0
        }
        delayBitPeriod();
    }

    // Stop bit (high for at least 1 bit period)
    nrf_gpio_pin_set(tx_pin_);
    delayBitPeriod();
}

uint8_t VirtualUART::receive() {
    uint8_t byte = 0;

    // Wait for start bit (RX goes low)
    while (nrf_gpio_pin_read(rx_pin_) == 1);

    // Wait half a bit period to sample in the middle of each bit
    nrf_delay_us(bit_period_us_ / 2);

    // Read each data bit
    for (int i = 0; i < 8; ++i) {
        delayBitPeriod(); // Move to the center of each bit period
        if (nrf_gpio_pin_read(rx_pin_)) {
            byte |= (1 << i); // Set the bit if RX is high
        }
    }

    // Wait for the stop bit
    delayBitPeriod();

    return byte;
}

void VirtualUART::transmitString(const char* str) {
    while (*str) {
        transmit(static_cast<uint8_t>(*str++));
    }
}
