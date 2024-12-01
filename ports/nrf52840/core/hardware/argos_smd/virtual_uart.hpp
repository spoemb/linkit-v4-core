#ifndef VIRTUAL_UART_HPP
#define VIRTUAL_UART_HPP

#include <stdint.h>
#include <nrfx_gpiote.h>
#include <nrf_delay.h>

class VirtualUART {
public:
    // Constructor to initialize with TX and RX pins and baud rate
    VirtualUART(uint32_t tx_pin, uint32_t rx_pin, uint32_t baud_rate = 9600);

    // Initialize the virtual UART (configures GPIO)
    void init();

    // Transmit a single byte
    void transmit(uint8_t byte);

    // Receive a single byte
    uint8_t receive();

    // Transmit a string (array of characters)
    void transmitString(const char* str);

private:
    uint32_t tx_pin_;
    uint32_t rx_pin_;
    uint32_t baud_rate_;
    uint32_t bit_period_us_;

    // Helper to delay for a single bit period
    void delayBitPeriod() const;
};

#endif // VIRTUAL_UART_HPP  