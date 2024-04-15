#ifndef ADS1292R_H_
#define ADS1292R_H_

#include "driver/spi_master.h"
#include "wifiudp.h"

#define RREG 0x20   // Read n nnnn registers starting at address r rrrr
                    // first byte 001r rrrr (2xh)(2) - second byte 000n nnnn(2)
#define WREG 0x40   // Write n nnnn registers starting at address r rrrr
                    // first byte 010r rrrr (2xh)(2) - second byte 000n nnnn(2)
#define START 0x08  // Start/restart (synchronize) conversions
#define STOP 0x0A   // Stop conversion
#define RDATAC 0x10 // Enable Read Data Continuous mode.

// This mode is the default mode at power-up.
#define SDATAC 0x11 // Stop Read Data Continuously mode
#define RDATA 0x12  // Read data by command; supports multiple read back.

// register address
#define ADS1292_REG_ID 0x00
#define ADS1292_REG_CONFIG1 0x01
#define ADS1292_REG_CONFIG2 0x02
#define ADS1292_REG_LOFF 0x03
#define ADS1292_REG_CH1SET 0x04
#define ADS1292_REG_CH2SET 0x05
#define ADS1292_REG_RLDSENS 0x06
#define ADS1292_REG_LOFFSENS 0x07
#define ADS1292_REG_LOFFSTAT 0x08
#define ADS1292_REG_RESP1 0x09
#define ADS1292_REG_RESP2 0x0A
#define ADS1292_REG_GPIO 0x0B
// SPI HOST
#define ADS1292R_HOST SPI2_HOST
// GPIO pins
#define GPIO_RESET 11
#define GPIO_DRDY 8
#define GPIO_START 10
#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_START) | \
                             (1ULL << GPIO_RESET) | \
                             (1ULL << GPIO_CS))
#define GPIO_LOW_POWER 0
#define GPIO_CS 18
#define GPIO_MOSI 7
#define GPIO_MISO 2
#define GPIO_SCLK 6

void ads1292r_cmd(spi_device_handle_t spi_handle, const uint8_t cmd, bool keep_cs_active);
void ads1292r_send_data(spi_device_handle_t spi_handle, uint8_t data);
uint8_t ads1292r_read_reg(spi_device_handle_t spi_handle, uint8_t reg_addr, uint8_t num);
void ads1292r_write_reg(spi_device_handle_t spi_handle, uint8_t reg_addr, uint8_t data);
void ads1292r_init(spi_device_handle_t spi_handle);
void ads1292r_data(spi_device_handle_t spi_handle, uint8_t *data);
void cs_low();
void cs_high();

#endif
