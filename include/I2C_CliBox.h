#ifndef _I2C_CLIBOX_H_
#define _I2C_CLIBOX_H_
#include "i2c_master.h"

#define BME280_I2C_ADDRESS      0xEC // Datasheet: 1110110 (0x76)

int8_t i2c_bme280_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t i2c_bme280_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

#endif