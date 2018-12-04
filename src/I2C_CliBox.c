#include "I2C_CliBox.h"
#include "stdint.h"
#include "osapi.h"
#include "BMP280.h"

int8_t ICACHE_FLASH_ATTR i2c_bme280_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {

    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    uint8_t wr_byte, rd_byte;
    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

    
    i2c_master_start();     // Start I2C communication

    wr_byte = BME280_I2C_ADDRESS | 0x00;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Write register byte
    wr_byte = REG_CTRL_MEAS;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Start I2C communication again (Module specific way of reading from register)
    i2c_master_start();

    // Write BMP address, set read bit
    wr_byte = BME280_I2C_ADDRESS | 0x01;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Read register byte
    rd_byte = i2c_master_readByte();
    i2c_master_send_nack();

    i2c_master_stop();

    i2c_master_start();

    // Write BMP address, set write bit
    wr_byte = BME280_I2C_ADDRESS | 0x00;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Write register byte
    wr_byte = REG_CTRL_MEAS;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Write data byte
    wr_byte = (rd_byte &                // Fetch current register value from read buffer
              ~CONF_MODE_SLEEP) |       // Disable sleep mode
              CONF_MODE_NORMAL;         // Enable normal mode
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    i2c_master_stop();

    return rslt;
}

int8_t ICACHE_FLASH_ATTR i2c_bme280_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {

    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */

    return rslt;
}