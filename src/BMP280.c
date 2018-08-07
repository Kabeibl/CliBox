#include "BMP280.h"
#include "i2c_master.h"

/* Device address */
#define I2C_ADDRESS     0xEA

/* Device registers */
#define REG_ID          0xD0
#define REG_RESET       0xE0
#define REG_STATUS      0xF3
#define REG_CTRL_MEAS   0xF4
#define REG_CONFIG      0xF5
#define REG_PRESS_MSB   0xF7
#define REG_PRESS_LSB   0xF8
#define REG_PRESS_XLSB  0xF9
#define REG_TEMP_MSB    0xFA
#define REG_TEMP_LSB    0xFB
#define REG_TEMP_XLSB   0xFC

double measure_humidity(void) {

    uint8 wr_byte,
          r_byte;

    i2c_master_start();

    wr_byte = I2C_ADDRESS |     // Set slave address 
              (1 << 0);         // Set read bit up
    i2c_master_wrByte(wr_byte);
    i2c_master_readAck();
    
    wr_byte = REG_

    i2c_master_readByte();
}