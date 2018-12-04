#include "ADS_1015.h"
#include "gpio.h"
#include "i2c_master.h"

/* Registers */
#define CONVERSION_REG          (0x00)
#define CONFIG_REG              (0x01)
#define LO_TRESH_REG            (0x02)
#define HI_TRESH_REG            (0x03)

/* MUX Settings */
#define CONF_MUX_AIN0_AIN1      (0x00)      // Default
#define CONF_MUX_AIN0_AIN3      (0x01)
#define CONF_MUX_AIN1_AIN3      (0x02)
#define CONF_MUX_AIN2_AIN3      (0x03)
#define CONF_MUX_AIN0           (0x04)
#define CONF_MUX_AIN1           (0x05)
#define CONF_MUX_AIN2           (0x06)
#define CONF_MUX_AIN3           (0x07)

/* PGA Settings */
#define CONF_PGA_FSR_6_144V     (0x00)
#define CONF_PGA_FSR_4_096V     (0x01)
#define CONF_PGA_FSR_2_046V     (0x02)      // Default
#define CONF_PGA_FSR_1_024V     (0x03)
#define CONF_PGA_FSR_0_512V     (0x04)
#define CONF_PGA_FSR_0_256V     (0x05)

/* Device modes */
#define CONTINUOUS_CONV_MODE    (0x00)
#define SINGLE_SHOT_MODE        (0x01)      // Default

/* Data rate */
#define CONF_DR_128             (0x00)
#define CONF_DR_250             (0x01)
#define CONF_DR_490             (0x02)
#define CONF_DR_920             (0x03)
#define CONF_DR_1600            (0x04)      // Default
#define CONF_DR_2400            (0x05)
#define CONF_DR_3300            (0x06)

/* Comparator queue and disable */
#define CONF_COMP_QUE_1         (0x00)
#define CONF_COMP_QUE_2         (0x01)
#define CONF_COMP_QUE_4         (0x02)
#define CONF_COMP_QUE_DISABLE   (0x03)

/* Latching comparator */
#define CONF_COMP_LATCH_OFF     (0x00)      // Nonlatching comparator (Default)
#define CONF_COMP_LATCH_ON      (0x01)      // Latching comparator (RDY pin remains latched until master reads data)

/* Function prototypes */
/* Comparator polarity */
#define CONF_COMP_POL_LOW       (0x00)      // Active low (Default)
#define CONF_COMP_POL_HIGH      (0x01)      // Active high

/* Comparator mode */
#define CONF_COMP_MODE_TRAD     (0x00)      // Traditional comparator (Default)
#define CONF_COMP_MODE_WINDOW   (0x01)      // Window comparator

/* Initialise ADS1015 */
void        ICACHE_FLASH_ATTR  ADS_init     (void) {

    uint8     wrByte;

    i2c_master_start();                     // Start communication (SCL high, SDA high)

    wrByte = SLAVE_ADDR_1_GND |             // Set slave address and write bit
             (0 << 0);                      // Set rd/wr-byte to write
    i2c_master_writeByte(wrByte);           // Write byte to slave
    if(!i2c_master_checkAck()) {            // Check ACK from slave
        // Error
    }

    wrByte = CONFIG_REG;                    // Set register to write to
    i2c_master_writeByte(wrByte);           // Write byte to slave
    if(!i2c_master_checkAck()) {            // Check ACK from slave
        // Error
    }

    wrByte = SINGLE_SHOT_MODE |             // Set ADC to single shot mode
             CONF_PGA_FSR_2_046V |          // Set amplifier to range to +- 2.046V
             CONF_MUX_AIN0 |                // Set MUX to single ended input from AIN0
             (1 << 7);                      // Start conversion
    i2c_master_writeByte(wrByte);           // Write byte to slave
    if(!i2c_master_checkAck()) {            // Check ACK from slave
        // Error
    }

    wrByte = CONF_COMP_QUE_DISABLE |        // Disable comparator and set ALERT/RDY pin high
             CONF_DR_1600;                  // Set data rate to 1600 SPS
    i2c_master_writeByte(wrByte);           // Write byte to slave
    if(!i2c_master_checkAck()) {            // Check ACK from slave
        // Error
    }

    i2c_master_stop();                     // Stop communication (SCL high, SDA low)       
}

/* Read data from ADS1015 */
uint16      ICACHE_FLASH_ATTR  ADS_read_adc (uint8 slave_addr) {

    /* Variables */
    uint8     wrByte,
                rdByte;
    uint16    measurement;

    i2c_master_start();                     // Start communication (SCL high, SDA high)

    wrByte = slave_addr |                   // Set slave address
             (1 << 0);                      // Set rd/wr-byte to read
    i2c_master_writeByte(wrByte);           // Write byte to slave
    if(!i2c_master_checkAck()) {            // Check ACK from slave
        // Error
    }

    wrByte = CONVERSION_REG;                // Set register to write to
    i2c_master_writeByte(wrByte);           // Write byte to slave
    if(!i2c_master_checkAck()) {            // Check ACK from slave
        // Error
    }

    if(!(rdByte = i2c_master_readByte())) { // Read MSB from conversion register
        // Error
    }
    i2c_master_send_ack();                  // Send ACK to slave   
    measurement = rdByte;                   // Set MSB of measurement

    if(!(rdByte = i2c_master_readByte())) { // Read MSB from conversion register
        // Error
    }
    i2c_master_send_ack();                  // Send ACK to slave          
    measurement |= rdByte;                  // Set LSB of measurement

    i2c_master_stop();                     // Stop communication (SCL high, SDA low)
    return measurement;
}
