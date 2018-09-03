#include "BMP280.h"
#include "i2c_master.h"
#include "stdint.h"

// Device address
#define I2C_ADDRESS                 0xEC // Datasheet: 1110110 (0x76) 

// Device registers
#define REG_ID                      0xD0
#define REG_RESET                   0xE0
#define REG_STATUS                  0xF3
#define REG_CTRL_MEAS               0xF4
#define REG_CONFIG                  0xF5
#define REG_PRESS_MSB               0xF7
#define REG_PRESS_LSB               0xF8
#define REG_PRESS_XLSB              0xF9
#define REG_TEMP_MSB                0xFA
#define REG_TEMP_LSB                0xFB
#define REG_TEMP_XLSB               0xFC

// Compensation registers
#define REG_DIG_T1_0                0x88
#define REG_DIG_T1_1                0x89
#define REG_DIG_T2_0                0x8A
#define REG_DIG_T2_1                0x8B
#define REG_DIG_T3_0                0x8C
#define REG_DIG_T3_1                0x8D
#define REG_DIG_P1_0                0x8E
#define REG_DIG_P1_1                0x8F
#define REG_DIG_P2_0                0x90
#define REG_DIG_P2_1                0x91
#define REG_DIG_P3_0                0x92
#define REG_DIG_P3_1                0x93
#define REG_DIG_P4_0                0x94
#define REG_DIG_P4_1                0x95
#define REG_DIG_P5_0                0x96
#define REG_DIG_P5_1                0x97
#define REG_DIG_P6_0                0x98
#define REG_DIG_P6_1                0x99
#define REG_DIG_P7_0                0x9A
#define REG_DIG_P7_1                0x9B
#define REG_DIG_P8_0                0x9C
#define REG_DIG_P8_1                0x9D
#define REG_DIG_P9_0                0x9E
#define REG_DIG_P9_1                0x9F

// Settings
#define CONF_RESET                  0xB6

// Power modes
#define CONF_MODE_SLEEP             (0x00 & MASK_POWER_MODE)
#define CONF_MODE_FORCED            (0x01 & MASK_POWER_MODE)
#define CONF_MODE_NORMAL            (0x03 & MASK_POWER_MODE)

// Pressure oversampling modes
#define CONF_OSRS_P_1               (0x04 & MASK_OSRS_P)
#define CONF_OSRS_P_2               (0x08 & MASK_OSRS_P)
#define CONF_OSRS_P_4               (0x0C & MASK_OSRS_P)
#define CONF_OSRS_P_8               (0x10 & MASK_OSRS_P)
#define CONF_OSRS_P_16              (0x14 & MASK_OSRS_P)

// Temperature oversampling modes
#define CONF_OSRS_T_1               (0x20 & MASK_ORSR_T)
#define CONF_OSRS_T_2               (0x40 & MASK_ORSR_T)
#define CONF_OSRS_T_4               (0x60 & MASK_ORSR_T)
#define CONF_OSRS_T_8               (0x80 & MASK_ORSR_T)
#define CONF_OSRS_T_16              (0xA0 & MASK_ORSR_T)

// SPI modes (4-wire / 3-wire)
#define CONF_SPI_4_WIRE             (0x00 & MASK_SPI_MODE)
#define CONF_SPI_3_WIRE             (0x01 & MASK_SPI_MODE)

// IIR filter formula coefficient (These values are based on assumptions
// The datasheet doesn't tell the actual register values.)
#define CONF_IIR_FILTER_OFF         (0x04 & MASK_IIR_FILTER)    
#define CONF_IIR_FILTER_2           (0x08 & MASK_IIR_FILTER)
#define CONF_IIR_FILTER_4           (0x0C & MASK_IIR_FILTER)    
#define CONF_IIR_FILTER_8           (0x10 & MASK_IIR_FILTER)
#define CONF_IIR_FILTER_16          (0x14 & MASK_IIR_FILTER)

// Standby settings in normal power mode
#define CONF_T_STANDBY_0_5_MS       (0x00 & MASK_T_STANDBY)       
#define CONF_T_STANDBY_62_5_MS      (0x20 & MASK_T_STANDBY)
#define CONF_T_STANDBY_125_MS       (0x40 & MASK_T_STANDBY)
#define CONF_T_STANDBY_250_MS       (0x60 & MASK_T_STANDBY)
#define CONF_T_STANDBY_500_MS       (0x80 & MASK_T_STANDBY)
#define CONF_T_STANDBY_1000_MS      (0xA0 & MASK_T_STANDBY)
#define CONF_T_STANDBY_2000_MS      (0xC0 & MASK_T_STANDBY)
#define CONF_T_STANDBY_4000_MS      (0xE0 & MASK_T_STANDBY)

// Masks
#define MASK_POWER_MODE     0x03
#define MASK_OSRS_P         0x1C
#define MASK_ORSR_T         0xE0
#define MASK_SPI_MODE       0x01
#define MASK_IIR_FILTER     0x1C
#define MASK_T_STANDBY      0xE0

// Temperature and pressure compensation parameters
uint16_t    dig_T1, dig_P1;
int16_t     dig_T2, dig_T3,
            dig_P2, dig_P3,
            dig_P4, dig_P5,
            dig_P6, dig_P7,
            dig_P8, dig_P9;
int         t_fine;

// Prototypes
double compensate_temperature(int adc_T);
double compensate_pressure(int adc_P);
void set_configurations(void);
void set_measurement_mode(void);
void read_compensation_parameters(void);
extern void i2c_master_writeByte(uint8_t);

void BMP_Init(void) {
    /**
     * Initialize BMP280-module.
     * */

    set_configurations();
    set_measurement_mode();
    read_compensation_parameters();
}

void BMP_Start(void) {

     /**
     * Set BMP-module to normal mode.
     * This will start measuring continuously.
     * */

    uint8_t wr_byte,
            rd_byte;

    // Start I2C communication
    i2c_master_start();

    // Write BMP address, set write bit
    wr_byte = I2C_ADDRESS | 0x00;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Write register byte
    wr_byte = REG_CTRL_MEAS;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Start I2C communication again (Module specific way of reading from register)
    i2c_master_start();

    // Write BMP address, set read bit
    wr_byte = I2C_ADDRESS | 0x01;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Read register byte
    rd_byte = i2c_master_readByte();
    i2c_master_send_nack();

    i2c_master_stop();

    i2c_master_start();

    // Write BMP address, set write bit
    wr_byte = I2C_ADDRESS | 0x00;
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
}

void BMP_Measure(double* meas_arr) {

    /**
     * Wake up the module and set it to continuous reading mode.
     * Burst read pressure and temperature data.
     * Put module back to sleep.
     * */

    uint8_t wr_byte,
            rd_bytes[6];
    int     adc_T, 
            adc_P;

    BMP_Start();

    // Start I2C communication
    i2c_master_start();

    // Send address byte with write bit
    wr_byte = I2C_ADDRESS | I2C_WRITE;             
    i2c_master_writeByte(wr_byte); 
    i2c_master_getAck();

    // Send register byte
    wr_byte = REG_PRESS_MSB;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Start I2C communication again (module specific way of reading)
    i2c_master_start();

    // Send address byte with read bit
    wr_byte = I2C_ADDRESS | I2C_READ;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Burst read 6 bytes from slave
    for(int i = 0; i < sizeof(rd_bytes); i++) {

        rd_bytes[i]= i2c_master_readByte();
        
        if(i < sizeof(rd_bytes - 1)) {
            i2c_master_send_ack();
        }
        else {
            i2c_master_send_nack();
        }
    }

    i2c_master_stop();

    BMP_Sleep();

    // Copy pressure and temperature values
    adc_P = (rd_bytes[0] << 16) |
            (rd_bytes[1] << 8) |
            rd_bytes[2];
    adc_T = (rd_bytes[3] << 16) |
            (rd_bytes[4] << 8) |
            rd_bytes[5];
    
    meas_arr[0] = compensate_temperature(adc_T);
    meas_arr[1] = compensate_pressure(adc_P);
}

void BMP_Reset(void) {
    /**
     * Reset BMP280-module.
     * */

    uint8_t wr_byte;

    // Start I2C communication
    i2c_master_start();

    // Write BMP address
    wr_byte = I2C_ADDRESS | 0x00;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Write register byte
    wr_byte = REG_RESET;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Write data byte
    wr_byte = CONF_RESET;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    i2c_master_stop();
}

void BMP_Sleep(void) {
    /**
     * Put BMP-module to sleep mode
     * */

    uint8_t wr_byte,
            rd_byte;

    // Start I2C communication
    i2c_master_start();

    // Write BMP address, set write bit
    wr_byte = I2C_ADDRESS | 0x00;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Write register byte
    wr_byte = REG_CTRL_MEAS;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Start I2C communication again (Module specific way of reading from register)
    i2c_master_start();

    // Write BMP address, set read bit
    wr_byte = I2C_ADDRESS | 0x01;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Read register byte
    rd_byte = i2c_master_readByte();
    i2c_master_send_nack();             // End communication

    i2c_master_stop();

    i2c_master_start();

    // Write BMP address, set write bit
    wr_byte = I2C_ADDRESS | 0x00;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Write register byte
    wr_byte = REG_CTRL_MEAS;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Write data byte
    wr_byte = (rd_byte &                // Fetch current register value from read buffer
              ~CONF_MODE_NORMAL) |      // Disable normal mode
              CONF_MODE_SLEEP;          // Enable sleep mode
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    i2c_master_stop();

}

double compensate_temperature(int adc_T) {

    /**
     * Convert adc temperature to degrees celcius.
     * (BMP280 Digital Pressure Sensor / 3.11.3 Compensation formula)
     * Resolution is 0.01C.
     * Returns temperature in degrees celcius.
     * */

    /* Variables */
    int     var1, var2, T_int;
    double  T;


    var1  = ((((adc_T>>3) - (dig_T1<<1))) * (dig_T2)) >> 11;
    var2  = (((((adc_T>>4) - (dig_T1)) * ((adc_T>>4) - (dig_T1))) >> 12) * (dig_T3)) >> 14;
    t_fine = var1 + var2;
    T_int  = (t_fine * 5 + 128) >> 8;
    T = (double)T_int / 100;

    return T;
}

double compensate_pressure(int adc_P) {

    /**
     * Convert adc pressure to pascals.
     * (BMP280 Digital Pressure Sensor / 3.11.3 Compensation formula)
     * Returns pressure in pascals
     * */

    /* Variables */
    int var1, var2, p;
    double p_out;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0){
        return 0; // Avoid exception caused by division by zero
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    p_out = p / 256;
    
    return p_out;
}

void set_configurations(void) {

    /**
     * Set configuration register
     * (BMP280 Digital Pressure Sensor / 4.3.5 Register 0xF5 "CONFIG")
     * ----------------------------------------
     * |7           5|4           2|1        0|
     * |--------------------------------------|
     * |     t_sb    |    filter   | spi3w_en |
     * ----------------------------------------
     * */

    uint8_t wr_byte;

    // Start I2C communication
    i2c_master_start();

    // Write BMP address
    wr_byte = I2C_ADDRESS | 0x00;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Write config register byte
    wr_byte = REG_CONFIG;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Write data byte
    wr_byte = CONF_SPI_4_WIRE |         // Keep SPI at normal mode (Doesn't matter, we're using I2C)
              CONF_IIR_FILTER_OFF |     // Disable IIR filtering
              CONF_T_STANDBY_0_5_MS;    // Set measurement interval to once every 0.5ms (This for averaging)
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    i2c_master_stop();
}

void set_measurement_mode(void) {
    
    /**
     * Set measurement mode 
     * (BMP280 Digital Pressure Sensor / 4.3.4 Register 0xF4 "CTRL_MEAS")
     * ---------------------------------------
     * |7           5|4           2|1       0|
     * |-------------------------------------|
     * |    osrs_t   |    osrs_p   |  mode   |
     * ---------------------------------------
     * */

     uint8_t wr_byte;

    // Start I2C communication
    i2c_master_start();

    // Write BMP address
    wr_byte = I2C_ADDRESS | 0x00;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Write register byte
    wr_byte = REG_CTRL_MEAS;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Write data byte
    wr_byte = CONF_MODE_SLEEP |         // Set module to sleep mode
              CONF_OSRS_P_1 |           // Set pressure oversampling to ultra low power mode
              CONF_OSRS_T_1;            // Set temperature oversampling to ultra low power mode
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    i2c_master_stop();
}

void read_compensation_parameters(void) {

    /**
     * Read compensation parameters from registers
     * at initialization. These parameters are used to
     * convert the adc values of temperature and pressure to
     * degrees celcius and pascals.
     * */

    uint8_t wr_byte,
            rd_bytes[24];
    // int     i;

    // Start I2C communication
    i2c_master_start();

    // Write BMP address, set write bit
    wr_byte = I2C_ADDRESS  | 0x00;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Write register byte
    wr_byte = REG_DIG_P1_0;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Start I2C communication again (Module specific way of reading from register)
    i2c_master_start();

    // Write BMP address, set read bit
    wr_byte = I2C_ADDRESS | 0x01;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Read all compensation values to an array
    for(int i = 0; i < sizeof(rd_bytes); i++) {

        rd_bytes[i]= i2c_master_readByte();
        
        if(i < sizeof(rd_bytes - 1)) {
            i2c_master_send_ack();
        }
        else {
            i2c_master_send_nack();
        }
    }

    // Copy compensation values to global variables
    dig_T1 = rd_bytes[0] | (rd_bytes[1] << 8);
    dig_T2 = rd_bytes[2] | (rd_bytes[3] << 8);
    dig_T3 = rd_bytes[4] | (rd_bytes[5] << 8);
    dig_P1 = rd_bytes[6] | (rd_bytes[7] << 8);
    dig_P2 = rd_bytes[8] | (rd_bytes[9] << 8);
    dig_P3 = rd_bytes[10] | (rd_bytes[11] << 8);
    dig_P4 = rd_bytes[12] | (rd_bytes[13] << 8);
    dig_P5 = rd_bytes[14] | (rd_bytes[15] << 8);
    dig_P6 = rd_bytes[16] | (rd_bytes[17] << 8);
    dig_P7 = rd_bytes[18] | (rd_bytes[19] << 8);
    dig_P8 = rd_bytes[20] | (rd_bytes[21] << 8);
    dig_P9 = rd_bytes[22] | (rd_bytes[23] << 8);

    i2c_master_stop();
}