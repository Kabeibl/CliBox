#include "BMP280.h"
#include "I2C_CliBox.h"
#include "osapi.h"
#include "CliBox_Debug.h"

#define USING_DEBUGGING

// Device registers
#define REG_ID                      0xD0
#define REG_RESET                   0xE0
#define REG_CTRL_HUM                0xF2
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
#define REG_DIG_H1_0                0xE1
#define REG_DIG_H1_1                0xE2
#define REG_DIG_H2_0                0xE1

// Settings
#define CONF_RESET                  0xB6

// Power modes
#define CONF_MODE_SLEEP             (0x00 & MASK_POWER_MODE)
#define CONF_MODE_FORCED            (0x01 & MASK_POWER_MODE)
#define CONF_MODE_NORMAL            (0x03 & MASK_POWER_MODE)

// Pressure oversampling modes
#define CONF_OSRS_P_0               (0x00 & MASK_OSRS_P)
#define CONF_OSRS_P_1               (0x04 & MASK_OSRS_P)
#define CONF_OSRS_P_2               (0x08 & MASK_OSRS_P)
#define CONF_OSRS_P_4               (0x0C & MASK_OSRS_P)
#define CONF_OSRS_P_8               (0x10 & MASK_OSRS_P)
#define CONF_OSRS_P_16              (0x14 & MASK_OSRS_P)

// Temperature oversampling modes
#define CONF_OSRS_T_0               (0x00 & MASK_OSRS_T)
#define CONF_OSRS_T_1               (0x20 & MASK_OSRS_T)
#define CONF_OSRS_T_2               (0x40 & MASK_OSRS_T)
#define CONF_OSRS_T_4               (0x60 & MASK_OSRS_T)
#define CONF_OSRS_T_8               (0x80 & MASK_OSRS_T)
#define CONF_OSRS_T_16              (0xA0 & MASK_OSRS_T)

// Humidity oversampling modes
#define CONF_OSRS_H_0               (0x00 & MASK_OSRS_H)
#define CONF_OSRS_H_1               (0x01 & MASK_OSRS_H)
#define CONF_OSRS_H_2               (0x02 & MASK_OSRS_H)
#define CONF_OSRS_H_4               (0x03 & MASK_OSRS_H)
#define CONF_OSRS_H_8               (0x04 & MASK_OSRS_H)
#define CONF_OSRS_H_16              (0x05 & MASK_OSRS_H)

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
#define MASK_OSRS_T         0xE0
#define MASK_OSRS_H         0x07
#define MASK_SPI_MODE       0x01
#define MASK_IIR_FILTER     0x1C
#define MASK_T_STANDBY      0xE0

// Temperature and pressure compensation parameters
uint16_t        dig_T1, dig_P1;
int16_t         dig_T2, dig_T3,
                dig_P2, dig_P3,
                dig_P4, dig_P5,
                dig_P6, dig_P7,
                dig_P8, dig_P9,
                dig_H2, dig_H4,
                dig_H5;
uint8_t         dig_H1, dig_H3;
int8_t          dig_H6;
int             t_fine;

// Device type and operating mode
int             USING_BME280 = 0;
BMP_OP_MODE     op_mode = 0;

// Prototypes
double          compensate_temperature(int adc_T);
double          compensate_pressure(int adc_P);
uint32_t        compensate_humidity(int adc_H);
void            set_operating_mode();
void            read_compensation_parameters(void);
void            parse_compensation_parameters(const uint8_t *arr);
void            read_chip_ID(void);
extern void     i2c_master_writeByte(uint8_t byte);

extern void     delay_ms(uint32_t ms);

void ICACHE_FLASH_ATTR BMP_Init(BMP_OP_MODE mode) {
    /**
     * Initialize BMP280-module.
     * */

    op_mode = mode;

    read_chip_ID();
    read_compensation_parameters();
    set_operating_mode();

    #ifdef USING_DEBUGGING
        debug_write("BMP280 Initialized.", 0);
    #endif
}

void ICACHE_FLASH_ATTR BMP_Start(void) {

     /**
     * Set BMP-module to normal mode.
     * This will start measuring continuously.
     * */

    uint8_t wr_byte,
            rd_byte;

    // Start I2C communication
    i2c_master_start();

    // Write BMP address, set write bit
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
}

void ICACHE_FLASH_ATTR BMP_Measure(double* meas_arr) {

    /**
     * Wake up the module and set it to continuous reading mode.
     * Burst read pressure and temperature data.
     * Put module back to sleep.
     * */

    uint8_t     wr_byte,
                rd_byte_arr[8];
    int         adc_T, 
                adc_P;
    uint32_t    adc_H;

    BMP_Start();

    // Start I2C communication
    i2c_master_start();

    // Send address byte with write bit
    wr_byte = BME280_I2C_ADDRESS | I2C_WRITE;             
    i2c_master_writeByte(wr_byte); 
    i2c_master_getAck();

    // Send register byte
    wr_byte = REG_PRESS_MSB;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Start I2C communication again (module specific way of reading)
    i2c_master_start();

    // Send address byte with read bit
    wr_byte = BME280_I2C_ADDRESS | I2C_READ;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    if(USING_BME280) {
        // Burst read from register 0xF7 to 0xFE (8 bytes)
        for(int i = 0; i < 8; i++) {

            rd_byte_arr[i]= i2c_master_readByte();
            
            if(i < 7) {
                i2c_master_send_ack();
            }
            else {
                i2c_master_send_nack();
            }
        }
    }
    else {
        // Burst read from register 0xF7 to 0xFC (6 bytes)
        for(int i = 0; i < 6; i++) {

            rd_byte_arr[i]= i2c_master_readByte();
            
            if(i < 5) {
                i2c_master_send_ack();
            }
            else {
                i2c_master_send_nack();
            }
        }
    }

    // Burst read 6 bytes from slave
    for(int i = 0; i < sizeof(rd_byte_arr); i++) {

        rd_byte_arr[i]= i2c_master_readByte();
        
        if(i < sizeof(rd_byte_arr - 1)) {
            i2c_master_send_ack();
        }
        else {
            i2c_master_send_nack();
        }
    }

    i2c_master_stop();

    BMP_Sleep();

    // Copy pressure and temperature values
    adc_P = (rd_byte_arr[0] << 16) |
            (rd_byte_arr[1] << 8) |
            rd_byte_arr[2];
    adc_T = (rd_byte_arr[3] << 16) |
            (rd_byte_arr[4] << 8) |
            rd_byte_arr[5];
    
    meas_arr[0] = compensate_temperature(adc_T);
    meas_arr[1] = compensate_pressure(adc_P);

    if(USING_BME280) {

        adc_H = (rd_byte_arr[6] << 8) |
                rd_byte_arr[7];
        meas_arr[2] = compensate_humidity(adc_H);
    }
}

void ICACHE_FLASH_ATTR BMP_Reset(void) {
    /**
     * Reset BMP280-module.
     * */

    uint8_t wr_byte;

    // Start I2C communication
    i2c_master_start();

    // Write BMP address
    wr_byte = BME280_I2C_ADDRESS | 0x00;
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

    delay_ms(2);
}

void ICACHE_FLASH_ATTR BMP_Sleep(void) {
    /**
     * Put BMP-module to sleep mode
     * */

    uint8_t wr_byte,
            rd_byte;

    // Start I2C communication
    i2c_master_start();

    // Write BMP address, set write bit
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
    i2c_master_send_nack();             // End communication

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
              (~CONF_MODE_NORMAL |      // Disable normal mode
              ~CONF_MODE_FORCED)) |     // Disable forced mode
              CONF_MODE_SLEEP;          // Enable sleep mode
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    i2c_master_stop();

}

double ICACHE_FLASH_ATTR compensate_temperature(int adc_T) {

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

double ICACHE_FLASH_ATTR compensate_pressure(int adc_P) {

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

uint32_t ICACHE_FLASH_ATTR compensate_humidity(int adc_H) {

    /**
     * BME280 - 4.2.3 Compensation formulas
     * 
     * Description: Returns humidity in %RH as unsigned 32 bit integer in 
     * Q22.10 format (22 integer and 10 fractional bits).
     * Output value of “47445” represents 47445/1024 = 46.333 %RH
     * */

    int v_x1_u32r;

    v_x1_u32r = (t_fine - ((int)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int)dig_H4) << 20) - (((int)dig_H5) * v_x1_u32r)) + 
    ((int)16384)) >> 15) * (((((((v_x1_u32r * ((int)dig_H6)) >> 10) * (((v_x1_u32r * 
    ((int)dig_H3)) >> 11) + ((int)32768))) >> 10) + ((int)2097152)) *
    ((int)dig_H2) + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t)(v_x1_u32r>>12);
}

void ICACHE_FLASH_ATTR set_operating_mode() {
    
    /**
     * Set measurement mode 
     * (BMP280 Digital Pressure Sensor / 4.3.4 Register 0xF4 "CTRL_MEAS")
     * ---------------------------------------
     * |7           5|4           2|1       0|
     * |-------------------------------------|
     * |    osrs_t   |    osrs_p   |  mode   |
     * ---------------------------------------
     * 
     * (BME280 Digital Pressure Sensor / 5.3.3 Register 0xF2 "CTRL_HUM")
     * ---------------------------------------
     * |7                       3|2         0|
     * |-------------------------------------|
     * |                         |  osrs_h   |
     * ---------------------------------------
     * 
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
    wr_byte = BME280_I2C_ADDRESS | 0x00;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    switch(op_mode) {

        default:
        case OP_MODE_WEATHER:
        /**
         * BME280 - 3.5.1 Weather monitoring
         * 
         * Description: Only a very low data rate is needed. Power consumption is minimal. Noise of
         * pressure values is of no concern. Humidity, pressure and temperature are monitored. 
         * */

            if(USING_BME280) {

                // Write register byte
                wr_byte = REG_CTRL_HUM;
                i2c_master_writeByte(wr_byte);
                i2c_master_getAck();

                // Write data byte
                wr_byte = CONF_OSRS_H_1;            // Set humidity oversampling to ultra low power mode
                i2c_master_writeByte(wr_byte);
                i2c_master_getAck();
            }
            

            // Write register byte
            wr_byte = REG_CTRL_MEAS;
            i2c_master_writeByte(wr_byte);
            i2c_master_getAck();

            // Write data byte
            wr_byte = CONF_MODE_FORCED |        // Set module to forced mode
                      CONF_OSRS_P_1 |           // Set pressure oversampling to ultra low power mode
                      CONF_OSRS_T_1;            // Set temperature oversampling to ultra low power mode
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

            break;
        
        case OP_MODE_HUMIDITY:
        /**
         * BME280 - 3.5.2 Humidity sensing
         * 
         * Description: A low data rate is needed. Power consumption is minimal. Forced mode is used to
         * minimize power consumption and to synchronize readout, but using normal mode would also be
         * possible.
         * */
            break;

        case OP_MODE_INDOOR_NAV:
        /**
         * BME - 3.5.3 Indoor navigation
         * Description: Lowest possible altitude noise is needed. A very low bandwidth is preferred. Increased power
         * consumption is tolerated. Humidity is measured to help detect room changes. This setting is
         * suggested for the Android settings ‘SENSOR_DELAY_NORMAL’ and ‘SENSOR_DELAY_UI’.
         * */
            break;

        case OP_MODE_GAMING:
        /**
         * BME - 3.5.4 Gaming
         * Description: Low altitude noise is needed. The required bandwidth is ~2 Hz in order to respond quickly to
         * altitude changes (e.g. be able to dodge a flying monster in a game). Increased power
         * consumption is tolerated. Humidity sensor is disabled. This setting is suggested for the Android
         * settings ‘SENSOR_DELAY_GAMING’ and ‘SENSOR_DELAY_FASTEST’.
         * */
            break;
    }

    i2c_master_stop();
}

void ICACHE_FLASH_ATTR read_compensation_parameters(void) {

    /**
     * Read compensation parameters from registers
     * at initialization. These parameters are used to
     * convert the ADC-values of temperature, pressure and humidity.
     * */

    uint8_t wr_byte,
            rd_byte_arr[34];

    // Start I2C communication
    i2c_master_start();

    // Write BMP address, set write bit
    wr_byte = BME280_I2C_ADDRESS  | 0x00;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Write register byte
    wr_byte = REG_DIG_T1_0;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Start I2C communication again (Module specific way of reading from register)
    i2c_master_start();

    // Write BMP address, set read bit
    wr_byte = BME280_I2C_ADDRESS | 0x01;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Read all compensation values to an array
    if(USING_BME280) {

        for(int i = 0; i < sizeof(rd_byte_arr - 7); i++) {

            rd_byte_arr[i]= i2c_master_readByte();
            
            if(i < sizeof(rd_byte_arr - 8)) {
                i2c_master_send_ack();
            }
            else {
                i2c_master_send_nack();
                i2c_master_stop();
            }
        }

        // Start I2C communication
        i2c_master_start();

        // Write BME address, set write bit
        wr_byte = BME280_I2C_ADDRESS  | 0x00;
        i2c_master_writeByte(wr_byte);
        i2c_master_getAck();

        // Write register byte
        wr_byte = REG_DIG_H1_0;
        i2c_master_writeByte(wr_byte);
        i2c_master_getAck();

        // Start I2C communication again (Module specific way of reading from register)
        i2c_master_start();

        // Write BMP address, set read bit
        wr_byte = BME280_I2C_ADDRESS | 0x01;
        i2c_master_writeByte(wr_byte);
        i2c_master_getAck();

        for(int i = sizeof(rd_byte_arr - 7); i < sizeof(rd_byte_arr); i++) {

            rd_byte_arr[i]= i2c_master_readByte();

            if(i < sizeof(rd_byte_arr - 1)) {
                i2c_master_send_ack();
            }
            else {
                i2c_master_send_nack();
                i2c_master_stop();
            }
        }
    }
    else {

        for(int i = 0; i < sizeof(rd_byte_arr - 9); i++) {

            rd_byte_arr[i]= i2c_master_readByte();
            
            if(i < sizeof(rd_byte_arr - 10)) {
                i2c_master_send_ack();
            }
            else {
                i2c_master_send_nack();
            }
        }
    }

    i2c_master_stop();

    parse_compensation_parameters(rd_byte_arr);
}

void ICACHE_FLASH_ATTR parse_compensation_parameters(const uint8_t* arr) {

    // Copy compensation values to global variables
    dig_T1 = arr[0] | (arr[1] << 8);
    dig_T2 = arr[2] | (arr[3] << 8);
    dig_T3 = arr[4] | (arr[5] << 8);

    dig_P1 = arr[6] | (arr[7] << 8);
    dig_P2 = arr[8] | (arr[9] << 8);
    dig_P3 = arr[10] | (arr[11] << 8);
    dig_P4 = arr[12] | (arr[13] << 8);
    dig_P5 = arr[14] | (arr[15] << 8);
    dig_P6 = arr[16] | (arr[17] << 8);
    dig_P7 = arr[18] | (arr[19] << 8);
    dig_P8 = arr[20] | (arr[21] << 8);
    dig_P9 = arr[22] | (arr[23] << 8);

    if(USING_BME280) {
        dig_H1 = arr[24];
        dig_H2 = arr[25] | (arr[26] << 8);
        dig_H3 = arr[27];
        dig_H4 = arr[28] | (arr[29] << 8);
        dig_H5 = arr[30] | (arr[31] << 8);
        dig_H6 = arr[32];
    }

    #ifdef USING_DEBUGGING
    
        debug_write("Compensation parameters:: ", 0);
        debug_write("T1: ", dig_T1);
        debug_write("T2: ", dig_T2);
        debug_write("T3: ", dig_T3);
        debug_write("P1: ", dig_P1);
        debug_write("P2: ", dig_P2);
        debug_write("P3: ", dig_P3);
        debug_write("P4: ", dig_P4);
        debug_write("P5: ", dig_P5);
        debug_write("P6: ", dig_P6);
        debug_write("P7: ", dig_P7);
        debug_write("P8: ", dig_P8);
        debug_write("P9: ", dig_P9);

        if(USING_BME280) {
            debug_write("H1: ", dig_H1);
            debug_write("H2: ", dig_H2);
            debug_write("H3: ", dig_H3);
            debug_write("H4: ", dig_H4);
            debug_write("H5: ", dig_H5);
            debug_write("H6: ", dig_H6);
        }

    #endif // USING_DEBUGGING
}

void ICACHE_FLASH_ATTR read_chip_ID(void) {

    /**
     * Read device ID.
     * BME280: 0x60
     * BMP280: 0x56/0x57 (samples)
     *         0x58 (mass production)
     * */

    uint8_t wr_byte,
            rd_byte,
            device_ID;
    
    // Start I2C communication
    i2c_master_start();

    // Write BMP address, set write bit
    wr_byte = BME280_I2C_ADDRESS  | 0x00;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Write register byte
    wr_byte = REG_ID;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    // Start I2C communication again (Module specific way of reading from register)
    i2c_master_start();

    // Write BMP address, set read bit
    wr_byte = BME280_I2C_ADDRESS | 0x01;
    i2c_master_writeByte(wr_byte);
    i2c_master_getAck();

    rd_byte = i2c_master_readByte();

    i2c_master_stop();

    device_ID = rd_byte;

    if(device_ID == 0x60) {
        USING_BME280 = 1;
    }

    #ifdef USING_DEBUGGING
        debug_write("Device ID:", device_ID);
        debug_write("\n", 0);
    #endif // USING_DEBUGGING
}