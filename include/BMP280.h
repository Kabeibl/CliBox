#ifndef _BMP280_H_
#define _BMP280_H_
#include "stdint.h"

/**
 * This is a driver for Bosch BMP280 Digital pressure
 * sensor. All the module detail is based on BMP280 Digital Pressure
 * Sensor -datasheet.
 * */

// BMP operating mode
typedef enum {

    OP_MODE_WEATHER = 0,
    OP_MODE_HUMIDITY,
    OP_MODE_INDOOR_NAV,
    OP_MODE_GAMING
} BMP_OP_MODE;

typedef struct {

    uint16_t    dig_T1;
	int16_t     dig_T2;
	int16_t     dig_T3;
	uint16_t    dig_P1;
	int16_t     dig_P2;
	int16_t     dig_P3;
	int16_t     dig_P4;
	int16_t     dig_P5;
	int16_t     dig_P6;
	int16_t     dig_P7;
	int16_t     dig_P8;
	int16_t     dig_P9;
	uint8_t     dig_H1;
	int16_t     dig_H2;
	uint8_t     dig_H3;
	int16_t     dig_H4;
	int16_t     dig_H5;
	int8_t      dig_H6;
	int32_t     t_fine;
} BME280_COMPENSATION_PARAMS;

void BMP_Init(BMP_OP_MODE mode);
void BMP_Start(void);
void BMP_Measure(double* meas_arr);
void BMP_Reset(void);
void BMP_Sleep(void);

extern int USING_BME280;

#endif