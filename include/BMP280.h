#ifndef _BMP280_
#define _BMP280_

/**
 * This is a driver for Bosch BMP280 Digital pressure
 * sensor. All the module detail is based on BMP280 Digital Pressure
 * Sensor -datasheet.
 * */

extern void     BMP_Init(void);
extern void     BMP_Start(void);
extern void     BMP_Measure(double* meas_arr);
extern void     BMP_Reset(void);
extern void     BMP_Sleep(void);

#endif