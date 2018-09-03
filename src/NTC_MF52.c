#include "NTC_MF52.h"
#include "osapi.h"
#include "math.h"

/* Definitions */
#define R_REF 	    10000.0	// Balance resistor
#define T_ROOM 		298.15	// Room temp
#define	R_ROOM 		11000.0	// Resistance at room temp
#define B 			3500.0	// B coefficient
#define MAX_ADC 	1023.0	// Maximum ADC value (10-bit ADC)
#define N_SAMPLES 	1000	// Number of samples for avg voltage
#define e           2.7183  // Natural number

/* Prototypes */
static double 	ICACHE_FLASH_ATTR read_avg_voltage      (void);
static double 	ICACHE_FLASH_ATTR calculate_res	        (double res, double volt);
static double 	ICACHE_FLASH_ATTR calculate_temp	    (int res);
static void 	ICACHE_FLASH_ATTR print_double	        (double d);
extern int      ICACHE_FLASH_ATTR system_adc_read       (void);

double          ICACHE_FLASH_ATTR measure_NTC_temperature   (void) {

    /* Variables */
	double volt = 0,
           res 	= 0,
		   temp = 0;

	volt = read_avg_voltage();		    // Read voltage
	res = calculate_res(res, volt);		// Calculate resistance
	temp = calculate_temp(res);			// Calculate temperature

	os_printf("Temperature: ");			// Print temperature
	print_double(temp);					//

    return temp;
}

static double 	ICACHE_FLASH_ATTR read_avg_voltage      (void) {

    double  voltage     = 0;
    int     i           = 0;
    
    for(i = 0; i < N_SAMPLES; i++) {
        voltage += system_adc_read();
    }

    voltage /= N_SAMPLES;
	return voltage;
}

static double 	ICACHE_FLASH_ATTR calculate_res	        (double res, double volt) {

    res = ((double)MAX_ADC / volt) - 1.0;
	res = R_REF / res;
	return res;
}

static double 	ICACHE_FLASH_ATTR calculate_temp	    (int res) {

    double temperature;

    /*
    *               B
    *   T = ______________________
    *
    *         /                   \
    *         |      R_THERM      |
    *         | _________________ |
    *      ln |             -B    |
    *         |            ------ |
    *         |            T_ROOM |
    *         | R_ROOM * e        |
    *         \                   /
    */

    temperature = B / log(res / 
                  (R_ROOM * pow(e, (-B / T_ROOM))));

	temperature -= 273.15;

	return temperature;
}

static void 	ICACHE_FLASH_ATTR print_double	        (double d) {

    int integer,
        decimal;

    /* Drop decimals */
	integer = (int)d;

    /* Convert first 2 decimals to integer */
	if(integer >= 0) {
		decimal = (d - (double)integer) * 100;
	}
	else {
		decimal = ((double)integer - d) * 100;
	}
	
    /* Print */
	os_printf("%d.%02d\n", integer, decimal);
}