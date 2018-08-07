#include "NTC_MF52.h"
#include "math.h"
#include "osapi.h"

extern double 	log				(double d);
extern int 		system_adc_read	(void);

NTC_MF52::NTC_MF52() {

}

void    ICACHE_FLASH_ATTR NTC_MF52::read_temperature  (void) {

    /* Variables */
	double res 	= 0,
		   temp 	= 0;

	
	if(!read_avg_voltage()) return;		// Read voltage
	res = calculate_res(res);			// Calculate resistance
	temp = steinhart_eq(res);			// Calculate temperature

	os_printf("Temperature: ");			// Print temperature
	print_double(temp);					//

	avg_voltage = 0;
}

int 	ICACHE_FLASH_ATTR NTC_MF52::read_avg_voltage  (void) {

    int voltage = system_adc_read();

	if(avg_counter++ < N_SAMPLES) {
		avg_voltage += voltage;
		return 0;
	}

	avg_voltage /= N_SAMPLES;
	avg_counter = 0;
	return 1;
}

double 	ICACHE_FLASH_ATTR NTC_MF52::calculate_res     (double res) {
    
    res = (1023.0 / avg_voltage) - 1.0;
	res = RESISTOR / res;
	return res;
}

double 	ICACHE_FLASH_ATTR NTC_MF52::steinhart_eq	    (int res) {


	double temperature;

	/*
	temperature = res / (double)R_ROOM;		// R / R_BODY
	os_printf("Temp before log: ");
	print_double(temperature);

	temperature = log((double)temperature);		// ln(R/R_BODY)
	os_printf("Temp after log: ");
	print_double(temperature);

	temperature /= B;					// 1/B * ln(R/R_BODY)
	temperature += 1.0 / (R_ROOM + T_ROOM);	// + (1/T_BODY)
	temperature = 1.0 / temperature;	// Invert
	temperature -= T_ROOM;				// Convert to celcius

	*/

	temperature = (B * T_ROOM) / 
				  (B + (T_ROOM * log(res / R_ROOM)));
	temperature -= 273.15;

	return temperature;
}

void 	ICACHE_FLASH_ATTR NTC_MF52::print_double	    (double d) {

    int integer, decimal;					// Declare variables

	integer = (int)d;						// Drop decimals

	if(integer >= 0) {
		decimal = (d - (double)integer) * 100;	// Convert first 2 decimals to integer
	}

	else {
		decimal = ((double)integer - d) * 100;
	}
	

	os_printf("%d.%02d\n", integer, decimal);	// Print
}

~NTC_MF52::NTC_MF52() {

}