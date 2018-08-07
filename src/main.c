/* HEADERS */
#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "NTC_MF52.h"
#include "GP2Y1010AU0F.h"

/* GLOBAL VARIABLES */
static const 	    int 		led_pin 		= 2;
static 			    os_timer_t 	timer;

/* PROTOTYPES */
extern void 	uart_div_modify	(int i, int j);
static void 	init 			(void);
static void		blink 			(void);

/* TIMER CALLBACK FUNCTION */
void timer_cb(void *arg) {

	/* Declare variables */
	double temperature,
		   fine_dust,
		   humidity,
		   pressure,
		   altitude;

	/* Measure temperature from NTC_MF52-thermistor */
	temperature = measure_temperature();

	/* Measure fine dust from GP2Y1010AU0F fine dust sensor */
	fine_dust = measure_fine_dust();

	/* Measure humidity from BMP280 Barometric Pressure & Altitude sensor */
	// humidity = measure_humidity();

	/* Measure pressure from BMP280 Barometric Pressure & Altitude sensor */
	// pressure = measure_pressure();

	/* Measure altitude from BMP280 Barometric Pressure & Altitude sensor */
	// altitude = measure_altitude();

	/* Send sensor data to cloud over WiFi */
	// send_sensor_data(temperature,
	// 				 fine_dust
	// 				 humidity,
	// 				 pressure,
	// 				 altitude);


	/* 
	 *  System goes to deep-sleep.
	 *  Upon waking up, the device boots up from user_init.
	 */
	system_deep_sleep(360000000);
}

/* MAIN */
void 	ICACHE_FLASH_ATTR user_init() {

	init();
}

/* FUNCTIONS */
void  	ICACHE_FLASH_ATTR init 				(void) {

	gpio_init(); 									/* Initialise GPIO */
	uart_div_modify(0, UART_CLK_FREQ / 115200);		/* Set UART baud rate */
	os_timer_setfn(&timer, timer_cb, NULL); 		/* Setup timer */
	os_timer_arm(&timer, 500, 1); 					/* Set timer repeating with 500ms delay */
	i2c_master_gpio_init(); 						/* Initialize I2C */
}

void	ICACHE_FLASH_ATTR blink 	   		(void) {

	if(GPIO_REG_READ(GPIO_OUT_ADDRESS) & (1 << led_pin)) {
		gpio_output_set(0, (1 << led_pin), (1 << led_pin), 0);
	}
	else {
		gpio_output_set((1 << led_pin),0 , (1 << led_pin), 0);
	}
}