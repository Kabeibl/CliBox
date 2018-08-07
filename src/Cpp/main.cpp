/* HEADERS */
#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "math.h"
#include "NTC_MF52.h"


/* GLOBAL VARIABLES */
static const 	    int 		led_pin 		= 2;
static 			    os_timer_t 	timer;
static              NTC_MF52    thermistor;

/* PROTOTYPES */
extern void 	uart_div_modify	(int i, int j);
static void 	init 			(void);
static void		blink 			(void);

/* TIMER CALLBACK FUNCTION */
void timer_cb(void *arg) {

	thermistor.read_temperature();
}

/* MAIN */
void 	ICACHE_FLASH_ATTR user_init() {

	init();
}

/* FUNCTIONS */
void  	ICACHE_FLASH_ATTR init 				(void) {

	gpio_init(); 							/* Initialise GPIO */
	uart_div_modify(0, UART_CLK_FREQ / 115200);
	os_timer_setfn(&timer, timer_cb, NULL); /* Setup timer */
	os_timer_arm(&timer, 100, 1); 			/* Set timer repeating with 100ms delay */
}

void	ICACHE_FLASH_ATTR blink 	   		(void) {

	if(GPIO_REG_READ(GPIO_OUT_ADDRESS) & (1 << led_pin)) {
		gpio_output_set(0, (1 << led_pin), (1 << led_pin), 0);
	}
	else {
		gpio_output_set((1 << led_pin),0 , (1 << led_pin), 0);
	}
}