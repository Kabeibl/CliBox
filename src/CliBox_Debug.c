#include "CliBox_Debug.h"
#include "osapi.h"

/* Function declarations */
static void print_double(double d);

void ICACHE_FLASH_ATTR debug_write(char* debug_msg, double value) {

    /**
     * Print debug message in form:
     * "Message value"
     * */

    os_printf("DEBUG:");
    os_printf(debug_msg);
    print_double(value);
    os_printf("\n");
}

static void ICACHE_FLASH_ATTR print_double(double d) {

    int integer,
        decimal;

    // Skip printing of zero
    if(d == 0) return;

    /* Drop decimals */
	integer = (int)d;

    /* Convert first 2 decimals to integer */
	if(integer > 0) {
		decimal = (d - (double)integer) * 100.0;
	}
	else {
		decimal = ((double)integer - d) * 100.0;
	}
	
    /* Print */
	os_printf("%d.%02d", integer, decimal);
}