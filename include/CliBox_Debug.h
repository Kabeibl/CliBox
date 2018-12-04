#ifndef _CLIBOX_DEBUG_
#define _CLIBOX_DEBUG_

#include "stdio.h"
#include "stdarg.h"

/**
 * Serial print-out for debugging purposes
 * */

extern void debug_write(char* debug_msg, double value);

#endif