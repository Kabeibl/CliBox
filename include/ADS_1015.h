#ifndef _ADS_1015_
#define _ADS_1015_
#include "osapi.h"

/* I2C Address pins */
#define SLAVE_ADDR_1_GND        (0x90)      // Binary: 1001000x
#define SLAVE_ADDR_2_VDD        (0x92)      // Binary: 1001001x
#define SLAVE_ADDR_3_SDA        (0x94)      // Binary: 1001010x
#define SLAVE_ADDR_4_SCL        (0x96)      // Binary: 1001011x

/* Function prototypes */
extern void        ICACHE_FLASH_ATTR  ADS_init      (void);
extern uint16      ICACHE_FLASH_ATTR  ADS_read_adc  (uint8 slave_addr);


#endif