#include "GP2Y1010AU0F.h"
#include "ADS_1015.h"
#include "osapi.h"
#include "math.h"
#include "gpio.h"

/* Constants */
#define SLOPE           7.86
#define Y_INTERSECT     0.614
#define LED_PIN         4

/* Function declarations */
void        ICACHE_FLASH_ATTR  led_on                   (void);
void        ICACHE_FLASH_ATTR  led_off                  (void);
double      ICACHE_FLASH_ATTR  voltage_to_dust_density  (uint16 voltage);

/* Function definitions */
/* Measure dust density (mg/m3) */
double      ICACHE_FLASH_ATTR  measure_fine_dust        (void) {

    uint16 voltage;
    double fine_dust;

    led_on();                                       // Turn led on
    voltage = ADS_read_adc(SLAVE_ADDR_1_GND);       // Read photoresistor
    led_off();                                      // Turn led off
    fine_dust = voltage_to_dust_density(voltage);   // Convert voltage to dust density

    return fine_dust;
}

/* Turn sensor led on */
void        ICACHE_FLASH_ATTR  led_on                    (void) {
    
    /* Turn led on */
    gpio_output_set((1 << LED_PIN),     // Set mask
                     0,                 // Clear mask
                     (1 << LED_PIN),    // Enable mask
                     0);                // Disable mask
}

/* Turn sensor led off */
void        ICACHE_FLASH_ATTR  led_off                  (void) {

    /* Turn led off */
    gpio_output_set(0,                 // Set mask
                    (1 << LED_PIN),    // Clear mask
                    (1 << LED_PIN),    // Enable mask
                    0);                // Disable mask
}

/* Convert voltage to dust density based on Sharp-datasheet graph */
double      ICACHE_FLASH_ATTR  voltage_to_dust_density  (uint16 voltage) {

    double dust_density;
    dust_density = ((double)voltage - Y_INTERSECT) / SLOPE;
    return dust_density;
}