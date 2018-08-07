#ifdef _NTC_MF52_
#define _NTC_MF52_

/* DEFINITIONS */
#define RESISTOR 	10000.0	// Constant resistor
#define T_ROOM 		293.15	// Room temp
#define	R_ROOM 		10000.0	// Resistance at room temp
#define B 			4150.0	// B coefficient
#define MAX_ADC 	1023.0	// Maximum ADC value (10-bit ADC)
#define N_SAMPLES 	10		// Number of samples for avg voltage

class NTC_MF52 {
    public:

        NTC_MF52();
        ~NTC_MF52();

        void read_temperature();

    private:

        static int 		read_avg_voltage    (void);
        static double 	calculate_res	    (double res);
        static double 	steinhart_eq	    (int res);
        static void 	print_double	    (double d);

        double  avg_volt;
        int     avg_volt_counter;
}

#endif