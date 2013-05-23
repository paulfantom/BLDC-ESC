//********************************************
// Options for HK Blueseries 30A ESC (TWI mod)
//********************************************
#ifndef _PARAMETERS_H
#define _PARAMETERS_H

#define F_CPU	12000000L
#define VERSION	(0x27)
//#define   SKIP_BEEP
#define   INPUT_RANGE     (0xFF)
#define   PWM_RANGE       200 // number of  counts for a full PWM cycle
#define   STARTUP_POWER   (20) // Start up Power (0-255)

#define COMPERTURN      (42) // 7 pole motor with 3 phase stator (poles*phases*2)
#define T1MHZFREQ       (F_CPU/8/1000000L) // T1 frequency in MegaHertz
#define TIMING_N_MEAS   (4) // number of comutations in "timing" variable
#define COM_DELAY_FRAC  (4) //  (1 / Fraction of cycle to delay after zero cross) 

#define RPM_PER_STEP    (30) //Used in governor mode
#define MAX_GOV_PWM     (0x80)

#define BAUD            38400

#define timeoutSTART	65000
#define timeoutMIN	48000

#define RPM_MIN         (200)
#define RPM_RANGE	4800	// ( RPM )
#define ENOUGH_GOODIES	12 // number of good startup cycles before going to Run mode

#define UART_TOT_MS      2000// milliseconds for UART, no-message timeout

//****************************
// TWI Slave library constants
//****************************
#define TWI_DEBUG	0
#define TWI_BUFFER_SIZE	2
#define TWI_OWN_ADDR	0x10

//------------- PORT DEFINITION -------------------
//
//  ***** IMPORTANT !!! ****
//   Pin asignment changes depending on ESC hardware design
//   Wrong asignment of pins might shor-circuit the P and N FETS
//   with catastrofic results, so pay ATTENTION to this definitions 
//   before burning the firmware.

//*********************
// PORT B definitions
//*********************
#define	AnFET_port	PORTB

#define	AnFET_pin       0


//*********************
// PORT C definitions
//*********************
#define	ApFET_port      PORTB

#define	ApFET_pin	1

//*********************
// PORT D definitions
//*********************  
#define	CpFET_port      PORTB
#define	CnFET_port      PORTB
#define	BpFET_port      PORTB
#define	BnFET_port      PORTB

#define	CpFET_pin	4
#define	CnFET_pin	5
#define	BpFET_pin	2
#define	BnFET_pin	3

//*********************
// ANALOGUE
//*********************  

#define ACCU_MUX  2	// ADC2 voltage control input
#define	c_comp	  6	// common comparator input (AIN0)

#define mux_a	  0	// ADC0 phase input
#define mux_b     7	// ADC7 phase input
#define mux_c	  6	// ADC6 phase input

#endif // _PARAMETERS_H

