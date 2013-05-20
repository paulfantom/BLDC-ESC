#ifndef _ESC_MAIN_H
#define _ESC_MAIN_H

#include "../parameters.h"
#include <stdint.h>

#define T1STOP	      0x00 // disable timer0
#define T1CK8	      0x02 // set timer0 to CLK/8 frequency

//Function like macros
#define WAIT_OCT1_TOT                   while(FLAG(OCT1_PENDING));//wait timer1 compare
#define CBI(REG, POS)                   (REG) &= ~(1<<POS)
#define SBI(REG, POS)                   (REG) |= (1<<POS)
#define NOP                             asm volatile ("nop \n\t"::)
#define LIMIT(X,MIN,MAX)                X = (X<MIN) ? MIN : ((X>MAX) ? MAX : X)
#define RPM_CURVE(RPM)                  (STARTUP_POWER + ((0xFF - STARTUP_POWER) * ((long)(RPM))) / RPM_RANGE )
#define LIMIT_PWM(RPM,P)                P = (RPM < RPM_RANGE) ? ((P < RPM_CURVE(RPM) ) ? P : RPM_CURVE(RPM) ) : P;

#define US_PER_COM(X)                   ((X)/TIMING_N_MEAS/T1MHZFREQ)
#define US_PER_TURN(X)                  (US_PER_COM(X) * COMPERTURN)
#define TURN_PER_SEC(X)                 (1000000L/US_PER_TURN(X))
#define CALC_RPM(X)                     (TURN_PER_SEC(X)*60)

#define UART_TOT                        (UART_TOT_MS * T1MHZFREQ / 65)

#define  CLI                            asm volatile ("cli"::)
#define  SEI                            asm volatile ("sei"::)


//#define  WAIT_MS(X)     _delay_ms(X)

#define ApFET_on	SBI(ApFET_port,ApFET_pin)
#define ApFET_off	CBI(ApFET_port,ApFET_pin)
#define AnFET_on	SBI(AnFET_port,AnFET_pin)
#define AnFET_off	CBI(AnFET_port,AnFET_pin)
#define BpFET_on	SBI(BpFET_port,BpFET_pin)
#define BpFET_off	CBI(BpFET_port,BpFET_pin)
#define BnFET_on	SBI(BnFET_port,BnFET_pin)
#define BnFET_off	CBI(BnFET_port,BnFET_pin)
#define CpFET_on	SBI(CpFET_port,CpFET_pin)
#define CpFET_off	CBI(CpFET_port,CpFET_pin)
#define CnFET_on	SBI(CnFET_port,CnFET_pin)
#define CnFET_off	CBI(CnFET_port,CnFET_pin)

#define INIT_PB	 0
#define INIT_PC	 0
#define	INIT_PD	0

//Flag handling Macros
#define FLAG(F)          (F##_R  & F##_B)
#define SET_FLAG(F)      (F##_R) |= (F##_B)
#define CLEAR_FLAG(F)    (F##_R) &= ~(F##_B)

//Flag deginition
#define OCT1_PENDING_B	_BV(0)	//  output compare interrunpt is pending
#define OCT1_PENDING_R  flag0
#define GOV_MODE_B        _BV(1)// if accu voltage low
#define GOV_MODE_R 	flag0
#define I_pFET_HIGH_B	_BV(2)//over-current detect
#define I_pFET_HIGH_R    flag0
#define GET_STATE_B	_BV(3)// state is to be send
#define GET_STATE_R      flag0
#define C_FET_B		_BV(4)// C-FET state is to be changed
#define C_FET_R	        flag0
#define A_FET_B		_BV(5)// A-FET state is to be changes. If neither 1 nor 2 is set, B-FET state is to be changed
#define A_FET_R	         flag0
#define I_OFF_CYCLE_B	_BV(6)	// current off cycle is active
#define I_OFF_CYCLE_R	flag0
#define T1OVFL_FLAG_B	_BV(7)	//imer1 overflow sets this flag - used for voltage + current watch
#define T1OVFL_FLAG_R	flag0

#define POWER_OFF_B	    _BV(00)	// switch fets on disabled
#define POWER_OFF_R	    flag1
#define FULL_POWER_B	    _BV(01)	//100% on - don't switch off, but do OFF_CYCLE working
#define FULL_POWER_R	    flag1
#define CALC_NEXT_OCT1_B    _BV(02)	// calculate OCT1 offset, when wait_OCT1_before_switch is called
#define CALC_NEXT_OCT1_R    flag1
#define GOV_CALC_B          _BV(03)	// calculate governer PWN
#define GOV_CALC_R          flag1
#define EVAL_UART_B	    _BV(04)	// if set, new rc puls is evaluated, while waiting for OCT1
#define EVAL_UART_R	    flag1
#define EVAL_SYS_STATE_B    _BV(05)	// if set, overcurrent and undervoltage are checked
#define EVAL_SYS_STATE_R     flag1
#define EVAL_RPM_B	    _BV(06)	// if set, next PWM on should look for current
#define EVAL_RPM_R	    flag1
#define EVAL_PWM_B	    _BV(07)	// if set, PWM should be updated
#define EVAL_PWM_R	    flag1

#define RPM_RANGE1_B	_BV(0) // if set RPM is lower than 1831 RPM
#define RPM_RANGE1_R	flag2
#define RPM_RANGE2_B	_BV(1) // if set RPM is between 1831 RPM and 3662 RPM
#define RPM_RANGE2_R      flag2
#define SCAN_TIMEOUT_B	_BV(2) // if set a startup timeout occurred
#define SCAN_TIMEOUT_R	flag2
#define POFF_CYCLE_B	_BV(3) // if set one commutation cycle is performed without power
#define POFF_CYCLE_R	flag2
#define COMP_HI_B	_BV(4) // if set ACO was high
#define COMP_HI_R	flag2
#define STARTUP_B		_BV(5) // if set startup-phase is active
#define STARTUP_R		flag2
#define IS_MAINTENANCE_B	_BV(6) //
#define IS_MAINTENANCE_R	flag2
#define NEW_SETPOINT_B		_BV(7) //
#define NEW_SETPOINT_R		flag2

// EEPROM Alocation
#define  EPRM_MID  0x10

#endif // _ESC_MAIN_H
