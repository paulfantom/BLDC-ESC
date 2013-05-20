

// --Disclaimer--
//The authors, of this software accept no responsibility 
//for damages resulting from the use of this product and makes
//no warranty or representation, either express or implied, 
//including but not limited to, any implied warranty of 
//merchantability or fitness for a particular purpose. 
//This software is provided "AS IS", and you, its user, 
//assume all risks when using it. 


// Ported to AVR-GCC
// by: Anton Dubniak
// Mar. 2011
//
// Ported to C and converted to UART control
// by: Renato Salles
// Dec. 2010
//
// Based on original ASM code from: 
// Bernhard Konze
// Dec. 2006
//
//
// This code is for 16MHz ATMega8 based ESCs
// It will probably work on 8MHz version with some small changes

#include "../parameters.h"
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "main.h"
#include "util.h"
//#include "UART.h"
#include "start.h"
#include "run.h"
#include "../interface.h"

//Used interrupts:
//	t1oca
//	t1ovfl
//	t2ovfl

// Global Variables
volatile char flag0, flag1, flag2;
volatile uint8_t  pwm_on_timer = 0;
volatile uint8_t  pwm_off_timer = 0xFF;
unsigned long timing;
unsigned meas_rpm;
unsigned com_advc=0;
unsigned target_rpm=0;
unsigned wt_OCT1_tot;
unsigned wt_comp_scan;
unsigned zero_wt;
uint8_t t1_timeout;
uint8_t uart_timeout;
uint8_t DIR_PB=0;
uint8_t DIR_PC=0;
uint8_t DIR_PD=0;
uint8_t phase =0;
volatile char dataStore[256];

void setup(void)
{
  CLI; //disable all interrupts
  //Automatic port direction setup
  DDRB =0;
  DDRC =0;
  DDRD =0;
  
#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) ||\
    defined(__AVR_ATmega168__)
  TIMSK0 = 0;
  TIMSK1 = 0;
  TIMSK2 = 0;
#else
  TIMSK = 0;
#endif
  
    if(&ApFET_port == &PORTB) 
        DDRB |= (1<<ApFET_pin);
    else if(&ApFET_port == &PORTC) 
        DDRC |= (1<<ApFET_pin);
    else if(&ApFET_port == &PORTD) 
        DDRD |= (1<<ApFET_pin);

    if(&AnFET_port == &PORTB) 
        DDRB |= (1<<AnFET_pin);
    else if(&AnFET_port == &PORTC) 
        DDRC |= (1<<AnFET_pin);
    else if(&AnFET_port == &PORTD) 
        DDRD |= (1<<AnFET_pin);

    if(&BpFET_port == &PORTB) 
        DDRB |= (1<<BpFET_pin);
    else if(&BpFET_port == &PORTC) 
        DDRC |= (1<<BpFET_pin);
    else if(&BpFET_port == &PORTD) 
        DDRD |= (1<<BpFET_pin);

    if(&BnFET_port == &PORTB) 
        DDRB |= (1<<BnFET_pin);
    else if(&BnFET_port == &PORTC) 
        DDRC |= (1<<BnFET_pin);
    else if(&BnFET_port == &PORTD) 
        DDRD |= (1<<BnFET_pin);

    if(&CpFET_port == &PORTB) 
        DDRB |= (1<<CpFET_pin);
    else if(&CpFET_port == &PORTC) 
        DDRC |= (1<<CpFET_pin);
    else if(&CpFET_port == &PORTD) 
        DDRD |= (1<<CpFET_pin);

    if(&CnFET_port == &PORTB) 
        DDRB |= (1<<CnFET_pin);
    else if(&CnFET_port == &PORTC) 
        DDRC |= (1<<CnFET_pin);
    else if(&CnFET_port == &PORTD) 
        DDRD |= (1<<CnFET_pin);


    PORTB = INIT_PB;	
    PORTC = INIT_PC;  
    PORTD = INIT_PD;
    
    ADCSRA &= ~(1<<ADEN); //switch to comparator multiplexed

#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) ||\
    defined(__AVR_ATmega168__)
    ADCSRB |= (1<<ACME);
#else
    SFIOR |= (1<<ACME);
#endif

//  start timer2 with CK/8 (0.5us/count  for a 16MHz clk)
#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) ||\
    defined(__AVR_ATmega168__)
    TCCR2B = (1 << CS21);
#else
    TCCR2 = (1 << CS21);
#endif

// start timer1 with CK/8 (0.5us/count  for a 16MHz clk) 
    TCCR1B = T1CK8;

// reset  flags
    flag0 =0;
    flag1 =0;
    flag2 =0;
    
    CLEAR_FLAG(IS_MAINTENANCE);
    CLEAR_FLAG(GOV_MODE);
    com_advc =0;
    /* initialize UART*/
/*    openSerial();
    putln(' '); 
    putch('*'); //signal software start 
    putch('v');
    putch('0' + (VERSION>>4));  
    putch('.');
    putch('0' + (VERSION & 0x0f));
*/
    /* initialize interface */
    interface_init();

#ifndef SKIP_BEEP
    _delay_ms(10); 		
    beep(15,10); // 1.5kHz, 0.3sec	
    beep(10,10); // 3kHz, 0.3sec
    
    _delay_ms(500);

     beep(20,20); // 2.0kHz, 0.6sec

/* 
   uint8_t i;
    for(i=0;i<myId;i++) //beep as many times as ID number
    {
      beep(20,20); // 2.0kHz, 0.6sec
      _delay_ms(500); //wait 0.5s	
    }
*/
#endif
   
    switch_power_off();
    // init registers and interrupts
#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) ||\
    defined(__AVR_ATmega168__)
    TIFR1 = TIMSK1 = (1<<TOIE1) | (1<<OCIE1A);	// Enable the timer1 interrupts we use in the code
    TIFR2 = TIMSK2 = (1<<TOIE2);		// Enable the timer2 interrupts we use in the code
#else
    TIFR = TIMSK = (1<<TOIE1) | (1<<OCIE1A) | (1<<TOIE2); // Enable the 3 interrupts we use in the code
#endif

    set_all_timings();
    SEI; //Enable all interrupts

}


void loop(void)
{
     while(init_startup()) //loop until successfull startup
     { 
       //wait for command  and startup the motor
     }
     

//DEBUG
    run(); //this function only returns motor stops, or error is found
    switch_power_off();
    //restart motor
    _delay_ms(1000);
    SET_FLAG(STARTUP);
    CLEAR_FLAG(POFF_CYCLE);
}



int main(void)
{

evaluate_rpm();


calc_governer();

	setup();
	for(;;)
	{
		loop();
	}
}
