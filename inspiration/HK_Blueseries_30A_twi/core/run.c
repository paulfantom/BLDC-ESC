#include <avr/io.h>
//#include "UART.h"
#include "main.h"
#include "util.h"
#include "run.h"

char run()
{
  for(;;)
  {

      // Phase 1
       set_OCT1_tot(); //set maximum waiting time for zero-crossing before quit
      if(wait_for_transition(0))
      {
        return -1; // return with error
      }
      commutationDelay(); // delay about 15deg between zero-crossing and commutation
      com1com2(); //perform next commutation
      update_timing(wt_comp_scan); // Calculate new commutation timing 
      WAIT_OCT1_TOT; // Wait "wt_comp_scan" before next zero-crossing scan
      set_OCT1_tot(); //set maximum waiting time for zero-crossing before quit
      
      // Phase 2
      if(wait_for_transition(1))
      {
        return -1; // return with error
      }
      commutationDelay();
      com2com3();
      update_timing(wt_comp_scan);
      WAIT_OCT1_TOT;
       set_OCT1_tot(); //set maximum waiting time for zero-crossing before quit
      
      // Phase 3
      if(wait_for_transition(0))
      {
        return -1; // return with error
      }
      commutationDelay();
      com3com4();
      update_timing(wt_comp_scan);
      WAIT_OCT1_TOT;
       set_OCT1_tot(); //set maximum waiting time for zero-crossing before quit
      
      // Phase 4
      if(wait_for_transition(1))
      {
        return -1; // return with error
      }
      commutationDelay();
      com4com5();
      update_timing(wt_comp_scan);
      WAIT_OCT1_TOT;
       set_OCT1_tot(); //set maximum waiting time for zero-crossing before quit
      
      // Phase 5
      if(wait_for_transition(0))
      {
        return -1; // return with error
      }
      commutationDelay();
      com5com6();
      update_timing(wt_comp_scan);
      WAIT_OCT1_TOT;
       set_OCT1_tot(); //set maximum waiting time for zero-crossing before quit
      
       // Phase 6
      if(wait_for_transition(1))
      {
        return -1; // return with error
      }
      commutationDelay();
      com6com1();
      update_timing(wt_comp_scan);
      WAIT_OCT1_TOT;

    if(!FLAG(IS_MAINTENANCE)) //don't check timeouts when in maintenance mode
    {
      if(uart_timeout== 0) //if communication time-out
      {
          // cut power to the motor
          SET_FLAG(POWER_OFF);
      }
    } 
    
    if(meas_rpm < (RPM_MIN/2))
    {
      return -1; //if too slow, quit
    }
  }
}


void com1com2()
{
  phase=1;
  BpFET_off;
  if(!FLAG(POWER_OFF))
  {
    ApFET_on;
  }
  ADMUX = mux_b;
}

void com2com3()
{
  phase=2;
  //Stop timer2 interrupt
//#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) ||\	//FIXME
//    defined(__AVR_ATmega168__)
  TIMSK2 = 0;
//#else
//  TIMSK = (1<<OCIE1A) | (1<<TOIE1);
//#endif
  NOP;
  CLEAR_FLAG(A_FET);
  CLEAR_FLAG(C_FET);

   CnFET_off;
   if(!FLAG(POWER_OFF) && !FLAG(I_OFF_CYCLE))
   {
      BnFET_on;
   }
//#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) ||\
//    defined(__AVR_ATmega168__)
  TIMSK2 = (1<<TOIE2);
//#else
//  TIMSK = (1<<TOIE1) | (1<<OCIE1A) | (1<<TOIE2);
//#endif
  ADMUX = mux_c;
}

void com3com4()
{
  phase=3;
  ApFET_off;
  if(!FLAG(POWER_OFF))
  {
    CpFET_on;
  }
  ADMUX = mux_a;
}

void com4com5()
{
  phase=4;
  //Stop timer2 interrupt
//#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) ||\	FIXME
//    defined(__AVR_ATmega168__)
  TIMSK2 = 0;
//#else
//  TIMSK = (1<<OCIE1A) | (1<<TOIE1) ;
//#endif
  NOP;
  SET_FLAG(A_FET);
  CLEAR_FLAG(C_FET);

   BnFET_off;
   if(!FLAG(POWER_OFF) && !FLAG(I_OFF_CYCLE))
   {
      AnFET_on;
    }
//#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) ||\
//    defined(__AVR_ATmega168__)
//  TIMSK2 = (1<<TOIE2);
//#else
//  TIMSK = (1<<TOIE1) | (1<<OCIE1A) | (1<<TOIE2);
//#endif
  ADMUX = mux_b;
}

void com5com6()
{
  phase=5;
  CpFET_off;
  if(!FLAG(POWER_OFF))
  {
    BpFET_on;
  }
  ADMUX = mux_c;
}

void com6com1()
{
  phase=6;
  //Stop timer2 interrupt
//#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) ||\	FIXME
//    defined(__AVR_ATmega168__)
  TIMSK2 = 0;
//#else
//  TIMSK = (1<<OCIE1A) | (1<<TOIE1) ;
//#endif
  NOP;
  CLEAR_FLAG(A_FET);
  SET_FLAG(C_FET);

   AnFET_off;
   if(!FLAG(POWER_OFF) && !FLAG(I_OFF_CYCLE))
   {
     CnFET_on;
   }
//#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) ||\	FIXME
//    defined(__AVR_ATmega168__)
  TIMSK2 = (1<<TOIE2);
//#else
//  TIMSK = (1<<TOIE1) | (1<<OCIE1A) | (1<<TOIE2);
//#endif
  ADMUX = mux_a;
}

