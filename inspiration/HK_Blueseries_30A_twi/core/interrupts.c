#include <avr/interrupt.h>
#include "main.h"

extern volatile char flag0, flag1, flag2;
extern volatile uint8_t pwm_on_timer;
extern volatile uint8_t pwm_off_timer;
extern uint8_t t1_timeout;
extern uint8_t uart_timeout;

ISR(TIMER2_OVF_vect) //timer2 overflow interrupt
{
  if(FLAG(I_OFF_CYCLE)) // Start ON cycle
  {
        TCNT2 = pwm_on_timer;  
        CLEAR_FLAG(I_OFF_CYCLE);
        
        if(FLAG(POWER_OFF))
        {
          return;
        }
        
        //switch next FET on
        if(FLAG(C_FET))
        {
            CnFET_on;
        }
        else if(FLAG(A_FET))
        {
            AnFET_on;
        }
        else 
        {
            BnFET_on;
        }
    }
    else // Start OFF cycle
    {
      SET_FLAG(I_OFF_CYCLE);
      TCNT2 = pwm_off_timer;  //reload timer0
          
      if(!(ACSR & (1<<ACO)))
      {
        CLEAR_FLAG(COMP_HI);
      }
      else
      {
        SET_FLAG(COMP_HI);
      }
  
      if(FLAG(FULL_POWER))
      {
        return;
      }
    
   //switch next FET off
      if(FLAG(C_FET))
      {
          CnFET_off;
      }
      else if(FLAG(A_FET))
      {
         AnFET_off;
      }
      else
      {
          BnFET_off;
      } 
    }
}

ISR(TIMER1_OVF_vect) //overflow timer1
{
  SET_FLAG(T1OVFL_FLAG);
  
  if(t1_timeout!=0)
      t1_timeout--;
  
  if(uart_timeout!=0)
    uart_timeout--;
}


ISR(TIMER1_COMPA_vect) //output compare timer1 interrupt
{
  CLEAR_FLAG(OCT1_PENDING);
}

