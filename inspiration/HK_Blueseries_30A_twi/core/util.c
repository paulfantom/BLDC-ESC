#include "main.h"
#include <avr/io.h>
#include <util/delay.h>
//#include "UART.h"
#include "../interface.h"
#include "util.h"

void switch_power_off()
{
  processSetPoint(0);
  AnFET_off;
  BnFET_off;
  CnFET_off;
  ApFET_off;
  BpFET_off;
  CpFET_off;
}

void beep(uint8_t freq, uint8_t time)
{
  
  //freq = Hz / 100
  //time = ms / 10
  uint8_t j=freq*time;
  
  while(j!=0)
  {
      // Power on for 32us
      BpFET_on;
      AnFET_on;
      _delay_us(30);
      
      BpFET_off;
      AnFET_off;
      _delay_us(70); //complete 100us

      uint8_t i;
      for(i=1;i<(100/freq);i++) //power on for 1/freq - 100us
         _delay_us(100);
   j--;
  }
}

uint16_t readTimer1()
{
  //stop timer 1
  TCCR1B = T1STOP;
  uint16_t   ret = TCNT1L + (TCNT1H<<8); //read timer1
  TCCR1B = T1CK8; //enable timer1
  return ret;
}


// Set timers to start-up value
void set_all_timings()
{
  wt_OCT1_tot = timeoutSTART;
  wt_comp_scan = 0x1FFF;
  timing = 0x03FFFF;
}

// Input: Timer1 Compare register 
// Return: timer1 value when this function was called the last time
uint16_t update_timing(uint16_t tIn)
{
  uint16_t		tcnt1_sav;
  static uint16_t	last_tcnt1;
  
  tcnt1_sav = readTimer1(); //stop timer1 and get present value
  tIn += tcnt1_sav;
#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) ||\
    defined(__AVR_ATmega168__)
  TIMSK1 = (1 << TOIE1);	//stop compare interrupt
#else
  TIMSK = (1 << TOIE1) | (1 << TOIE2); //stop compare interrupt
#endif
  OCR1AH = (tIn>>8) & 0xFF; //  set compare registers = tInput
  OCR1AL = tIn & 0xFF;
  SET_FLAG(OCT1_PENDING);

#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) ||\
    defined(__AVR_ATmega168__)
  TIMSK1 = (1 << TOIE1) | (1 << OCIE1A); // enable interrupt again
#else
  TIMSK = (1 << TOIE1) | (1 << OCIE1A) | (1 << TOIE2); // enable interrupt again
#endif

  if((timing & 0xFFFF0000)) //if timing > 0xFFFF
  {
    zero_wt = 0x0000FFFF; //saturate zero_wt af 0xFFFF
  }
  else
  {
    zero_wt = timing;
  }
  
  uint16_t last_com = tcnt1_sav - last_tcnt1;
  uint16_t  ret = last_tcnt1;
  last_tcnt1 = tcnt1_sav;
  
  timing = timing - (timing / TIMING_N_MEAS) + last_com; //average of last 4 comutations
  

  if(timing > 0x3FFFF) // limit range to 0x3ffff
  {
    timing = 0x3FFFF;
  }
  
  wt_comp_scan = timing / TIMING_N_MEAS / COM_DELAY_FRAC;
  
  wt_comp_scan -= com_advc; // subtract comutation advance time 
  
  
  return ret; 
}

void evaluate_rpm()
{
  meas_rpm = CALC_RPM(timing);
  return;
}

void calc_governer()
{
  static unsigned long avg_rpm =0;
  
  #define GOV_AVG        (16)
  
  #define GOV_P          (05L)
  #define GOV_P_LIMIT    (100) // Limit for proportional part
  #define GOV_I_US       (10000L) //Integral update in us 
  //#define GOV_I_DIV      (32) // Integral 1/gain
  #define GOV_I_RES      (1<<8) //extra resolutin for integrator
  #define GOV_I_LIMIT    (GOV_I_RES * 250) //limit for integrator
  #define GOV_D           (1)
  
  // calculate setpoint
  
  avg_rpm = avg_rpm - (avg_rpm / GOV_AVG) + ((unsigned long)meas_rpm);

  long  err = ((long)target_rpm) - (avg_rpm / GOV_AVG); // error (in RPM)
  
  long  govP = (10L * err) / GOV_P;
  
  LIMIT(govP ,1,MAX_GOV_PWM);
  
  processSetPoint(govP);

  return;
}

void commutationDelay()
{
  unsigned int tmp = readTimer1();
#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) ||\
    defined(__AVR_ATmega168__)
  TIMSK1 = (1 << TOIE1);	//stop compare interrupt
#else
  TIMSK = (1 << TOIE1) | (1 << TOIE2); //stop compare interrupt
#endif

  tmp += wt_comp_scan; //set compare register to next commutation time
  OCR1AH = tmp>>8;
  OCR1AL = tmp & 0xFF;
  SET_FLAG(OCT1_PENDING);
#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) ||\
    defined(__AVR_ATmega168__)
  TIMSK1 = (1 << OCIE1A) | (1 << TOIE1);	// start compare interrupt
#else
  TIMSK = (1 << OCIE1A) | (1 << TOIE1) | (1 << TOIE2);
#endif
  
  evaluate_rpm();
  
  if(FLAG(GOV_MODE)) //not in gorvernor mode, exit
  {
    calc_governer();
  }
  
  // As we have to wait anyway, Perform some house keeping
  static char st=0;
  switch(st++) //Perform a different task every call
  {
    case 0:
      getCommand();
      break;
    case 1:
      break;
    case 2:
      break;
    case 3:
     // EMPTY
      break;
    case 4:
      // EMPTY
      break;
    case 5:
      // EMPTY
      st=0;
      break;
    default:
      st=0;
      break;
  }

  WAIT_OCT1_TOT;
}

char wait_for_transition(char isHigh)
{
  if(isHigh)
  {
    //wait for High
    do
    {
        do
        { 
            if(!FLAG(OCT1_PENDING)) //if timeout
            {
                return -1; //return error
            }
        }while((ACSR & (1<<ACO)));//wait ACO == 0
         _delay_us(4); //wait for spike
    }while((ACSR & (1<<ACO))); //wait ACO == 0 and not just a spike, continue...

    //wait for Low
    do
    {
        do
        { 
            if(!FLAG(OCT1_PENDING)) //if timeout
            {   return -1; //return error
            }
        }while(!(ACSR & (1<<ACO)));//wait ACO == 1
         _delay_us(4); //wait for spike
    }while(!(ACSR & (1<<ACO))); //wait ACO == 1 and not just a spike, continue...
  }
  else //----------------------- Is Low phase
  {
    //wait for Low
    do
    {
        do
        { 
            if(!FLAG(OCT1_PENDING)) //if timeout
            {
                return -1; //return error
            }
        }while(!(ACSR & (1<<ACO)));//wait ACO == 1
         _delay_us(4); //wait for spike
    }while(!(ACSR & (1<<ACO))); //wait ACO == 1 and not just a spike, continue...

    //wait for High
    do
    {
        do
        { 
            if(!FLAG(OCT1_PENDING)) //if timeout
            {
                return -1; //return error
            }
        }while((ACSR & (1<<ACO)));//wait ACO == 0
         _delay_us(4); //wait for spike
    }while((ACSR & (1<<ACO))); //wait ACO == 0 and not just a spike, continue...
  }
  
  return 0;
}

void syncPowerOn()
{
  while(FLAG(I_OFF_CYCLE)); //wait power on
  while(!FLAG(I_OFF_CYCLE)); //wait power off
}


void set_OCT1_tot()
{
  unsigned int tmp = readTimer1();
#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) ||\
    defined(__AVR_ATmega168__)
  TIMSK1 = (1 << TOIE1);	//stop compare interrupt
#else
  TIMSK = (1 << TOIE1) | (1 << TOIE2); //stop compare interrupt
#endif
  tmp += zero_wt;
  OCR1AH = tmp >>8;
  OCR1AL = tmp & 0xFF;
  SET_FLAG(OCT1_PENDING);
#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) ||\
    defined(__AVR_ATmega168__)
  TIMSK1 = (1 << OCIE1A) | (1 << TOIE1);	// start compare interrupt
#else
  TIMSK = (1 << OCIE1A) | (1 << TOIE1) | (1 << TOIE2);
#endif
}



