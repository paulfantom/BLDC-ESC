#include "main.h"
#include <avr/io.h>
#include <util/delay.h>
//#include "UART.h"
#include "run.h"
#include "util.h"
#include "../interface.h"
#include "start.h"

char startMotor() //startup sequence
{
    //bool make120Jump = false; 
    uint8_t goodies =0;
    meas_rpm=0;
    
    do
    {
        startTimeOut();
        //Start1
        if(startStep()) //if final state is HI
        {
          com1com2();
          startTimeOut();
        }
       else //if Low, jump 120 deg
        {
            goodies=0;
            com2com3();
            com3com4(); // set FET state to 4
            com4com5(); 
            com5com6(); //go to state 1 and wait there for 300ms
            SET_FLAG(SCAN_TIMEOUT);
            //putch('J');
        //    _delay_ms(10);
            com6com1(); 
            continue; // try again
        }
    

        //Start2
        startStep();
        com2com3();
        startTimeOut();
        
        //start3
        startStep();
        com3com4();
        startTimeOut();   
        
        //start4
        startStep();
        com4com5();
        startTimeOut(); 
        
        //start5
        startStep();
        com5com6();
        startTimeOut(); 
        
        //start6
        startStep();
        com6com1();
        if( FLAG(POWER_OFF) || t1_timeout==0)
        {
          switch_power_off();
          _delay_ms(500);
          return -1; //if no power, or timeout, stop motor
        }
    
        
       if(FLAG(SCAN_TIMEOUT))
       {
         goodies=0;
         meas_rpm=0;
       }
       else
       {
         evaluate_rpm();
       }
       
       
        if(meas_rpm > RPM_MIN)
        {
              break; //speed is fast! exit startup sequence
        }
       
       goodies++;
       CLEAR_FLAG(SCAN_TIMEOUT);
   }while(goodies<ENOUGH_GOODIES); //enough successfull comutations, start run mode
   

    //s6_run1:
    uart_timeout = UART_TOT;
    update_timing(wt_comp_scan);
    set_OCT1_tot();
    CLEAR_FLAG(STARTUP);
    //putch('S');
    return 0; //goto run1
}


void startTimeOut()
{

  // Apply some "dithering" to the estimated motor timming
  // This will help the motor to sync at startup
  uint16_t temp = update_timing(wt_OCT1_tot) - ((TCNT1L & 0x0f)<<8);
  
  if(temp<timeoutMIN)
  {
      temp = timeoutSTART;
  }
  
  wt_OCT1_tot = temp;
  syncPowerOn();
}


char startStep()
{
    char startState = FLAG(COMP_HI); //get initial comparator state
    
    while(startState == FLAG(COMP_HI)) //wait for comparator change
    {
      if(!FLAG(OCT1_PENDING)) // check for timeout
      {
        SET_FLAG(SCAN_TIMEOUT);
        return 1;
      }
      syncPowerOn();
    }
   
  unsigned int tmp = readTimer1();

#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) ||\
    defined(__AVR_ATmega168__)
  TIMSK1 = (1 << TOIE1);
  TIMSK2 = (1 << TOIE2);
#else
  TIMSK = (1 << TOIE1) | (1 << TOIE2);
#endif
  tmp += 1000;
  OCR1AH = tmp >>8;
  OCR1AL = tmp & 0xFF;
  SET_FLAG(OCT1_PENDING);
#if defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) ||\
    defined(__AVR_ATmega168__)
  TIMSK1 = (1 << TOIE1) | (1 << OCIE1A);
  TIMSK2 = (1 << TOIE2);
#else
  TIMSK = (1 << TOIE1) | (1 << OCIE1A) | (1 << TOIE2);
#endif
  WAIT_OCT1_TOT;
    
  return FLAG(COMP_HI);
}


char init_startup()
{
    switch_power_off();
    meas_rpm=0;
    CLEAR_FLAG(NEW_SETPOINT);
    while(!FLAG(NEW_SETPOINT)) //wait for new setpoint 
    {
      getCommand();
    }
    
    t1_timeout = 80; //about 2.5sec
    set_all_timings();

    CLEAR_FLAG(SCAN_TIMEOUT);
    processSetPoint(STARTUP_POWER);
    
    com2com3(); 
    com3com4(); // set FET state to 4
    syncPowerOn(); //Wait a full timer cycle...
    syncPowerOn(); //...as to make sure PWM timer is loaded with Startup Power value
    CLEAR_FLAG(POWER_OFF); //Power on to FETS
    com4com5(); 
    com5com6();
    _delay_ms(10);
    com6com1(); //go to state 1 
    return startMotor();
}

