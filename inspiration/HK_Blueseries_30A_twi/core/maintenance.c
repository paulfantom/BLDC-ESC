#include <avr/io.h>
#include <avr/eeprom.h>
#include "main.h"
//#include "UART.h"
#include "maintenance.h"
#include "interface.h"

void doMaintenance(uint8_t dataIn)
{
  t_mntCmd data = (t_mntCmd)dataIn;
  
  static t_mntCmd lastCmd = mnt_none;
  
  //is this a state report?
 /*if( IS_MAINT(dataIn) && (dataIn & 0x01))
 {
   putch('M');
   return;
 }*/
  
  //Process 2 byte commands
  switch(lastCmd)
  {
    case mnt_none:
      lastCmd = data;
      break;
    case mnt_testFet:
      if(FLAG(POWER_OFF)) //only test FETs when power is off
      {
        
        ApFET_off;
        AnFET_off;
        BpFET_off;
        BnFET_off;
        CpFET_off;
        CnFET_off;
        
        switch(data)
        {
          case mnt_none:
            ApFET_on;
            //putch('A');
          break;
          case mnt_exit:
            SET_FLAG(C_FET);
            CLEAR_FLAG(A_FET);
            AnFET_on;
            //putch('a');
          break;
          case mnt_setPwm:
            BpFET_on;
            //putch('B');
          break;
          case mnt_setID:
            SET_FLAG(C_FET);
            CLEAR_FLAG(A_FET);
            BnFET_on;
            //putch('b');
          break;
          case mnt_getRpm:
            CpFET_on;
            //putch('C');
          break;
          case mnt_testFet:
            SET_FLAG(A_FET);
            CLEAR_FLAG(C_FET);
            CnFET_on;
            //putch('n');
          break;
	  case mnt_GovModeOn:
	  case mnt_GovModeOff:
	  case mnt_AdvcTime:
	  //
	  break;
        }
      }
      else
      {
        //putch('E'); // Error exit maintenance mode
        processSetPoint(0);
        CLEAR_FLAG(IS_MAINTENANCE);
        lastCmd = mnt_none;
      }
      lastCmd = mnt_none;
      break;
    case mnt_setPwm:
      if(FLAG(GOV_MODE))
       {
         target_rpm = dataIn ;
         target_rpm *= RPM_PER_STEP;
         //putch('G');
         //putch((target_rpm >> 8) & 0xFF); 
         //putch(target_rpm & 0xFF);  
       }
       else
       {
          //putch('s'); 
         processSetPoint(data);
       }
      lastCmd = mnt_none;
      break;
    case mnt_setID:
      //putch('i');
/*
      if(data>0 && data < 9)
      {
	 eeprom_write_byte((uint8_t*)EPRM_MID, (uint8_t)data);
         myId=data;
      }
      putch(myId); //report last ID
*/
      lastCmd = mnt_none;
      break;
    case mnt_AdvcTime:
      com_advc = (unsigned char)data;
      lastCmd = mnt_none;
      //putch('a');
      break;
    default:
      //putch('E'); // exit maintenance mode
      processSetPoint(0);
      CLEAR_FLAG(IS_MAINTENANCE);
      lastCmd = mnt_none;
      break;  
  }
  
  // Process 1 byte commands
  switch(lastCmd)
  {
    case mnt_none:
      break;
    case mnt_exit:
      //putch('x'); // exit maintenance mode
      processSetPoint(0);
      CLEAR_FLAG(IS_MAINTENANCE);
      lastCmd = mnt_none;
      break;  
    case  mnt_getRpm:
      //putch((meas_rpm >> 8)& 0xFF);
      //putch((meas_rpm & 0xFF));
      lastCmd = mnt_none;
      break;
    case mnt_GovModeOn:
      target_rpm = 700; //RPM_MIN * 2 / 3; // Set speed at 2/3 of minimum RPM
      SET_FLAG(GOV_MODE);
      //putch('G');
      //putch('1');
      lastCmd = mnt_none;
      break;
    case mnt_GovModeOff:
      CLEAR_FLAG(GOV_MODE);
      processSetPoint(0);
      //putch('G');
      //putch('0');
      lastCmd = mnt_none;
      break;
    default:
      break; 
  }

}
