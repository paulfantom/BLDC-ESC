#ifndef _MAINTENANCE_H
#define _MAINTENANCE_H

#include <stdint.h>

extern uint8_t myId;
extern unsigned com_advc;

typedef enum
{
  mnt_none =0,
  mnt_exit =1,
  mnt_setPwm =2,
  mnt_setID =3,
  mnt_getRpm =4,
  mnt_testFet =5,
  mnt_GovModeOn =6,
  mnt_GovModeOff =7,
  mnt_AdvcTime = 8
}t_mntCmd;

void doMaintenance(uint8_t);

#endif // _MAINTENANCE_H
