#ifndef _INTERFACE_H
#define _INTERFACE_H

#include <stdint.h>

extern volatile char flag0, flag1, flag2;
extern unsigned target_rpm;
extern uint8_t uart_timeout;
extern unsigned meas_rpm;
extern volatile uint8_t pwm_off_timer;
extern volatile uint8_t pwm_on_timer;

void interface_init(void);
void getCommand(void);
void processSetPoint(uint8_t);

#endif // _INTERFACE_H
