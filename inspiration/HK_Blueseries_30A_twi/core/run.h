#ifndef _RUN_H
#define _RUN_H

extern volatile char flag0, flag1, flag2;
extern unsigned wt_comp_scan;
extern uint8_t uart_timeout;
extern unsigned meas_rpm;
extern uint8_t phase;

char run(void);
void com1com2(void);
void com2com3(void);
void com3com4(void);
void com4com5(void);
void com5com6(void);
void com6com1(void);

#endif // _RUN_H
