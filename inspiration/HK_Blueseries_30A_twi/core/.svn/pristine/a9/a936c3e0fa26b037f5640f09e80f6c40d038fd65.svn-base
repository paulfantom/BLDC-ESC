#ifndef _UTIL_H
#define _UTIL_H

#include <stdint.h>

extern unsigned wt_OCT1_tot;
extern unsigned wt_comp_scan;
extern unsigned long timing;
extern unsigned zero_wt;
extern unsigned com_advc;

void switch_power_off(void);
void beep(uint8_t, uint8_t);
uint16_t readTimer1(void);
void set_all_timings(void);
uint16_t update_timing(uint16_t);
void evaluate_rpm(void);
void calc_governer(void);
void commutationDelay(void);
char wait_for_transition(char);
void syncPowerOn(void);
void set_OCT1_tot(void);

#endif // _UTIL_H
