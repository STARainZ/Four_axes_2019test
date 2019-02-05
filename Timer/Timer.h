#ifndef _TIMER_H
#define _TIMER_H

#include "head.h"

//void MotorContolTimer(void);
void Timer0_Config(void);
void Timer1_Config(void);
void Timer2_Config(void);
void Timer2IntHandler(void);
uint32_t MyFrequence;
uint8_t KeyPress0;


#endif
