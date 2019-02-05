#ifndef __PWM_H__
#define __PWM_H__

#include "head.h"

void BeepPwmInit (void);
void PwmConfig(void);
/*
 * ��Ƶ100000 pwm����20000us
 * ����ֵ*5 = ң����ң��ֵ
 */
#define PwmControl_1(Pulse) PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, Pulse*5); // PB7
#define PwmControl_2(Pulse) PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, Pulse*5); // PB4
#define PwmControl_3(Pulse) PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, Pulse*5); // PB5
#define PwmControl_4(Pulse) PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, Pulse*5); // PE4
#define PwmControl_5(Pulse) PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, Pulse*5); // PE5

#endif
