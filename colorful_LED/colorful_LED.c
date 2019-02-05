/**
  ******************************************************************************
  * 文件名程: ColorFulLed.c
  * 作    者: By Sw Young
  * 版    本: V1.0
  * 功    能:
  * 编写日期: 2018.7.6
  ******************************************************************************
  * 说明：
  * 硬件平台：TM4C123G
  *   *****
  * 软件设计说明：
  *   *****
  * Github：
  ******************************************************************************
**/
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/fpu.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "utils/uartstdio.h"
#include "driverlib/pwm.h"
#include "delay/delay.h"
#include "colorful_LED.h"
/**
  * 函 数 名:LED_ColorInit
  * 函数功能: 彩灯初始化（PWM控制）
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young
  *   2017.7.6
  */
void LED_ColorInit(void)
{
    // PWM时钟配置：4分频
    SysCtlPWMClockSet(SYSCTL_PWMDIV_4);
    //使能PWM1模块
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    //使能PWM输出所在GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinConfigure(GPIO_PF1_M1PWM5);  //该函数不能连续位操作
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    //配置GPIO为PWM功能
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    //配置PWM发生器1：加减计数
    PWMGenConfigure(PWM1_BASE,PWM_GEN_2,PWM_GEN_MODE_UP_DOWN| PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE,PWM_GEN_3,PWM_GEN_MODE_UP_DOWN| PWM_GEN_MODE_NO_SYNC);
     //设置PWM发生器1的周期
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 25500);        //f~1k
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 25500);        //f~1k
    //设置GPIO输出的脉冲宽度
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 00000);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 00000);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 00000);
    //使能输出
    PWMOutputState(PWM1_BASE, (PWM_OUT_5_BIT|PWM_OUT_6_BIT|PWM_OUT_7_BIT), true);
    //使能PWM发生器2、3
    PWMGenEnable(PWM1_BASE,PWM_GEN_2);
    PWMGenEnable(PWM1_BASE,PWM_GEN_3);

}
/**
  * 函 数 名:LED_Color
  * 函数功能: 颜色控制函数
  * 输入参数: 所需颜色的RGB值
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young
  *   2017.7.6
  */
void LED_Color(uint8_t r,uint8_t g,uint8_t b)
{
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, r*100);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, g*100);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, b*100);
}
/**
  * 函 数 名:LED_Config
  * 函数功能: LED初始化（高低电平控制）
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young
  *   2017.7.6
  */
void LED_Config(void)
{
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
}
/**
  * 函 数 名:LED_Set
  * 函数功能: LED颜色设置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young
  *   2017.7.6
  */
void LED_Set(uint8_t color)
{
    if(color==0)//红
    {
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1, GPIO_PIN_1);
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, 0);
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3, 0);
    }
    if(color==1)//蓝
    {
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1, 0);
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, GPIO_PIN_2);
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3, 0);
    }
    if(color==2)//绿
    {
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1, 0);
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, 0);
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3, GPIO_PIN_3);
    }
    if(color==3)//品红
    {
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1, GPIO_PIN_1);
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, GPIO_PIN_2);
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3, 0);
    }
    if(color==4)//黄
    {
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1, GPIO_PIN_1);
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, 0);
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3, GPIO_PIN_3);
    }
    if(color==5)//浅蓝
    {
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1, 0);
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, GPIO_PIN_2);
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3, GPIO_PIN_3);
    }
    if(color==6)//白
    {
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1, GPIO_PIN_1);
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, GPIO_PIN_2);
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3, GPIO_PIN_3);
    }
    if(color==7)
    {
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1, 0);
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, 0);
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3, 0);
    }
}
/**
  * 函 数 名:Led_Twinkle
  * 函数功能:   LED闪烁函数
  * 输入参数: 颜色、闪烁次数
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young
  *   2017.7.6
  */
void Led_Twinkle(uint8_t color,uint8_t times)
{
    uint8_t t=0;
    for(t=0;t<times;t++)
    {
        LED_Set(color);
        Delay_ms(30);
        LED_Set(LedOff);
    }
}
