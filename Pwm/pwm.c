/**
  ******************************************************************************
  * 文件名程: pwm.c
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
#include "pwm.h"
uint16_t PwmChannel_1 = 7600,\
         PwmChannel_2 = 7600,\
         PwmChannel_3 = 5500,\
         PwmChannel_4 = 7600,\
         PwmChannel_5 = 9750;
//channel5 1100 自稳 1520 定高 1950 降落
void BeepPwmInit (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);//使能PWM0模块
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//使能PWM0和PWM1输出所在GPIO
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);//配置PH0/PH1为PWM功能
    GPIOPinConfigure(GPIO_PB6_M0PWM0);    //#define GPIO_PB6_M0PWM0         0x00011804
    GPIOPinConfigure(GPIO_PB7_M0PWM1);    //#define GPIO_PB7_M0PWM1         0x00011C04
    //驱动电流8MA，推挽输出
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_6,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_32);     // PWM时钟配置：32不分频
    //配置PWM发生器0：加减计数，不同步
    PWMGenConfigure(PWM0_BASE,PWM_GEN_0,PWM_GEN_MODE_UP_DOWN| PWM_GEN_MODE_NO_SYNC);
    //设置PWM发生器1的频率，时钟频率/PWM分频数/n，80M/32/5000=500HZ
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 5000);
    //设置PWM0/PWM1输出的脉冲宽度
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,2500);//50%占空比
    //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 500);//25%占空比
    PWMDeadBandEnable(PWM0_BASE, PWM_GEN_0, 100, 100);
    //使能PWM0和PWM1的输出
    PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT |PWM_OUT_1_BIT), true);
    //使能PWM发生器
    PWMGenDisable(PWM0_BASE, PWM_GEN_0);
}
/**
  * 函 数 名:PwmConfig
  * 函数功能: pwm初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young
  *   2017.7.6
  */
void PwmConfig(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);//使能PWM0模块
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//使能PWM输出所在GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//使能PWM输出所在GPIO

    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);//配置PB4为PWM功能
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);//配置PB5为PWM功能
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);//配置PB7为PWM功能
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);//配置PE4为PWM功能
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);//配置PE5为PWM功能

    GPIOPinConfigure(GPIO_PB7_M0PWM1);    //#define GPIO_PB7_M0PWM1
    GPIOPinConfigure(GPIO_PB4_M0PWM2);    //#define GPIO_PB4_M0PWM2
    GPIOPinConfigure(GPIO_PB5_M0PWM3);    //#define GPIO_PB5_M0PWM3
    GPIOPinConfigure(GPIO_PE4_M0PWM4);    //#define GPIO_PE4_M0PWM4
    GPIOPinConfigure(GPIO_PE5_M0PWM5);    //#define GPIO_PE5_M0PWM5
    //驱动电流8MA，推挽输出
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_7,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_4,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);

    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_5,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);


      // PWM时钟配置：不分频
      SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
      //配置PWM发生器：加减计数
      PWMGenConfigure(PWM0_BASE,
                      PWM_GEN_0,PWM_GEN_MODE_UP_DOWN
                      | PWM_GEN_MODE_NO_SYNC);
      PWMGenConfigure(PWM0_BASE,
                      PWM_GEN_1,PWM_GEN_MODE_UP_DOWN
                      | PWM_GEN_MODE_NO_SYNC);
      PWMGenConfigure(PWM0_BASE,
                      PWM_GEN_2,PWM_GEN_MODE_UP_DOWN
                      | PWM_GEN_MODE_NO_SYNC);
      //设置PWM发生器的频率，时钟频率/PWM分频数/n，80M/16/100000=50HZ
      PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 100000);
      PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 100000);
      PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 100000);


      //设置PWM0/PWM1输出的脉冲宽度
      PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PwmChannel_1); //默认初始化为10% PB7
      PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PwmChannel_2); //默认初始化为10% PB4
      PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, PwmChannel_3); //默认初始化为10% PB5
      PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, PwmChannel_4); //默认初始化为10% PE4
      PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, PwmChannel_5); //默认初始化为10% PE5
      //使能PWM1和PWM2和PWM3和PWM4和PWM5的输出
      PWMOutputState(PWM0_BASE, (   PWM_OUT_1_BIT |
                                    PWM_OUT_2_BIT |
                                    PWM_OUT_3_BIT |
                                    PWM_OUT_4_BIT |
                                    PWM_OUT_5_BIT ), true);
      //使能PWM发生器
      PWMGenEnable(PWM0_BASE, PWM_GEN_0);
      PWMGenEnable(PWM0_BASE, PWM_GEN_1);
      PWMGenEnable(PWM0_BASE, PWM_GEN_2);

}
