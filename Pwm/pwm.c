/**
  ******************************************************************************
  * �ļ�����: pwm.c
  * ��    ��: By Sw Young
  * ��    ��: V1.0
  * ��    ��:
  * ��д����: 2018.7.6
  ******************************************************************************
  * ˵����
  * Ӳ��ƽ̨��TM4C123G
  *   *****
  * ������˵����
  *   *****
  * Github��
  ******************************************************************************
**/
#include "pwm.h"
uint16_t PwmChannel_1 = 7600,\
         PwmChannel_2 = 7600,\
         PwmChannel_3 = 5500,\
         PwmChannel_4 = 7600,\
         PwmChannel_5 = 9750;
//channel5 1100 ���� 1520 ���� 1950 ����
void BeepPwmInit (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);//ʹ��PWM0ģ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//ʹ��PWM0��PWM1�������GPIO
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);//����PH0/PH1ΪPWM����
    GPIOPinConfigure(GPIO_PB6_M0PWM0);    //#define GPIO_PB6_M0PWM0         0x00011804
    GPIOPinConfigure(GPIO_PB7_M0PWM1);    //#define GPIO_PB7_M0PWM1         0x00011C04
    //��������8MA���������
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_6,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_32);     // PWMʱ�����ã�32����Ƶ
    //����PWM������0���Ӽ���������ͬ��
    PWMGenConfigure(PWM0_BASE,PWM_GEN_0,PWM_GEN_MODE_UP_DOWN| PWM_GEN_MODE_NO_SYNC);
    //����PWM������1��Ƶ�ʣ�ʱ��Ƶ��/PWM��Ƶ��/n��80M/32/5000=500HZ
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 5000);
    //����PWM0/PWM1�����������
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,2500);//50%ռ�ձ�
    //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 500);//25%ռ�ձ�
    PWMDeadBandEnable(PWM0_BASE, PWM_GEN_0, 100, 100);
    //ʹ��PWM0��PWM1�����
    PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT |PWM_OUT_1_BIT), true);
    //ʹ��PWM������
    PWMGenDisable(PWM0_BASE, PWM_GEN_0);
}
/**
  * �� �� ��:PwmConfig
  * ��������: pwm��ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  *   By Sw Young
  *   2017.7.6
  */
void PwmConfig(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);//ʹ��PWM0ģ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//ʹ��PWM�������GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//ʹ��PWM�������GPIO

    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);//����PB4ΪPWM����
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);//����PB5ΪPWM����
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);//����PB7ΪPWM����
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);//����PE4ΪPWM����
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);//����PE5ΪPWM����

    GPIOPinConfigure(GPIO_PB7_M0PWM1);    //#define GPIO_PB7_M0PWM1
    GPIOPinConfigure(GPIO_PB4_M0PWM2);    //#define GPIO_PB4_M0PWM2
    GPIOPinConfigure(GPIO_PB5_M0PWM3);    //#define GPIO_PB5_M0PWM3
    GPIOPinConfigure(GPIO_PE4_M0PWM4);    //#define GPIO_PE4_M0PWM4
    GPIOPinConfigure(GPIO_PE5_M0PWM5);    //#define GPIO_PE5_M0PWM5
    //��������8MA���������
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_7,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_4,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);

    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_5,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);


      // PWMʱ�����ã�����Ƶ
      SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
      //����PWM���������Ӽ�����
      PWMGenConfigure(PWM0_BASE,
                      PWM_GEN_0,PWM_GEN_MODE_UP_DOWN
                      | PWM_GEN_MODE_NO_SYNC);
      PWMGenConfigure(PWM0_BASE,
                      PWM_GEN_1,PWM_GEN_MODE_UP_DOWN
                      | PWM_GEN_MODE_NO_SYNC);
      PWMGenConfigure(PWM0_BASE,
                      PWM_GEN_2,PWM_GEN_MODE_UP_DOWN
                      | PWM_GEN_MODE_NO_SYNC);
      //����PWM��������Ƶ�ʣ�ʱ��Ƶ��/PWM��Ƶ��/n��80M/16/100000=50HZ
      PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 100000);
      PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 100000);
      PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 100000);


      //����PWM0/PWM1�����������
      PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PwmChannel_1); //Ĭ�ϳ�ʼ��Ϊ10% PB7
      PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PwmChannel_2); //Ĭ�ϳ�ʼ��Ϊ10% PB4
      PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, PwmChannel_3); //Ĭ�ϳ�ʼ��Ϊ10% PB5
      PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, PwmChannel_4); //Ĭ�ϳ�ʼ��Ϊ10% PE4
      PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, PwmChannel_5); //Ĭ�ϳ�ʼ��Ϊ10% PE5
      //ʹ��PWM1��PWM2��PWM3��PWM4��PWM5�����
      PWMOutputState(PWM0_BASE, (   PWM_OUT_1_BIT |
                                    PWM_OUT_2_BIT |
                                    PWM_OUT_3_BIT |
                                    PWM_OUT_4_BIT |
                                    PWM_OUT_5_BIT ), true);
      //ʹ��PWM������
      PWMGenEnable(PWM0_BASE, PWM_GEN_0);
      PWMGenEnable(PWM0_BASE, PWM_GEN_1);
      PWMGenEnable(PWM0_BASE, PWM_GEN_2);

}
