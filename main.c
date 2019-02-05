/**
  ******************************************************************************
  * �ļ�����: main.c
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
#include <stdint.h>
#include <stdbool.h>
/**
  ******************************************************************************
  * �ļ�����: main.c
  * ��    ��: By Sw Young
  * ��    ��: V1.2
  * ��    ��:
  * ��д����: 2018.7.6
  ******************************************************************************
  * ˵����
  * Ӳ��ƽ̨��
  *   MCUc:TM4C123��Pixhawk�����ߴ��ڡ���ˢ���
  * ������˵����
  *     ��ߡ���ȡͼ�������ڶ�ʱ������ɣ���߰���������ʱ�����ܣ��ұ߰����رն�ʱ�����ܡ�
  * Github��
  ******************************************************************************
**/

#include <stdint.h>
#include <stdbool.h>
#include "stdio.h"
#include "math.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "colorful_LED/colorful_LED.h"
#include "Pwm/pwm.h"
#include "Timer/Timer.h"
#include "delay/delay.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "uart/uart.h"
#include "uart/uartstdio.h"
#include "Beep/Beep.h"
#include "head.h"
#include "Control/Control.h"
#include "sonar/sonar.h"
#include "key/key.h"
#include "MavLink_Receive/mavlink_recieve.h"
#include "0.96'OLED/OLED.h"

/**
  * �� �� ��:HardwareConfig
  * ��������: Ӳ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  *   By Sw Young
  *   2017.7.6
  */
void HardwareConfig(void)
{
    Uart0Iint();        //����0��ʼ��
    Uart1Iint();        //����1��ʼ��
    UART3Iint();        //����3��ʼ��

    PwmConfig();            //��ʼ��PWM

    LED_Config();    //LED��ʼ��
    LED_Set(BLUE);

    OLED_Init();            //��ʼ��OLED
    OLED_Clear();
    Delay_ms(5);            //��ʱ�ȴ�OLED��ʼ��
/*
 * ����
 */
//    Sonar_Configure();      //��������ʼ��
//    Sonar_GPIOA_Configure();
    Sonar_GPIOA_Interrupt();//pix_hawk��������ȡ��Ҫ��ʼ����IO�ж�

    Timer0_Config();    //��ʱ����ʼ��
    Timer1_Config();
    Timer2_Config();

    Key_Configure();    //������ʼ��
    Key_Interrupt();    //�����ж�

    Mavlink_DateInit();
    IntMasterEnable();
    PID_Init();

}

extern uint16_t Real_XCoordinate,Real_YCoordinate;//��������
extern uint16_t Goal_Distance;
extern float    Real_Distance;
extern int16_t  RealAttitude_roll,\
                RealAttitude_pitch,\
                RealAttitude_yaw;//��������
extern uint16_t err_roll,err_pitch;

extern uint8_t Control_Open;
extern bool start_PID_X;
extern bool start_PID_Y;
extern volatile bool start_PID_H;
extern uint8_t Para_Report;//�ش�
extern float DEFAULT_KP_X,DEFAULT_KD_X,DEFAULT_KP_Y,DEFAULT_KD_Y;//����
extern uint16_t Chane1_Stable,Chane2_Stable,Chane3_Stable,Chane4_Stable;//��ƫУ׼
extern uint8_t Mode_Flag;
bool mode4 =false;
//extern uint16_t get_x, get_y;   //�������ⱨ��
extern double fPeriod;//�����Ҫ��pix_hawk������������ϣ�������Ȩֵ�˲�
/**
  * �� �� ��:Main
  * ��������: ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  *   By Sw Young
  *   2017.7.6
  */
int main(void)
{
    bool Control_Open_Flag = true;//ϵͳ���Ʊ�־λ��ͨ�������򿪣�һ����Ч
    bool Coordinate_Open_Flag = true;//X��Y����pid���ڱ�־��һ����Ч

    FPUEnable();        //������������
    FPULazyStackingEnable();

    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); //��Ƶ����80M
    HardwareConfig();   //Ӳ����ʼ��
    Delay_ms(10);       //��ʱ�ȴ�Ӳ����ʼ��
    LED_Set(RED);       //����LEDָʾ��
    OledDisplayInit();  //��ʼ��OLED��ʾ����
    while(1)
    {
       //UARTprintf("Hello");//������
       // Real_Distance = GetAverageDistance();//��ȡ�߶��ڶ�ʱ����
       if(DataIsReady == true)//pix_hawk sonor
       {
            Real_Distance=fPeriod*0.002125;
//            UARTprintf("ALT:\t%d\t\n",(int)Real_Distance);
        }
        if(Control_Open&&Control_Open_Flag)
        {
            Control_Open_Flag=true;
            Control_Open_Flag = false;
            UnlockPixhawk();
            start_PID_H = true;
            LED_Set(GREEN);
            UARTprintf("alt_control\n");
        }
        if(Real_Distance>200&&Control_Open&&Coordinate_Open_Flag)
        {
            Coordinate_Open_Flag = false;
            start_PID_X = true;
            start_PID_Y = true;
        }
        calculate_test();
        Display();
        if(start_PID_H!=true)
        {
//            UARTprintf("RealDistance: %d\n",(int)Real_Distance);
//            UARTprintf("GoalDistance: %d\n",Goal_Distance);
//            UARTprintf("x=%d,y=%d\n",Real_XCoordinate,Real_YCoordinate);
//            UARTprintf("Roll: %d;pitch: %d;yaw:%d\n",RealAttitude_roll,RealAttitude_pitch,RealAttitude_yaw);
//            UARTprintf("err_roll:%d;err_pitch%d;\n",err_roll=fabs(RealAttitude_roll),err_pitch=fabs(RealAttitude_pitch));
        }
        if(Para_Report!=0)
        {
            if(Para_Report==1)
            {
                UARTprintf("XP%d\tXD%d\tYP%d\tYD%d\tROLL%d\tPITCH%d\tYAW%d\t\n",(int)(PID_X.Kp*1000),(int)(PID_X.Kd*1000),(int)(PID_Y.Kp*1000),(int)(PID_Y.Kd*1000),(int)(Chane1_Stable),(int)(Chane2_Stable),(int)(Chane4_Stable));
//                SysCtlDelay(SysCtlClockGet() / (1000 * 3));
//                UARTprintf("XD00%d",(int)(DEFAULT_KD_X*100));
                Para_Report++;
            }
//            else if(Para_Report==2)
//            {
//                UARTprintf("YP0%d",(int)(DEFAULT_KP_Y*100));
//                SysCtlDelay(SysCtlClockGet() / (1000 * 3));
//                UARTprintf("YD00%d",(int)(DEFAULT_KD_Y*100));
//                Para_Report=0;
//            }
        }
        mode4=false;
        //С���ɻ���0.5-1.5m�����ⱨ��,ģʽ4
        if(Real_XCoordinate!=80&&Real_YCoordinate!=60&&(int)Real_Distance>500&&(int)Real_Distance<1500&&Mode_Flag==4)
        {
            //BeepSet(0);
            LED_Set(WHITE);
            UARTprintf("WARN");
            mode4=true;
        }
//        if(!(Real_XCoordinate!=80&&Real_YCoordinate!=60&&(int)Real_Distance>500&&(int)Real_Distance<1500)&&Mode_Flag==4)
        if(!mode4&&Mode_Flag==4)
        {
            //BeepSet(1);
            LED_Set(LedOff);
            UARTprintf("OFF");
        }

        Delay_ms(500);
    }
}
