/**
  ******************************************************************************
  * 文件名程: main.c
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
/**
  ******************************************************************************
  * 文件名程: main.c
  * 作    者: By Sw Young
  * 版    本: V1.2
  * 功    能:
  * 编写日期: 2018.7.6
  ******************************************************************************
  * 说明：
  * 硬件平台：
  *   MCUc:TM4C123、Pixhawk、无线串口、无刷电机
  * 软件设计说明：
  *     测高、获取图像坐标在定时器中完成，左边按键开启定时器功能，右边按键关闭定时器功能。
  * Github：
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
  * 函 数 名:HardwareConfig
  * 函数功能: 硬件初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young
  *   2017.7.6
  */
void HardwareConfig(void)
{
    Uart0Iint();        //串口0初始化
    Uart1Iint();        //串口1初始化
    UART3Iint();        //串口3初始化

    PwmConfig();            //初始化PWM

    LED_Config();    //LED初始化
    LED_Set(BLUE);

    OLED_Init();            //初始化OLED
    OLED_Clear();
    Delay_ms(5);            //延时等待OLED初始化
/*
 * 更换
 */
//    Sonar_Configure();      //超声波初始化
//    Sonar_GPIOA_Configure();
    Sonar_GPIOA_Interrupt();//pix_hawk超声波读取需要初始化的IO中断

    Timer0_Config();    //定时器初始化
    Timer1_Config();
    Timer2_Config();

    Key_Configure();    //按键初始化
    Key_Interrupt();    //按键中断

    Mavlink_DateInit();
    IntMasterEnable();
    PID_Init();

}

extern uint16_t Real_XCoordinate,Real_YCoordinate;//申明坐标
extern uint16_t Goal_Distance;
extern float    Real_Distance;
extern int16_t  RealAttitude_roll,\
                RealAttitude_pitch,\
                RealAttitude_yaw;//申明参数
extern uint16_t err_roll,err_pitch;

extern uint8_t Control_Open;
extern bool start_PID_X;
extern bool start_PID_Y;
extern volatile bool start_PID_H;
extern uint8_t Para_Report;//回传
extern float DEFAULT_KP_X,DEFAULT_KD_X,DEFAULT_KP_Y,DEFAULT_KD_Y;//调参
extern uint16_t Chane1_Stable,Chane2_Stable,Chane3_Stable,Chane4_Stable;//零偏校准
extern uint8_t Mode_Flag;
bool mode4 =false;
//extern uint16_t get_x, get_y;   //用于声光报警
extern double fPeriod;//如果需要用pix_hawk超声波，则不需希尔排序加权值滤波
/**
  * 函 数 名:Main
  * 函数功能: 主函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young
  *   2017.7.6
  */
int main(void)
{
    bool Control_Open_Flag = true;//系统控制标志位，通过按键打开，一次有效
    bool Coordinate_Open_Flag = true;//X、Y方向pid调节标志，一次有效

    FPUEnable();        //开启浮点运算
    FPULazyStackingEnable();

    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); //主频设置80M
    HardwareConfig();   //硬件初始化
    Delay_ms(10);       //延时等待硬件初始化
    LED_Set(RED);       //设置LED指示灯
    OledDisplayInit();  //初始化OLED显示界面
    while(1)
    {
       //UARTprintf("Hello");//调试用
       // Real_Distance = GetAverageDistance();//获取高度在定时器中
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
        //小车飞机在0.5-1.5m内声光报警,模式4
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
