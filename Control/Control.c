/**
  ******************************************************************************
  * 文件名程: contral.c
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
#include <0.96'OLED/OLED.h>
#include <0.96'OLED/OLED.h>
#include "head.h"
#include "colorful_LED/colorful_LED.h"
#include "Pwm/pwm.h"
#include "Timer/Timer.h"
#include "delay/delay.h"
#include "uart/uart.h"
#include "Beep/Beep.h"
#include "Control.h"

/*
 * 参数初始化
 */
//高度:单位 MM
uint16_t Goal_Distance = 1100;//默认定高值800mm
float volatile Real_Distance = 0;
uint16_t Error_Distance = 0;

int16_t RealAttitude_roll;
int16_t RealAttitude_pitch;
int16_t RealAttitude_yaw;

uint16_t err_roll=0,err_pitch=0;

uint16_t Goal_XCoordinate,Goal_YCoordinate;
uint16_t Real_XCoordinate,Real_YCoordinate;
extern volatile uint8_t get_x, get_y;
//extern int last_err_x,last_err_y;
//extern int pre_last_err_x,pre_last_err_y;

uint8_t Control_Open = 0;
bool Control_Serial = true;

/*遥控器校准值
 *CH1 1100-1950
 *CH2 1100-1950
 *CH3 1100-1950
 *CH4 1100-1950
 *CH5 1100(自稳)-1520(定高)-1950(降落)
 */
//零偏校准
uint16_t    Chane1_Stable=1520;
uint16_t    Chane2_Stable=1580;
uint16_t    Chane3_Stable=1100;
uint16_t    Chane4_Stable=1520;
/**
  * 函 数 名:UnlockPixhawk
  * 函数功能: Pixhawk解锁函数
  * 输入参数:
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young
  *   2017.7.6
  */
void UnlockPixhawk(void)
{
    PwmControl_1(1520);
    PwmControl_2(1520);
    PwmControl_3(1100);
    PwmControl_4(1520);
    PwmControl_5(1100); //自稳模式
    Delay_ms(1000);//延时等待
    PwmControl_3(1100);
    PwmControl_4(1950);//解锁
    Delay_ms(1000);
    Delay_ms(1000);
    Delay_ms(1000);
    PwmControl_4(1520);

    PwmControl_1(Chane1_Stable);
    PwmControl_2(Chane2_Stable);//零偏校准
    PwmControl_3(Chane3_Stable);
    PwmControl_4(Chane4_Stable);

    PwmControl_5(1520); //定高模式

}
/**
  * 函 数 名:LockPixhawk
  * 函数功能: Pixhawk上锁函数函数
  * 输入参数:
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young
  *   2017.7.6
  */
void LockPixhawk(void)
{
    PwmControl_3(1100);
    PwmControl_4(1100);//上锁
    Delay_ms(1000);
    Delay_ms(1000);
    Delay_ms(1000);
    PwmControl_4(1520);
}
/**
  * 函 数 名:LandMode
  * 函数功能: 飞行器降落模式函数
  * 输入参数:
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young
  *   2017.7.6
  */
void LandMode(void)
{
    PwmControl_5(1950); //降落模式
    PwmControl_1(1520);
    PwmControl_2(1520);
    PwmControl_3(1100);
    PwmControl_4(1520);
}

void Point_Filter(int* pre_err,int* last_err,int* err)
{
    int tempa,tempb,tempc,max,min;//用于防跳变滤波
    //均值滤波，保证得到数据不跳变，避免期望阶跃时，微分输出异常
    tempa=*pre_err;
    tempb=*last_err;
    tempc=*err;
    max = tempa > tempb ? tempa:tempb;
    max = max > tempc ? max:tempc;
    min = tempa < tempb ? tempa:tempb;
    min = min < tempc ? min:tempc;
    if(tempa > min && tempa < max)    *err = tempa;
    if(tempb > min  && tempb < max )  *err = tempb;
    if(tempc > min  &&  tempc < max)  *err = tempc;
    *pre_err = *last_err;
    *last_err = *err;
}
/**
  * 函 数 名:Get_Coordinate
  * 函数功能: 获取XY坐标函数
  * 输入参数:
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young
  *   2017.7.6
  */
void Get_Coordinate(void)
{
//    if(Real_XCoordinate==80&&Real_YCoordinate==60)
//    {
//        get_x = 60;
//        get_y = 90;
//    }
//    else
//    {
        get_x = Real_XCoordinate;
        get_y = Real_YCoordinate;
//    }
}
/**
  * 函 数 名:Get_Distance
  * 函数功能: 获取飞行器高度函数
  * 输入参数:
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young
  *   2017.7.6
  */
void Get_Distance(void)//舍弃
{
    Real_Distance = GetAverageDistance();
}
/**
  * 函 数 名:AltitudeHold
  * 函数功能: 自稳模式飞行
  * 输入参数:
  * 返 回 值: 无
  * 说    明: 舍弃（飞行模式控制移至pid）
  *   By Sw Young
  *   2017.7.6
  */
void AltitudeHold(void)//舍弃
{
    Real_Distance = GetAverageDistance();
    if(Real_Distance>Goal_Distance)
        Error_Distance = Real_Distance-Goal_Distance;
    else if(Goal_Distance>Real_Distance)
        Error_Distance = Goal_Distance-Real_Distance;
}
/**
  * 函 数 名:OledDisplayInit
  * 函数功能: OLED显示初始化函数
  * 输入参数:
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young
  *   2017.7.6
  */
void OledDisplayInit(void)
{
    OLED_Clear();
    OLED_ShowString(16,0,"FourAxes2018",16);
    OLED_ShowString(6,3,"AxesState:",16);
    OLED_ShowString(90,3,"OFF",16);
    OLED_ShowString(6,6,"GoalDis:",16);
    OLED_ShowNum(50,6,Goal_Distance,4,16);
    OLED_ShowString(80,88,"MM",16);
}
/**
  * 函 数 名:Display
  * 函数功能: OLED显示函数
  * 输入参数:
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young
  *   2017.7.6
  */
void Display(void)
{
    OLED_ShowString(6,3,"AxesState:",16);
    if(Control_Open)
        OLED_ShowString(90,3,"ON ",16);
    else
        OLED_ShowString(90,3,"OFF",16);
    OLED_ShowString(6,6,"GoalDis:",16);
    OLED_ShowNum(56,6,Goal_Distance,4,16);
    OLED_ShowString(88,6,"MM",16);
}
/**
  * 函 数 名:AttitudeProtection
  * 函数功能: 姿态保护函数
  * 输入参数:
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young
  *   2017.7.6
  */
void AttitudeProtection(void)
{
    err_roll=fabs(RealAttitude_roll);
    err_pitch=fabs(RealAttitude_pitch);
    if(err_pitch>38||err_roll>38)
    {
        Control_Open = false;
        PwmControl_3(1100);
        //Delay_ms(1000);//延时等待
        PwmControl_5(1100); //自稳模式
        Delay_ms(1000);//延时等待
        PwmControl_4(1100);//上锁
        Delay_ms(1000);
        Delay_ms(1000);
        Delay_ms(1000);
        PwmControl_4(1520);
        //LandMode();
    }
}
