/**
  ******************************************************************************
  * �ļ�����: contral.c
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
 * ������ʼ��
 */
//�߶�:��λ MM
uint16_t Goal_Distance = 1100;//Ĭ�϶���ֵ800mm
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

/*ң����У׼ֵ
 *CH1 1100-1950
 *CH2 1100-1950
 *CH3 1100-1950
 *CH4 1100-1950
 *CH5 1100(����)-1520(����)-1950(����)
 */
//��ƫУ׼
uint16_t    Chane1_Stable=1520;
uint16_t    Chane2_Stable=1580;
uint16_t    Chane3_Stable=1100;
uint16_t    Chane4_Stable=1520;
/**
  * �� �� ��:UnlockPixhawk
  * ��������: Pixhawk��������
  * �������:
  * �� �� ֵ: ��
  * ˵    ��: ��
  *   By Sw Young
  *   2017.7.6
  */
void UnlockPixhawk(void)
{
    PwmControl_1(1520);
    PwmControl_2(1520);
    PwmControl_3(1100);
    PwmControl_4(1520);
    PwmControl_5(1100); //����ģʽ
    Delay_ms(1000);//��ʱ�ȴ�
    PwmControl_3(1100);
    PwmControl_4(1950);//����
    Delay_ms(1000);
    Delay_ms(1000);
    Delay_ms(1000);
    PwmControl_4(1520);

    PwmControl_1(Chane1_Stable);
    PwmControl_2(Chane2_Stable);//��ƫУ׼
    PwmControl_3(Chane3_Stable);
    PwmControl_4(Chane4_Stable);

    PwmControl_5(1520); //����ģʽ

}
/**
  * �� �� ��:LockPixhawk
  * ��������: Pixhawk������������
  * �������:
  * �� �� ֵ: ��
  * ˵    ��: ��
  *   By Sw Young
  *   2017.7.6
  */
void LockPixhawk(void)
{
    PwmControl_3(1100);
    PwmControl_4(1100);//����
    Delay_ms(1000);
    Delay_ms(1000);
    Delay_ms(1000);
    PwmControl_4(1520);
}
/**
  * �� �� ��:LandMode
  * ��������: ����������ģʽ����
  * �������:
  * �� �� ֵ: ��
  * ˵    ��: ��
  *   By Sw Young
  *   2017.7.6
  */
void LandMode(void)
{
    PwmControl_5(1950); //����ģʽ
    PwmControl_1(1520);
    PwmControl_2(1520);
    PwmControl_3(1100);
    PwmControl_4(1520);
}

void Point_Filter(int* pre_err,int* last_err,int* err)
{
    int tempa,tempb,tempc,max,min;//���ڷ������˲�
    //��ֵ�˲�����֤�õ����ݲ����䣬����������Ծʱ��΢������쳣
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
  * �� �� ��:Get_Coordinate
  * ��������: ��ȡXY���꺯��
  * �������:
  * �� �� ֵ: ��
  * ˵    ��: ��
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
  * �� �� ��:Get_Distance
  * ��������: ��ȡ�������߶Ⱥ���
  * �������:
  * �� �� ֵ: ��
  * ˵    ��: ��
  *   By Sw Young
  *   2017.7.6
  */
void Get_Distance(void)//����
{
    Real_Distance = GetAverageDistance();
}
/**
  * �� �� ��:AltitudeHold
  * ��������: ����ģʽ����
  * �������:
  * �� �� ֵ: ��
  * ˵    ��: ����������ģʽ��������pid��
  *   By Sw Young
  *   2017.7.6
  */
void AltitudeHold(void)//����
{
    Real_Distance = GetAverageDistance();
    if(Real_Distance>Goal_Distance)
        Error_Distance = Real_Distance-Goal_Distance;
    else if(Goal_Distance>Real_Distance)
        Error_Distance = Goal_Distance-Real_Distance;
}
/**
  * �� �� ��:OledDisplayInit
  * ��������: OLED��ʾ��ʼ������
  * �������:
  * �� �� ֵ: ��
  * ˵    ��: ��
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
  * �� �� ��:Display
  * ��������: OLED��ʾ����
  * �������:
  * �� �� ֵ: ��
  * ˵    ��: ��
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
  * �� �� ��:AttitudeProtection
  * ��������: ��̬��������
  * �������:
  * �� �� ֵ: ��
  * ˵    ��: ��
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
        //Delay_ms(1000);//��ʱ�ȴ�
        PwmControl_5(1100); //����ģʽ
        Delay_ms(1000);//��ʱ�ȴ�
        PwmControl_4(1100);//����
        Delay_ms(1000);
        Delay_ms(1000);
        Delay_ms(1000);
        PwmControl_4(1520);
        //LandMode();
    }
}
