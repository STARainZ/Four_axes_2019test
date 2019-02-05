/**
  ******************************************************************************
  * �ļ�����: Timer.c
  * ��    ��: By Sw Young
  * ��    ��: V1.0
  * ��    ��:
  * ��д����: 2018.3.29
  ******************************************************************************
  * ˵����
  * Ӳ��ƽ̨��
  *   MCUc:TM4C123
  * ������˵����
  *
  * Github��
  ******************************************************************************
**/
#include "timer.h"
#include "Control/Control.h"
#include "Pwm/pwm.h"
#include "delay/delay.h"

char Time_Flag = 0;
uint32_t Counter = 0;
uint8_t Beep_Flag = 0;
uint32_t Beep_Counter = 0;
uint32_t Beep_Fre = 40;
uint8_t land_counter=0;
uint8_t land_counter3=0;//����ģʽ3����
bool land_coun_sta=false;   //���ڵ�
bool land_mode1_stablize_sta=false;//ģʽ1�����ҵ�
bool land_car_sta=false;    //��ؽ���
bool land_carcount_sta=false;   //��ؽ��������־λ
uint8_t land_carcount_sta3=0;  //ģʽ3��ؽ��������־λ
bool followcount_sta=false;//����������־λ
bool follow_flag=false;//Ŀ����ʹfollowcount_staֻ��trueһ��
uint8_t land_away_car=0;  //Զ��С�������ʱ
bool awaycount_sta=false;//��ʼԶ�����
bool away_flag=false;//Զ������Ƿ����
bool away_over=false;//Զ������Ƿ����
extern float volatile  Real_Distance;
extern volatile bool start_PID_H;
extern bool start_PID_Y;
extern uint8_t Control_Open;
extern uint16_t Goal_Distance;
extern uint8_t Mode_Flag;

bool Delay_s_Timer(int Time,int Freq_Timer,int* Counter)//s,hz,varience for
{
    static bool Flag=false;
    *Counter+=1;
    if(*Counter<Time*Freq_Timer)
    {
        Flag=false;
    }
    else if(*Counter>=Time*Freq_Timer)
    {
        *Counter=0;
        Flag=true;
    }
    return Flag;
}


/**
  * �� �� ��:MotorContolTimer.c
  * ��������:
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��:
  *   By Sw Young
  *   2018.03.29
  */
void Timer0_Config(void)
{
       //
       // Enable the peripherals used by this example.
       //
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
       //
       // Enable processor interrupts.
       //
       //����Ƶ
       //����Ϊ���ϼ���ģʽ
        TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);
       //
       //�Ѽ���ֵװ��
       TimerLoadSet(TIMER0_BASE, TIMER_A,0xFFFFFFFF);//2
       //
       //***************************�ж�ʹ��*******************************************
       //Ĭ�ϲ��ж�
       //ʹ��TIME0B�ڻ�������ʱ�ж�
       //TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
       //ʹ��TIMER0B�ж�
       //IntEnable(INT_TIMER0A);
       IntDisable(INT_TIMER0A);//2
       //��̬ע��
       //TimerIntRegister(ui32Base, ui32Timer, pfnHandler)
       //ʹ��TIMER0B,��ʼ����
       //TimerEnable(TIMER0_BASE, TIMER_B);
       TimerEnable(TIMER0_BASE, TIMER_A);//2
       //ʹ�ܴ������ж�
       IntMasterEnable();
}
/**
  * �� �� ��:Timer0IntHandler.c
  * ��������:
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��:
  *   By Sw Young
  *   2018.03.29
  */
uint8_t k=0;
void Timer0IntHandler(void)
{
    uint32_t ui32IntStatus;
    ui32IntStatus = TimerIntStatus(TIMER0_BASE, true);
    TimerIntClear(TIMER0_BASE, ui32IntStatus);//����жϱ�־λ
    if((GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0) & GPIO_PIN_0)  != GPIO_PIN_0)
    {
    KeyPress0=(1+KeyPress0)%2;
    }

    if(k)
        {k=0;GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, GPIO_PIN_2);}
    else
        {k=1;GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, 0);}
}
void Timer1_Config(void)
{
       //
       // Enable the peripherals used by this example.
       //
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
       //
       // Enable processor interrupts.
       //
        IntMasterEnable();

       //
       // Configure the two 32-bit periodic timers.
       //
        TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);

        TimerLoadSet(TIMER1_BASE, TIMER_A,  1600000); //Fre = ��Ƶ/1600000 = 50HZ

       //
       // Setup the interrupts for the timer timeouts.
       //
        IntEnable(INT_TIMER1A);

        TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
       //
       // Enable the timers.
       //
        TimerEnable(TIMER1_BASE, TIMER_A);
}
/**
  * �� �� ��:Timer1IntHandler.c
  * ��������:
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��:
  *   By Sw Young
  *   2018.03.29
  */
/*
 * ��ʱ��1���жϷ�������PID.C��
 */

/**
  * �� �� ��:Timer2Init,Timer2IntHandler.c
  * ��������:
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��ʱ����ʹ��
  *   By xx
  *   2019.1.30
  */
void Timer2_Config(void)
{
       //
       // Enable the peripherals used by this example.
       //
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
       //
       // Enable processor interrupts.
       //
        IntMasterEnable();

       //
       // Configure the two 32-bit periodic timers.
       //
        TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);

        TimerLoadSet(TIMER2_BASE, TIMER_A,  16000000); //Fre = ��Ƶ/1600000 = 5HZ

       //
       // Setup the interrupts for the timer timeouts.
       //
        IntEnable(INT_TIMER2A);

        TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
       //
       // Enable the timers.
       //
        TimerEnable(TIMER2_BASE, TIMER_A);
}

void Timer2IntHandler(void)
{
    uint32_t ui32IntStatus;
    ui32IntStatus = TimerIntStatus(TIMER2_BASE, true);
    TimerIntClear(TIMER2_BASE, ui32IntStatus);//����жϱ�־λ
    TimerIntDisable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    //����ʱ�����,ֻ��һ�Σ�����
    if(((Real_Distance>(Goal_Distance-50))&&Real_Distance<Goal_Distance)||((Real_Distance<(Goal_Distance+50))&&Real_Distance>Goal_Distance))
    {
        //ģʽ1��ʱ
        if(Mode_Flag==1&&start_PID_H)
            land_coun_sta=true;
        //ģʽ2��ʱ��PITCH��������ͬʱ���������ʱ��ģʽ��ר��
        else if(Mode_Flag==2&&start_PID_H)
            land_carcount_sta=true;
        //ģʽ3������ʱ,PITCH������
        else if(Mode_Flag==3&&!follow_flag) //ֻ��һ��
        {
            followcount_sta=true;
            follow_flag=true;
        }
    }

    //ģʽ1����10s
    if(land_coun_sta)
    {
        land_counter++;
        if(land_counter>45&&Mode_Flag==1) //����
        {
            start_PID_H=false;
            land_coun_sta=false;
            land_counter=0;
            LandMode();
            land_mode1_stablize_sta=true;
            UARTprintf("Land\n");
        }
    }
    if(land_mode1_stablize_sta&&(int)Real_Distance<200)
    {
        Control_Open=false; //x,y����ر�
        PwmControl_1(1520);
        PwmControl_2(1520);
        PwmControl_3(1100); //���������
        PwmControl_5(1100); //����ģʽ
        land_mode1_stablize_sta=false;
    }
    //ģʽ2����5s����ؽ���
    if(land_carcount_sta)
    {
        land_counter++;
        //uart3����ֵ
        if(land_counter==34&&Mode_Flag==2)
        {
            UART3Send("12345",5);
        }
        //��ʱ��1�д��
        if(land_counter>85) //����
        {
            land_car_sta=true;
            land_carcount_sta=false;
            land_counter=0;
            UARTprintf("Land car\n");
        }
    }

    //ģʽ3������ʱ,pitch��ʱ
    if(followcount_sta)
    {
        land_counter3++;
        if(land_counter3==24&&Mode_Flag==3)  //uart3����ֵ
        {
            UART3Send("12345",5);
        }
        else if(land_counter3>30&&Mode_Flag==3)
        {
            followcount_sta=false;
            land_counter3=0;
            UARTprintf("follow car\n");
        }
    }
    //ģʽ3����5s����ؽ���
    if(land_carcount_sta3==1)
    {
        land_counter3++;
        if(land_counter3>25) //����
        {
            land_car_sta=true;
            land_carcount_sta3=2;
            land_counter3=0;
            UARTprintf("Land car\n");
        }
    }

    //��ؽ���ģʽ2��3ͨ�ã�ʱ�����
    if(land_car_sta)
    {
        PwmControl_5(1950); //����ģʽ
        start_PID_H=false;  //���㽵
        awaycount_sta=true;
        land_car_sta=false;
    }
    //Զ��С����ʱ�����ڸı�flag
    if(awaycount_sta)
    {
        land_away_car++;
        away_over=false;
        if(land_away_car==2)
        {
            Control_Open=false; //x,y����ر�
            PwmControl_1(1520); //x�������
            away_flag=true;
        }
        if(land_away_car>8)
        {
            away_flag=false;
            away_over=true;
            land_away_car=0;
            awaycount_sta=false;
            UARTprintf("away car\n");
        }
    }
    //����flag���ж�������
    if(away_flag)
        PwmControl_2(1560);     //���ƫ��
    if(away_over)
        PwmControl_2(1520);     //����


    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
}
