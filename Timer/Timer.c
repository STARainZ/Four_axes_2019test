/**
  ******************************************************************************
  * 文件名程: Timer.c
  * 作    者: By Sw Young
  * 版    本: V1.0
  * 功    能:
  * 编写日期: 2018.3.29
  ******************************************************************************
  * 说明：
  * 硬件平台：
  *   MCUc:TM4C123
  * 软件设计说明：
  *
  * Github：
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
uint8_t land_counter3=0;//用于模式3计数
bool land_coun_sta=false;   //降黑点
bool land_mode1_stablize_sta=false;//模式1自稳砸地
bool land_car_sta=false;    //择地降落
bool land_carcount_sta=false;   //择地降落计数标志位
uint8_t land_carcount_sta3=0;  //模式3择地降落计数标志位
bool followcount_sta=false;//跟车计数标志位
bool follow_flag=false;//目的是使followcount_sta只置true一次
uint8_t land_away_car=0;  //远离小车降落计时
bool awaycount_sta=false;//开始远离计数
bool away_flag=false;//远离计数是否完成
bool away_over=false;//远离计数是否完成
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
  * 函 数 名:MotorContolTimer.c
  * 函数功能:
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明:
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
       //不分频
       //设置为向上计数模式
        TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);
       //
       //把计数值装满
       TimerLoadSet(TIMER0_BASE, TIMER_A,0xFFFFFFFF);//2
       //
       //***************************中断使能*******************************************
       //默认不中断
       //使能TIME0B在基数结束时中断
       //TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
       //使能TIMER0B中断
       //IntEnable(INT_TIMER0A);
       IntDisable(INT_TIMER0A);//2
       //动态注册
       //TimerIntRegister(ui32Base, ui32Timer, pfnHandler)
       //使能TIMER0B,开始计数
       //TimerEnable(TIMER0_BASE, TIMER_B);
       TimerEnable(TIMER0_BASE, TIMER_A);//2
       //使能处理器中断
       IntMasterEnable();
}
/**
  * 函 数 名:Timer0IntHandler.c
  * 函数功能:
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明:
  *   By Sw Young
  *   2018.03.29
  */
uint8_t k=0;
void Timer0IntHandler(void)
{
    uint32_t ui32IntStatus;
    ui32IntStatus = TimerIntStatus(TIMER0_BASE, true);
    TimerIntClear(TIMER0_BASE, ui32IntStatus);//清除中断标志位
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

        TimerLoadSet(TIMER1_BASE, TIMER_A,  1600000); //Fre = 主频/1600000 = 50HZ

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
  * 函 数 名:Timer1IntHandler.c
  * 函数功能:
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明:
  *   By Sw Young
  *   2018.03.29
  */
/*
 * 定时器1的中断服务函数在PID.C中
 */

/**
  * 函 数 名:Timer2Init,Timer2IntHandler.c
  * 函数功能:
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 定时任务使用
  *   By xx
  *   2018.03.29
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

        TimerLoadSet(TIMER2_BASE, TIMER_A,  16000000); //Fre = 主频/1600000 = 5HZ

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
    TimerIntClear(TIMER2_BASE, ui32IntStatus);//清除中断标志位
    TimerIntDisable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    //模式1计数15s
    if(land_coun_sta)
    {
        land_counter++;
        if(land_counter>45&&Mode_Flag==1) //降落
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
        Control_Open=false; //x,y定点关闭
        PwmControl_1(1520);
        PwmControl_2(1520);
        PwmControl_3(1100); //油门拉最低
        PwmControl_5(1100); //自稳模式
        land_mode1_stablize_sta=false;
    }
    //模式2计数5s，择地降落
    if(land_carcount_sta)
    {
        land_counter++;
        //uart3改阈值
        if(land_counter==34&&Mode_Flag==2)
        {
            UART3Send("12345",5);
        }
        if(land_counter>85) //降落
        {
            land_car_sta=true;
            land_carcount_sta=false;
            land_counter=0;
            UARTprintf("Land car\n");
        }
    }

    //模式3计数5s，择地降落
    if(land_carcount_sta3==1)
    {
        land_counter3++;
        if(land_counter3>25) //降落
        {
            land_car_sta=true;
            land_carcount_sta3=2;
            land_counter3=0;
            UARTprintf("Land car\n");
        }
    }

    if(land_car_sta)
    {
        PwmControl_5(1950); //降落模式
        start_PID_H=false;  //定点降
        awaycount_sta=true;
        land_car_sta=false;
    }

    //远离小车计时，用于改变flag
    if(awaycount_sta)
    {
        land_away_car++;
        away_over=false;
        if(land_away_car==2)
        {
            Control_Open=false; //x,y定点关闭
            PwmControl_1(1520); //x方向回中
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
    //根据flag进行舵量控制
    if(away_flag)
        PwmControl_2(1560);     //向后偏航
    if(away_over)
        PwmControl_2(1520);     //回中
    //模式3跟车计时,pitch计时
    if(followcount_sta)
    {
        land_counter3++;
        if(land_counter3==24&&Mode_Flag==3)  //uart3改阈值
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
//    //模式3五秒落地计时
//    {
//
//    }
    //定点时间计数,不能多次进！！！
    if(((Real_Distance>(Goal_Distance-50))&&Real_Distance<Goal_Distance)||((Real_Distance<(Goal_Distance+50))&&Real_Distance>Goal_Distance))
     {
        //模式1计时
        if(Mode_Flag==1&&start_PID_H)
            land_coun_sta=true;
        //模式2计时，PITCH计数器，同时包含降落计时，模式二专用
        else if(Mode_Flag==2&&start_PID_H)
            land_carcount_sta=true;
        //模式3跟车计时,PITCH计数器
        else if(Mode_Flag==3&&!follow_flag) //只进一次
        {
            followcount_sta=true;
            follow_flag=true;
        }
     }

//    ;
//    UARTprintf("Land calculate:%d\n",(int)(86-land_counter));
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);    //开关中断不能离太近

}
