/**
  ******************************************************************************
  * 文件名程: sonar.c
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
#include "sonar.h"
double fPeriod=0;

volatile bool DataIsReady  = false;

void Sonar_Configure()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    //设置PA6为脉冲输出端--->Trig
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
}

//触发超声波测量
void SonarTrig(void)
{

    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);

    Delay_us(30);

    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);

}




uint32_t ui32Frequency;
volatile bool FinishFlag;

void Int_Handler_GPIOA(void)
  {
      /***********标准中断服务函数所需操作**********/
       uint32_t ui32IntStatus;
       static uint32_t ui32Time[2] = {0};
       static uint8_t u8Coun=0;
        ui32IntStatus = GPIOIntStatus(GPIO_PORTA_BASE, true);
        GPIOIntClear(GPIO_PORTA_BASE, ui32IntStatus);//清除中断标志位
        if((ui32IntStatus & GPIO_PIN_7)  == GPIO_PIN_7)
        {
            //已经开启了FPU,为什么u8Coun在后面++就不可以，而在前面++就可以？
            //              浮点运算？时间复杂度？
            //
            ui32Time[u8Coun++]= TimerValueGet(TIMER0_BASE, TIMER_A);
            GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_FALLING_EDGE);

            if(u8Coun >1)  //进行两次中断，中间的差值就是计数值
            {
                u8Coun = 0;
                fPeriod = ui32Time[1] > ui32Time[0] ? (ui32Time[1] - ui32Time[0]) : (ui32Time[1] - ui32Time[0] + 0xFFFFFFFF);
                GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_RISING_EDGE);
                DataIsReady = true;
            }
            //u8Coun++;
        }
}

//测五次，去掉最大值最小值，取平均值。
double GetAverageDistance()
{
    int i = 0;
    double dis[11] = {0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0};
    double AverageDistance = 0;
    for( i=0 ; i<=10 ; i++)
    {
        SonarTrig();
        if(DataIsReady == true)
        {
        dis[i] = fPeriod * 0.002125;
        }
     }
    AverageDistance = LimitingFilter(dis,11);
    return AverageDistance;
}



void Sonar_GPIOA_Configure(void)
{
    //使能GPIO外设
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
                ;
      //输入GPIO配置
      GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7);

}

void Sonar_GPIOA_Interrupt(void)
{
    Sonar_GPIOA_Configure();
    //中断配置
       /****初始化外部中断并且设置外部中断为上升沿触发方式********/
       GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_7);                     //打开外部中断
       GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_RISING_EDGE);//上升沿触发
//       GPIOIntRegister(GPIO_PORTA_BASE, Int_Handler_GPIOA);
       IntEnable(INT_GPIOA);
      // IntPrioritySet(INT_GPIOA, 0);                                   //中断优先级
       //GPIOIntDisable(GPIO_PORTA_BASE, GPIO_PIN_7);
       IntMasterEnable();

}



