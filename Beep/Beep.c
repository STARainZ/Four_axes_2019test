/**
  ******************************************************************************
  * 文件名程: BEEP.c
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
#include "beep.h"
#include "head.h"

//默认蜂鸣器低电平有效
void Beep_Configure()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    //输出GPIO配置
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE,GPIO_PIN_7);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);//关蜂鸣器
}

void BeepSet(uint8_t status)
{
    if(status)
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0);//开蜂鸣器
    if(status==0)
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);//关蜂鸣器

}
