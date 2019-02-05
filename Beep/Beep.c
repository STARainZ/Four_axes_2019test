/**
  ******************************************************************************
  * �ļ�����: BEEP.c
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
#include "beep.h"
#include "head.h"

//Ĭ�Ϸ������͵�ƽ��Ч
void Beep_Configure()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    //���GPIO����
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE,GPIO_PIN_7);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);//�ط�����
}

void BeepSet(uint8_t status)
{
    if(status)
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0);//��������
    if(status==0)
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);//�ط�����

}
