/**
  ******************************************************************************
  * �ļ�����: sonar.c
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
#include "sonar.h"
double fPeriod=0;

volatile bool DataIsReady  = false;

void Sonar_Configure()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    //����PA6Ϊ���������--->Trig
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
}

//��������������
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
      /***********��׼�жϷ������������**********/
       uint32_t ui32IntStatus;
       static uint32_t ui32Time[2] = {0};
       static uint8_t u8Coun=0;
        ui32IntStatus = GPIOIntStatus(GPIO_PORTA_BASE, true);
        GPIOIntClear(GPIO_PORTA_BASE, ui32IntStatus);//����жϱ�־λ
        if((ui32IntStatus & GPIO_PIN_7)  == GPIO_PIN_7)
        {
            //�Ѿ�������FPU,Ϊʲôu8Coun�ں���++�Ͳ����ԣ�����ǰ��++�Ϳ��ԣ�
            //              �������㣿ʱ�临�Ӷȣ�
            //
            ui32Time[u8Coun++]= TimerValueGet(TIMER0_BASE, TIMER_A);
            GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_FALLING_EDGE);

            if(u8Coun >1)  //���������жϣ��м�Ĳ�ֵ���Ǽ���ֵ
            {
                u8Coun = 0;
                fPeriod = ui32Time[1] > ui32Time[0] ? (ui32Time[1] - ui32Time[0]) : (ui32Time[1] - ui32Time[0] + 0xFFFFFFFF);
                GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_RISING_EDGE);
                DataIsReady = true;
            }
            //u8Coun++;
        }
}

//����Σ�ȥ�����ֵ��Сֵ��ȡƽ��ֵ��
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
    //ʹ��GPIO����
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
                ;
      //����GPIO����
      GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7);

}

void Sonar_GPIOA_Interrupt(void)
{
    Sonar_GPIOA_Configure();
    //�ж�����
       /****��ʼ���ⲿ�жϲ��������ⲿ�ж�Ϊ�����ش�����ʽ********/
       GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_7);                     //���ⲿ�ж�
       GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_RISING_EDGE);//�����ش���
//       GPIOIntRegister(GPIO_PORTA_BASE, Int_Handler_GPIOA);
       IntEnable(INT_GPIOA);
      // IntPrioritySet(INT_GPIOA, 0);                                   //�ж����ȼ�
       //GPIOIntDisable(GPIO_PORTA_BASE, GPIO_PIN_7);
       IntMasterEnable();

}



