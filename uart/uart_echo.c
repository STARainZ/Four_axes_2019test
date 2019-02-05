/**
  ******************************************************************************
  * 文件名程: Uart_echo.c
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
//*****************************************************************************
//
// uart_echo.c - Example for reading data from and writing data to the UART in
//               an interrupt driven fashion.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "uart.h"
#include "Timer/Timer.h"
#include "head.h"
#include "Control/Control.h"
#include "Pwm/pwm.h"
//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART Echo (uart_echo)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the USB debug virtual serial port on the evaluation board)
//! will be configured in 115,200 baud, 8-n-1 mode.  All characters received on
//! the UART are transmitted back to the UART.
//
//*****************************************************************************

/*---------------------------------------------------------------Beautiful Line---------------------------------------------------------*/
//UART0
uint8_t ReciveData_UART0[16];
uint8_t ReciveData_i_UART0 = 0;
extern bool Control_Serial;
extern bool Control_Open;
extern uint8_t Mode_Flag;
extern uint8_t land_carcount_sta3;
extern float DEFAULT_KP_X,DEFAULT_KD_X,DEFAULT_KP_Y,DEFAULT_KD_Y;//参数校准
extern uint16_t Chane1_Stable,Chane2_Stable,Chane3_Stable,Chane4_Stable;//零偏校准
//extern uint8_t land_counter;
int Para_Compared;
uint8_t Para_Report=0;
//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UARTIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART0_BASE, ui32Status);

    ReciveData_i_UART0 = 0;
    //
    // Loop while there are characters in the receive FIFO.
    //
    while(UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
//        UARTCharPutNonBlocking(UART0_BASE,
//                                   UARTCharGetNonBlocking(UART0_BASE));

        ReciveData_UART0[ReciveData_i_UART0] = UARTCharGetNonBlocking(UART0_BASE);
        ReciveData_i_UART0++;
        //
        // Blink the LED to show a character transfer is occuring.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        SysCtlDelay(SysCtlClockGet() / (1000 * 3));

        //
        // Turn off the LED
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

    }

 //   UARTSend()
    UART0Send(ReciveData_UART0, ReciveData_i_UART0);
}


//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UART0Send(uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
    }
}



//*****************************************************************************
//
// uart0 init
//
//*****************************************************************************
void Uart0Iint(void)
{
//    //
//     // Enable the peripherals used by this example.
//     //
//     SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
//     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//
//     //
//     // Enable processor interrupts.
//     //
//     IntMasterEnable();
//
//     //
//     // Set GPIO A0 and A1 as UART pins.
//     //
//     GPIOPinConfigure(GPIO_PA0_U0RX);
//     GPIOPinConfigure(GPIO_PA1_U0TX);
//     GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
//
//     //
//     // Configure the UART for 115,200, 8-N-1 operation.
//     //
//     UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
//                             (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
//                              UART_CONFIG_PAR_NONE));
//
//     //
//     // Enable the UART interrupt.
//     //
//     IntEnable(INT_UART0);
//     UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
//
//     //
//     // Prompt for text to be entered.
//     //
//     UART0Send((uint8_t *)"\n UART0 Is OK!!\n\n ", 17);


/*
 * 另一种配置
 */
    // Enable GPIOC
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
       while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)));
       //
       // Enable UART0
       //
       SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
       while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)));


       //
       // Configure GPIO Pins for UART mode.
       //
        // Set GPIO A0 and A1 as UART pins.
        //
        GPIOPinConfigure(GPIO_PA0_U0RX);
        GPIOPinConfigure(GPIO_PA1_U0TX);
        GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

       //
       // Use the internal 16MHz oscillator as the UART clock source.
       //
       UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

       //
       // Initialize the UART for console I/O.
       //
       UARTStdioConfig(1, 115200, 16000000);

       UARTprintf("UART0 Is Ok!\n");


}



/*---------------------------------------------------------------Beautiful Line---------------------------------------------------------*/

//UART1
uint8_t ReciveData_UART1[16];
uint8_t ReciveData_i_UART1 = 0;

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UART1IntHandler(void)
{
    uint32_t ui32Status;
    //
    // Get the interrrupt status.
    //
    ui32Status = UARTIntStatus(UART1_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART1_BASE, ui32Status);

    ReciveData_i_UART1 = 0;
    //
    // Loop while there are characters in the receive FIFO.
    //
    while(UARTCharsAvail(UART1_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
//        UARTCharPutNonBlocking(UART1_BASE,
//                                   UARTCharGetNonBlocking(UART1_BASE));

        ReciveData_UART1[ReciveData_i_UART1] = UARTCharGetNonBlocking(UART1_BASE);
        ReciveData_i_UART1++;
        //
        // Blink the LED to show a character transfer is occuring.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        SysCtlDelay(SysCtlClockGet() / (1000 * 3));

        //
        // Turn off the LED
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

    }

 //   UARTSend()
//    UART1Send("Received: ",10);
//    UART1Send(ReciveData_UART1, ReciveData_i_UART1);
        if(ReciveData_UART1[0]=='S'&&ReciveData_UART1[1]=='T'&&ReciveData_UART1[2]=='O'&&ReciveData_UART1[3]=='P')
        {
            LandMode();
            Control_Serial= false;
            UARTprintf("Land\n");
        }
        else if(ReciveData_UART1[0]=='S'&&ReciveData_UART1[1]=='T'&&ReciveData_UART1[2]=='A'&&ReciveData_UART1[3]=='T')
        {
            Control_Open=true;
            UARTprintf("USTA\n");

        }
//        else if(ReciveData_UART1[0]=='E'&&ReciveData_UART1[1]=='M'&&ReciveData_UART1[2]=='R'&&ReciveData_UART1[3]=='G')//紧急制动
//        {
//            PwmControl_3(1100); //油门拉最低
//            PwmControl_5(1100); //自稳模式
//            Control_Serial= false;
//            //是否需要置flag 主函数上锁？
//            UARTprintf("EMER\n");
//        }
        //择地降落
        else if(ReciveData_UART1[0]=='S'&&ReciveData_UART1[1]=='S'&&ReciveData_UART1[2]=='S'&&ReciveData_UART1[3]=='S'&&Mode_Flag==3)
        {
//            land_counter=60;
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
            if(land_carcount_sta3!=2)
                land_carcount_sta3=1;
//            UARTprintf("USTA\n");
        }
//        else if(ReciveData_UART1[0]=='F'&&ReciveData_UART1[1]=='0'&&ReciveData_UART1[2]=='0'&&ReciveData_UART1[3]=='5')
//        {
//            PwmControl_2((int)(channel_val_MID+40));
//            UARTprintf("f\n");
//        }
//        else if (ReciveData_UART1[0]=='B'&&ReciveData_UART1[1]=='0'&&ReciveData_UART1[2]=='0'&&ReciveData_UART1[3]=='5')
//        {
//            PwmControl_2((int)(channel_val_MID-40));
//            UARTprintf("b\n");
//        }
//        else if (ReciveData_UART1[0]=='L'&&ReciveData_UART1[1]=='0'&&ReciveData_UART1[2]=='0'&&ReciveData_UART1[3]=='5')
//        {
//            PwmControl_1((int)(channel_val_MID+40));
//            UARTprintf("l\n");
//        }
//        else if (ReciveData_UART1[0]=='R'&&ReciveData_UART1[1]=='0'&&ReciveData_UART1[2]=='0'&&ReciveData_UART1[3]=='5')
//        {
//            PwmControl_1((int)(channel_val_MID-40));
//            UARTprintf("r\n");
//        }
//        else if (ReciveData_UART1[0]=='M'&&ReciveData_UART1[1]=='0'&&ReciveData_UART1[2]=='0'&&ReciveData_UART1[3]=='5')
//        {
//            PwmControl_1((int)(channel_val_MID));
//            PwmControl_2((int)(channel_val_MID));
//            UARTprintf("m\n");
//        }

////参数校准
    else if (ReciveData_UART1[0]=='P'&&ReciveData_UART1[1]=='A'&&ReciveData_UART1[2]=='R'&&ReciveData_UART1[3]=='A')
    {
        Para_Report=1;
    }
    else if (ReciveData_UART1[0]=='X'&&ReciveData_UART1[1]=='A')
    {
        PID_X.Kp=PID_X.Kp + 0.05;
        Para_Compared=(int)(PID_X.Kp*1000);
//        if(Para_Compared<1000)
//            UARTprintf("XP0%d",Para_Compared);
//        else
//            UARTprintf("XP%d",Para_Compared);
        UARTprintf("XP%d\n",Para_Compared);
    }
    else if (ReciveData_UART1[0]=='X'&&ReciveData_UART1[1]=='B')
    {
        PID_X.Kp=PID_X.Kp - 0.05;
        Para_Compared=(int)(PID_X.Kp*1000);
//        if(Para_Compared<1000)
//            UARTprintf("XP0%d",Para_Compared);
//        else
//            UARTprintf("XP%d",Para_Compared);
        UARTprintf("XP%d\n",Para_Compared);
    }
    else if (ReciveData_UART1[0]=='X'&&ReciveData_UART1[1]=='C')
    {
        PID_X.Kd=PID_X.Kd + 0.005f;
        Para_Compared=(int)(PID_X.Kd*1000);
//        if(Para_Compared<1000)
//        {
//            if(Para_Compared<100)
//            UARTprintf("XD00%d",Para_Compared);
//            else
//            UARTprintf("XD0%d",Para_Compared);
//        }
//        else
//        {
//            UARTprintf("XD%d",Para_Compared);
//        }
        UARTprintf("XD%d\n",Para_Compared);
    }
    else if (ReciveData_UART1[0]=='X'&&ReciveData_UART1[1]=='D')
    {
        PID_X.Kd=PID_X.Kd - 0.005f;
        Para_Compared=(int)(PID_X.Kd*1000);
//        if(Para_Compared<1000)
//        {
//            if(Para_Compared<100)
//            UARTprintf("XD00%d",Para_Compared);
//            else
//            UARTprintf("XD0%d",Para_Compared);
//        }
//        else
//        {
//            UARTprintf("XD%d",Para_Compared);
//        }
        UARTprintf("XD%d\n",Para_Compared);
    }
    else if (ReciveData_UART1[0]=='Y'&&ReciveData_UART1[1]=='A')
    {
        PID_Y.Kp=PID_Y.Kp + 0.05;
        Para_Compared=(int)(PID_Y.Kp*1000);
//        if(Para_Compared<1000)
//            UARTprintf("YP0%d",Para_Compared);
//        else
//            UARTprintf("YP%d",Para_Compared);
        UARTprintf("YP%d\n",Para_Compared);
    }
    else if (ReciveData_UART1[0]=='Y'&&ReciveData_UART1[1]=='B')
    {
        PID_Y.Kp=PID_Y.Kp - 0.05;
        Para_Compared=(int)(PID_Y.Kp*1000);
//        if(Para_Compared<1000)
//            UARTprintf("YP0%d",Para_Compared);
//        else
//            UARTprintf("YP%d",Para_Compared);
        UARTprintf("YP%d\n",Para_Compared);
    }
    else if (ReciveData_UART1[0]=='Y'&&ReciveData_UART1[1]=='C')
    {
        PID_Y.Kd=PID_Y.Kd + 0.005f;
        Para_Compared=(int)(PID_Y.Kd*1000);
//        if(Para_Compared<1000)
//        {
//            if(Para_Compared<100)
//            UARTprintf("YD00%d",Para_Compared);
//            else
//            UARTprintf("YD0%d",Para_Compared);
//        }
//        else
//        {
//            UARTprintf("YD%d",Para_Compared);
//        }
        UARTprintf("YD%d\n",Para_Compared);
    }
    else if (ReciveData_UART1[0]=='Y'&&ReciveData_UART1[1]=='D')
    {
        PID_Y.Kd=PID_Y.Kd - 0.005f;
        Para_Compared=(int)(PID_Y.Kd*1000);
//        if(Para_Compared<1000)
//        {
//            if(Para_Compared<100)
//            UARTprintf("YD00%d",Para_Compared);
//            else
//            UARTprintf("YD0%d",Para_Compared);
//        }
//        else
//        {
//            UARTprintf("YD%d",Para_Compared);
//        }
        UARTprintf("YD%d\n",Para_Compared);
    }
    else if (ReciveData_UART1[0]=='K'&&ReciveData_UART1[1]=='A')
    {
        Chane1_Stable=Chane1_Stable + 1;
        Para_Compared=(int)(Chane1_Stable);
        UARTprintf("ROLL%d\n",Para_Compared);
    }
    else if (ReciveData_UART1[0]=='K'&&ReciveData_UART1[1]=='B')
    {
        Chane1_Stable=Chane1_Stable - 1;
        Para_Compared=(int)(Chane1_Stable);
        UARTprintf("ROLL%d\n",Para_Compared);
    }
    else if (ReciveData_UART1[0]=='K'&&ReciveData_UART1[1]=='C')
    {
        Chane2_Stable=Chane2_Stable + 1;
        Para_Compared=(int)(Chane2_Stable);
        UARTprintf("PITCH%d\n",Para_Compared);
    }
    else if (ReciveData_UART1[0]=='K'&&ReciveData_UART1[1]=='D')
    {
        Chane2_Stable=Chane2_Stable - 1;
        Para_Compared=(int)(Chane2_Stable);
        UARTprintf("PITCH%d\n",Para_Compared);
    }
    else if (ReciveData_UART1[0]=='K'&&ReciveData_UART1[1]=='E')
    {
        Chane4_Stable=Chane4_Stable + 1;
        Para_Compared=(int)(Chane4_Stable);
        UARTprintf("YAW%d\n",Para_Compared);
    }
    else if (ReciveData_UART1[0]=='K'&&ReciveData_UART1[1]=='F')
    {
        Chane4_Stable=Chane4_Stable - 1;
        Para_Compared=(int)(Chane4_Stable);
        UARTprintf("YAW%d\n",Para_Compared);
    }
     ReciveData_UART1[0]=' ';
//    if(ReciveData_UART1[0]=='F'||ReciveData_UART1[0]=='B'||ReciveData_UART1[0]=='R'||ReciveData_UART1[0]=='L')
//        MotorOrderDisplacement = (ReciveData_UART1[1]-48)*100+(ReciveData_UART1[2]-48)*10+(ReciveData_UART1[3]-48);
}


//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UART1Send(uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        UARTCharPutNonBlocking(UART1_BASE, *pui8Buffer++);
    }
}



//*****************************************************************************
//
// uart1 init
//
//*****************************************************************************
void Uart1Iint(void)
{
    //
     // Enable the peripherals used by this example.
     //
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
     SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

     //
     // Enable processor interrupts.
     //
     IntMasterEnable();

     //
     // Set GPIO B0 and B1 as UART pins.
     //
     GPIOPinConfigure(GPIO_PB0_U1RX);
     GPIOPinConfigure(GPIO_PB1_U1TX);
     GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

     //
     // Configure the UART for 115,200, 8-N-1 operation.
     //
     UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
                             (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                              UART_CONFIG_PAR_NONE));
     //
     // Enable the UART interrupt.
     //
     IntEnable(INT_UART1);
     UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

     //
     // Prompt for text to be entered.
     //
     UART1Send((uint8_t *)"\n UART1 Is OK!!\n\n ", 17);
}

/*---------------------------------------------------------------Beautiful Line---------------------------------------------------------*/

//UART3
//uint8_t ReciveData_UART3[16];
//uint8_t ReciveData_i_UART3 = 0;
uint8_t temp;
uint8_t count=0;
extern uint16_t Real_XCoordinate,Real_YCoordinate;

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UART3IntHandler(void)
{
       UARTIntClear(UART3_BASE,UART_INT_RX);
       count++;
       temp= UARTCharGetNonBlocking(UART3_BASE);
       if(temp==0x00)
       {
           count=0;
       }
       if(count==1)
       {
           Real_XCoordinate= temp;
//           GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
//          //
//          // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
//          //
//          SysCtlDelay(SysCtlClockGet() / (1000 * 3));
//          //
//          // Turn off the LED
//          //
//          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
//          UART0Send(&temp, 1);
       }
       if(count==2)
       {
           Real_YCoordinate= temp;
//           GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
//          //
//          // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
//          //
//          SysCtlDelay(SysCtlClockGet() / (1000 * 3));
//          //
//          // Turn off the LED
//          //
//          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
//          UART0Send(&temp, 1);
       }

//       UARTprintf("X_r%d,Y_r%d");
//    uint32_t ui32Status;
//    ui32Status = UARTIntStatus(UART3_BASE, true);
//    UARTIntClear(UART3_BASE, ui32Status);
//    ReciveData_i_UART3 = 0;
//    while(UARTCharsAvail(UART3_BASE))
//    {
//        // UARTCharPutNonBlocking(UART1_BASE,
//        // UARTCharGetNonBlocking(UART1_BASE));
//        ReciveData_UART3[ReciveData_i_UART3] = UARTCharGetNonBlocking(UART3_BASE);
//        ReciveData_i_UART3++;
//        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
//        SysCtlDelay(SysCtlClockGet() / (1000 * 3));
//        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
//    }
//
// //   UARTSend()
//    UART3Send("Rec: ",4);
//    UART3Send(ReciveData_UART3, ReciveData_i_UART3);
////参数获取
//    if(ReciveData_UART3[0]=='X'&&ReciveData_UART3[3]=='Y')
//    {
////        Real_XCoordinate = (ReciveData_UART3[1]-48)*100+(ReciveData_UART3[2]-48)*10+(ReciveData_UART3[3]-48);
////        Real_YCoordinate = (ReciveData_UART3[4]-48)*100+(ReciveData_UART3[5]-48)*10+(ReciveData_UART3[6]-48);
//        Real_XCoordinate = ReciveData_UART3[1];
//        Real_YCoordinate = ReciveData_UART3[2];
//    }
}


//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UART3Send(uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        UARTCharPutNonBlocking(UART3_BASE, *pui8Buffer++);
    }
}



//*****************************************************************************
//
// uart1 init
//
//*****************************************************************************
void UART3Iint(void)
{
    //
     // Enable the peripherals used by this example.
     //
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
     SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);

     // Enable processor interrupts.
     //
     IntMasterEnable();

     //
     // Set GPIO B0 and B1 as UART pins.
     //
     GPIOPinConfigure(GPIO_PC6_U3RX);
     GPIOPinConfigure(GPIO_PC7_U3TX);
     GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);

     //
     // Configure the UART for 115,200, 8-N-1 operation.
     //
     UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet(), 115200,
                             (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                              UART_CONFIG_PAR_NONE));
     //
     // Enable the UART interrupt.
     //
     IntEnable(INT_UART3);
     UARTIntEnable(UART3_BASE, UART_INT_RX | UART_INT_RT);

  //       SysCtlDelay(SysCtlClockGet()/(2*1000));
     //
     // Prompt for text to be entered.
     //
     UART3Send((uint8_t *)"\n UART3 Is OK!!\n\n ", 17);
}


