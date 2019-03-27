//*****************************************************************************
//
// timers.c - Timers example.
//
// Copyright (c) 2012-2015 Texas Instruments Incorporated.  All rights reserved.
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
// This is part of revision 2.1.2.111 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "drivers/ppm_encoder.h"
#include "drivers/mavlink_recieve.h"
#include "drivers/UART1.h"
#include "drivers/Control Command.h"
#include "drivers/PID.h"
#include "drivers/Timer.h"
//#include "drivers/sonic.h"
//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Timer (timers)</h1>
//!
//! This example application demonstrates the use of the timers to generate
//! periodic interrupts.  One timer is set up to interrupt once per second and
//! the other to interrupt twice per second; each interrupt handler will toggle
//! its own indicator on the display.
//
//*****************************************************************************



//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//#define PPM_GPIO_CLK   SYSCTL_PERIPH_GPIOB
//#define PPM_GPIO_PORT  GPIO_PORTB_BASE
//#define PPM_GPIO_PIN   GPIO_PIN_5

void ConfigureUART(void)	//��ʼ��UART0 ������921600
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
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
    UARTStdioConfig(0, 921600, 16000000);
}

//*****************************************************************************
//
// This example application demonstrates the use of the timers to generate
// periodic interrupts.
//
//*****************************************************************************
int
main(void)
{
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //80M
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);


    ppm_encoder_init();	//ppm��ʼ��
    ConfigureUART();	//UART0��ʼ��
    ConfigureUART1 ();	//UART1��ʼ��
    ConfigureTimer1 ();	//��ʼ��Timer1�����ڴ��ڶ�ʱ����
    ConfigureTimer3 ();	//��ʼ��Timer3�����ڶ�ʱ�ж�
//    WTimer1BConfigure();//��ʼ��WTimer1B�����ڳ�����
    Mavlink_DateInit();

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();


//�ȴ��ɿس�ʼ��
    Flight_Initial();
 	ROM_SysCtlDelay((SysCtlClockGet()/3000)*5000);


//�ȴ�����ź�
 	while(FL_FLAG==0)
 		ROM_SysCtlDelay((SysCtlClockGet()/3000)*10);


//�ɿؽ���
 	Flight_Armed();
 	ROM_SysCtlDelay((SysCtlClockGet()/3000)*3000);
//
// 	AltHold_Rise_ready();
//
// 	AltHold_Maintain();
// 	ROM_SysCtlDelay((SysCtlClockGet()/3000)*3000);

//����1

 	dir_Distance=90;	//�趨�µ�Ŀ��߶�106cm
 	AltHold_Rise1();
 	while(int_Distance<dir_Distance)
 	{

 		ROM_SysCtlDelay((SysCtlClockGet()/3000)*10);

 	}

//����2

 	dir_Distance=130;	//�趨�µ�Ŀ��߶�106cm
	AltHold_Rise2();
	while(int_Distance<dir_Distance)
	{

		ROM_SysCtlDelay((SysCtlClockGet()/3000)*10);

	}


// 	dir_Distance=120;	//�趨�µ�Ŀ��߶�120cm
//	while(int_Distance<dir_Distance)
//	{
//
//		AltHold_Rise_Control();
//		ROM_SysCtlDelay((SysCtlClockGet()/3000)*10);
//
//	}


//�߶ȱ���

	dir_Distance=126;	//�趨�µ�Ŀ��߶�106cm

 	AltHold_flag=true;
 	while(AltHold_flag==true)
 	{

 		Position_Control();

 		if((int_Distance>(dir_Distance-3))&&(int_Distance<(dir_Distance+3))&&(Ns!=1000))
 			delayNs(80);	//ʹ�ܶ�ʱ���жϣ���ʱ(0.2*80)s

 		ROM_SysCtlDelay((SysCtlClockGet()/3000)*10);

 		if(ST_FLAG==1)
 		{
 			break;
 		}

 		if(PA_FLAG==1)
 		{
 			STOP();
 			goto next;
 		}

 	}


//Ѳ��

//	dir_Distance=156;	//�趨�µ�Ŀ��߶�156cm
//
// 	AltHold_flag=true;
// 	while(AltHold_flag==true)
// 	{
//
// 		Position_Control();
//
// 		ROM_SysCtlDelay((SysCtlClockGet()/3000)*10);
//
// 		AltHold_Maintain();
// 		ROM_SysCtlDelay((SysCtlClockGet()/3000)*200);
//
// 	}


//����

 	dir_Distance=60;	//�趨�µ�Ŀ��߶�10cm
 	AltHold_Fall();
 	while(int_Distance>dir_Distance)
 	{
 		AltHold_Fall();
 		ROM_SysCtlDelay((SysCtlClockGet()/3000)*98);
 	}

 	AltHold_Fall_Continue();
 	ROM_SysCtlDelay((SysCtlClockGet()/3000)*3000);

 	next:
//�ɿ�����

 	Flight_Disarmed();


           while(1)
            {

        	   ROM_SysCtlDelay((SysCtlClockGet()/3000)*10);

            }
}


