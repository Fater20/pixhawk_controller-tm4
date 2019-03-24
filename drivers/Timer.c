/*
 * Timer.c
 *
 *  Created on: 2019Äê3ÔÂ11ÈÕ
 *      Author: dell
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "UART1.h"
#include "mavlink_recieve.h"
//#include "sonic.h"

bool AltHold_flag=true;

int Ns;

 void Timer1BIntHandler(void)
 {
     int dis[3];
     int i,Distance;
     uint32_t ui32Status;
     ui32Status = TimerIntStatus(TIMER1_BASE, true);
     TimerIntClear(TIMER1_BASE, ui32Status);
     Distance=int_Distance;
     dis[0]=Distance/100;
     dis[1]=Distance/10%10;
     dis[2]=Distance%10;
     for(i=0;i<=2;i++)
     {
         UARTCharPut(UART1_BASE,'0'+dis[i]);
     }
     UARTCharPut(UART1_BASE,'H');

 }
 void ConfigureTimer1 (void)
 {
     SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
     TimerClockSourceSet(TIMER1_BASE, TIMER_CLOCK_SYSTEM);
     TimerPrescaleSet(TIMER1_BASE, TIMER_B, 160 - 1);
     TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PERIODIC);
     TimerLoadSet(TIMER1_BASE, TIMER_B, ((SysCtlClockGet() / (TimerPrescaleGet(TIMER1_BASE, TIMER_B) + 1)) / 100) - 1);
     TimerIntEnable(TIMER1_BASE, TIMER_TIMB_TIMEOUT);
     IntEnable(INT_TIMER1B);
//     TimerIntRegister(TIMER1_BASE, TIMER_B, Timer1BIntHandler);
     TimerEnable(TIMER1_BASE, TIMER_B);
     IntPrioritySet(INT_TIMER1B,0xD0);
 }


 void Timer3BIntHandler(void)
 {
	 static int i=0;
     uint32_t ui32Status;
     ui32Status = TimerIntStatus(TIMER3_BASE, true);
     TimerIntClear(TIMER3_BASE, ui32Status);

     i++;
     if(i>=Ns)
     {
    	 AltHold_flag=false;
     	 TimerDisable(TIMER3_BASE, TIMER_B);
     	 i=0;

     }


 }
 void ConfigureTimer3 (void)
 {
     SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
     TimerClockSourceSet(TIMER3_BASE, TIMER_CLOCK_SYSTEM);
     TimerPrescaleSet(TIMER3_BASE, TIMER_B, 255);
     TimerConfigure(TIMER3_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PERIODIC);
     TimerLoadSet(TIMER3_BASE, TIMER_B, ((SysCtlClockGet() / (TimerPrescaleGet(TIMER3_BASE, TIMER_B) + 1)) / 5) - 1);
     TimerIntEnable(TIMER3_BASE, TIMER_TIMB_TIMEOUT);
     IntEnable(INT_TIMER3B);
//     TimerIntRegister(TIMER3_BASE, TIMER_B, Timer3BIntHandler);
//     TimerEnable(TIMER3_BASE, TIMER_B);
     IntPrioritySet(INT_TIMER3B,0xC0);
 }

void delayNs(int n)
{
	TimerEnable(TIMER3_BASE, TIMER_B);
	Ns=n;
}

