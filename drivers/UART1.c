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
//*****************************************************************************

 int dir_x;//横坐标
 int dir_y;//纵坐标
 int ST_FLAG=0;//迫降标志位
 int FL_FLAG=0;//起飞标志位
 int PA_FLAG=0;//停桨标志位

void ConfigureUART1 (void)
{
    //串口设置
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);


    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_1);

    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(1,921600, 16000000);


    UARTFIFOEnable(UART1_BASE);                                      //使能FIFO
    UARTFIFOLevelSet(UART1_BASE,UART_FIFO_TX4_8, UART_FIFO_RX2_8);   //FIFO 8字节深度

    //无线串口配置
  //GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7);
  //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_7, 0x0);
  //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_6, 0);
    //中断设置

    //串口接收中断
    UARTIntEnable(UART1_BASE, UART_INT_RT);
    UARTIntRegister(UART1_BASE, UART1IntHandler);
    IntEnable(INT_UART1);
    IntPrioritySet(INT_UART1,0xB0);


//    //无线串口
//
//
//    //
//    // Check if the peripheral access is enabled.
//    //
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
//    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
//        ;
//
//    //
//    // Enable the GPIO pin for the LED (PF 123).  Set the direction as output, and
//    // enable the GPIO pin for digital function.
//    //
//    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 );
////    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
////    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
////    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
//
//    //
//    // Turn off all led.
//    //
//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x0);
//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

}



void UART1IntHandler(void)
{
    //得到数据
    uint32_t ui32IntStatus;
    uint8_t  ui8RxBuffer[9];
    uint8_t  i = 0;
//    int* p=x;
//    int* q=y;
    ui32IntStatus = UARTIntStatus(UART1_BASE, true);
    UARTIntClear(UART1_BASE, ui32IntStatus);
    while(UARTCharsAvail(UART1_BASE)) {
        ui8RxBuffer[i++] =(uint8_t)UARTCharGetNonBlocking(UART1_BASE);
        if(i > 8) {
            break;
                  }
    }

    UARTDataDeal(ui8RxBuffer,&dir_x,&dir_y);

//   UARTCharPut(UART1_BASE,'0'+dir_x);
//   for (i=0;i<=10;i++);
//   UARTCharPut(UART1_BASE,'0'+dir_y);
 }

//发送数据
//void send_data(int distance)
//{
//    char* t;
//    t=sprintf("%dH",distance);
//    while(1)
//    {
//
//    }
//}



//串口数据处理
void UARTDataDeal(uint8_t *pui8Data,int*x,int*y){
    int count1=0;
    int count2=0;
    int i=1;
    int num_x[3];
    int num_y[3];
    int res=0;
    int j,k;



    //起飞
        if(pui8Data[0] == 'f'&&pui8Data[1]=='l')
        {
            FL_FLAG=1;
        }
    //停桨
        if(pui8Data[0] == 'p'&&pui8Data[1]=='a')
                {
                    PA_FLAG=1;
                }

    //迫降
    if(pui8Data[0] == 's'&&pui8Data[1]=='t')
    {
            ST_FLAG=1;
    }

    //坐标
    if(pui8Data[0] == '('){
   while(pui8Data[i]>=48&&pui8Data[i]<=57)
     {
        num_x[i-1]=pui8Data[i]-48;
        count1++;
        i++;
     }
   for(k=0;k<count1;k++)
   {
     for(j=0;j<(count1-k-1);j++)
     {
        num_x[k]*=10;
     }
        res+=num_x[k];
        *x=res;
     }
   }

    res=0;
  if(pui8Data[i]==','){
      i++;
    while(pui8Data[i]>=48&&pui8Data[i]<=57)
        {
           num_y[i-count1-2]=pui8Data[i]-48;
           count2++;
           i++;
        }
      for(k=0;k<count2;k++)
      {
        for(j=0;j<(count2-k-1);j++)
        {
           num_y[k]*=10;
        }
           res+=num_y[k];
//           UARTCharPut(UART1_BASE,'0'+num_y[0]);
           *y=res;
      }
  //    UARTCharPutNonBlocking(UART1_BASE,x+'0');


}

////  UARTCharPut(UART1_BASE,'0'+num_y[0]);
//  UARTCharPut(UART1_BASE,'0'+num_y[1]);
}

