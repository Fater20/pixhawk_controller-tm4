#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "ppm_encoder.h"
#include <string.h>

////unit: us
//#define PPM_ENCODER_RESOLUTION 1
//unit: us
#define PPM_ENCODER_TOTAL_CH_VAL 20000
#define PPM_ENCODER_NEG_CH_VAL 500
//calculate arr value
#define CAL_ARR(ch_val)  (ch_val * ppm_period_us)
//gpio configure
#define PPM_GPIO_CLK   SYSCTL_PERIPH_GPIOB     //GPIOB ����
#define PPM_GPIO_PORT  GPIO_PORTB_BASE        //GPIOB base
#define PPM_GPIO_PIN   GPIO_PIN_5             //pin5
//timer configure
#define PPM_TIMER_BASE    TIMER0_BASE          //timer0

typedef enum
{
    PPM_STATE_CH_NEG,
    PPM_STATE_CH_POS,
    PPM_STATE_IDLE_NEG,
    PPM_STATE_IDLE_POS,
    PPM_STATE_STOP,
}ppm_state_e;

//gpio cache
//static bool ppm_gpio_next_val;

//timer period per us, this will be caclulated when initialization
static uint32_t ppm_period_us = 1;

//ppm data buffer,  used to save user setting
#define PPM_DATA_BUF_NUM 2
static ppm_data_t ppm_data_buf[PPM_DATA_BUF_NUM];
static uint32_t ppm_data_buf_index;

//ppm data shadow buffer,  used to encode ppm 
static ppm_data_t ppm_data_shadow;
static ppm_state_e ppm_shadow_state;
static uint32_t ppm_shadow_ch_idx;

static bool first_time = true;

static void ppm_gpio_init(void);
static void ppm_gpio_negative(void);
static void ppm_gpio_positive(void);
static void ppm_gpio_set_next(bool is_pos);
static void ppm_timer_config(void);
static void ppm_data_calculate_idle(ppm_data_t *ppm_data);
static void ppm_data_set_default(ppm_data_t *ppm_data);
static void ppm_data_buf_init(void);
static void ppm_data_shadow_init(void);
static void ppm_data_shadow_update(void);
static bool ppm_data_shadow_one_step(void);

void ppm_encoder_init(void)
{
    ppm_data_buf_init();
    ppm_data_shadow_init();  //�� �ṹ��ppm_data_buf+i ��ֵ���Ƶ��ṹ��ppm_data_shadow ���� ppm_shadow_state��ֵPPM_STATE_CH_NEG
    ppm_gpio_init();  //ʹ��GPIOB ,����GPIOB pin5Ϊ��� �����ø�pin5
    //delay_ms(500);
    ppm_timer_config();
}

static void ppm_gpio_init(void)
{
	ROM_SysCtlPeripheralEnable(PPM_GPIO_CLK);
	ROM_GPIOPinTypeGPIOOutput(PPM_GPIO_PORT, PPM_GPIO_PIN);
//    ppm_gpio_next_val = true;
    ppm_gpio_set_next(true);
}

static void ppm_gpio_negative(void)
{
	GPIOPinWrite(PPM_GPIO_PORT, PPM_GPIO_PIN, 0x00);
}

static void ppm_gpio_positive(void)
{
	GPIOPinWrite(PPM_GPIO_PORT, PPM_GPIO_PIN, 0xff);
}

static void ppm_gpio_set_next(bool is_pos)
{
//    if(ppm_gpio_next_val)
//    {
//        ppm_gpio_positive();
//    }
//    else
//    {
//        ppm_gpio_negative();
//    }
//    ppm_gpio_next_val = is_pos;

	if(is_pos)
	{
		ppm_gpio_positive();   //B5 �ø�
	}
	else
	{
		ppm_gpio_negative();   //B5��0
	}
}

static void ppm_timer_config(void)   //��ppm���䶨ʱ��ģ��/������� timer0 �� timerA
{
	ppm_period_us = ROM_SysCtlClockGet() / 1000000;  //8

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

	//
	// Configure the two 32-bit periodic timers. //�����Լ�ʱ
	//
	ROM_TimerConfigure(PPM_TIMER_BASE, TIMER_CFG_PERIODIC);
	ROM_TimerLoadSet(PPM_TIMER_BASE, TIMER_A, CAL_ARR(PPM_ENCODER_TOTAL_CH_VAL));   //20000*8 =160 000

	//
	// Setup the interrupts for the timer timeouts.
	//
	ROM_IntEnable(INT_TIMER0A);
	ROM_TimerIntEnable(PPM_TIMER_BASE, TIMER_TIMA_TIMEOUT);

	//
	// Enable the timers.
	//
	ROM_TimerEnable(PPM_TIMER_BASE, TIMER_A);
    ROM_IntPrioritySet(INT_TIMER0A,0x00);
}

//*****************************************************************************
//
// The interrupt handler for the first timer interrupt.
//
//*****************************************************************************
void Timer0IntHandler(void)
{
	ROM_TimerIntClear(PPM_TIMER_BASE, TIMER_TIMA_TIMEOUT);

    if(first_time)
    {
        first_time = false;
        return;
    }

    if(ppm_data_shadow_one_step())
    {
        ppm_data_shadow_update();
    }
}


void ppm_encoder_set_data(ppm_data_t *ppm_data)
{
    memcpy(ppm_data_buf + ppm_data_buf_index, ppm_data, sizeof(ppm_data_t));
    ppm_data_calculate_idle(ppm_data_buf + ppm_data_buf_index);
    ppm_data_buf_index++;
    if(ppm_data_buf_index >= PPM_DATA_BUF_NUM)
    {
        ppm_data_buf_index = 0;
    }
}

static void ppm_data_calculate_idle(ppm_data_t *ppm_data)   //����  ����ʱ�� idle_val ��
{
    uint32_t j;
    uint16_t idle_val = PPM_ENCODER_TOTAL_CH_VAL;  // ��ʼ����ֵΪ20000
    for(j = 0; j < PPM_ENCODER_CHANNEL_NUM; j++)
    {
        idle_val -= PPM_ENCODER_NEG_CH_VAL;  // idle_val = idle_val - PPM_ENCODER_NEG_CH_VAL ���� 19500��18500����18000��17000�� �� ����
        idle_val -= ppm_data->ch_val[j];    //  ��8�κ� idle_val=8000
    }
    idle_val -= PPM_ENCODER_NEG_CH_VAL;   // ����ΪΪ7500us
    ppm_data->idle_val = idle_val;
}


static void ppm_data_set_default(ppm_data_t *ppm_data)   //��ch_val��ֵ��8��ppm���ĸߵ�ƽʱ�䣩 �� �Լ�8��ppm���Ŀ���ʱ��
{
    uint32_t i;
    for(i = 0; i < PPM_ENCODER_CHANNEL_NUM; i++)
    {
        ppm_data->ch_val[i] = PPM_ENCODER_DEFFAULT_CH_VAL;  //��ch_val[8]��0~7 ��ֵ1000��PPM_ENCODER_DEFFAULT_CH_VAL��
    }
    ppm_data_calculate_idle(ppm_data);
}

static void ppm_data_buf_init(void)
{
    uint32_t i;
    ppm_data_buf_index = 0;
    for(i = 0; i < PPM_DATA_BUF_NUM; i++)
    {
        ppm_data_set_default(ppm_data_buf + i);//����ʱ��ppm_data_buf[2] ����0��1��Ԫ�ض�Ϊ�ṹ������ ppm_data_t
    }
}

static void ppm_data_shadow_init(void)
{
    //ppm_data_set_default(&ppm_data_shadow);
    //ppm_shadow_state = PPM_STATE_CH_NEG;
    //ppm_shadow_ch_idx = 0;
    ppm_data_shadow_update();
}

static void ppm_data_shadow_update(void)
{
    uint32_t i;
    for(i = 0; i < PPM_DATA_BUF_NUM; i++)
    {
        if(i != ppm_data_buf_index)
        {   
            memcpy(&ppm_data_shadow, ppm_data_buf + i, sizeof(ppm_data_t));
            break;
        }
    }                    //��ppm_data_buf + i ��ַ�����ݿ���ppm_data_shadow

    if(i == PPM_DATA_BUF_NUM)
    {
        //impossible 
        ppm_data_set_default(&ppm_data_shadow);
    }
    ppm_shadow_state = PPM_STATE_CH_NEG; //0
    ppm_shadow_ch_idx = 0;
}


static void handle_negative(void)
{
    //ppm_gpio_negative();
    ppm_gpio_set_next(false);
    ROM_TimerLoadSet(PPM_TIMER_BASE, TIMER_A, CAL_ARR(PPM_ENCODER_NEG_CH_VAL));  //500*1 = 0.5 ms
    ppm_shadow_state = PPM_STATE_CH_POS;

}

static void handle_positive(void)
{
    //ppm_gpio_positive();
    ppm_gpio_set_next(true);                    //ppm_shadow_ch_idx�ڳ�ʼ����ppm_data_shadow_update�����и�ֵΪ0������
    ROM_TimerLoadSet(PPM_TIMER_BASE, TIMER_A, CAL_ARR(ppm_data_shadow.ch_val[ppm_shadow_ch_idx])); //����ȡppm_data_shadow.ch_val[ppm_shadow_ch_idx]Ҳ����ȡ0~7��ͨ����ch_val
    ppm_shadow_ch_idx++;  //�ı���ppm_shadow_ch_idx��ֵ��ʹ���ɼ�1
    if(ppm_shadow_ch_idx >= PPM_ENCODER_CHANNEL_NUM)
    {
        ppm_shadow_state = PPM_STATE_IDLE_NEG;
    }
    else
    {
        ppm_shadow_state = PPM_STATE_CH_NEG;
    }

}

static void handle_idle_neg(void)
{
    //ppm_gpio_negative();
    ppm_gpio_set_next(false);
    ROM_TimerLoadSet(PPM_TIMER_BASE, TIMER_A, CAL_ARR(PPM_ENCODER_NEG_CH_VAL));
    ppm_shadow_state = PPM_STATE_IDLE_POS;
}

static void handle_idle_pos(void)
{
    //ppm_gpio_positive();
    ppm_gpio_set_next(true);
    ROM_TimerLoadSet(PPM_TIMER_BASE, TIMER_A, CAL_ARR(ppm_data_shadow.idle_val));
}


static bool ppm_data_shadow_one_step(void)
{
    bool should_update = false;
    
    switch(ppm_shadow_state)  //��ʼʱ ppm_shadow_state �� ppm_data_shadow_update�����и�ֵΪ0
    {
        case PPM_STATE_CH_NEG:
            handle_negative();  //ʹB5�͵�ƽ����0.5ms
            break;
        case PPM_STATE_CH_POS:
            handle_positive();  // ִ��ppm_shadow_ch_idx++, ppm_shadow_ch_idx��0��ʼ�ӵ�7��ʹB5�ߵ�ƽ������ppm_data_shadow.ch_val[ppm_shadow_ch_idx] us
            break;
        case PPM_STATE_IDLE_NEG:
            handle_idle_neg();  // ʹB5�͵�ƽ����0.5ms
            break;
        case PPM_STATE_IDLE_POS:
            handle_idle_pos();  // ʹB5�ߵ�ƽ������ppm_data_shadow.idle_val us
            //ppm_shadow_state = PPM_STATE_STOP;
            should_update = true;
        default:
            //
            break;
    }

    return should_update;
}



