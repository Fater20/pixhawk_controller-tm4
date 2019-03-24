/*
 * Control Command.c
 *
 *  Created on: 2019��3��9��
 *      Author: dell
 */

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "ppm_encoder.h"
#include "UART1.h"
#include "PID.h"
#include "mavlink_recieve.h"


ppm_data_t ppm_data;
int Thro_value=0;
int dir_Distance=0;

int center_x=80;
int center_y=60;

void set_ppm_channel(ppm_data_t *ppm_data, uint16_t low_ch_val, uint16_t hight_ch_val)
{
    int i;
    for(i = 0; i < PPM_ENCODER_CHANNEL_NUM / 2; i++)  //����ǰ4��ͨ��ֵΪ low_ch_val����
    {
        ppm_data->ch_val[i] = low_ch_val;
    }
    for(; i < PPM_ENCODER_CHANNEL_NUM; i++)     //���ú�4��ͨ��ֵΪ hight_ch_val����
    {
        ppm_data->ch_val[i] = hight_ch_val;
    }
}

void set_ppm_channel_5(ppm_data_t *ppm_data, uint16_t ppm_roll, uint16_t ppm_pitch, uint16_t ppm_power, uint16_t ppm_yaw,uint16_t ppm_station)
{
	ppm_data->ch_val[0] = ppm_roll;
	ppm_data->ch_val[1] = ppm_pitch;
	ppm_data->ch_val[2] = ppm_power;
	ppm_data->ch_val[3] = ppm_yaw;
	ppm_data->ch_val[4] = ppm_station;

}

//ͨ��У׼
void channel_calibration(void)
{
	ppm_data_t ppm_data;
	set_ppm_channel(&ppm_data, 1000,1500);
	ppm_encoder_set_data(&ppm_data);  //��ppm_data�����ݸ��Ƶ�ppm_data_buf + ppm_data_buf_index ��
	set_ppm_channel_5(&ppm_data,500,500,500,500,500);
	ppm_encoder_set_data(&ppm_data);
	SysCtlDelay((SysCtlClockGet()/3000)*10000);
	set_ppm_channel_5(&ppm_data,1500,1500,1500,1500,1500);
	ppm_encoder_set_data(&ppm_data);
	SysCtlDelay((SysCtlClockGet()/3000)*10000);

}

//�ȴ��ɿس�ʼ��
void Flight_Initial(void)
{
 	set_ppm_channel_5(&ppm_data,1000,1000,1000,1000,500);
 	ppm_encoder_set_data(&ppm_data);
}

//�ɿؽ���
void Flight_Armed(void)
{
 	set_ppm_channel_5(&ppm_data,1000,1000,500,1550,500);
 	ppm_encoder_set_data(&ppm_data);
}

//�ɿ�����
void Flight_Disarmed(void)
{
 	set_ppm_channel_5(&ppm_data,1000,1000,500,1000,750);
 	ppm_encoder_set_data(&ppm_data);
}

void AltHold_Rise_ready(void)
{
 	set_ppm_channel_5(&ppm_data,1000,1000,600,1000,750);
 	ppm_encoder_set_data(&ppm_data);
 	ROM_SysCtlDelay((SysCtlClockGet()/3000)*1000);
 	set_ppm_channel_5(&ppm_data,1000,1000,800,1000,750);
 	ppm_encoder_set_data(&ppm_data);
 	ROM_SysCtlDelay((SysCtlClockGet()/3000)*1000);
 	set_ppm_channel_5(&ppm_data,1000,1000,900,1000,750);
 	ppm_encoder_set_data(&ppm_data);
 	ROM_SysCtlDelay((SysCtlClockGet()/3000)*1000);
}
//�ɿض���ģʽ������
void AltHold_Rise1(void)
{
 	set_ppm_channel_5(&ppm_data,1000,1000,1100,1000,750);
 	ppm_encoder_set_data(&ppm_data);
}
void AltHold_Rise2(void)
{
 	set_ppm_channel_5(&ppm_data,1000,1000,1040,1000,750);
 	ppm_encoder_set_data(&ppm_data);
}

void AltHold_Rise_Control(void)
{
	int CH_1=1000,CH_2=1000;
	//x��λ����
		if((dir_x<center_x+1)&&(dir_x>center_x-1))
			CH_2=1000;
		else CH_2=1000+calcuIncre(dir_x-center_x);

	//y��λ����
		if((dir_y<center_y+1)&&(dir_y>center_y-1))
			CH_1=1000;
		else CH_1=1000+calcuIncre(center_y-dir_y);

//��������ֵ
 	set_ppm_channel_5(&ppm_data,CH_1,CH_2,1130,1000,750);
 	ppm_encoder_set_data(&ppm_data);
}

//�ɿض���ģʽ���½�
void AltHold_Fall(void)
{
	int CH_1=1000,CH_2=1000;
	//x��λ����
		if((dir_x<center_x+1)&&(dir_x>center_x-1))
			CH_2=1000;
		else CH_2=1000+calcuIncre(dir_x-center_x);

	//y��λ����
		if((dir_y<center_y+1)&&(dir_y>center_y-1))
			CH_1=1000;
		else CH_1=1000+calcuIncre(center_y-dir_y);

//��������ֵ
 	set_ppm_channel_5(&ppm_data,CH_1,CH_2,870,1000,750);
 	ppm_encoder_set_data(&ppm_data);
}

void AltHold_Fall_Continue(void)
{
 	set_ppm_channel_5(&ppm_data,1000,1000,880,1000,750);
 	ppm_encoder_set_data(&ppm_data);
}

//�ɿض���ģʽ�¸߶ȱ���
void AltHold_Maintain(void)
{
 	set_ppm_channel_5(&ppm_data,1000,1000,1000,1000,750);
 	ppm_encoder_set_data(&ppm_data);
}

//�ɿض���ģʽ�µ����߶���Ŀ��߶�
void AltHold_Control(void)
{
	if(int_Distance>=(dir_Distance+5))
		set_ppm_channel_5(&ppm_data,1000,1000,895,1000,750);
	if(int_Distance<=(dir_Distance-5))
		set_ppm_channel_5(&ppm_data,1000,1000,1105,1000,750);
 	ppm_encoder_set_data(&ppm_data);
}


void Position_Control(void)
{
	int CH_1=1000,CH_2=1000,CH_3=1000;
////�߶�����
//	if(int_Distance<120)
//	{
//	if(int_Distance>=(dir_Distance+10))
//		CH_3=895;
//	else if(int_Distance<=(dir_Distance-10))
//		CH_3=1105;
//
//	else CH_3=1000;
//	}
	//x��λ����
		if((dir_x<center_x+1)&&(dir_x>center_x-1))
			CH_2=1000;
		else CH_2=1000+calcuIncre(dir_x-center_x);

	//y��λ����
		if((dir_y<center_y+1)&&(dir_y>center_y-1))
			CH_1=1000;
		else CH_1=1000+calcuIncre(center_y-dir_y);

//��������ֵ
 	set_ppm_channel_5(&ppm_data,CH_1,CH_2,CH_3,1000,750);
 	ppm_encoder_set_data(&ppm_data);

}

void STOP(void)
{
 	set_ppm_channel_5(&ppm_data,1000,1000,500,1000,750);
 	ppm_encoder_set_data(&ppm_data);
}

