#ifndef __PPM_ENCODER__
#define __PPM_ENCODER__

#include <stdint.h>

#define PPM_ENCODER_CHANNEL_NUM 8
#define PPM_ENCODER_DEFFAULT_CH_VAL 1000


typedef struct
{
    uint16_t ch_val[PPM_ENCODER_CHANNEL_NUM];
    //user needn't set
    uint16_t idle_val;
}ppm_data_t;

extern void ppm_encoder_init(void);
extern void ppm_encoder_set_data(ppm_data_t *ppm_data);

#endif

