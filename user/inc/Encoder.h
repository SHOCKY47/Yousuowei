#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "zf_common_headfile.h"
#include "headfile.h"

#define ENCODER_QUADDEC_2   (TIM3_ENCOEDER)
#define ENCODER_QUADDEC_A_2 (TIM3_ENCOEDER_CH1_B4)
#define ENCODER_QUADDEC_B_2 (TIM3_ENCOEDER_CH2_B5)

#define ENCODER_QUADDEC_1   (TIM4_ENCOEDER)
#define ENCODER_QUADDEC_A_1 (TIM4_ENCOEDER_CH1_B6)
#define ENCODER_QUADDEC_B_1 (TIM4_ENCOEDER_CH2_B7)

extern int16 encoder_1;
extern int16 encoder_2;

extern int64 LeftMotor_TotalEncoder;
extern int64 RightMotor_TotalEncoder;
extern const int64 EncoderPerMeter;

extern float V_L;
extern float V_R;

void Get_Speed(void);

void EncoderCount(void);
int64 GetEncoder(void);

#endif