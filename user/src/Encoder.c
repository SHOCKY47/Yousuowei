#include "Encoder.h"

int16 encoder_1 = 0;
int16 encoder_2 = 0;

/***测距离***/
const int64 EncoderPerMeter   = 6480 * 2;
int64 LeftMotor_TotalEncoder  = 0;
int64 RightMotor_TotalEncoder = 0;

float V_L = 0;
float V_R = 0;

void Get_Speed(void)
{

    encoder_1 = encoder_get_count(ENCODER_QUADDEC_1); // 获取编码器计数

    encoder_clear_count(ENCODER_QUADDEC_1); // 清空编码器计数

    encoder_2 = -encoder_get_count(ENCODER_QUADDEC_2); // 获取编码器计数
    encoder_clear_count(ENCODER_QUADDEC_2);            // 清空编码器计数

    // V_R = encoder_data_quaddec_1 * 20.4 / (2355.2 * 0.008); // 右轮速度

    // V_L = encoder_data_quaddec_2 * 20.4 / (2355.2 * 0.008); // 左轮速度
}

// 用编码器计算里程
void EncoderCount(void)
{
    // if (g_TrackType.m_u8YjunctionFlag == YJUNCTION_NONE && g_TrackType.m_u8GarageFlag == GARAGE_NONE && (g_TrackType.m_u8RightRoundaboutFlag == ROUNDABOUT_NONE || g_TrackType.m_u8RightRoundaboutFlag == ROUNDABOUT_BEGIN) && (g_TrackType.m_u8LeftRoundaboutFlag == ROUNDABOUT_NONE || g_TrackType.m_u8LeftRoundaboutFlag == ROUNDABOUT_BEGIN)) {
    //     LeftMotor_TotalEncoder = RightMotor_TotalEncoder = 0;
    // } else {
    LeftMotor_TotalEncoder += encoder_1;
    RightMotor_TotalEncoder += encoder_2;
    // }
}

// 获取测里程
int64 GetEncoder(void)
{
    return (LeftMotor_TotalEncoder + RightMotor_TotalEncoder) / 2;
}