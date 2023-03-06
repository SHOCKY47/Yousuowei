/*
 * data.c
 *
 *  Created on: 13 Nov 2021
 *      Author: kare
 */
#include "data.h"

/*---------------------------------变量定义-----------------------------*/

TRACK_BORDER_INFO g_Border;
TRACK_TYPE_INFO g_TrackType; /*赛道类型*/
/***用于找线版本的数据***/
LINE_ERROR_INFO g_LineError; /*存放图像与边线相关的偏差信息*/

uint16 Protect_Frame;

uint32 tim1;
uint32 tim2;

int16 BlackBlock_Monitor = 0;

float32 Dist_Monitor = 0;

int16 Pnt = 0;

int16 GrowCount  = 0;
int16 GrowCount2 = 0;
int16 GrowCount3 = 0;
int16 GrowCount4 = 0;

uint8 BinaryImg[90][188];

/*---------------------------------函数定义-----------------------------*/
void data_init(void)
{
    CenterX       = Inv_x[10998];
    CenterY       = Inv_y[10998];
    Protect_Frame = 0;

    // control_init();
    // Nomal_Charge = 1;

    //    Stop_to_Charge = 1;

    g_LineError.m_u8LeftCenterValid  = 0;
    g_LineError.m_u8RightCenterValid = 0;

    g_LineError.m_f32RightBorderKappa = 0;
    g_LineError.m_f32LeftBorderKappa  = 0;
    g_LineError.m_f32RampError        = 0;
    g_LineError.m_u8RampErrorValid    = 0;
    //     //初始化运行圈数

    //     g_TrackType.m_u8Garage_Mode = 0;

    //     g_TrackType.m_u8Round_Count = 0;
    //     g_TrackType.m_u8GarageFlag = GARAGE_NONE;  //出车库
    //     g_TrackType.m_u8GarageDirection = Garage_Out_LEFT;
    //     g_TrackType.m_u8GarageTracking = Garage_Tracking_Near;
    //     g_TrackType.m_u8YjunctionDirection = YJUNCTION_LEFT;

    //     /*特殊赛道元素初始化*/
    // //    g_TrackType.m_u8PRoadDir = PROAD_INIT;
    //     g_TrackType.m_u8CrossFlag = CROSS_NONE;
    //     g_TrackType.m_u8LeftSideCrossFlag = CROSS_NONE;
    //     g_TrackType.m_u8RightSideCrossFlag = CROSS_NONE;
    //     g_TrackType.m_u8YjunctionFlag = YJUNCTION_NONE;
    //     g_TrackType.m_u8RightRoundaboutFlag = ROUNDABOUT_NONE;
    //     g_TrackType.m_u8LeftRoundaboutFlag = ROUNDABOUT_NONE;
    //     g_TrackType.m_u8RightPRoadFlag = PROAD_NONE;
    //     g_TrackType.m_u8LeftPRoadFlag = PROAD_NONE;
    //     g_TrackType.m_u8BendSFlag = BENDS_NONE;
    //     g_TrackType.m_u8RampFlag = RAMP_NONE;
    //     g_TrackType.m_int16BendSCount = 0;
    //     CornerLossCount = 0;
    //     StopCharging = 0;
    //     Beep.flag = 0;
    //     Beep.time = 0;
    //     Beep.num = 0;
    //     memset(g_Border.m_LRemotePnt, 0 , sizeof(g_Border.m_LRemotePnt[0]) * IMGH);
    //     memset(g_Border.m_RRemotePnt, 0 , sizeof(g_Border.m_RRemotePnt[0]) * IMGH);
}
