#include "Cross.h"

// 十字的处理可以寻远线也可以将远线拉到车头

uint16 Border_Lost_Count   = 0;
uint16 Border_Refind_Count = 0;

/*---------------------------------以下画线部分放在cpu0的pram中-----------------------------------------------------------*/

void DrawRemoteLine(TRACK_BORDER_INFO *p_Border)
{
    int16 int16_numL = p_Border->m_i16LRemotePointCnt;
    int16 int16_numR = p_Border->m_i16RRemotePointCnt;

    int16 int16_step = -1;
    while (++int16_step < int16_numL) {
        ips200_draw_point(p_Border->m_LRemotePnt[int16_step].m_i16x, p_Border->m_LRemotePnt[int16_step].m_i16y, RGB565_RED);
    }

    int16_step = -1;
    while (++int16_step < int16_numR) {
        ips200_draw_point(p_Border->m_RRemotePnt[int16_step].m_i16x, p_Border->m_RRemotePnt[int16_step].m_i16y, RGB565_BLUE);
    }
}

/*---------------------------------处理部分放在cpu1的pram中-----------------------------------------------------------*/

void Check_Cross(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_Type)
{

    if (p_Border->LL_CornerPos != -1 && p_Border->RL_CornerPos != -1 && p_Type->m_u8CrossFlag == CROSS_NONE) {
        p_Type->m_u8CrossFlag          = CROSS_FAR; // CROSS_FAR
        p_Type->m_u8RightSideCrossFlag = CROSS_NONE;
        p_Type->m_u8LeftSideCrossFlag  = CROSS_NONE;
    }

    // 拐点距离较近时切换CROSS_NEAR,切换寻远端边线
    if (p_Type->m_u8CrossFlag == CROSS_FAR && ((p_Border->LL_CornerPos != -1 && p_Border->LL_CornerPos <= 10) || (p_Border->RL_CornerPos != -1 && p_Border->RL_CornerPos <= 10))) p_Type->m_u8CrossFlag = CROSS_NEAR; // CROSS_NEAR

    // 有十字状态时搜索远端拐点
    if (p_Type->m_u8CrossFlag != CROSS_NONE) {
        FindRemoteLine(InImg, p_Border);
        FindRemoteCorner(p_Border);
    }

    // 近端边线先丢再有后刷掉十字状态
    if (p_Type->m_u8CrossFlag == CROSS_NEAR && p_Border->m_i16LPointCntRS < 5 && p_Border->m_i16RPointCntRS < 5) Border_Lost_Count++;

    if (p_Type->m_u8CrossFlag == CROSS_NEAR && Border_Lost_Count > 1 && (p_Border->m_i16LPointCntRS > 25 || p_Border->m_i16RPointCntRS > 25) /*&& (p_Border ->LL_CornerPos == -1 || p_Border ->LL_CornerPos > 35) && (p_Border ->RL_CornerPos == -1 || p_Border ->RL_CornerPos > 35)*/) {
        Border_Refind_Count++;
    }

    if (p_Type->m_u8CrossFlag == CROSS_NEAR && Border_Refind_Count > 0) {

        // 清空使用的变量
        Border_Lost_Count     = 0;
        Border_Refind_Count   = 0;
        p_Type->m_u8CrossFlag = CROSS_NONE;
        Protect_Frame         = 6;

        // pitch_angle = 0;
    }
}

/*搜索远端L拐点*/
void FindRemoteLine(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border)
{
    int16 thresholdclip          = 5;
    int16 int16_half             = BinaryBlock / 2;
    INT_POINT_INFO t_SeedLRemote = {-1, -1};
    INT_POINT_INFO t_SeedRRemote = {-1, -1};
    int16 y1, x1;
    int16 int16_dy, int16_dx;

    //
    if (p_Border->m_i16LPointCnt > 40) {
        y1 = p_Border->m_LPnt[p_Border->m_i16LPointCnt - 10].m_i16y - 8;
        x1 = p_Border->m_LPnt[p_Border->m_i16LPointCnt - 10].m_i16x;
        if (x1 < 5) x1 = 5;
    } else {
        y1 = 77;
        x1 = 20;
    }

    while (--y1 >= 30) {
        int int16_localthre = 0;
        int16_dy            = -int16_half - 1;
        while (++int16_dy <= int16_half) {
            int16_dx = -int16_half - 1;
            while (++int16_dx <= int16_half) {
                int16_localthre += InImg[y1 + int16_dy][x1 + int16_dx];
            }
        }

        int16_localthre /= (BinaryBlock * BinaryBlock);

        int16_localthre -= thresholdclip;

        if (InImg[y1 - 1][x1] < int16_localthre) {
            t_SeedLRemote.m_i16x = x1;
            t_SeedLRemote.m_i16y = y1;
            break;
        }
    }

    p_Border->m_i16LRemotePointCnt = IMGH;
    LeftLine_SeedGrow_Adaptive(InImg, t_SeedLRemote, p_Border->m_LRemotePnt, p_Border->m_LRemotePntGrowDirection, BinaryBlock, Threclip, &p_Border->m_i16LRemotePointCnt);

    if (p_Border->m_i16RPointCnt > 40) {
        y1 = p_Border->m_RPnt[p_Border->m_i16RPointCnt - 10].m_i16y - 8;
        x1 = p_Border->m_RPnt[p_Border->m_i16RPointCnt - 10].m_i16x;
        if (x1 > 176) x1 = 176;
    } else {
        y1 = 77;
        x1 = 167;
    }
    while (--y1 >= 30) {
        int int16_localthre = 0;
        int16_dy            = -int16_half - 1;
        while (++int16_dy <= int16_half) {
            int16_dx = -int16_half - 1;
            while (++int16_dx <= int16_half) {
                int16_localthre += InImg[y1 + int16_dy][x1 + int16_dx];
            }
        }

        int16_localthre /= (BinaryBlock * BinaryBlock);

        int16_localthre -= thresholdclip;

        if (InImg[y1 - 1][x1] < int16_localthre) {
            t_SeedRRemote.m_i16x = x1;
            t_SeedRRemote.m_i16y = y1;
            break;
        }
    }

    RightLine_SeedGrow_Adaptive(InImg, t_SeedRRemote, p_Border->m_RRemotePnt, p_Border->m_RRemotePntGrowDirection, BinaryBlock, Threclip, &p_Border->m_i16RRemotePointCnt);

    Inverse_Perspective(p_Border->m_LRemotePnt, (INT_POINT_INFO *)p_Border->m_LPntInvpRemote, p_Border->m_i16LRemotePointCnt);
    Inverse_Perspective(p_Border->m_RRemotePnt, (INT_POINT_INFO *)p_Border->m_RPntInvpRemote, p_Border->m_i16RRemotePointCnt);

    Points_Blur((INT_POINT_INFO *)p_Border->m_LPntInvpRemote, p_Border->m_LPntoutRemote, p_Border->m_i16LRemotePointCnt, PointsBlurKernel);
    Points_Blur((INT_POINT_INFO *)p_Border->m_RPntInvpRemote, p_Border->m_RPntoutRemote, p_Border->m_i16RRemotePointCnt, PointsBlurKernel);

    p_Border->m_i16LPointCntRSRemote = IMGH;
    p_Border->m_i16RPointCntRSRemote = IMGH;
    Points_Resample(p_Border->m_LPntoutRemote, p_Border->m_LPntRSRemote, p_Border->m_i16LRemotePointCnt, &p_Border->m_i16LPointCntRSRemote, SampleDist * PixelperMeter);
    Points_Resample(p_Border->m_RPntoutRemote, p_Border->m_RPntRSRemote, p_Border->m_i16RRemotePointCnt, &p_Border->m_i16RPointCntRSRemote, SampleDist * PixelperMeter);

    Border_Local_Angle(p_Border->m_LPntRSRemote, p_Border->LdAngleRemote, p_Border->m_i16LPointCntRSRemote, InterPoint);
    Border_Local_Angle(p_Border->m_RPntRSRemote, p_Border->RdAngleRemote, p_Border->m_i16RPointCntRSRemote, InterPoint);

    Angle_NMS(p_Border->LdAngleRemote, p_Border->LdAngleNMSRemote, p_Border->m_i16LPointCntRSRemote, NMSKernel);
    Angle_NMS(p_Border->RdAngleRemote, p_Border->RdAngleNMSRemote, p_Border->m_i16RPointCntRSRemote, NMSKernel);

    LeftBorderTrackingCenter(p_Border->m_LPntRSRemote, p_Border->m_LCPntRemote, p_Border->m_i16LPointCntRSRemote, InterPoint, (RoadWith * PixelperMeter / 2));
    RightBorderTrackingCenter(p_Border->m_RPntRSRemote, p_Border->m_RCPntRemote, p_Border->m_i16RPointCntRSRemote, InterPoint, (RoadWith * PixelperMeter / 2));

    p_Border->m_i16LCnterCntRSRemote = IMGH;
    p_Border->m_i16RCnterCntRSRemote = IMGH;
    Points_Resample(p_Border->m_LCPntRemote, p_Border->m_LCPntRSRemote, p_Border->m_i16LPointCntRSRemote, &p_Border->m_i16LCnterCntRSRemote, SampleDist * PixelperMeter);
    Points_Resample(p_Border->m_RCPntRemote, p_Border->m_RCPntRSRemote, p_Border->m_i16RPointCntRSRemote, &p_Border->m_i16RCnterCntRSRemote, SampleDist * PixelperMeter);
}

void FindRemoteCorner(TRACK_BORDER_INFO *p_Border)
{
    p_Border->LL_CornerPosRemote = p_Border->RL_CornerPosRemote = -1;

    if (p_Border->m_i16LPointCntRSRemote > 40) {
        int16 int16_Loopi;

        int16_Loopi = -1;
        while (++int16_Loopi < p_Border->m_i16LPointCntRSRemote - 1) {
            if (p_Border->LdAngleNMSRemote[int16_Loopi] == 0) continue;

            int16 upper = clip(int16_Loopi + InterPoint, 0, p_Border->m_i16LPointCntRSRemote - 1);
            int16 lower = clip(int16_Loopi - InterPoint, 0, p_Border->m_i16LPointCntRSRemote - 1);

            float32 f32_Corn = Fabs(p_Border->LdAngleRemote[int16_Loopi]) - (Fabs(p_Border->LdAngleRemote[upper]) + Fabs(p_Border->LdAngleRemote[lower])) / 2;

            if (f32_Corn > L_CORNER_LOWERBOUND && f32_Corn < L_CORNER_UPPERBOUND && int16_Loopi < 1.2 / SampleDist) {
                p_Border->LL_CornerPosRemote = int16_Loopi;
            }

            if (p_Border->LL_CornerPosRemote != -1) break;
        }
    }

    if (p_Border->m_i16RPointCntRSRemote > 40) {
        int16 int16_Loopi = -1;
        while (++int16_Loopi < p_Border->m_i16RPointCntRSRemote - 1) {
            if (p_Border->RdAngleNMSRemote[int16_Loopi] == 0) continue;

            int16 upper = clip(int16_Loopi + InterPoint, 0, p_Border->m_i16RPointCntRSRemote - 1);
            int16 lower = clip(int16_Loopi - InterPoint, 0, p_Border->m_i16RPointCntRSRemote - 1);

            float32 f32_Corn = Fabs(p_Border->RdAngleRemote[int16_Loopi]) - (Fabs(p_Border->RdAngleRemote[upper]) + Fabs(p_Border->RdAngleRemote[lower])) / 2;

            if (f32_Corn > L_CORNER_LOWERBOUND && f32_Corn < L_CORNER_UPPERBOUND && int16_Loopi < 1.2 / SampleDist) {
                p_Border->RL_CornerPosRemote = int16_Loopi;
            }
            if (p_Border->RL_CornerPosRemote != -1) break;
        }
    }

    if (p_Border->LL_CornerPosRemote != -1 && p_Border->RL_CornerPosRemote != -1) {
        float32 f32_dx = p_Border->m_LPntRSRemote[p_Border->LL_CornerPosRemote].m_i16x - p_Border->m_RPntRSRemote[p_Border->RL_CornerPosRemote].m_i16x;
        float32 f32_dy = p_Border->m_LPntRSRemote[p_Border->LL_CornerPosRemote].m_i16y - p_Border->m_RPntRSRemote[p_Border->RL_CornerPosRemote].m_i16y;
        float32 f32_dz = FSqrt(f32_dx * f32_dx + f32_dy * f32_dy);

        if (Fabs(f32_dz - 0.4 * PixelperMeter) < 0.15 * PixelperMeter) {
            if (!(/*f32_dzf > 0.7 * PixelperMeter &&*/ p_Border->m_LPntRSRemote[clip(p_Border->LL_CornerPosRemote - 50, 0, p_Border->m_i16LPointCntRSRemote - 1)].m_i16x < p_Border->m_LPntRSRemote[p_Border->LL_CornerPosRemote].m_i16x && p_Border->m_RPntRSRemote[clip(p_Border->RL_CornerPosRemote - 50, 0, p_Border->m_i16RPointCntRSRemote - 1)].m_i16x > p_Border->m_RPntRSRemote[p_Border->RL_CornerPosRemote].m_i16x)) {
                p_Border->LL_CornerPosRemote = p_Border->RL_CornerPosRemote = -1;
            }
        } else {
            p_Border->LL_CornerPosRemote = p_Border->RL_CornerPosRemote = -1;
        }
    } else if (p_Border->LL_CornerPosRemote != -1) {
        if (!(p_Border->m_LPntRSRemote[clip(p_Border->LL_CornerPosRemote - 50, 0, p_Border->m_i16LPointCntRSRemote - 1)].m_i16x < p_Border->m_LPntRSRemote[p_Border->LL_CornerPosRemote].m_i16x)) p_Border->LL_CornerPosRemote = -1;
    }

    else if (p_Border->RL_CornerPos != -1) {
        if (!(p_Border->m_RPntRSRemote[clip(p_Border->RL_CornerPosRemote - 50, 0, p_Border->m_i16RPointCntRSRemote - 1)].m_i16x > p_Border->m_RPntRSRemote[p_Border->RL_CornerPosRemote].m_i16x)) p_Border->RL_CornerPosRemote = -1;
    }
}

// 中入十字检查
void Check_MIDCross(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_Type)
{
    if (p_Border->m_i16LPointCntRS < 1 && p_Border->m_i16RPointCntRS < 1 && g_TrackType.m_u8CrossFlag == CROSS_NONE && p_Type->m_u8RightSideCrossFlag == CROSS_NONE && g_TrackType.m_u8LeftSideCrossFlag == CROSS_NONE) {
        FindRemoteLine(InImg, p_Border);
        FindRemoteCorner(p_Border);

        if (p_Border->RL_CornerPosRemote != -1 && p_Border->LL_CornerPosRemote != -1) {
            p_Type->m_u8CrossFlag = CROSS_NEAR;
        }
    }
}

/*斜入搜索远端L拐点*/
void SideCrossFindRemoteLine(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border)
{
    int16 thresholdclip          = 5;
    int16 int16_half             = BinaryBlock / 2;
    INT_POINT_INFO t_SeedLRemote = {-1, -1};
    INT_POINT_INFO t_SeedRRemote = {-1, -1};
    int16 y1, x1;
    int16 int16_dy, int16_dx;

    //
    if (p_Border->m_i16LPointCnt > 50) {
        y1 = p_Border->m_LPnt[p_Border->m_i16LPointCnt - 40].m_i16y - 8;
        x1 = p_Border->m_LPnt[p_Border->m_i16LPointCnt - 40].m_i16x;
        if (x1 < 5) x1 = 5;
    } else {
        y1 = 77;
        x1 = 10;
    }

    while (--y1 >= 30) {
        int int16_localthre = 0;
        int16_dy            = -int16_half - 1;
        while (++int16_dy <= int16_half) {
            int16_dx = -int16_half - 1;
            while (++int16_dx <= int16_half) {
                int16_localthre += InImg[y1 + int16_dy][x1 + int16_dx];
            }
        }

        int16_localthre /= (BinaryBlock * BinaryBlock);

        int16_localthre -= thresholdclip;

        if (InImg[y1 - 1][x1] < int16_localthre) {
            t_SeedLRemote.m_i16x = x1;
            t_SeedLRemote.m_i16y = y1;
            break;
        }
    }

    p_Border->m_i16LRemotePointCnt = IMGH;
    LeftLine_SeedGrow_Adaptive(InImg, t_SeedLRemote, p_Border->m_LRemotePnt, p_Border->m_LRemotePntGrowDirection, BinaryBlock, Threclip, &p_Border->m_i16LRemotePointCnt);

    if (p_Border->m_i16RPointCnt > 50) {
        y1 = p_Border->m_RPnt[p_Border->m_i16RPointCnt - 40].m_i16y - 8;
        x1 = p_Border->m_RPnt[p_Border->m_i16RPointCnt - 40].m_i16x;
        if (x1 > 176) x1 = 176;
    } else {
        y1 = 77;
        x1 = 177;
    }
    while (--y1 >= 30) {
        int int16_localthre = 0;
        int16_dy            = -int16_half - 1;
        while (++int16_dy <= int16_half) {
            int16_dx = -int16_half - 1;
            while (++int16_dx <= int16_half) {
                int16_localthre += InImg[y1 + int16_dy][x1 + int16_dx];
            }
        }

        int16_localthre /= (BinaryBlock * BinaryBlock);

        int16_localthre -= thresholdclip;

        if (InImg[y1 - 1][x1] < int16_localthre) {
            t_SeedRRemote.m_i16x = x1;
            t_SeedRRemote.m_i16y = y1;
            break;
        }
    }

    RightLine_SeedGrow_Adaptive(InImg, t_SeedRRemote, p_Border->m_RRemotePnt, p_Border->m_RRemotePntGrowDirection, BinaryBlock, Threclip, &p_Border->m_i16RRemotePointCnt);

    Inverse_Perspective(p_Border->m_LRemotePnt, (INT_POINT_INFO *)p_Border->m_LPntInvpRemote, p_Border->m_i16LRemotePointCnt);
    Inverse_Perspective(p_Border->m_RRemotePnt, (INT_POINT_INFO *)p_Border->m_RPntInvpRemote, p_Border->m_i16RRemotePointCnt);

    Points_Blur((INT_POINT_INFO *)p_Border->m_LPntInvpRemote, p_Border->m_LPntoutRemote, p_Border->m_i16LRemotePointCnt, PointsBlurKernel);
    Points_Blur((INT_POINT_INFO *)p_Border->m_RPntInvpRemote, p_Border->m_RPntoutRemote, p_Border->m_i16RRemotePointCnt, PointsBlurKernel);

    p_Border->m_i16LPointCntRSRemote = IMGH;
    p_Border->m_i16RPointCntRSRemote = IMGH;
    Points_Resample(p_Border->m_LPntoutRemote, p_Border->m_LPntRSRemote, p_Border->m_i16LRemotePointCnt, &p_Border->m_i16LPointCntRSRemote, SampleDist * PixelperMeter);
    Points_Resample(p_Border->m_RPntoutRemote, p_Border->m_RPntRSRemote, p_Border->m_i16RRemotePointCnt, &p_Border->m_i16RPointCntRSRemote, SampleDist * PixelperMeter);

    Border_Local_Angle(p_Border->m_LPntRSRemote, p_Border->LdAngleRemote, p_Border->m_i16LPointCntRSRemote, InterPoint);
    Border_Local_Angle(p_Border->m_RPntRSRemote, p_Border->RdAngleRemote, p_Border->m_i16RPointCntRSRemote, InterPoint);

    Angle_NMS(p_Border->LdAngleRemote, p_Border->LdAngleNMSRemote, p_Border->m_i16LPointCntRSRemote, NMSKernel);
    Angle_NMS(p_Border->RdAngleRemote, p_Border->RdAngleNMSRemote, p_Border->m_i16RPointCntRSRemote, NMSKernel);

    LeftBorderTrackingCenter(p_Border->m_LPntRSRemote, p_Border->m_LCPntRemote, p_Border->m_i16LPointCntRSRemote, InterPoint, (RoadWith * PixelperMeter / 2));
    RightBorderTrackingCenter(p_Border->m_RPntRSRemote, p_Border->m_RCPntRemote, p_Border->m_i16RPointCntRSRemote, InterPoint, (RoadWith * PixelperMeter / 2));

    p_Border->m_i16LCnterCntRSRemote = IMGH;
    p_Border->m_i16RCnterCntRSRemote = IMGH;
    Points_Resample(p_Border->m_LCPntRemote, p_Border->m_LCPntRSRemote, p_Border->m_i16LPointCntRSRemote, &p_Border->m_i16LCnterCntRSRemote, SampleDist * PixelperMeter);
    Points_Resample(p_Border->m_RCPntRemote, p_Border->m_RCPntRSRemote, p_Border->m_i16RPointCntRSRemote, &p_Border->m_i16RCnterCntRSRemote, SampleDist * PixelperMeter);
}
// 右斜入三拐点十字检查
void RightThreeCornerCross(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_Type)
{
    if (p_Border->m_i16LPointCntRS < 1 && p_Border->RL_CornerPos != -1 && p_Type->m_u8CrossFlag == CROSS_NONE && p_Type->m_u8RightSideCrossFlag == CROSS_NONE && g_TrackType.m_u8LeftSideCrossFlag == CROSS_NONE) {
        SideCrossFindRemoteLine(InImg, p_Border);
        FindRemoteCorner(p_Border);

        if (p_Border->RL_CornerPosRemote != -1 && p_Border->LL_CornerPosRemote != -1) {
            p_Type->m_u8RightSideCrossFlag = CROSS_FAR;
        }
    }

    if (p_Type->m_u8RightSideCrossFlag == CROSS_FAR) {
        SideCrossFindRemoteLine(InImg, p_Border);
        FindRemoteCorner(p_Border);

        if (p_Border->RL_CornerPos != -1 && p_Border->RL_CornerPos <= 10) {
            p_Type->m_u8RightSideCrossFlag = CROSS_NEAR;
        }
    }

    if (p_Type->m_u8RightSideCrossFlag == CROSS_NEAR) {
        SideCrossFindRemoteLine(InImg, p_Border);
        FindRemoteCorner(p_Border);

        if (p_Border->m_i16LPointCntRS < 5 && p_Border->m_i16RPointCntRS < 5) Border_Lost_Count++;

        if (Border_Lost_Count > 1 && (p_Border->m_i16LPointCntRS > 25 || p_Border->m_i16RPointCntRS > 25)) {
            Border_Refind_Count++;
        }

        if (Border_Refind_Count > 0) {
            Border_Lost_Count              = 0;
            Border_Refind_Count            = 0;
            p_Type->m_u8RightSideCrossFlag = CROSS_NONE;
            Protect_Frame                  = 6;

            // pitch_angle = 0;
        }
    }
}

// 左斜入三拐点十字检查
void LeftThreeCornerCross(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_Type)
{
    if (p_Border->m_i16RPointCntRS < 1 && p_Border->LL_CornerPos != -1 && p_Type->m_u8CrossFlag == CROSS_NONE && p_Type->m_u8LeftSideCrossFlag == CROSS_NONE && g_TrackType.m_u8RightSideCrossFlag == CROSS_NONE) {
        SideCrossFindRemoteLine(InImg, p_Border);
        FindRemoteCorner(p_Border);

        if (p_Border->RL_CornerPosRemote != -1 && p_Border->LL_CornerPosRemote != -1) {
            p_Type->m_u8LeftSideCrossFlag = CROSS_FAR;
        }
    }

    if (p_Type->m_u8LeftSideCrossFlag == CROSS_FAR) {
        SideCrossFindRemoteLine(InImg, p_Border);
        FindRemoteCorner(p_Border);

        if (p_Border->LL_CornerPos != -1 && p_Border->LL_CornerPos <= 10) {
            p_Type->m_u8LeftSideCrossFlag = CROSS_NEAR;
        }
    }

    if (p_Type->m_u8LeftSideCrossFlag == CROSS_NEAR) {
        SideCrossFindRemoteLine(InImg, p_Border);
        FindRemoteCorner(p_Border);

        if (p_Border->m_i16LPointCntRS < 5 && p_Border->m_i16RPointCntRS < 5) Border_Lost_Count++;

        if (Border_Lost_Count > 1 && (p_Border->m_i16LPointCntRS > 25 || p_Border->m_i16RPointCntRS > 25)) {
            Border_Refind_Count++;
        }

        if (Border_Refind_Count > 0) {
            Border_Lost_Count             = 0;
            Border_Refind_Count           = 0;
            p_Type->m_u8LeftSideCrossFlag = CROSS_NONE;
            Protect_Frame                 = 6;

            // pitch_angle = 0;
        }
    }
}

// // 右斜入两拐点十字检查  TODO
// void RightTwoCornerCross(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_Type)
// {
// }

// // 左斜入两拐点十字检查  TODO
// void LeftTwoCornerCross(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_Type)
// {
// }
