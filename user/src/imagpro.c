#include "imagpro.h"
// #include <stdlib.h>

// TRACK_BORDER_INFO g_Border;
// TRACK_TYPE_INFO g_TrackType;

/*赛道宽度*/
const float32 RoadWith = 0.4;
/*采样距离*/
const float32 SampleDist = 0.02;
/*间隔点数*/
const int16 InterPoint = 10;
/*差比和步长*/
const int16 SARStep = 5;
/*差比和阈值*/
const int16 SAR_thres = 50;
/*自适应二值化核大小*/
const int16 BinaryBlock = 7;
/*阈值补偿*/
const int16 Threclip = 8;
/*边线滤波核大小*/
const int16 PointsBlurKernel = 7;
/*非极大值抑制核大小*/
const int16 NMSKernel = 7;

/*纯跟踪起点*/
float32 CenterX;
float32 CenterY;

/*目前的速度,决定部分元素的纯跟踪预瞄点*/
float32 SpeedThres;
float32 SpeedDistance;
uint8 outimage[MT9V03X_H / 2][MT9V03X_W / 2];
uint8 inv_image[MT9V03X_H][MT9V03X_W];
// uint8 outimage1[MT9V03X_H][MT9V03X_W];

/* 前进方向定义：
 *   0
 * 3   1
 *   2
 */
/*八邻域数组*/
const int8 arr_frontdir[4][2] =
    {
        {-1, 0},
        {0, 1},
        {1, 0},
        {0, -1}};

const int8 arr_frontleftdir[4][2] =
    {
        {-1, -1},
        {-1, 1},
        {1, 1},
        {1, -1}};

const int8 arr_frontrightdir[4][2] =
    {
        {-1, 1},
        {1, 1},
        {1, -1},
        {-1, -1}};

#define GrayScale 256
int pixelCount[GrayScale] = {0}; // 每个灰度值所占像素个数
float pixelPro[GrayScale] = {0}; // 每个灰度值所占总像素比例

/*----------------------------调试代码----------------------------*/
/*画原边线*/
void DrawBoarder(TRACK_BORDER_INFO *p_Border)
{
    int16 int16_numL = p_Border->m_i16LPointCnt;

    int16 int16_numR = p_Border->m_i16RPointCnt;

    int16 int16_step = -1;

    while (++int16_step < int16_numL) {
        ips200_draw_point(p_Border->m_LPnt[int16_step].m_i16x, p_Border->m_LPnt[int16_step].m_i16y, RGB565_RED);
    }

    int16_step = -1;

    while (++int16_step < int16_numR) {
        ips200_draw_point(p_Border->m_RPnt[int16_step].m_i16x, p_Border->m_RPnt[int16_step].m_i16y, RGB565_BLUE);
    }
}

/*画逆透视后边线*/
void DrawBoarderInvp(TRACK_BORDER_INFO *p_Border)
{
    int16 int16_numL = p_Border->m_i16LPointCntRS;

    int16 int16_numR = p_Border->m_i16RPointCntRS;

    int16 int16_step = -1;

    while (++int16_step < int16_numL) {
        if (p_Border->m_LPntRS[int16_step].m_i16y > 3 && p_Border->m_LPntRS[int16_step].m_i16x > 3 && p_Border->m_LPntRS[int16_step].m_i16y < 115 && p_Border->m_LPntRS[int16_step].m_i16x < 183) {
            ips200_draw_point((uint16)p_Border->m_LPntRS[int16_step].m_i16x, (uint16)p_Border->m_LPntRS[int16_step].m_i16y, RGB565_RED);
        }
    }

    int16_step = -1;
    while (++int16_step < int16_numR) {
        if (p_Border->m_RPntRS[int16_step].m_i16y > 3 && p_Border->m_RPntRS[int16_step].m_i16x > 3 && p_Border->m_RPntRS[int16_step].m_i16y < 115 && p_Border->m_RPntRS[int16_step].m_i16x < 183) {
            ips200_draw_point((uint16)p_Border->m_RPntRS[int16_step].m_i16x, (uint16)p_Border->m_RPntRS[int16_step].m_i16y, RGB565_GREEN);
        }
    }
}

/*画逆透视后中线*/
void DrawCenter(TRACK_BORDER_INFO *p_Border)
{
    int16 int16_numL = p_Border->m_i16LCnterCntRS;

    int16 int16_numR = p_Border->m_i16RCnterCntRS;

    int16 int16_step = -1;

    while (++int16_step < int16_numL) {
        if (p_Border->m_LCPntRS[int16_step].m_i16y > 0 && p_Border->m_LCPntRS[int16_step].m_i16x > 0 && p_Border->m_LCPntRS[int16_step].m_i16y < 118 && p_Border->m_LCPntRS[int16_step].m_i16x < 188) {
            ips200_draw_point((uint16)p_Border->m_LCPntRS[int16_step].m_i16x, (uint16)p_Border->m_LCPntRS[int16_step].m_i16y, RGB565_PURPLE);
        }
    }

    int16_step = -1;
    while (++int16_step < int16_numR) {
        if (p_Border->m_RCPntRS[int16_step].m_i16y > 0 && p_Border->m_RCPntRS[int16_step].m_i16x > 0 && p_Border->m_RCPntRS[int16_step].m_i16y < 118 && p_Border->m_RCPntRS[int16_step].m_i16x < 188) {
            ips200_draw_point((uint16)p_Border->m_RCPntRS[int16_step].m_i16x, (uint16)p_Border->m_RCPntRS[int16_step].m_i16y, RGB565_BROWN);
        }
    }
}

// 变阈值二值化
void adaptiveThreshold_1(uint8 (*InImg)[MT9V03X_W], uint8 (*OutImg)[MT9V03X_W], int width, int height, int block, uint8_t clip_value)
{
    int count = 0;
    // assert(block % 2 == 1); // block必须为奇数
    int half_block = block / 2;
    for (int y = half_block; y < height - half_block; y++) {
        for (int x = half_block; x < width - half_block; x++) {
            int thres = 0;
            for (int dy = -half_block; dy <= half_block; dy++) {
                for (int dx = -half_block; dx <= half_block; dx++) {
                    thres += InImg[y + dy + 1][x + dx];
                    count++;
                }
            }
            thres        = thres / (block * block) - clip_value; // 求得平均阈值在减去经验变量
            OutImg[y][x] = InImg[y][x] > thres ? 254 : 1;
        }
    }
    SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\r\n count=%d.", count);
}
// 池化变阈值二值化
//  uint8 Through_Img[IMGH][IMGW];
void adaptiveThreshold_2(void)
{
    // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> A success.");
    // int half_block = block / 2;
    /*2x2最小池化(赛道边界是黑色，最小池化可以较好保留赛道边界)*/
    int16 min_value;
    // 先遍历y后遍历x比较cache-friendly

    for (int16 i = 1; i < IMGH; i += 2) {
        for (int16 j = 1; j < IMGW; j += 2) {
            min_value = 254;
            if (mt9v03x_image[i][j] < min_value) min_value = mt9v03x_image[i][j];
            if (mt9v03x_image[i - 1][j] < min_value) min_value = mt9v03x_image[i - 1][j];
            if (mt9v03x_image[i][j - 1] < min_value) min_value = mt9v03x_image[i][j - 1];
            if (mt9v03x_image[i - 1][j - 1] < min_value) min_value = mt9v03x_image[i - 1][j - 1];
            outimage[i / 2][j / 2] = min_value;
        }
    }
}

void wusuowei(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_Type)
{

    // adaptiveThreshold_2(image_read_buffer, outimage[MT9V03X_H / 2][MT9V03X_W / 2]);
    // timer_start(TIM_2);
    // 初始化左右边线种子
    INT_POINT_INFO t_SeedL = {-1, -1};
    INT_POINT_INFO t_SeedR = {-1, -1};

    int16 int16_x, int16_y;
    int16 int16_dx, int16_dy;
    int16 BinaryBlock = 7;

    int16 int16_half = BinaryBlock / 2;
    // 阈值补偿
    int16 thresholdclip = 10;

    /*----------在图像底部1/3搜索第一个点------------*/
    uint8 LYEND = IMGH * 2 / 3;
    // if (p_Type->m_u8GarageFlag == GARAGE_LEFT_IN) LYEND = 95;
    //    else if(p_Type ->m_u8GarageFlag == OUT_GARAGE) LYEND = 95;

    // 忽略底部5行
    int16_y = IMGH - 5;
    // 找到种子标志位
    p_Border->u8_FindLeftSeed = 0;
    while (--int16_y > LYEND) {
        /*从图像的中线向左搜索*/
        int16_x = IMGW / 2;
        // if (p_Type->m_u8GarageFlag == OUT_GARAGE) int16_x = IMGW / 2 - 40;
        int16 int16_xEnd;

        /*-------防止数组越界---------*/
        int16_xEnd = 4;

        while (--int16_x > int16_xEnd) {
            int16 int16_localthre = 0;
            int16_dy              = -int16_half - 1;
            while (++int16_dy <= int16_half) {
                int16_dx = -int16_half - 1;
                while (++int16_dx <= int16_half) {
                    int16_localthre += InImg[int16_y + int16_dy][int16_x + int16_dx];
                }
            }
            /*获得图像7X7个像素点的平均阈值*/
            int16_localthre /= (BinaryBlock * BinaryBlock);

            /*阈值补偿*/
            int16_localthre -= thresholdclip;

            // 根据阈值判断是否扫到黑点
            if (InImg[int16_y][int16_x - 1] < int16_localthre) {
                p_Border->u8_FindLeftSeed = 1;
                t_SeedL.m_i16x            = int16_x;
                t_SeedL.m_i16y            = int16_y;
                // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> x=%d.", t_SeedL.m_i16x);
                // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> y=%d.", t_SeedL.m_i16y);
                // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> y=%d.", 1);
                break;
            }
        }

        // 扫到种子后跳出循环
        if (p_Border->u8_FindLeftSeed == 1) break;
    }
    LeftLine_SeedGrow_Adaptive(mt9v03x_image, t_SeedL, p_Border->m_LPnt, p_Border->m_LPntGrowDirection, BinaryBlock, Threclip, &p_Border->m_i16LPointCnt);

    int16_y                    = IMGH - 5;
    p_Border->u8_FindRightSeed = 0;

    uint8 RYEND = IMGH * 2 / 3;
    // if(p_Type ->m_u8GarageFlag == GARAGE_RIGHT_IN) RYEND = 95;
    //    else if(p_Type ->m_u8GarageFlag == OUT_GARAGE) LYEND = 95;

    while (--int16_y > RYEND) {
        int16_x = IMGW / 2;
        // if(p_Type ->m_u8GarageFlag == OUT_GARAGE) int16_x = IMGW/2 + 40;
        int16 int16_xEnd;

        int16_xEnd = IMGW - 6;
        while (++int16_x < int16_xEnd) {

            int16 int16_localthre = 0;
            int16_dy              = -int16_half - 1;
            while (++int16_dy <= int16_half) {
                int16_dx = -int16_half - 1;
                while (++int16_dx <= int16_half) {
                    int16_localthre += InImg[int16_y + int16_dy][int16_x + int16_dx];
                }
            }
            int16_localthre /= (BinaryBlock * BinaryBlock);

            int16_localthre -= thresholdclip;

            if (InImg[int16_y][int16_x + 1] < int16_localthre) {
                p_Border->u8_FindRightSeed = 1;
                t_SeedR.m_i16x             = int16_x;
                t_SeedR.m_i16y             = int16_y;
                break;
            }
        }

        if (p_Border->u8_FindRightSeed == 1) break;
    }

    /*初始化右边界数组个数*/
    p_Border->m_i16RPointCnt = IMGH;
    RightLine_SeedGrow_Adaptive(mt9v03x_image, t_SeedR, p_Border->m_RPnt, p_Border->m_RPntGrowDirection, BinaryBlock, Threclip, &p_Border->m_i16RPointCnt);
    // timer_stop(TIM_2);
    // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\r\n LOG -> POCEESS TIME ==%d", timer_get(TIM_2));
    // timer_clear(TIM_2);
    /*边线数组重新压栈*/
    BorderLineReCoor(p_Border->m_LPnt, p_Border->m_i16LPointCnt, p_Border->m_LeftLineCoor);
    BorderLineReCoor(p_Border->m_RPnt, p_Border->m_i16RPointCnt, p_Border->m_RightLineCoor);

    /*计算赛道宽度,用于计算超宽*/
    BorderWidth_Calc(p_Border, p_Border->m_LeftLineCoor, p_Border->m_RightLineCoor);

    /*逆透视变换*/
    Inverse_Perspective(p_Border->m_LPnt, p_Border->m_LPntInvp, p_Border->m_i16LPointCnt);
    Inverse_Perspective(p_Border->m_RPnt, p_Border->m_RPntInvp, p_Border->m_i16RPointCnt);

    //    timer_start(TIM_2);

    /*边线滤波*/
    Points_Blur(p_Border->m_LPntInvp, p_Border->m_LPntout, p_Border->m_i16LPointCnt, PointsBlurKernel);
    Points_Blur(p_Border->m_RPntInvp, p_Border->m_RPntout, p_Border->m_i16RPointCnt, PointsBlurKernel);

    /*等距离采样*/
    p_Border->m_i16LPointCntRS = IMGH;
    p_Border->m_i16RPointCntRS = IMGH;
    Points_Resample(p_Border->m_LPntout, p_Border->m_LPntRS, p_Border->m_i16LPointCnt, &p_Border->m_i16LPointCntRS, SampleDist * PixelperMeter);
    Points_Resample(p_Border->m_RPntout, p_Border->m_RPntRS, p_Border->m_i16RPointCnt, &p_Border->m_i16RPointCntRS, SampleDist * PixelperMeter);

    // 对最多一半的边线计算斜率和方差
    uint8 m_u8StartLine    = 5;
    uint8 m_u8LeftEndLine  = p_Border->m_i16LPointCnt > 60 ? 60 : p_Border->m_i16LPointCnt;
    uint8 m_u8RightEndLine = p_Border->m_i16RPointCnt > 60 ? 60 : p_Border->m_i16RPointCnt;

    /*计算边线斜率*/
    // Regression(p_Border->m_LPnt, &p_Border->m_f64LeftSlope, &p_Border->m_f64LeftIntersect, m_u8StartLine, m_u8LeftEndLine);
    // Regression(p_Border->m_RPnt, &p_Border->m_f64RightSlope, &p_Border->m_f64RightIntersect, m_u8StartLine, m_u8RightEndLine);

    /*计算边线方差*/
    // Variance(p_Border->m_LPnt, p_Border->m_f64LeftSlope, p_Border->m_f64LeftIntersect, &p_Border->m_f64LeftVariance, m_u8StartLine, m_u8LeftEndLine);
    // Variance(p_Border->m_RPnt, p_Border->m_f64RightSlope, p_Border->m_f64RightIntersect, &p_Border->m_f64RightVariance, m_u8StartLine, m_u8RightEndLine);

    /*计算边线角度变化率*/
    // Border_Local_Angle(p_Border->m_LPntRS, p_Border->LdAngle, p_Border->m_i16LPointCntRS, InterPoint);
    // Border_Local_Angle(p_Border->m_RPntRS, p_Border->RdAngle, p_Border->m_i16RPointCntRS, InterPoint);

    /*角度非极大值抑制*/
    //    Angle_NMS(p_Border, NMSKernel);
    // Angle_NMS(p_Border->LdAngle, p_Border->LdAngleNMS, p_Border->m_i16LPointCntRS, NMSKernel);
    // Angle_NMS(p_Border->RdAngle, p_Border->RdAngleNMS, p_Border->m_i16RPointCntRS, NMSKernel);

    /*边线跟踪中线*/
    LeftBorderTrackingCenter(p_Border->m_LPntRS, p_Border->m_LCPnt, p_Border->m_i16LPointCntRS, InterPoint, (RoadWith * PixelperMeter / 2));
    RightBorderTrackingCenter(p_Border->m_RPntRS, p_Border->m_RCPnt, p_Border->m_i16RPointCntRS, InterPoint, (RoadWith * PixelperMeter / 2));
    // timer_stop(TIM_2);
    // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\r\n LOG -> POCEESS TIME ==%d", timer_get(TIM_2));
    // timer_clear(TIM_2);

    // 统一中线起点,把中线的起始点拉到车头,防止左右边线平移出的中线算出的曲率相差过大

    // 找中线距离车头的最近点
    float32 LeftMinDist  = 1e10;
    int16 LeftStartPoint = -1;
    int16 Loopi          = -1;
    while (++Loopi < p_Border->m_i16LPointCntRS - 1) {
        float32 f32_dx = p_Border->m_LCPnt[Loopi].m_i16x - CenterX;
        float32 f32_dy = p_Border->m_LCPnt[Loopi].m_i16y - CenterY;
        float32 f32_dz = FSqrt(f32_dx * f32_dx + f32_dy * f32_dy);

        if (f32_dz < LeftMinDist) // 找最小Dist
        {
            LeftMinDist    = f32_dz;
            LeftStartPoint = Loopi;
        }
    }

    float32 RightMinDist  = 1e10;
    int16 RightStartPoint = -1;
    Loopi                 = -1;
    while (++Loopi < p_Border->m_i16RPointCntRS - 1) {
        float32 f32_dx = p_Border->m_RCPnt[Loopi].m_i16x - CenterX;
        float32 f32_dy = p_Border->m_RCPnt[Loopi].m_i16y - CenterY;
        float32 f32_dz = FSqrt(f32_dx * f32_dx + f32_dy * f32_dy);

        if (f32_dz < RightMinDist) {
            RightMinDist    = f32_dz;
            RightStartPoint = Loopi;
        }
    }

    if (LeftStartPoint != -1 && p_Border->m_i16LPointCntRS - LeftStartPoint >= 3) {
        p_Border->m_LCPnt[LeftStartPoint].m_i16x = CenterX;
        p_Border->m_LCPnt[LeftStartPoint].m_i16y = CenterY;
        Points_Resample(p_Border->m_LCPnt + LeftStartPoint, p_Border->m_LCPntRS, p_Border->m_i16LPointCntRS - LeftStartPoint, &p_Border->m_i16LCnterCntRS, SampleDist * PixelperMeter);
    } else {
        p_Border->m_i16LCnterCntRS = 1;
    }

    if (RightStartPoint != -1 && p_Border->m_i16RPointCntRS - RightStartPoint >= 3) {
        p_Border->m_RCPnt[RightStartPoint].m_i16x = CenterX;
        p_Border->m_RCPnt[RightStartPoint].m_i16y = CenterY;
        Points_Resample(p_Border->m_RCPnt + RightStartPoint, p_Border->m_RCPntRS, p_Border->m_i16RPointCntRS - RightStartPoint, &p_Border->m_i16RCnterCntRS, SampleDist * PixelperMeter);
    } else {
        p_Border->m_i16RCnterCntRS = 1;
    }
}

/*左边线自适应二值化八邻域*/
void LeftLine_SeedGrow_Adaptive(uint8 (*InImg)[IMGW], INT_POINT_INFO t_Seed, INT_POINT_INFO point[], uint8 pointDirection[], int16 blocksize, int16 threclip, int16 *num)
{
    int16 int16_half = blocksize / 2;
    // INT_POINT_INFO t_tempPoint;
    int16 int16_y, int16_x, int16_dx, int16_dy;
    int16 int16_steps = 0;
    int8 u8_dir = 0, u8_turn = 0;

    memset(point, 0, sizeof(point[0]) * (*num));                   // 初始化边线数组
    memset(pointDirection, 0, sizeof(pointDirection[0]) * (*num)); // 初始化生长方向
    *num = 0;

    // 检查是否搜索到种子
    if (t_Seed.m_i16y == -1) {
        return;
    }

    point[int16_steps++] = t_Seed;

    /*种子作为边线第一个点*/
    int16_x = t_Seed.m_i16x;
    int16_y = t_Seed.m_i16y;

    // 忽略图像的边界，防止数组越界
    while (int16_x < IMGW - 6 && int16_x > 4 && int16_y < IMGH - 5 && int16_y >= 4 && u8_turn < 4 && int16_steps < IMGH) {
        int16 int16_localthre = 0;
        int16_dy              = -int16_half - 1;
        while (++int16_dy <= int16_half) {
            int16_dx = -int16_half - 1;
            // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\r\n LOG -> A.");
            while (++int16_dx <= int16_half) {
                int16_localthre += InImg[int16_y + int16_dy][int16_x + int16_dx];
                // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\r\n LOG -> B.");
            }
        }

        int16_localthre /= (blocksize * blocksize);
        /*阈值补偿*/
        int16_localthre -= threclip;

        int16 int16_frontlevel     = InImg[int16_y + arr_frontdir[u8_dir][0]][int16_x + arr_frontdir[u8_dir][1]];
        int16 int16_frontleftlevel = InImg[int16_y + arr_frontleftdir[u8_dir][0]][int16_x + arr_frontleftdir[u8_dir][1]];

        if (int16_frontlevel < int16_localthre) // 如果前方小于阈值，往右转
        {
            u8_turn++;
            u8_dir = (u8_dir + 1) % 4;
            // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\r\n LOG -> C.");

        } else if (int16_frontleftlevel < int16_localthre) // 左前方小于阈值并且前方大于阈值，往前方
        {
            int16_y += arr_frontdir[u8_dir][0];
            int16_x += arr_frontdir[u8_dir][1];

            pointDirection[int16_steps] = u8_dir + 1;

            point[int16_steps].m_i16y = int16_y;
            point[int16_steps].m_i16x = int16_x;
            int16_steps++;

            // point[int16_steps++]        = t_tempPoint;
            // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\r\n LOG -> else1 xx=%d.", point[int16_steps].m_i16x);
            // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\r\n LOG -> else1 yy=%d.", point[int16_steps].m_i16y);
            u8_turn = 0;
        } else // 前方大于阈值并且左前方大于阈值，往左前方
        {
            int16_y += arr_frontleftdir[u8_dir][0];
            int16_x += arr_frontleftdir[u8_dir][1];

            pointDirection[int16_steps] = u8_dir + 5;

            point[int16_steps].m_i16y = int16_y;
            point[int16_steps].m_i16x = int16_x;
            int16_steps++;

            // point[int16_steps++]        = t_tempPoint;

            // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\r\n LOG ->else2 xx=%d.", point[int16_steps].m_i16x);
            // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\r\n LOG ->else2 yy=%d.", point[int16_steps].m_i16y);

            u8_dir  = (u8_dir + 3) % 4;
            u8_turn = 0;
        }
    }
    // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\r\n LOG -> D.");
    *num = int16_steps;
}

/*右边线自适应八邻域*/
void RightLine_SeedGrow_Adaptive(uint8 (*InImg)[IMGW], INT_POINT_INFO t_Seed, INT_POINT_INFO point[], uint8 pointDirection[], int16 blocksize, int16 threclip, int16 *num)
{
    int16 int16_half = blocksize / 2;
    // INT_POINT_INFO t_tempPoint;
    int16 int16_y, int16_x, int16_dx, int16_dy;
    int16 int16_steps = 0;
    int8 u8_dir = 0, u8_turn = 0;

    /*初始化边线数组和生长方向*/
    memset(point, 0, sizeof(point[0]) * (*num));
    memset(pointDirection, 0, sizeof(pointDirection[0]) * (*num));
    *num = 0;

    // 检查是否搜索到种子
    if (t_Seed.m_i16y == -1) {
        return;
    }

    point[int16_steps++] = t_Seed;
    /*赋值种子作为边线第一个点*/
    int16_x = t_Seed.m_i16x;
    int16_y = t_Seed.m_i16y;
    while (int16_x < IMGW - 6 && int16_x > 4 && int16_y < IMGH - 5 && int16_y >= 4 && u8_turn < 4 && int16_steps < IMGH) {
        int16 int16_localthre = 0;
        int16_dy              = -int16_half - 1;
        while (++int16_dy <= int16_half) {
            int16_dx = -int16_half - 1;
            while (++int16_dx <= int16_half) {
                int16_localthre += InImg[int16_y + int16_dy][int16_x + int16_dx];
            }
        }

        int16_localthre /= (blocksize * blocksize);
        /*阈值补偿*/
        int16_localthre -= threclip;

        int16 frontlevel      = InImg[int16_y + arr_frontdir[u8_dir][0]][int16_x + arr_frontdir[u8_dir][1]];
        int16 frontrightlevel = InImg[int16_y + arr_frontrightdir[u8_dir][0]][int16_x + arr_frontrightdir[u8_dir][1]];

        if (frontlevel < int16_localthre) // 前方小于阈值，往左
        {
            u8_turn++;
            u8_dir = (u8_dir + 3) % 4;
        } else if (frontrightlevel < int16_localthre) // 前方大于阈值并且右前方小于阈值，往前
        {
            int16_y += arr_frontdir[u8_dir][0];
            int16_x += arr_frontdir[u8_dir][1];

            pointDirection[int16_steps] = u8_dir + 1;

            point[int16_steps].m_i16y = int16_y;
            point[int16_steps].m_i16x = int16_x;

            int16_steps++;
            // t_tempPoint.m_i16y = int16_y;
            // t_tempPoint.m_i16x = int16_x;

            // point[int16_steps++] = t_tempPoint;

            u8_turn = 0;
        } else // 前方大于阈值并且左前方也大于阈值，往右前
        {
            int16_y += arr_frontrightdir[u8_dir][0];
            int16_x += arr_frontrightdir[u8_dir][1];

            // t_tempPoint.m_i16y = int16_y;
            // t_tempPoint.m_i16x = int16_x;
            pointDirection[int16_steps] = u8_dir + 5;
            // point[int16_steps++] = t_tempPoint;

            point[int16_steps].m_i16y = int16_y;
            point[int16_steps].m_i16x = int16_x;
            int16_steps++;
            u8_dir  = (u8_dir + 1) % 4;
            u8_turn = 0;
        }
    }

    *num = int16_steps;
}

/*边线重新压栈*/
void BorderLineReCoor(INT_POINT_INFO PointIN[], int16 PointNum, int16 PointOUT[])
{
    int16 int16_i = -1;
    memset(PointOUT, 0, sizeof(PointOUT[0]) * IMGH);

    if (PointNum < 25) return;

    while (++int16_i < PointNum - 1) {
        // 一行取一个点
        if (PointOUT[PointIN[int16_i].m_i16y] == 0) {
            PointOUT[PointIN[int16_i].m_i16y] = PointIN[int16_i].m_i16x;
        }
    }
}

/*赛道宽度计算*/
void BorderWidth_Calc(TRACK_BORDER_INFO *p_Border, int16 PointLeft[], int16 PointRight[])
{
    /*初始化赛道宽度*/
    p_Border->WidthRow100 = p_Border->WidthRow90 = p_Border->WidthRow80 = p_Border->WidthRow70 = p_Border->WidthRow60 = p_Border->WidthRow50 = p_Border->WidthRow40 = p_Border->WidthRow30 = 0;

    if (PointLeft[100] != 0 && PointRight[100] != 0) p_Border->WidthRow100 = PointRight[100] - PointLeft[100];
    if (PointLeft[90] != 0 && PointRight[90] != 0) p_Border->WidthRow90 = PointRight[90] - PointLeft[90];
    if (PointLeft[80] != 0 && PointRight[80] != 0) p_Border->WidthRow80 = PointRight[80] - PointLeft[80];
    if (PointLeft[70] != 0 && PointRight[70] != 0) p_Border->WidthRow70 = PointRight[70] - PointLeft[70];
    if (PointLeft[60] != 0 && PointRight[60] != 0) p_Border->WidthRow60 = PointRight[60] - PointLeft[60];
    if (PointLeft[50] != 0 && PointRight[50] != 0) p_Border->WidthRow50 = PointRight[50] - PointLeft[50];
    if (PointLeft[40] != 0 && PointRight[40] != 0) p_Border->WidthRow40 = PointRight[40] - PointLeft[40];
    if (PointLeft[30] != 0 && PointRight[30] != 0) p_Border->WidthRow30 = PointRight[30] - PointLeft[30];
}

/*逆透视打表变化*/
void Inverse_Perspective(INT_POINT_INFO PointIN[], INT_POINT_INFO PointOUT[], int16 PointNum)
{
    int16 int16_i;

    int16_i = -1;
    // int num=113;
    memset(PointOUT, 0, sizeof(PointOUT[0]) * PointNum);

    while (++int16_i < PointNum) {
        // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> lx=%d.", PointIN[int16_i].m_i16x);
        // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> ly=%d.", PointIN[int16_i].m_i16y);
        // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> cnt=%d.", PointNum);
        // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> num=%d.", num);

        PointOUT[int16_i].m_i16x = Inv_x[PointIN[int16_i].m_i16y * 188 + PointIN[int16_i].m_i16x];
        PointOUT[int16_i].m_i16y = Inv_y[PointIN[int16_i].m_i16y * 188 + PointIN[int16_i].m_i16x];
        // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> invx=%d.", Inv_x[ pn ]);
        // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> invy=%d.", Inv_y[ pn ]);
        // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> out=%d.", PointOUT[int16_i].m_i16x);
        // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> out=%d.", PointOUT[int16_i].m_i16y);
        // --num;
        // unsigned char length;
        // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> inv=%s.", out_float(PointOUT[int16_i].m_i16x, 4, &length));
    }
}

/*边线滤波*/
void Points_Blur(INT_POINT_INFO PointIN[], FLOAT_POINT_INFO PointOUT[], int16 PointNum, int16 kernelSize)
{
    int16 int16_i, int16_j;
    int16 int16_half = kernelSize / 2;

    memset(PointOUT, 0, sizeof(PointOUT[0]) * PointNum); // 初始化滤波后的数组

    int16_i = -1;
    while (++int16_i < PointNum) {
        PointOUT[int16_i].m_i16x = 0;
        PointOUT[int16_i].m_i16y = 0;
        // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> lx=%d.", PointIN[int16_i].m_i16x);
        // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> ly=%d.", PointIN[int16_i].m_i16y);
        int16_j = -int16_half - 1;
        while (++int16_j <= int16_half) {
            PointOUT[int16_i].m_i16x += PointIN[clip((int16_i + int16_j), 0, PointNum - 1)].m_i16x * (int16_half + 1 - Fabs(int16_j));
            PointOUT[int16_i].m_i16y += PointIN[clip((int16_i + int16_j), 0, PointNum - 1)].m_i16y * (int16_half + 1 - Fabs(int16_j));
        }

        PointOUT[int16_i].m_i16x /= (2 * int16_half + 2) * (int16_half + 1) / 2;
        PointOUT[int16_i].m_i16y /= (2 * int16_half + 2) * (int16_half + 1) / 2;
        // unsigned char length;
        // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> bx=%s.", out_float(PointOUT[int16_i].m_i16x, 4, &length));
        // // unsigned char length;
        // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> djx=%s.", out_float(PointOUT[int16_i].m_i16x, 4, &length));
        // fprintf("bxx=%f", &PointOUT[int16_i].m_i16x);
        // fprintf("bxy=%f", &PointOUT[int16_i].m_i16y);
        // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> bxx=%f.", PointOUT[int16_i].m_i16x);
        // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> bxy=%f.", PointOUT[int16_i].m_i16y);
    }
}

/*等距离采样*/
void Points_Resample(FLOAT_POINT_INFO PointIN[], FLOAT_POINT_INFO PointOUT[], int16 PointINNum, int16 *PointOUTNum, float32 dist) // Dist = pixel per meter * 0.02m
{
    memset(PointOUT, 0, sizeof(PointOUT[0]) * *PointOUTNum); // 初始化采样后数组
    *PointOUTNum = 0;

    /*边线有点*/
    if (PointINNum > 0) {
        int16 int16_len        = 0;
        float32 f32_LineLength = 0;

        int16 int16_loopi = -1;
        while (++int16_loopi < PointINNum - 1 && int16_len < IMGH) {

            float32 f32_x0 = PointIN[int16_loopi].m_i16x;
            float32 f32_y0 = PointIN[int16_loopi].m_i16y;
            // char c[120]    = {"\0"};
            // char *str1     = gcvt(f32_x0, 4, c);
            // unsigned char length;
            // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> djx=%s.", out_float(f32_x0, 4, &length));
            // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> djy=%lf.", f32_y0);

            float32 f32_dx0 = PointIN[int16_loopi + 1].m_i16x - f32_x0;
            float32 f32_dy0 = PointIN[int16_loopi + 1].m_i16y - f32_y0;

            float32 f32_dist0 = FSqrt(f32_dx0 * f32_dx0 + f32_dy0 * f32_dy0); // 勾股定理开方

            // 线性插值
            f32_dx0 /= f32_dist0;
            f32_dy0 /= f32_dist0;

            while (f32_LineLength < f32_dist0 && int16_len < IMGH) {
                f32_x0 += f32_dx0 * f32_LineLength;
                PointOUT[int16_len].m_i16x = f32_x0;
                f32_y0 += f32_dy0 * f32_LineLength;
                PointOUT[int16_len].m_i16y = f32_y0;
                // unsigned char length;
                // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> bx=%s.", out_float(f32_x0, 4, &length));

                int16_len++;
                f32_dist0 -= f32_LineLength;
                f32_LineLength = dist;
            }

            f32_LineLength -= f32_dist0;
        }

        *PointOUTNum = int16_len;
    }
}

/*边线角度变化率*/
void Border_Local_Angle(FLOAT_POINT_INFO PointIN[], float32 AngleOUT[], int16 PointNum, int16 len)
{
    int16 int16_loopi;

    int16_loopi = -1;

    memset((void *)AngleOUT, 0, sizeof(AngleOUT[0]) * PointNum);

    while (++int16_loopi < PointNum) {

        if (int16_loopi <= 0 || int16_loopi >= PointNum - 1) {
            AngleOUT[int16_loopi] = 0;
            continue;
        }

        float32 f32_dx1 = PointIN[int16_loopi].m_i16x - PointIN[clip(int16_loopi - len, 0, PointNum - 1)].m_i16x; // 与前面点的横竖距离以及斜边长度
        float32 f32_dy1 = PointIN[int16_loopi].m_i16y - PointIN[clip(int16_loopi - len, 0, PointNum - 1)].m_i16y;
        float32 f32_dz1 = FSqrt(f32_dx1 * f32_dx1 + f32_dy1 * f32_dy1);

        float32 f32_dx2 = PointIN[clip(int16_loopi + len, 0, PointNum - 1)].m_i16x - PointIN[int16_loopi].m_i16x; // 与后面点的横竖距离以及斜边长度
        float32 f32_dy2 = PointIN[clip(int16_loopi + len, 0, PointNum - 1)].m_i16y - PointIN[int16_loopi].m_i16y;
        float32 f32_dz2 = FSqrt(f32_dx2 * f32_dx2 + f32_dy2 * f32_dy2);

        // 除数不为0
        if (f32_dz1 == 0 || f32_dz2 == 0) continue;
        float32 f32_cos1 = f32_dx1 / f32_dz1;
        float32 f32_sin1 = f32_dy1 / f32_dz1;

        float32 f32_cos2 = f32_dx2 / f32_dz2;
        float32 f32_sin2 = f32_dy2 / f32_dz2;

        if (f32_cos1 * f32_sin2 - f32_cos2 * f32_sin1 == 0 || f32_cos1 * f32_cos2 + f32_sin1 * f32_sin2 == 0) continue;
        AngleOUT[int16_loopi] = atan2f(f32_cos1 * f32_sin2 - f32_cos2 * f32_sin1, f32_cos1 * f32_cos2 + f32_sin1 * f32_sin2);
    }
}

/*非极大值抑制*/
void Angle_NMS(float32 AngleIN[], float32 AngleOUT[], int16 AngleNum, int16 kernelSize)
{
    int16 int16_half = kernelSize / 2;

    int16 int16_loopi, int16_loopj;

    memset((void *)AngleOUT, 0, sizeof(AngleOUT[0]) * AngleNum);

    int16_loopi = -1;
    while (++int16_loopi < AngleNum) {
        AngleOUT[int16_loopi] = AngleIN[int16_loopi];
        int16_loopj           = -int16_half - 1;
        while (++int16_loopj <= int16_half) {
            if (Fabs(AngleIN[clip(int16_loopi + int16_loopj, 0, AngleNum - 1)]) > Fabs(AngleOUT[int16_loopi])) {
                AngleOUT[int16_loopi] = 0;
                break;
            }
        }
    }
}

/*左边线跟踪中线*/
void LeftBorderTrackingCenter(FLOAT_POINT_INFO LeftPoint[], FLOAT_POINT_INFO CenterPoint[], int16 PointNum, int16 InterPoint, float32 dist)
{
    int16 int16_loopi;

    memset(CenterPoint, 0, sizeof(CenterPoint[0]) * IMGH);

    int16_loopi = -1;

    while (++int16_loopi < PointNum) {

        // 根据前后两点连接的斜率近似该点切线的斜率，沿斜率的法线方向平移赛道的半宽得到中线
        float32 f32_dx = LeftPoint[clip(int16_loopi + InterPoint, 0, PointNum - 1)].m_i16x - LeftPoint[clip(int16_loopi - InterPoint, 0, PointNum - 1)].m_i16x;
        float32 f32_dy = LeftPoint[clip(int16_loopi + InterPoint, 0, PointNum - 1)].m_i16y - LeftPoint[clip(int16_loopi - InterPoint, 0, PointNum - 1)].m_i16y;
        // unsigned char length;
        // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> djx=%s.", out_float(LeftPoint[clip(int16_loopi + InterPoint, 0, PointNum - 1)].m_i16x, 4, &length));

        float32 f32_dz = FSqrt(f32_dx * f32_dx + f32_dy * f32_dy);

        if (f32_dz == 0) continue;

        f32_dx = f32_dx / f32_dz;
        f32_dy = f32_dy / f32_dz;

        CenterPoint[int16_loopi].m_i16x = LeftPoint[int16_loopi].m_i16x - f32_dy * dist;
        CenterPoint[int16_loopi].m_i16y = LeftPoint[int16_loopi].m_i16y + f32_dx * dist;
        // unsigned char length;
        // SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n LOG -> djx=%s.", out_float(CenterPoint[int16_loopi].m_i16x, 4, &length));
    }
}

/*右边线跟踪中线*/
void RightBorderTrackingCenter(FLOAT_POINT_INFO RightPoint[], FLOAT_POINT_INFO CenterPoint[], int16 PointNum, int16 InterPoint, float32 dist)
{
    int16 int16_loopi;

    memset(CenterPoint, 0, sizeof(CenterPoint[0]) * IMGH);

    int16_loopi = -1;
    while (++int16_loopi < PointNum) {
        // 根据前后两点连接的斜率近似该点切线的斜率，沿斜率的法线方向平移赛道的半宽得到中线
        float32 f32_dx = RightPoint[clip(int16_loopi + InterPoint, 0, PointNum - 1)].m_i16x - RightPoint[clip(int16_loopi - InterPoint, 0, PointNum - 1)].m_i16x;
        float32 f32_dy = RightPoint[clip(int16_loopi + InterPoint, 0, PointNum - 1)].m_i16y - RightPoint[clip(int16_loopi - InterPoint, 0, PointNum - 1)].m_i16y;

        float32 f32_dz = FSqrt(f32_dx * f32_dx + f32_dy * f32_dy);

        if (f32_dz == 0) continue;

        f32_dx = f32_dx / f32_dz;
        f32_dy = f32_dy / f32_dz;

        CenterPoint[int16_loopi].m_i16x = RightPoint[int16_loopi].m_i16x + f32_dy * dist;
        CenterPoint[int16_loopi].m_i16y = RightPoint[int16_loopi].m_i16y - f32_dx * dist;
    }
}

unsigned char *out_float(double value, unsigned char decimal_digit, unsigned char *output_length)
{
    unsigned char _output[20];
    unsigned long integer;
    unsigned long decimal;
    unsigned char _output_length = 0;
    unsigned char _length_buff   = 0;
    static unsigned char *return_pointer;
    unsigned char signal_flag;
    if (value < 0)
        signal_flag = 1;
    else
        signal_flag = 0;
    value   = fabs(value);
    integer = (unsigned long)value;
    decimal = (unsigned long)((value - integer) * pow(10, decimal_digit));

    unsigned long integer_buff = integer;
    unsigned long decimal_buff = decimal;

    while (1) {
        if (integer / 10 != 0)
            _length_buff++;
        else {
            _length_buff++;
            break;
        }
        integer = integer / 10;
    }
    for (int i = 0; i < _length_buff; i++) {
        if (i == _length_buff - 1)
            _output[_output_length] = integer_buff % 10 + 0x30;
        else {
            //_output[_output_length] = integer_buff / 10 % 10 + 0x30;
            _output[_output_length] = integer_buff / (unsigned long)pow(10, _length_buff - i - 1) % 10 + 0x30;
            integer_buff            = integer_buff % (unsigned long)pow(10, _length_buff - i - 1);
            // integer_buff = integer_buff % 10;
        }
        _output_length++;
    }
    _output[_output_length] = '.';
    _output_length++;
    _length_buff = 0;
    while (1) {
        if (decimal / 10 != 0)
            _length_buff++;
        else {
            _length_buff++;
            break;
        }
        decimal = decimal / 10;
    }
    for (int i = 0; i < _length_buff; i++) {
        if (i == _length_buff - 1)
            _output[_output_length] = decimal_buff % 10 + 0x30;
        else {
            _output[_output_length] = decimal_buff / (unsigned long)pow(10, _length_buff - i - 1) % 10 + 0x30;
            decimal_buff            = decimal_buff % (unsigned long)pow(10, _length_buff - i - 1);
        }

        _output_length++;
    }
    _output[_output_length] = 0x00;
    _output_length++;
    return_pointer = (unsigned char *)realloc(return_pointer, _output_length);

    *output_length = _output_length - 1;
    if (return_pointer == 0)
        return 0;
    else {
        if (signal_flag == 1) {
            return_pointer[0] = '-';
            memcpy(return_pointer + 1, _output, _output_length);
        } else
            memcpy(return_pointer, _output, _output_length);
    }
    return return_pointer;
}

/*寻找拐点*/
void FindCorner(TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_Type)
{
    /*直道点数个数需要大于一定值*/
    p_Type->m_u8LeftLineStraightFlag  = p_Border->m_i16LPointCntRS > STRAIGHTNUMBOUND;
    p_Type->m_u8RightLineStraightFlag = p_Border->m_i16RPointCntRS > STRAIGHTNUMBOUND;

    p_Type->m_u8ShortLeftLineStraightFlag  = p_Border->m_i16LPointCntRS > SHORTSTRAIGHTNUMBOUND;
    p_Type->m_u8ShortRightLineStraightFlag = p_Border->m_i16RPointCntRS > SHORTSTRAIGHTNUMBOUND;

    p_Type->m_u8MidLeftLineStraightFlag  = p_Border->m_i16LPointCntRS > MIDSTRAIGHTNUMBOUND;
    p_Type->m_u8MidRightLineStraightFlag = p_Border->m_i16RPointCntRS > MIDSTRAIGHTNUMBOUND;

    /*初始化拐点*/
    p_Border->LL_CornerPos = p_Border->RL_CornerPos = p_Border->LY_CornerPos = p_Border->RY_CornerPos = -1;

    p_Border->LL_CornerNUM = p_Border->RL_CornerNUM = 0;

    int16 int16_Loopi;

    int16_Loopi = -1;
    while (++int16_Loopi < p_Border->m_i16LPointCntRS - 1) {
        /*如果不是局部极大值则跳过判断*/
        if (p_Border->LdAngleNMS[int16_Loopi] == 0) continue;

        int16 upper = clip(int16_Loopi + InterPoint, 0, p_Border->m_i16LPointCntRS - 1);
        int16 lower = clip(int16_Loopi - InterPoint, 0, p_Border->m_i16LPointCntRS - 1);

        float32 f32_Corn = Fabs(p_Border->LdAngle[int16_Loopi]) - (Fabs(p_Border->LdAngle[upper]) + Fabs(p_Border->LdAngle[lower])) / 2;

        if (f32_Corn > L_CORNER_LOWERBOUND && f32_Corn < L_CORNER_UPPERBOUND && int16_Loopi < 0.8 / SampleDist && p_Border->LL_CornerPos == -1) {
            p_Border->LL_CornerPos = int16_Loopi;
        }
        if (f32_Corn > Y_CORNER_LOWERBOUND && f32_Corn < Y_CORNER_UPPERBOUND && int16_Loopi < 0.5 / SampleDist && p_Border->LY_CornerPos == -1) {
            p_Border->LY_CornerPos = int16_Loopi;
        }
        if (f32_Corn > L_CORNER_LOWERBOUND) {
            p_Border->LL_CornerNUM++;
        }

        /*和阈值进行比较*/
        if (f32_Corn > STRAIGHTBOUND && int16_Loopi < 0.8 / SampleDist) p_Type->m_u8ShortLeftLineStraightFlag = 0;
        if (f32_Corn > STRAIGHTBOUND && int16_Loopi < 1. / SampleDist) p_Type->m_u8LeftLineStraightFlag = 0;
        if (f32_Corn > STRAIGHTBOUND && int16_Loopi < 1.2 / SampleDist && int16_Loopi > 0.6 / SampleDist) p_Type->m_u8MidLeftLineStraightFlag = 0;
        if (p_Border->LL_CornerPos != -1 && p_Border->LY_CornerPos != -1 && p_Type->m_u8LeftLineStraightFlag == 0) break;
    }

    int16_Loopi = -1;
    while (++int16_Loopi < p_Border->m_i16RPointCntRS - 1) {
        if (p_Border->RdAngleNMS[int16_Loopi] == 0) continue;

        int16 upper = clip(int16_Loopi + InterPoint, 0, p_Border->m_i16RPointCntRS - 1);
        int16 lower = clip(int16_Loopi - InterPoint, 0, p_Border->m_i16RPointCntRS - 1);

        float32 f32_Corn = Fabs(p_Border->RdAngle[int16_Loopi]) - (Fabs(p_Border->RdAngle[upper]) + Fabs(p_Border->RdAngle[lower])) / 2;

        if (f32_Corn > L_CORNER_LOWERBOUND && f32_Corn < L_CORNER_UPPERBOUND && int16_Loopi < 0.8 / SampleDist && p_Border->RL_CornerPos == -1) {
            p_Border->RL_CornerPos = int16_Loopi;
        }
        if (f32_Corn > Y_CORNER_LOWERBOUND && f32_Corn < Y_CORNER_UPPERBOUND && int16_Loopi < 0.5 / SampleDist && p_Border->RY_CornerPos == -1) {
            p_Border->RY_CornerPos = int16_Loopi;
        }
        if (f32_Corn > L_CORNER_LOWERBOUND) {
            p_Border->RL_CornerNUM++;
        }

        if (f32_Corn > STRAIGHTBOUND && int16_Loopi < 0.8 / SampleDist) p_Type->m_u8ShortRightLineStraightFlag = 0;
        if (f32_Corn > STRAIGHTBOUND && int16_Loopi < 1. / SampleDist) p_Type->m_u8RightLineStraightFlag = 0;
        if (f32_Corn > STRAIGHTBOUND && int16_Loopi < 1.2 / SampleDist && int16_Loopi > 0.6 / SampleDist) p_Type->m_u8MidRightLineStraightFlag = 0;

        if (p_Border->RL_CornerPos != -1 && p_Border->RY_CornerPos != -1 && p_Type->m_u8RightLineStraightFlag == 0) break;
    }

    // Y拐点二次检查
    // 根据拐点之间的距离和拐点后赛道延展的特征和y坐标插值
    if (p_Border->LY_CornerPos != -1 && p_Border->RY_CornerPos != -1) {
        float32 f32_dx = p_Border->m_LPntRS[p_Border->LY_CornerPos].m_i16x - p_Border->m_RPntRS[p_Border->RY_CornerPos].m_i16x;
        float32 f32_dy = p_Border->m_LPntRS[p_Border->LY_CornerPos].m_i16y - p_Border->m_RPntRS[p_Border->RY_CornerPos].m_i16y;
        float32 f32_dz = FSqrt(f32_dx * f32_dx + f32_dy * f32_dy);

        if (Fabs(f32_dz - 0.4 * PixelperMeter) < 0.15 * PixelperMeter) {
            float32 f32_dxf = p_Border->m_LPntRS[clip(p_Border->LY_CornerPos + 50, 0, p_Border->m_i16LPointCntRS - 1)].m_i16x - p_Border->m_RPntRS[clip(p_Border->RY_CornerPos + 50, 0, p_Border->m_i16RPointCntRS - 1)].m_i16x;
            float32 f32_dyf = p_Border->m_LPntRS[clip(p_Border->LY_CornerPos + 50, 0, p_Border->m_i16LPointCntRS - 1)].m_i16y - p_Border->m_RPntRS[clip(p_Border->RY_CornerPos + 50, 0, p_Border->m_i16RPointCntRS - 1)].m_i16y;
            float32 f32_dzf = FSqrt(f32_dxf * f32_dxf + f32_dyf * f32_dyf);

            if (!(f32_dzf > 0.7 * PixelperMeter && p_Border->m_LPntRS[clip(p_Border->LY_CornerPos + 50, 0, p_Border->m_i16LPointCntRS - 1)].m_i16x < p_Border->m_LPntRS[p_Border->LY_CornerPos].m_i16x && p_Border->m_RPntRS[clip(p_Border->RY_CornerPos + 50, 0, p_Border->m_i16RPointCntRS - 1)].m_i16x > p_Border->m_RPntRS[p_Border->RY_CornerPos].m_i16x)) {
                p_Border->LY_CornerPos = p_Border->RY_CornerPos = -1;
            }
        } else {
            p_Border->LY_CornerPos = p_Border->RY_CornerPos = -1;
        }
    }
    // 根据拐点后赛道延展的特征和y坐标差值
    else if (p_Border->LY_CornerPos != -1) {

        p_Border->LY_CornerPos = -1;
    }
    // 根据拐点后赛道延展的特征
    else if (p_Border->RY_CornerPos != -1) {

        p_Border->RY_CornerPos = -1;
    }

    // L拐点二次检查
    // 根据L拐点的间的距离和拐点后赛道延展的特征
    // if (p_Type->m_u8GarageFlag == GARAGE_NONE) {
    if (p_Border->LL_CornerPos != -1 && p_Border->RL_CornerPos != -1) {
        float32 f32_dx = p_Border->m_LPntRS[p_Border->LL_CornerPos].m_i16x - p_Border->m_RPntRS[p_Border->RL_CornerPos].m_i16x;
        float32 f32_dy = p_Border->m_LPntRS[p_Border->LL_CornerPos].m_i16y - p_Border->m_RPntRS[p_Border->RL_CornerPos].m_i16y;
        float32 f32_dz = FSqrt(f32_dx * f32_dx + f32_dy * f32_dy);

        if (Fabs(f32_dz - 0.4 * PixelperMeter) < 0.15 * PixelperMeter) {

            if (!(/*f32_dzf > 0.7 * PixelperMeter &&*/ p_Border->m_LPntRS[clip(p_Border->LL_CornerPos + 50, 0, p_Border->m_i16LPointCntRS - 1)].m_i16x < p_Border->m_LPntRS[p_Border->LL_CornerPos].m_i16x && p_Border->m_RPntRS[clip(p_Border->RL_CornerPos + 50, 0, p_Border->m_i16RPointCntRS - 1)].m_i16x > p_Border->m_RPntRS[p_Border->RL_CornerPos].m_i16x)) {
                p_Border->LL_CornerPos = p_Border->RL_CornerPos = -1;
            }
        } else {
            p_Border->LL_CornerPos = p_Border->RL_CornerPos = -1;
        }

    }
    /*根据L拐点后赛道延展的特征*/
    else if (p_Border->LL_CornerPos != -1) {
        if (!(p_Border->m_LPntRS[clip(p_Border->LL_CornerPos + 50, 0, p_Border->m_i16LPointCntRS - 1)].m_i16x < p_Border->m_LPntRS[p_Border->LL_CornerPos].m_i16x)) p_Border->LL_CornerPos = -1;

    }
    /*根据L拐点后赛道延展的特征*/
    else if (p_Border->RL_CornerPos != -1) {
        if (!(p_Border->m_RPntRS[clip(p_Border->RL_CornerPos + 50, 0, p_Border->m_i16RPointCntRS - 1)].m_i16x > p_Border->m_RPntRS[p_Border->RL_CornerPos].m_i16x)) p_Border->RL_CornerPos = -1;
    }
    // }
}

/*获取预瞄距离*/
void GetAimingDist(TRACK_BORDER_INFO *p_Border, LINE_ERROR_INFO *p_Error, TRACK_TYPE_INFO *p_Type)
{
    SpeedThres    = normal_speed * 0.85;
    SpeedDistance = Avg_speed < SpeedThres ? Avg_speed : (now_speed > 130 ? (float32)normal_speed : now_speed); // 速度控制

    if (SpeedDistance >= 150)
        SpeedDistance += 25;
    else if (SpeedDistance >= 140)
        SpeedDistance += 25;

    /*-------------------------------------------------------------左边线----------------------------------------------------------------------------------*/

    // 十字
    if (p_Type->m_u8CrossFlag != CROSS_NONE) {
        if (p_Type->m_u8CrossFlag == CROSS_FAR) {
            if (p_Border->LL_CornerPos != -1) {
                p_Error->m_f32LeftBorderAimingMin = fclip(0.1, 0., (p_Border->LL_CornerPos - 9) * SampleDist);
                p_Error->m_f32LeftBorderAimingMax = fclip(0.2, 0., (p_Border->LL_CornerPos - 7) * SampleDist);
            }
            p_Error->m_u8TackingType = TRACKINGBOTH;
        }

        else if (p_Type->m_u8CrossFlag == CROSS_NEAR) {
            if (p_Border->LL_CornerPosRemote != -1) {
                p_Error->m_f32LeftBorderAimingMin = fclip(0.6, (p_Border->LL_CornerPosRemote + 7) * SampleDist, (p_Border->m_i16LCnterCntRSRemote - 1) * SampleDist);
                p_Error->m_f32LeftBorderAimingMax = fclip(0.7, (p_Border->LL_CornerPosRemote + 10) * SampleDist, (p_Border->m_i16LCnterCntRSRemote - 1) * SampleDist);
            }
            p_Error->m_u8TackingType = TRACKINGBOTH;
        }
    }

    // 右斜入十字
    else if (p_Type->m_u8RightSideCrossFlag != CROSS_NONE) {
        if (p_Type->m_u8RightSideCrossFlag == CROSS_FAR) {
            if (p_Border->LL_CornerPos != -1) {
                p_Error->m_f32LeftBorderAimingMin = fclip(0.1, 0., (p_Border->LL_CornerPos - 9) * SampleDist);
                p_Error->m_f32LeftBorderAimingMax = fclip(0.2, 0., (p_Border->LL_CornerPos - 7) * SampleDist);
            }
            p_Error->m_u8TackingType = TRACKINGRIGHT;
        }

        else if (p_Type->m_u8RightSideCrossFlag == CROSS_NEAR) {
            if (p_Border->LL_CornerPosRemote != -1) {
                p_Error->m_f32LeftBorderAimingMin = fclip(0.6, (p_Border->LL_CornerPosRemote + 7) * SampleDist, (p_Border->m_i16LCnterCntRSRemote - 1) * SampleDist);
                p_Error->m_f32LeftBorderAimingMax = fclip(0.7, (p_Border->LL_CornerPosRemote + 10) * SampleDist, (p_Border->m_i16LCnterCntRSRemote - 1) * SampleDist);
            }
            p_Error->m_u8TackingType = TRACKINGBOTH;
        }
    }

    // 左斜入十字
    else if (p_Type->m_u8LeftSideCrossFlag != CROSS_NONE) {
        if (p_Type->m_u8LeftSideCrossFlag == CROSS_FAR) {
            if (p_Border->LL_CornerPos != -1) {
                p_Error->m_f32LeftBorderAimingMin = fclip(0.1, 0., (p_Border->LL_CornerPos - 9) * SampleDist);
                p_Error->m_f32LeftBorderAimingMax = fclip(0.2, 0., (p_Border->LL_CornerPos - 7) * SampleDist);
            }
            p_Error->m_u8TackingType = TRACKINGLEFT;
        }

        else if (p_Type->m_u8LeftSideCrossFlag == CROSS_NEAR) {
            if (p_Border->LL_CornerPosRemote != -1) {
                p_Error->m_f32LeftBorderAimingMin = fclip(0.6, (p_Border->LL_CornerPosRemote + 7) * SampleDist, (p_Border->m_i16LCnterCntRSRemote - 1) * SampleDist);
                p_Error->m_f32LeftBorderAimingMax = fclip(0.7, (p_Border->LL_CornerPosRemote + 10) * SampleDist, (p_Border->m_i16LCnterCntRSRemote - 1) * SampleDist);
            }
            p_Error->m_u8TackingType = TRACKINGBOTH;
        }
    }
    // 无特殊元素弯道
    else if (p_Type->m_u8ShortLeftLineStraightFlag == 0) {
        //        p_Error ->m_f32LeftBorderAimingMin = 0.2;
        p_Error->m_f32LeftBorderAimingMin = fclip(((int16)(now_speed / 5.0 + 0.5)) * 5, 90, 130) * 0.004 - 0.24;
        p_Error->m_f32LeftBorderAimingMax = p_Error->m_f32LeftBorderAimingMin + 0.1;

        p_Error->m_u8TackingType = TRACKINGBOTH;
    }

    // 无特殊元素直道
    else {
        p_Error->m_f32LeftBorderAimingMin = fclip(((int16)(now_speed / 5.0 + 0.5)) * 5, 130, 145) * 0.004 - 0.22;
        p_Error->m_f32LeftBorderAimingMax = p_Error->m_f32LeftBorderAimingMin + 0.1;
        p_Error->m_u8TackingType          = TRACKINGBOTH;
    }

    /*-------------------------------------------------------------右边线----------------------------------------------------------------------------------*/

    // 十字
    if (p_Type->m_u8CrossFlag != CROSS_NONE) {
        if (p_Type->m_u8CrossFlag == CROSS_FAR) {
            if (p_Border->RL_CornerPos != -1) {
                p_Error->m_f32RightBorderAimingMin = fclip(0.1, 0., (p_Border->RL_CornerPos - 9) * SampleDist);
                p_Error->m_f32RightBorderAimingMax = fclip(0.2, 0., (p_Border->RL_CornerPos - 7) * SampleDist);
            }
        }

        else if (p_Type->m_u8CrossFlag == CROSS_NEAR) {
            if (p_Border->RL_CornerPosRemote != -1) {
                p_Error->m_f32RightBorderAimingMin = fclip(0.6, (p_Border->RL_CornerPosRemote + 7) * SampleDist, (p_Border->m_i16RCnterCntRSRemote - 1) * SampleDist);
                p_Error->m_f32RightBorderAimingMax = fclip(0.7, (p_Border->RL_CornerPosRemote + 10) * SampleDist, (p_Border->m_i16RCnterCntRSRemote - 1) * SampleDist);
            }
        }
    }

    // 右斜入十字
    else if (p_Type->m_u8RightSideCrossFlag != CROSS_NONE) {
        if (p_Type->m_u8RightSideCrossFlag == CROSS_FAR) {
            if (p_Border->RL_CornerPos != -1) {
                p_Error->m_f32RightBorderAimingMin = fclip(0.1, 0., (p_Border->RL_CornerPos - 9) * SampleDist);
                p_Error->m_f32RightBorderAimingMax = fclip(0.2, 0., (p_Border->RL_CornerPos - 7) * SampleDist);
            }
        }

        else if (p_Type->m_u8RightSideCrossFlag == CROSS_NEAR) {
            if (p_Border->RL_CornerPosRemote != -1) {
                p_Error->m_f32RightBorderAimingMin = fclip(0.6, (p_Border->RL_CornerPosRemote + 7) * SampleDist, (p_Border->m_i16RCnterCntRSRemote - 1) * SampleDist);
                p_Error->m_f32RightBorderAimingMax = fclip(0.7, (p_Border->RL_CornerPosRemote + 10) * SampleDist, (p_Border->m_i16RCnterCntRSRemote - 1) * SampleDist);
            }
        }
    }

    // 左斜入十字
    else if (p_Type->m_u8LeftSideCrossFlag != CROSS_NONE) {
        if (p_Type->m_u8LeftSideCrossFlag == CROSS_FAR) {
            if (p_Border->RL_CornerPos != -1) {
                p_Error->m_f32RightBorderAimingMin = fclip(0.1, 0., (p_Border->RL_CornerPos - 9) * SampleDist);
                p_Error->m_f32RightBorderAimingMax = fclip(0.2, 0., (p_Border->RL_CornerPos - 7) * SampleDist);
            }
        }

        else if (p_Type->m_u8LeftSideCrossFlag == CROSS_NEAR) {
            if (p_Border->RL_CornerPosRemote != -1) {
                p_Error->m_f32RightBorderAimingMin = fclip(0.6, (p_Border->RL_CornerPosRemote + 7) * SampleDist, (p_Border->m_i16RCnterCntRSRemote - 1) * SampleDist);
                p_Error->m_f32RightBorderAimingMax = fclip(0.7, (p_Border->RL_CornerPosRemote + 10) * SampleDist, (p_Border->m_i16RCnterCntRSRemote - 1) * SampleDist);
            }
        }
    }
    // 非特殊元素弯道
    else if (p_Type->m_u8ShortRightLineStraightFlag == 0) {
        //        p_Error ->m_f32RightBorderAimingMin = 0.2;
        p_Error->m_f32RightBorderAimingMin = fclip(((int16)(now_speed / 5.0 + 0.5)) * 5, 90, 130) * 0.004 - 0.24;
        p_Error->m_f32RightBorderAimingMax = p_Error->m_f32RightBorderAimingMin + 0.1;
    }
    // 非特殊元素直道
    else {
        p_Error->m_f32RightBorderAimingMin = fclip(((int16)(now_speed / 5.0 + 0.5)) * 5, 130, 145) * 0.004 - 0.22;
        p_Error->m_f32RightBorderAimingMax = p_Error->m_f32RightBorderAimingMin + 0.1;
    }
}

/*纯跟踪计算曲率*/

void PurePursuit(TRACK_BORDER_INFO *p_Border, LINE_ERROR_INFO *p_Error, TRACK_TYPE_INFO *p_Type)
{
    int16 int16_i    = clip((int16)(p_Error->m_f32LeftBorderAimingMin / SampleDist), 0, p_Border->m_i16LCnterCntRS - 1);
    int16 int16_iEnd = clip((int16)(p_Error->m_f32LeftBorderAimingMax / SampleDist), 0, p_Border->m_i16LCnterCntRS - 1);
    float32 dx, dy, VerticalDist, HoriError, Kappa, Kappatotal;
    float32 gain;
    int16 norm = 0;
    if ((p_Error->m_u8TackingType == TRACKINGBOTH || p_Error->m_u8TackingType == TRACKINGLEFT) && p_Type->m_u8CrossFlag != CROSS_NEAR && p_Type->m_u8RightSideCrossFlag != CROSS_NEAR && p_Type->m_u8LeftSideCrossFlag != CROSS_NEAR /* && p_Type->m_u8LeftPRoadFlag != PROAD_END && p_Type->m_u8RightPRoadFlag != PROAD_END && p_Type->m_u8GarageFlag != GARAGE_RIGHT_TURN && p_Type->m_u8GarageFlag != GARAGE_LEFT_TURN && !(p_Type->m_u8GarageFlag == OUT_GARAGE && p_Type->m_u8GarageTracking == Garage_Tracking_Remote)*/) {
        while (int16_i++ < int16_iEnd) {
            if (p_Border->m_LCPntRS[int16_i].m_i16y != 0) {
                dx = p_Border->m_LCPntRS[int16_i].m_i16x - CenterX;
                dy = Fabs(p_Border->m_LCPntRS[int16_i].m_i16y - CenterY);

                VerticalDist = dy / PixelperMeter + 0.243;
                HoriError    = dx / PixelperMeter;

                Kappa = 2 * HoriError / (VerticalDist * VerticalDist + HoriError * HoriError);

                if (Fabs(Kappa) > 1) {
                    if (dx == 0 || Avg_speed == 0) gain = 0;
                    //                    else gain = (float32)atan((double)dx * 1000 / Avg_speed *200 / EncoderPerMeter);
                } else {
                    gain = 0;
                }

                Kappa += gain;

                norm += 1;
                Kappatotal += Kappa;
            }
        }

        if (norm != 0) {
            p_Error->m_f32LeftBorderKappa = Kappatotal / norm;
            p_Error->m_u8LeftCenterValid  = 1;

            if (p_Type->m_u8CrossFlag == CROSS_FAR && p_Border->LL_CornerPos == -1) p_Error->m_u8LeftCenterValid = 0;

            // if( (p_Type ->m_u8GarageFlag == GARAGE_RIGHT_PASS || p_Type ->m_u8GarageFlag == GARAGE_RIGHT_IN) && p_Border -> m_i16LPointCntRS < 60) p_Error -> m_u8LeftCenterValid = 0;

        } else {
            p_Error->m_u8LeftCenterValid = 0;
        }
    }

    else if (p_Type->m_u8CrossFlag == CROSS_NEAR || p_Type->m_u8RightSideCrossFlag == CROSS_NEAR || p_Type->m_u8LeftSideCrossFlag == CROSS_NEAR /*|| p_Type->m_u8LeftPRoadFlag == PROAD_END || p_Type->m_u8GarageFlag == GARAGE_LEFT_TURN || (p_Type->m_u8GarageFlag == OUT_GARAGE && p_Type->m_u8GarageDirection == Garage_Out_RIGHT && p_Type->m_u8GarageTracking == Garage_Tracking_Remote)*/) {
        int16_i    = clip((int16)(p_Error->m_f32LeftBorderAimingMin / SampleDist), 0, p_Border->m_i16LCnterCntRSRemote - 1);
        int16_iEnd = clip((int16)(p_Error->m_f32LeftBorderAimingMax / SampleDist), 0, p_Border->m_i16LCnterCntRSRemote - 1);

        while (int16_i++ < int16_iEnd) {
            if (p_Border->m_LCPntRSRemote[int16_i].m_i16y != 0) {
                dx = p_Border->m_LCPntRSRemote[int16_i].m_i16x - CenterX;
                dy = Fabs(p_Border->m_LCPntRSRemote[int16_i].m_i16y - CenterY);

                VerticalDist = dy / PixelperMeter + 0.243;
                HoriError    = dx / PixelperMeter;

                Kappa = 2 * HoriError / (VerticalDist * VerticalDist + HoriError * HoriError);

                if (Fabs(Kappa) > 1) {
                    if (dx == 0 || Avg_speed == 0) gain = 0;
                    //                    else gain = (float32)atan((double)dx * 1000 / Avg_speed *200 / EncoderPerMeter);
                } else {
                    gain = 0;
                }

                Kappa += gain;

                norm += 1;
                Kappatotal += Kappa;
            }
        }

        if (norm != 0) {
            p_Error->m_f32LeftBorderKappa = Kappatotal / norm;
            p_Error->m_u8LeftCenterValid  = 1;
            // if (p_Border->LL_CornerPosRemote == -1 && p_Type->m_u8LeftPRoadFlag != PROAD_END && p_Type->m_u8GarageFlag != OUT_GARAGE) p_Error->m_u8LeftCenterValid = 0;
        } else {
            p_Error->m_u8LeftCenterValid = 0;
        }
    }

    else {
        p_Error->m_f32LeftBorderKappa = 0;
        p_Error->m_u8LeftCenterValid  = 0;
    }

    int16_i    = clip((int16)(p_Error->m_f32RightBorderAimingMin / SampleDist), 0, p_Border->m_i16RCnterCntRS - 1);
    int16_iEnd = clip((int16)(p_Error->m_f32RightBorderAimingMax / SampleDist), 0, p_Border->m_i16RCnterCntRS - 1);
    norm = Kappatotal = 0;

    if ((p_Error->m_u8TackingType == TRACKINGBOTH || p_Error->m_u8TackingType == TRACKINGRIGHT) && p_Type->m_u8CrossFlag != CROSS_NEAR && p_Type->m_u8RightSideCrossFlag != CROSS_NEAR && p_Type->m_u8LeftSideCrossFlag != CROSS_NEAR /*&& p_Type->m_u8LeftPRoadFlag != PROAD_END && p_Type->m_u8RightPRoadFlag != PROAD_END && p_Type->m_u8GarageFlag != GARAGE_RIGHT_TURN && p_Type->m_u8GarageFlag != GARAGE_LEFT_TURN && !(p_Type->m_u8GarageFlag == OUT_GARAGE && p_Type->m_u8GarageTracking == Garage_Tracking_Remote)*/) {
        while (int16_i++ < int16_iEnd) {
            if (p_Border->m_RCPntRS[int16_i].m_i16y != 0) {
                dx = p_Border->m_RCPntRS[int16_i].m_i16x - CenterX;
                dy = Fabs(p_Border->m_RCPntRS[int16_i].m_i16y - CenterY);

                VerticalDist = dy / PixelperMeter + 0.243;
                HoriError    = dx / PixelperMeter;

                Kappa = 2 * HoriError / (VerticalDist * VerticalDist + HoriError * HoriError);

                if (Fabs(Kappa) > 1) {
                    if (dx == 0 || Avg_speed == 0) gain = 0;
                    //                    else gain = (float32)atan((double)dx * 1000 / Avg_speed *200 / EncoderPerMeter);
                } else {
                    gain = 0;
                }

                Kappa += gain;

                norm += 1;
                Kappatotal += Kappa;
            }
        }

        if (norm != 0) {
            p_Error->m_f32RightBorderKappa = Kappatotal / norm;
            p_Error->m_u8RightCenterValid  = 1;

            if (p_Type->m_u8CrossFlag == CROSS_FAR && p_Border->RL_CornerPos == -1) p_Error->m_u8RightCenterValid = 0;

            // if( (p_Type ->m_u8GarageFlag == GARAGE_LEFT_PASS || p_Type ->m_u8GarageFlag == GARAGE_LEFT_IN )&& p_Border ->m_i16RPointCntRS < 60) p_Error -> m_u8RightCenterValid = 0;

        } else {
            p_Error->m_u8RightCenterValid = 0;
        }
    }

    else if (p_Type->m_u8CrossFlag == CROSS_NEAR || p_Type->m_u8RightSideCrossFlag == CROSS_NEAR || p_Type->m_u8LeftSideCrossFlag == CROSS_NEAR /* || p_Type->m_u8RightPRoadFlag == PROAD_END || p_Type->m_u8GarageFlag == GARAGE_RIGHT_TURN || (p_Type->m_u8GarageFlag == OUT_GARAGE && p_Type->m_u8GarageDirection == Garage_Out_LEFT && p_Type->m_u8GarageTracking == Garage_Tracking_Remote)*/) {
        int16_i    = clip((int16)(p_Error->m_f32RightBorderAimingMin / SampleDist), 0, p_Border->m_i16RCnterCntRSRemote - 1);
        int16_iEnd = clip((int16)(p_Error->m_f32RightBorderAimingMax / SampleDist), 0, p_Border->m_i16RCnterCntRSRemote - 1);

        while (int16_i++ < int16_iEnd) {
            if (p_Border->m_RCPntRSRemote[int16_i].m_i16y != 0) {
                dx = p_Border->m_RCPntRSRemote[int16_i].m_i16x - CenterX;
                dy = Fabs(p_Border->m_RCPntRSRemote[int16_i].m_i16y - CenterY);

                VerticalDist = dy / PixelperMeter + 0.243;
                HoriError    = dx / PixelperMeter;

                Kappa = 2 * HoriError / (VerticalDist * VerticalDist + HoriError * HoriError);

                if (Fabs(Kappa) > 1) {
                    if (dx == 0 || Avg_speed == 0) gain = 0;
                    //                     else gain = (float32)atan((double)dx * 1000 / Avg_speed *200 / EncoderPerMeter);
                } else {
                    gain = 0;
                }

                Kappa += gain;

                norm += 1;
                Kappatotal += Kappa;
            }
        }

        if (norm != 0) {
            p_Error->m_f32RightBorderKappa = Kappatotal / norm;
            p_Error->m_u8RightCenterValid  = 1;
            // if (p_Border->RL_CornerPosRemote == -1 && p_Type->m_u8RightPRoadFlag != PROAD_END && p_Type->m_u8GarageFlag != OUT_GARAGE) p_Error->m_u8RightCenterValid = 0;
        } else {
            p_Error->m_u8RightCenterValid = 0;
        }
    }

    else {
        p_Error->m_f32RightBorderKappa = 0;
        p_Error->m_u8RightCenterValid  = 0;
    }
}

/*全图逆透视打表变化*/
void Full_Inverse_Perspective(void)
{
    uint8 i;
    uint8 j;

    // int num=113;
    // memset(InImg, 0, sizeof(OutImg[0]) * PointNum);

    for (i = 0; i < IMGH; i++) {

        for (j = 0; j < IMGW; j++) {
            inv_image[i][j] = mt9v03x_image[Inv_y[i * IMGW + j]][Inv_x[i * IMGW + j]];
        }
    }
}

// void adaptiveThreshold_1(uint8 (*InImg)[MT9V03X_W], uint8 (*OutImg)[MT9V03X_W], int width, int height, int block, uint8_t clip_value)
// {
//     int count = 0;
//     // assert(block % 2 == 1); // block必须为奇数
//     int half_block = block / 2;
//     for (int y = half_block; y < height - half_block; y++) {
//         for (int x = half_block; x < width - half_block; x++) {
//             int thres = 0;
//             for (int dy = -half_block; dy <= half_block; dy++) {
//                 for (int dx = -half_block; dx <= half_block; dx++) {
//                     thres += InImg[y + dy + 1][x + dx];
//                     count++;
//                 }
//             }
//             thres        = thres / (block * block) - clip_value; // 求得平均阈值在减去经验变量
//             OutImg[y][x] = InImg[y][x] > thres ? 254 : 1;
//         }
//     }
//     SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\r\n count=%d.", count);
// }
