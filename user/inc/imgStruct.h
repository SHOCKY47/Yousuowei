
#ifndef _IMGSTRUCT_H_
#define _IMGSTRUCT_H_

#include "zf_common_headfile.h"
#include "headfile.h"

typedef enum {
    TRACKINGBOTH,
    TRACKINGLEFT,
    TRACKINGRIGHT
} TRACKINGTYPE;

typedef enum {
    LEFTLINE_FRONT = 1,
    LEFTLINE_RIGHT,
    LEFTLINE_REAR,
    LEFTLINE_LEFT,
    LEFTLINE_FRONT_LEFT,
    LEFTLINE_FRONT_RIGHT,
    LEFTLINE_REAR_RIGHT,
    LEFTLINE_REAR_LEFT
} LeftLine_GrowDirection;

typedef enum {
    RIGHTLINE_FRONT = 1,
    RIGHTLINE_RIGHT,
    RIGHTLINE_REAR,
    RIGHTLINE_LEFT,
    RIGHTLINE_FRONT_RIGHT,
    RIGHTLINE_REAR_RIGHT,
    RIGHTLINE_REAR_LEFT,
    RIGHTLINE_FRONT_LEFT
} RightLine_GrowDirection;

#define IMGH MT9V03X_H
#define IMGW MT9V03X_W

typedef struct float_point_info_ {
    float32 m_i16x;
    float32 m_i16y;
} FLOAT_POINT_INFO;

typedef struct int_point_info_ {
    int16 m_i16x;
    int16 m_i16y;
} INT_POINT_INFO;

typedef struct track_border_info_ {
    /*经典算法边界数据*/
    INT_POINT_INFO m_LPnt[IMGH];     /*左边边线数组*/
    INT_POINT_INFO m_RPnt[IMGH];     /*右边边线数组*/
    INT_POINT_INFO m_CPnt[IMGH];     /*中线数组*/
    INT_POINT_INFO pre_m_CPnt[IMGH]; /*上次中线数组*/

    /*边界生长方向*/
    uint8 m_LPntGrowDirection[IMGH]; /*左边界生长方向*/
    uint8 m_RPntGrowDirection[IMGH]; /*右边界生长方向*/

    int16 m_LeftLineCoor[IMGH];
    int16 m_RightLineCoor[IMGH];

    int16 m_i16LPointCnt; /*左边找到点数*/
    int16 m_i16RPointCnt; /*右边找到点数*/

    int16 m_i16LLostPointCnt; /*左边丢失点总数*/
    int16 m_i16RLostPointCnt; /*右边丢失点总数*/

    int16 WidthRow100;
    int16 WidthRow90;
    int16 WidthRow80;
    int16 WidthRow70;
    int16 WidthRow60;
    int16 WidthRow50;
    int16 WidthRow40;
    int16 WidthRow30;

    uint8 m_u8LeftMonoFlag;  /*左边界单调标志位*/
    uint8 m_u8RightMonoFlag; /*右边界单调标志位*/

    float64 m_f64LeftSlope;
    float64 m_f64LeftIntersect;
    float64 m_f64RightSlope;
    float64 m_f64RightIntersect;

    float64 m_f64LeftVariance;
    float64 m_f64RightVariance;

    uint8 u8_FindLeftSeed;  /*找到左边种子标志*/
    uint8 u8_FindRightSeed; /*找到右边种子标志*/

    INT_POINT_INFO m_LPntInvp[IMGH]; /*左边线逆透视数组*/
    INT_POINT_INFO m_RPntInvp[IMGH]; /*右边线逆透视数组*/

    /*三角滤波边线数组*/
    FLOAT_POINT_INFO m_LPntout[IMGH];
    FLOAT_POINT_INFO m_RPntout[IMGH];

    /*重新采样后边界数组大小*/
    int16 m_i16LPointCntRS;
    int16 m_i16RPointCntRS;

    /*重采样边界*/
    FLOAT_POINT_INFO m_LPntRS[IMGH];
    FLOAT_POINT_INFO m_RPntRS[IMGH];

    /*边线角度变化率*/
    float32 LdAngle[IMGH];
    float32 RdAngle[IMGH];

    /*边线角度变化率非极大值抑制*/
    float32 LdAngleNMS[IMGH];
    float32 RdAngleNMS[IMGH];

    /*左边边线跟踪中线*/
    FLOAT_POINT_INFO m_LCPnt[IMGH];
    /*右边边线跟踪中线*/
    FLOAT_POINT_INFO m_RCPnt[IMGH];

    /*左中线等距采样*/
    FLOAT_POINT_INFO m_LCPntRS[IMGH];
    /*左中线采样点数*/
    int16 m_i16LCnterCntRS;

    /*右中线等距采样*/
    FLOAT_POINT_INFO m_RCPntRS[IMGH];
    /*右中线采样点数*/
    int16 m_i16RCnterCntRS;

    /*左右边线Y角点位置*/
    int16 LY_CornerPos;
    int16 RY_CornerPos;

    /*左右边线L角点位置*/
    int16 LL_CornerPos;
    int16 RL_CornerPos;

    int16 LL_CornerNUM;
    int16 RL_CornerNUM;

    int16 LeftLocalMaximum;
    int16 LeftLocalMinimum;
    int16 RightLocalMaximum;
    int16 RightLocalMinimum;

    int8 m_i8LMonotonicity[IMGH]; /*左边单调性 (复用:找线阶段用它来描述需要是否需要补线,或者补线的类型,左边)*/
    int8 m_i8LMonotonicityCnt;    /*左拐点*/
    int8 m_i8RMonotonicity[IMGH]; /*右边单调性 (复用:同上,描述右边)*/
    int8 m_i8RMonotonicityCnt;    /*右拐点*/
    INT_POINT_INFO Cturn;         /*中间顶点 主要用于三叉*/

    INT_POINT_INFO m_LMaxPoint; /*左边最大点,在正常情况下,最大点不应该出现在边线的中间,只会出现在边线结束,出现在中间,那么这个最大点应该就*/
    INT_POINT_INFO m_RMinPoint;

    /*新思路算法定义数据*/
    INT_POINT_INFO m_OptimalPoint;    // 实际最大列所在的最优点
    INT_POINT_INFO m_LOptimalPoint;   // 实际左半边最大列所在的最优点
    INT_POINT_INFO m_ROptimalPoint;   // 实际右半边最大列所在的最优点
    INT_POINT_INFO m_CenterLinePoint; // 图像中间优所在的最优点
    uint32 m_LeftArea;
    uint32 m_RightArea;

    uint32 m_u32LAllArea; /*左边白色点总面积（纵向）*/
    uint32 m_u32RAllArea; /*右边白色点总面积（纵向）*/
    uint32 m_u32AllArea;  /*中间部分总面积（横向）*/

    /*记录每列从底部向上的白点数数组*/
    uint16 m_u16LineBAr[IMGW]; //

    /**************************************Remote part**************************************/
    /*左右远端边线L角点位置*/
    int16 LL_CornerPosRemote;
    int16 RL_CornerPosRemote;

    uint8 m_u8LFindRemoteSeed; /*找到左边远端种子标志*/
    uint8 m_u8RFindRemoteSeed; /*找到右边远端种子标志*/

    INT_POINT_INFO m_LRemotePnt[IMGH]; /*远端左边边线数组*/
    INT_POINT_INFO m_RRemotePnt[IMGH]; /*远端右边边线数组*/

    /*远端边界生长方向*/
    uint8 m_LRemotePntGrowDirection[IMGH]; /*远端左边界生长方向*/
    uint8 m_RRemotePntGrowDirection[IMGH]; /*远端右边界生长方向*/

    int16 m_i16LRemotePointCnt; /*左边远端边线点数*/
    int16 m_i16RRemotePointCnt; /*右边远端边线点数*/

    FLOAT_POINT_INFO m_LPntInvpRemote[IMGH]; /*左边线远端逆透视数组*/
    FLOAT_POINT_INFO m_RPntInvpRemote[IMGH]; /*右边线远端逆透视数组*/

    FLOAT_POINT_INFO m_LPntoutRemote[IMGH]; /*左边线远端边线滤波数组*/
    FLOAT_POINT_INFO m_RPntoutRemote[IMGH]; /*右边线远端边线滤波数组*/

    int16 m_i16LPointCntRSRemote; /*远端左边线重新采样后边界数组大小*/
    int16 m_i16RPointCntRSRemote; /*远端右边线重新采样后边界数组大小*/

    FLOAT_POINT_INFO m_LPntRSRemote[IMGH]; /*远端左边重采样边界*/
    FLOAT_POINT_INFO m_RPntRSRemote[IMGH]; /*远端右边重采样边界*/

    FLOAT_POINT_INFO m_LCPntRemote[IMGH]; /*远端左边边线跟踪中线*/
    FLOAT_POINT_INFO m_RCPntRemote[IMGH]; /*远端右边边线跟踪中线*/

    FLOAT_POINT_INFO m_LCPntRSRemote[IMGH]; /*远端左边边线跟踪中线重采样数组*/
    FLOAT_POINT_INFO m_RCPntRSRemote[IMGH]; /*远端右边边线跟踪中线重采样数组*/

    int16 m_i16LCnterCntRSRemote; /*远端左边边线跟踪中线重采样数组大小*/
    int16 m_i16RCnterCntRSRemote; /*远端右边边线跟踪中线重采样数组大小*/

    float32 LdAngleRemote[IMGH]; /*远端左边线局部角度变化率*/
    float32 RdAngleRemote[IMGH]; /*远端右边线局部角度变化率*/

    float32 LdAngleNMSRemote[IMGH]; /*远端左边线局部角度变化率非极大值抑制*/
    float32 RdAngleNMSRemote[IMGH]; /*远端右边线局部角度变化率非极大值抑制*/

} TRACK_BORDER_INFO;

/*赛道类型结构体*/
typedef struct track_type_info_ {
    // 车库模式
    uint32 m_u8Garage_Mode;
    // 0 非车库发车，不累计运行圈数
    // 1 车库发车，两圈后入库


uint8 Outframe;//出界
    // 运行圈数计数
    uint8 m_u8Round_Count;

    // 左边线长直道标志位
    uint8 m_u8LeftLineStraightFlag;
    // 右边线长直道标志位
    uint8 m_u8RightLineStraightFlag;

    // 左边线短直道标志位
    uint8 m_u8ShortLeftLineStraightFlag;
    // 右边线段直道标志位
    uint8 m_u8ShortRightLineStraightFlag;

    // 左边线图像中部直弯标志
    uint8 m_u8MidLeftLineStraightFlag;
    // 右边线图像中部直弯标志
    uint8 m_u8MidRightLineStraightFlag;

    /*出车库方向*/
    uint32 m_u8GarageDirection;
    /*车库标志位*/
    uint8 m_u8GarageFlag;
    /*出库巡近远线*/
    uint8 m_u8GarageTracking;

    /*十字标志*/
    uint8 m_u8CrossFlag; /*正入十字标志位*/
    // 0 无十字
    // 1 进入十字前，近端边线点数较多
    // 2 十字中，近端丢线

    uint8 m_u8RightSideCrossFlag; // 右斜入十字标志
    uint8 m_u8LeftSideCrossFlag;  // 左斜入十字标志

    /*三岔标志*/
    uint8 m_u8YjunctionFlag;
    // 0 无三岔
    // 1 进入三岔
    // 2 三岔内
    // 3 出三叉
    uint32 m_u8YjunctionDirection;
    // 0 左进三岔
    // 1 右进三岔

    uint8 m_u8LeftRoundaboutFlag;
    // 0 无环岛
    // 1 检测到环岛，跟随直道右边线
    // 2 进入左环岛，跟随内环左边线
    // 3 环岛内，跟随外环右边线
    // 4 出环岛，跟随内环左边线
    // 5 环岛结束，跟随直道右边线

    uint8 m_u8RightRoundaboutFlag;
    // 0 无环岛
    // 1 检测到环岛，跟随直道左边线
    // 2 进入右环岛，跟随内环右边线
    // 3 环岛内，跟随外环左边线
    // 4 出环岛，跟随内环右边线
    // 5 环岛结束，跟随直道左边线

    // 记录大小环岛
    uint8 m_u8RoundaboutSize;

    uint8 m_u8RightPRoadFlag;

    uint8 m_u8LeftPRoadFlag;

    //        //P字方向
    //        uint8 m_u8PRoadDir;

    uint8 m_u8RampFlag;

    uint8 m_u8BendSFlag;

    int16 m_int16BendSCount;
    //        uint8 m_u8StraightFlag;   /*直道标志位*/
    //        uint8 m_u8BendFlag;       /*弯道标志位*/
    //        uint8 m_u8RoundaboutFlag; /*环岛标志位*/
    //        uint8 m_u8RoundaboutDir;  /*环岛方向*/
    //        uint8 m_u8RoundaboutSave;
    //        uint8 m_u8RampFlag;       /*坡道标志位*/
    //
    //        uint8 m_u8ThreeRoadsFlag; /*三岔口标志位 0:未检测到 1:检测到 2:左进 3:右进 */
    //        uint8 m_u8ThreeRoadsDir;  /*三叉路口左转右转标志位*/
    //        uint8 m_u8ThreeRoadsBug;  /*三叉bug修复用*/
    //
    //        uint8 m_u8speedupFlag;/*加速标志位*/
    //        uint8 m_u8CarBarnState;    /*出库状态*/
    //        // 出库状态
    //        // 0 出库
    //        // 1 进入赛道
    //        // 2 穿过 补两线
    //        // 3 在赛道上
    //        // 4 入库
    //        // 5 停车
    //
    //        uint8 m_u8CarBarnDir;      /*出库入库方向,什么方向出库就什么方向入库*/
    //
    //        uint8 m_u8CrossCarBarn;
    //
    //        uint8 m_u8RunTurns; /*运行圈数*/
    //
    //        uint8 m_u8ZebraCrossingFlag; /*斑马线*/
    //
    //        uint8 m_u8EndTime;
    //        // 斑马线状态
    //        // 0 未处于斑马线
    //        // 1 处于斑马线

} TRACK_TYPE_INFO;

typedef struct line_error_info_ {
    uint8 m_u8TackingType;
    /*偏差计算起始行*/
    int16 m_int16StartPnt;

    /*偏差计算结尾行*/
    int16 m_int16EndPnt;

    float32 m_f32LeftCenterError;
    float32 m_f32LeftCenterErrorLast;

    uint8 m_u8LeftCenterValid;
    // 0 左中线丢线
    // 1 左中线误差有效

    // 曲率偏差，速度决策
    //        float64 LeftCurve;
    //        float64 RightCurve;
    float64 Curve;

    float32 m_f32RightCenterError;
    float32 m_f32RightCenterErrorLast;

    uint8 m_u8RightCenterValid;

    /*远端偏差计算起始行*/
    int16 m_int16StartPntRemote;

    /*远端偏差计算结尾行*/
    int16 m_int16EndPntRemote;

    float32 m_f32LeftCenterErrorRemote;
    float32 m_f32RightCenterErrorRemote;

    float32 m_f32LeftBorderAimingMin;
    float32 m_f32LeftBorderAimingMax;

    float32 m_f32LeftBorderKappa;

    float32 m_f32RightBorderAimingMin;
    float32 m_f32RightBorderAimingMax;
    float32 m_f32RightBorderKappa;

    float32 m_f32RampError;
    uint8 m_u8RampErrorValid;
} LINE_ERROR_INFO;

#endif /* CODE_IMGSTRUCT_H_ */
