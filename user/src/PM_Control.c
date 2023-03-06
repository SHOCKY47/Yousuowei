#include "PM_Control.h"

int16 normal_speed;
int16 now_speed;
float32 Avg_speed;
float32 Correct_speed;
float filter_speed[30] = {0}; // 速度滑动滤波

// 充电前改变KP参数
uint8 change_kp_flag  = 0;
uint8 leave_kp_flag   = 0;
uint8 choose_flag     = 0;
uint8 charging_switch = 1;

// 曲率环PD控制
Curvature_para Curvature_Para;

// 内环角速度PI控制
Angular_Control_Para Angular_Para;

void Control(void)
{
    float32 Img_Error;
    static float32 Img_LastError;

    now_speed    = 40;
    normal_speed = 100;
    Avg_speed    = (Motor_Left.setpoint + Motor_Left.setpoint) / 2;

    if (g_LineError.m_u8LeftCenterValid == 1 && g_LineError.m_u8RightCenterValid == 1) {
        Img_Error = (g_LineError.m_f32LeftBorderKappa + g_LineError.m_f32RightBorderKappa) / 2.0;
    } else if (g_LineError.m_u8LeftCenterValid == 1 && g_LineError.m_u8RightCenterValid == 0) {
        Img_Error = g_LineError.m_f32LeftBorderKappa;
    } else if (g_LineError.m_u8LeftCenterValid == 0 && g_LineError.m_u8RightCenterValid == 1) {
        Img_Error = g_LineError.m_f32RightBorderKappa;
    } else if (g_LineError.m_u8LeftCenterValid == 0 && g_LineError.m_u8RightCenterValid == 0) {
        Img_Error = Img_LastError;
    }

    // 需要处理的元素:环岛、P字、S路、坡道、车库
    // 需要固定速度的元素：坡道上、P字出阶段、入库阶段
    // 直道弯道区分 无元素

    // 直道
    if (g_TrackType.m_u8ShortLeftLineStraightFlag == 1 || g_TrackType.m_u8ShortRightLineStraightFlag == 1 /*|| g_TrackType.m_u8CrossFlag != CROSS_NONE || g_TrackType.m_u8RightSideCrossFlag != CROSS_NONE || g_TrackType.m_u8LeftSideCrossFlag != CROSS_NONE*/) {

        Curvature_Para.Bas_KP  = 0.4;
        Curvature_Para.Gain_KP = 2;
        Curvature_Para.KD      = 0.1;
        Curvature_PD(&Curvature_Para, Img_Error);

        Curvature_Para.result = Curvature_Para.result < 2.1 ? Curvature_Para.result : 2.1;

    } else { // 弯道
    }

    Curvature_PD(&Curvature_Para, Img_Error);

    float M_error         = 3.5;
    Curvature_Para.result = (Curvature_Para.result > M_error ? M_error : Curvature_Para.result) < -M_error ? -M_error : (Curvature_Para.result > M_error ? M_error : Curvature_Para.result);

    Img_LastError = Img_Error;
}

//--------------------------------------------------------------
//  @brief     曲率环(位置型PD)---转向环外环
//  @param     Curvature_para *Curvature_Para          PID结构体
//             Curvature_pid *Curvature_PID            曲率环参数
//             float32 processValue                    图像曲率误差(注意不是int16型，是float32)
//  @return    void        没求得
//  @note      位置型变结构PD
//--------------------------------------------------------------
void Curvature_PD(Curvature_para *Curvature_Para, float32 processValue)
{
    float thisError;
    float result;
    float Change_P;
    static float lasterror = 0; // 前一拍偏差

    thisError = processValue;
    result    = Curvature_Para->result;
    Change_P  = Curvature_Para->Bas_KP + Curvature_Para->Gain_KP * Fabs(thisError);

    result = Change_P * thisError + Curvature_Para->KD * (thisError - lasterror);

    /*对输出限幅，避免超调和积分饱和问题*/
    result = result >= Curvature_Para->maximum ? Curvature_Para->maximum : result;
    result = result <= Curvature_Para->minimum ? Curvature_Para->minimum : result;

    lasterror              = thisError;
    Curvature_Para->result = result;
}

//--------------------------------------------------------------
//  @brief     角速度环(PI)---转向环内环
//  @param     Angular_Control_Para *Angular_Para,  //角度环结构体
//             Curvature_para *Curvature_Para,      //曲率环结构体
//             float32 processValue                 //陀螺仪偏航角速度值(两轮胎连线方向)
//  @return    void        没求得
//  @note      增量式PI
//--------------------------------------------------------------
void Angular_Control_PI(Angular_Control_Para *Angular_Para, Curvature_para *Curvature_Para, float32 processValue)
{

    float thisError;
    float result;
    float increment;
    float pError, iError;
    static float lasterror = 0;                                                                         // 前一拍偏差
    static float preerror  = 0;                                                                         // 前两拍偏差
                                                                                                        // !EncoderPerMeter要自己确定
    thisError = Curvature_Para->result * Avg_speed * 180.0 / PI * 200 / EncoderPerMeter - processValue; //(期望曲率*线速度 - 实际角速度)
    // 这里线速度就是平均轮胎速度(编码器值M)，角速度也是两轮胎连线的角度变化率，轮胎速度要进行编码器的转换(M * m/M)。200应该是时间，因为周期是5ms(1/200 单位是1/s),最后得出m/s,然后进行弧度制转换。

    result = Angular_Para->result;

    if (Angular_Para->result > Angular_Para->maximum - 500) { // 如果上次偏差大于(最大值-1000)，积分值只累计负值
        if (thisError <= 0) {
            iError = (thisError + lasterror) / 2.0;
        }
    } else if (Angular_Para->result < Angular_Para->minimum + 500) { // 如果上次输出结果小于(最小值+1000)，积分值只累计正值
        if (thisError >= 0) {
            iError = (thisError + lasterror) / 2.0;
        }
    } else {
        iError = (thisError + lasterror) / 2.0; // 如果上次输出结果处于正常范围，正常积分
    }
    pError = thisError - lasterror;

    // if (BetaGeneration(thisError, vPID->epsilon) > 0) // 如果偏差小，加入积分作用
    // {
    //     increment = Angular_Para->KP * pError + Angular_Para->KI * iError;
    // } else // 如果偏差大，取消积分作用
    // {
    //     increment = Motor->R_P * pError + Motor->R_D * dError;
    // }

    result = result + increment;

    /*对输出限幅，避免超调和积分饱和问题*/
    result = result >= Angular_Para->maximum ? Angular_Para->maximum : result;
    result = result <= Angular_Para->minimum ? Angular_Para->minimum : result;

    preerror             = lasterror; // 存放偏差，用于下次运算
    lasterror            = thisError;
    Angular_Para->result = result;
}
