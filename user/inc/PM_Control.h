#ifndef __PM_CONTROL_H__
#define __PM_CONTROL_H__

#include "zf_common_headfile.h"
#include "headfile.h"
#include "imgStruct.h"

typedef struct {

    float result;
    float maximum;
    float minimum;
    float KP;
    float KI;

} Angular_Control_Para;

typedef struct // 曲率结构体参数
{

    // 变比例参数
    float Bas_KP;
    float Gain_KP;

    // 微分
    float KD;

    // PID基本参数
    float maximum;   /*输出值上限*/
    float minimum;   /*输出值下限*/
    float lasterror; // 前一拍偏差
    float result;    // 输出值

} Curvature_para;

// 曲率环PD控制
extern Curvature_para Curvature_Para;

// 内环角速度PI控制
extern Angular_Control_Para Angular_Para;

extern int16 normal_speed;
extern int16 now_speed;
extern float32 Avg_speed;
extern float filter_speed[30];
extern float32 Correct_speed;

extern uint8 change_kp_flag;
extern uint8 leave_kp_flag;
extern uint8 choose_flag;
extern uint8 charging_switch;

void Control(void);
void Curvature_PD(Curvature_para *Curvature_Para, float32 processValue);
void Angular_Control_PI(Angular_Control_Para *Angular_Para, Curvature_para *Curvature_Para, float32 processValue);

#endif