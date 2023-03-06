#include "Back_Wheel.h"

//----------------------------------------------------------变量定义--------------------------------------------------------------//
//----------------------------------------------------------变量定义--------------------------------------------------------------//
//----------------------------------------------------------变量定义--------------------------------------------------------------//
//----------------------------------------------------------变量定义--------------------------------------------------------------//
PID_2 Motor_Left;  // 左轮电机变量
PID_2 Motor_Right; // 右轮电机变量
Motor_Para MOTOR;  // 电机参数结构体变量

float history[4];

//----------------------------------------------------------后轮电机控制-------------------------------------------------------------//
//----------------------------------------------------------后轮电机控制-------------------------------------------------------------//
//----------------------------------------------------------后轮电机控制-------------------------------------------------------------//
//----------------------------------------------------------后轮电机控制-------------------------------------------------------------//

//--------------------------------------------------------------
//  @brief     左轮增量式PID(变积分)
//  @param     PID_1 *vPID              速度环PID常用量
//             Motor_Para *Motor        PID参数结构体
//             processValue             转速(编码器值)
//  @return    void        没求得
//  @note      增量式PID,死区限制，变积分
//--------------------------------------------------------------
void Motor_L_Control_Change_Integral(PID_2 *vPID, Motor_Para *Motor, int16 processValue)
{
    float thisError;
    float result;
    float increment;
    float pError, iError;
    float Change_P;
    float Change_I;
    static float lasterror = 0; // 前一拍偏差
    static float preerror  = 0; // 前两拍偏差

    thisError = vPID->setpoint - processValue; // 得到偏差值
    Change_P  = Motor->L_Bas_KP + Motor->L_Gain_KP * (1 - 1.0 / FExp(Motor->L_Cp * Fabs(thisError)));
    Change_I  = (1.0 / FExp(Motor->L_Ci * Fabs(thisError))) * Motor->L_Max_I;

    result = vPID->result;

    if (vPID->result > vPID->maximum - 500) { // 如果上次输出结果大于(最大值-1000)，积分值只累计负值
        if (thisError <= 0) {
            iError = thisError;
        }
    } else if (vPID->result < vPID->minimum + 500) { // 如果上次输出结果小于(最小值+1000)，积分值只累计正值
        if (thisError >= 0) {
            iError = thisError;
        }
    } else {
        iError = (thisError + lasterror) / 2.0; // 如果上次输出结果处于正常范围内，正常积分
    }

    pError    = thisError - lasterror;
    increment = Change_P * pError + Change_I * iError; // 变积分不用积分分离，因为积分分离本质就是变积分的一种特殊情况
    result    = result + increment;

    /*对输出限值，避免超调和积分饱和问题*/
    result = result >= vPID->maximum ? vPID->maximum : result;
    result = result <= vPID->minimum ? vPID->minimum : result;

    preerror     = lasterror; // 存放偏差用于下次运算
    lasterror    = thisError;
    vPID->result = result;
}

//--------------------------------------------------------------
//  @brief     右轮增量式PID(变积分)
//  @param     PID_1 *vPID             速度环PID常用量
//             Motor_Para *Motor       PID参数结构体;
//             processValue            转速(编码器值)
//  @return    void        没求得
//  @note      增量式PID，死区限制，变积分
//--------------------------------------------------------------
void Motor_R_Control_Change_Integral(PID_2 *vPID, Motor_Para *Motor, int16 processValue)
{
    float thisError;
    float result;
    float increment;
    float pError, iError;
    float Change_P;
    float Change_I;
    static float lasterror = 0; // 前一拍偏差
    static float preerror  = 0; // 前两拍偏差

    thisError = vPID->setpoint - processValue; // 得到偏差值
    Change_P  = Motor->R_Bas_KP + Motor->R_Gain_KP * (1 - 1.0 / FExp(Motor->R_Cp * Fabs(thisError)));
    Change_I  = (1.0 / FExp(Motor->R_Ci * Fabs(thisError))) * Motor->R_Max_I;
    result    = vPID->result;

    if (vPID->result > vPID->maximum - 500) { // 如果上次输出结果大于(最大值-1000)，积分值只累计负值
        if (thisError <= 0) {
            iError = thisError;
        }
    } else if (vPID->result < vPID->minimum + 500) { // 如果上次输出结果小于(最小值+1000)，积分值只累计正值
        if (thisError >= 0) {
            iError = thisError;
        }
    } else {
        iError = (thisError + lasterror) / 2.0; // 如果上次输出结果处于正常范围内，正常积分
    }

    pError    = thisError - lasterror;
    increment = Change_P * pError + Change_I * iError; // 变积分不用积分分离，因为积分分离本质就是变积分的一种特殊情况
    result    = result + increment;

    /*对输出限值，避免超调和积分饱和问题*/
    result = result >= vPID->maximum ? vPID->maximum : result;
    result = result <= vPID->minimum ? vPID->minimum : result;

    preerror     = lasterror; // 存放偏差用于下次运算
    lasterror    = thisError;
    vPID->result = result;
}
//--------------------------------------------------------------
//  @brief     左轮增量式PID
//  @param     PID_1 *vPID              速度环PID常用量;
//             Motor_Para *Motor        PID参数结构体
//             processValue             转速(编码器值)
//  @return    void        没求得
//  @note      增量式PID，死区限制，梯形积分
//--------------------------------------------------------------
void Motor_L_Control(PID_2 *vPID, Motor_Para *Motor, int16 processValue)
{

    float thisError;
    float result;
    float increment;
    float pError, dError, iError;
    static float lasterror = 0; // 前一拍偏差
    static float preerror  = 0; // 前两拍偏差

    thisError = vPID->setpoint - processValue; // 得到偏差值

    result = vPID->result;

    if (vPID->result > vPID->maximum - 500) { // 如果上次偏差大于(最大值-1000)，积分值只累计负值
        if (thisError <= 0) {
            iError = (thisError + lasterror) / 2.0;
        }
    } else if (vPID->result < vPID->minimum + 500) { // 如果上次输出结果小于(最小值+1000)，积分值只累计正值
        if (thisError >= 0) {
            iError = (thisError + lasterror) / 2.0;
        }
    } else {
        iError = (thisError + lasterror) / 2.0; // 如果上次输出结果处于正常范围，正常积分
    }
    pError = thisError - lasterror;
    dError = thisError - 2 * lasterror + preerror;

    if (BetaGeneration(thisError, vPID->epsilon) > 0) // 如果偏差小，加入积分作用
    {
        increment = Motor->L_P * pError + Motor->L_I * iError + Motor->L_D * dError;
    } else // 如果偏差大，取消积分作用
    {
        increment = Motor->L_P * pError + Motor->L_D * dError;
    }

    result = result + increment;

    /*对输出限幅，避免超调和积分饱和问题*/
    result = result >= vPID->maximum ? vPID->maximum : result;
    result = result <= vPID->minimum ? vPID->minimum : result;

    preerror     = lasterror; // 存放偏差用于下次运算
    lasterror    = thisError;
    vPID->result = result;
}

//--------------------------------------------------------------
//  @brief     右轮增量式PID
//  @param     PID_2 *vPID              速度环PID常用量;
//             Motor_Para *Motor        PID参数结构体;
//             processValue             转速(编码器值)
//  @return    void        没求得
//  @note      增量式PI，死区限制，梯形积分
//--------------------------------------------------------------
void Motor_R_Control(PID_2 *vPID, Motor_Para *Motor, int16 processValue)
{

    float thisError;
    float result;
    float increment;
    float pError, dError, iError;
    static float lasterror = 0; // 前一拍偏差
    static float preerror  = 0; // 前两拍偏差

    thisError = vPID->setpoint - processValue; // 得到偏差值

    result = vPID->result;

    if (vPID->result > vPID->maximum - 500) { // 如果上次偏差大于(最大值-1000)，积分值只累计负值
        if (thisError <= 0) {
            iError = (thisError + lasterror) / 2.0;
        }
    } else if (vPID->result < vPID->minimum + 500) { // 如果上次输出结果小于(最小值+1000)，积分值只累计正值
        if (thisError >= 0) {
            iError = (thisError + lasterror) / 2.0;
        }
    } else {
        iError = (thisError + lasterror) / 2.0; // 如果上次输出结果处于正常范围，正常积分
    }
    pError = thisError - lasterror;
    dError = thisError - 2 * lasterror + preerror;

    if (BetaGeneration(thisError, vPID->epsilon) > 0) // 如果偏差小，加入积分作用
    {
        increment = Motor->R_P * pError + Motor->R_I * iError + Motor->R_D * dError;
    } else // 如果偏差大，取消积分作用
    {
        increment = Motor->R_P * pError + Motor->R_D * dError;
    }

    result = result + increment;

    /*对输出限幅，避免超调和积分饱和问题*/
    result = result >= vPID->maximum ? vPID->maximum : result;
    result = result <= vPID->minimum ? vPID->minimum : result;

    preerror     = lasterror; // 存放偏差，用于下次运算
    lasterror    = thisError;
    vPID->result = result;
}

//--------------------------------------------------------------
//  @brief     左轮位置式PID
//  @param     PID_2 *vPID              速度环PID常用量
//             Motor_Para *Motor        PID参数结构体
//             processValue             转速(编码器值)
//  @return    void        没求得
//  @note      位置式PI，死区限制
//--------------------------------------------------------------
void Motor_L_Control_Position(PID_2 *vPID, Motor_Para *Motor, int16 processValue)
{

    float thisError;
    float result;
    static float integral  = 0;
    static float lasterror = 0; // 前一拍偏差

    thisError = vPID->setpoint - processValue;
    if (Fabs(thisError) > vPID->deadband) {
        integral += (thisError + lasterror) / 2;
        result = Motor->L_P * thisError + Motor->L_I * integral + Motor->L_D * (thisError - lasterror);

    } else {
        if ((Fabs(vPID->setpoint - vPID->minimum) < vPID->deadband) && (Fabs(processValue - vPID->minimum) < vPID->deadband)) {
            result = vPID->minimum;
        }
    }
    /*对输出限幅，避免超调和积分饱和问题*/
    result = result >= vPID->maximum ? vPID->maximum : result;
    result = result <= vPID->minimum ? vPID->minimum : result;

    lasterror    = thisError;
    vPID->result = result;
}

//--------------------------------------------------------------
//  @brief     右轮位置式PID
//  @param     PID_2 *vPID                   速度环常用量
//             Motor_Para *Motor             PID参数结构体
//             processValue                  转速(编码器值)
//  @return    void        没求得
//  @note      位置式，死区限制
//--------------------------------------------------------------
void Motor_R_Control_Position(PID_2 *vPID, Motor_Para *Motor, int16 processValue)
{

    float thisError;
    float result;
    static float integral  = 0;
    static float lasterror = 0; // 前一拍偏差
    thisError              = vPID->setpoint - processValue;
    if (Fabs(thisError) > vPID->deadband) {
        integral += (thisError + lasterror) / 2;
        result = Motor->R_P * thisError + Motor->R_I * integral + Motor->R_D * (thisError - lasterror);

    } else {
        if ((Fabs(vPID->setpoint - vPID->minimum) < vPID->deadband) && (Fabs(processValue - vPID->minimum) < vPID->deadband)) {
            result = vPID->minimum;
        }
    }
    /*对输出限值，避免超调和积分饱和问题*/
    result = result >= vPID->maximum ? vPID->maximum : result;
    result = result <= vPID->minimum ? vPID->minimum : result;

    lasterror    = thisError;
    vPID->result = result;
}
//--------------------------------------------------------------
//  @brief     左轮位置式微分先行PID
//  @param     PID_2 *vPID              速度环PID常用量
//             Motor_Para *Motor        PID参数结构体
//             processValue             转速(编码器值)
//  @return    void        没求得
//  @note      位置式PI，死区限制
//--------------------------------------------------------------
void Motor_L_Control_Position_Advance_differential(PID_2 *vPID, Motor_Para *Motor, int16 processValue)

{

    float thisError;
    float c1, c2, c3, temp;
    static float lasterror = 0; // 前一拍偏差

    thisError = vPID->setpoint - processValue;
    vPID->integral += thisError;
    temp             = vPID->gama * Motor->L_D + Motor->L_P;
    c3               = Motor->L_D / temp;
    c2               = (Motor->L_D + Motor->L_P) / temp;
    c1               = vPID->gama * c3;
    vPID->derivative = c1 * vPID->derivative + c2 * processValue + c3 * vPID->lastPv;
    vPID->result     = Motor->L_P * thisError + Motor->L_I * vPID->integral + vPID->derivative;
    lasterror        = thisError;
    vPID->lastPv     = processValue;
}

//--------------------------------------------------------------差速控制----------------------------------------------------------------//
//--------------------------------------------------------------差速控制----------------------------------------------------------------//
//--------------------------------------------------------------差速控制----------------------------------------------------------------//
//--------------------------------------------------------------差速控制----------------------------------------------------------------//

//--------------------------------------------------------------
//  @brief     lsd被动差速限滑动
//  @param
//  @return    void        没求得
//  @note
//--------------------------------------------------------------

// float lsd_p=0.1,lsd_pl=0.6,lsd_d=0.8;
// float lsd_p_base=0.15;
// int16 speed_set_l,speed_set_r;
// extern int16 speed_set;     //57大概为1m/s
// extern float speed_p;
// extern float speed_i;
// extern float speed_d;
//
// extern float lsd_show;
// void LSD(int16 encoder_l,int16 encoder_r)
//{
//   static int16 encoder_err;
//   static int16 encoder_det;
//   static int16 encoder_err_last;
//   static float lsd;
//   int turn_err_lsd;
//
//   turn_err_lsd=(int)center-39;
//
//   encoder_err=encoder_l-encoder_r;
////  if((encoder_err>10)||(encoder_err<-10))
////  {
//    encoder_det=encoder_err-encoder_err_last;
//
//    //lsd_p=lsd_p_base+turn_err_lsd*turn_err_lsd/7000;
//
//    //if(lsd_p>0.25) lsd_p=0.25;
//    lsd=encoder_err*lsd_p
//       +encoder_err_last*lsd_pl
//       +encoder_det*lsd_d;
//
//    //在这里加lsd的限幅
//
//    if(lsd>120) lsd=120;
//    if(lsd<-120) lsd=-120;
//    lsd_show=lsd;
//
//    encoder_err_last=encoder_err;
//    speed_set_l=speed_set+(int16)lsd;
//    speed_set_r=speed_set-(int16)lsd;
//
//
//}

//------------------------------------------------------------结果输出函数--------------------------------------------------------------//
//------------------------------------------------------------结果输出函数--------------------------------------------------------------//
//------------------------------------------------------------结果输出函数--------------------------------------------------------------//
//------------------------------------------------------------结果输出函数--------------------------------------------------------------//

//--------------------------------------------------------------
//  @brief     后轮PWM输出
//  @param     int32 L_outPWM     左电机pwm
//             int32 R_outPWM     右电机pwm
//  @return    void        没求得
//  @note
//--------------------------------------------------------------
void Back_Wheel_Out(int32 R_outPWM, int32 L_outPWM)
{
    if (0 <= L_outPWM) // 电机1 正转
    {

        pwm_set_duty(MOTOR1_PWM, L_outPWM);
        gpio_set_level(MOTOR1_DIR, 0);
    } else // 电机2 反转
    {
        pwm_set_duty(MOTOR1_PWM, -L_outPWM);
        gpio_set_level(MOTOR1_DIR, 1);
    }
    if (0 <= R_outPWM) // 电机2 正转
    {
        pwm_set_duty(MOTOR2_PWM, R_outPWM);
        gpio_set_level(MOTOR2_DIR, 0);
    } else // 电机2 反转
    {
        pwm_set_duty(MOTOR2_PWM, -R_outPWM);
        gpio_set_level(MOTOR2_DIR, 1);
    }
}

//----------------------------------------------------------滤波及功能性函数-------------------------------------------------------------//
//----------------------------------------------------------滤波及功能性函数-------------------------------------------------------------//
//----------------------------------------------------------滤波及功能性函数-------------------------------------------------------------//
//----------------------------------------------------------滤波及功能性函数-------------------------------------------------------------//

//--------------------------------------------------------------
//  @brief       积分分离因子判断函数
//  @param        float error            差值
//                float epsilon        界限
//  @return       beta
//  @note
//--------------------------------------------------------------
uint16 BetaGeneration(float error, float epsilon)
{
    uint16 beta = 0;
    if (Fabs(error) <= epsilon) {
        beta = 1;
    }
    return beta;
}

//--------------------------------------------------------------
//  @brief        均值滤波
//  @param        float inData       输入数据
//                float a            滤波系数
//  @return       outData            输出数据
//  @note         float型
//--------------------------------------------------------------
float data_filtering(float *filter, const float filter_data, const uint8 filter_depth)
{
    float filter_sum = 0;
    uint8 i;
    // 更新数据
    filter[filter_depth] = filter_data;

    for (i = 0; i < filter_depth; i++) {
        filter[i] = filter[i + 1]; // 数据左移，扔掉末尾数据
        filter_sum += filter[i];
    }
    return ((float)filter_sum / (float)filter_depth);
}

//--------------------------------------------------------------
//  @brief        一阶低通滤波
//  @param        float inData       输入数据
//                float a            滤波系数(a* 当前数据 + (1-a)*上次数据)
//  @return       outData            输出数据
//  @note         float型
//--------------------------------------------------------------
float errorfilter(float inData, float a)
{
    float outData;
    static float prevData1R = 0;
    outData                 = (1 - a) * prevData1R + a * inData;
    prevData1R              = inData;
    return outData;
}

//--------------------------------------------------------------
//  @brief        滑动平均滤波(原始数据)
//  @param        float value        输入值
//                uint8 time         取平均个数
//  @return       sum / K            输出数据
//  @note         float型
//--------------------------------------------------------------
float Filter_ave_DuojiData(float value, uint8 time)
{
    int i, j;
    float sum = 0;

    int factor[time];
    int K                   = 0;
    static int filter_index = 0;
    static int buff_init    = 0;

    for (i = 0; i < time; i++) {
        factor[i] = i + 1;
        K += i + 1;
    }
    if (buff_init == 0) {
        history[filter_index] = value;
        filter_index++;

        if (filter_index >= time - 1) {

            buff_init = 1;
        }

    } else {
        history[filter_index] = value;
        filter_index++;
        if (filter_index >= time - 1) {
            filter_index = 0;
        }
        j = filter_index;
        for (i = 0; i <= time - 1; i++) {
            sum += history[j] * factor[i];
            j++;
            if (j == time - 1) {
                j = 0;
            }
        }
    }
    return sum / K;
}
