#ifndef __IMU_H__
#define __IMU_H__

#include "zf_common_headfile.h"
#include "headfile.h"

extern int8 getgyro_flag; // 获取零飘的按键，出发前静止按一下
extern float gyro_0, gyroy_0;
extern float g_z, yaw_angle, pitch_angle, dt, last_icm_gyro_z, Kalman_pitch_angle;
extern float g_z_degree;
extern float ramp_angle[150];
extern uint8 ramp_flag;

void gyro_calculate(void);
// void ramp_ang(void);
float gy_Kalman_Filter(float angle_m, float gyro_m);
float gz_Kalman_Filter(float angle_m, float gyro_m);

// typedef struct {
//     int16 Xdata;
//     int16 Ydata;
//     int16 Zdata;
// } gyro_param_t;

// typedef struct {
//     float acc_x;
//     float acc_y;
//     float acc_z;
//     float gyro_x;
//     float gyro_y;
//     float gyro_z;
// } icm_param_t;

// typedef struct {
//     float q0;
//     float q1;
//     float q2;
//     float q3;
// } quater_param_t;

// typedef struct {
//     float pitch;
//     float roll;
//     float yaw;
// } euler_param_t;

// extern euler_param_t eulerAngle;
// extern gyro_param_t GyroOffsets;
// extern icm_param_t icm_data;

// gyro_param_t gyroOffsetInit(void);
// icm_param_t icmGetValues(gyro_param_t GyroOffset);
// void icmAHRSupdate(icm_param_t *icm);

#endif
