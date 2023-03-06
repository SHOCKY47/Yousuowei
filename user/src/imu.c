#include "imu.h"

int8 getgyro_flag = 0; // 获取零飘的按键，出发前静止按一下
float gyro_0;          //! 值要自己确定
float g_z, last_g_z, yaw_angle = 0, dt = 0.005, last_icm_gyro_z, Kalman_pitch_angle;
float g_y, gyroy_0, pitch_angle = 0;
float g_z_degree;

float ramp_angle[150];
uint8 ramp_flag = 0;

float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
float euler_pitch, euler_yaw, euler_roll;

// 偏航角速度 ： 去零飘 卡尔曼滤波 计算偏航角
void gyro_calculate(void)
{
    static float gz;
    gz              = (float)imu660ra_gyro_z;
    gz              = gz - gyro_0;
    g_z             = gz_Kalman_Filter(gz, gz - last_icm_gyro_z);
    g_z_degree      = g_z / 16.4;
    last_icm_gyro_z = gz;
    yaw_angle += g_z / 16.0 * dt;
    if (yaw_angle >= 360 || yaw_angle <= -360)
        yaw_angle = 0;
}

//
// void ramp_ang(void)
// {
//     // 遇到非坡道元素，做一次偏航角清零
//     if (start_flag)
//         if (g_TrackType.m_u8YjunctionFlag == YJUNCTION_YES || g_TrackType.m_u8RightRoundaboutFlag == ROUNDABOUT_END || g_TrackType.m_u8LeftRoundaboutFlag == ROUNDABOUT_END || g_TrackType.m_u8RightPRoadFlag == PROAD_END || g_TrackType.m_u8LeftPRoadFlag == PROAD_END || g_TrackType.m_u8CrossFlag == CROSS_NEAR || g_TrackType.m_u8GarageFlag == GARAGE_RIGHT_PASS || g_TrackType.m_u8GarageFlag == GARAGE_LEFT_PASS || g_TrackType.m_u8GarageFlag == OUT_GARAGE) pitch_angle = 0;

//     // 俯仰角速度： 去零飘 计算
//     static float gy;
//     gy = (float)icm_gyro_y;
//     gy = gy - gyroy_0;
//     pitch_angle += gy / 16.0 * dt;
//     if (pitch_angle >= 90 || pitch_angle <= -90)
//         pitch_angle = 0;
// }

// 偏航角卡尔曼滤波
float gz_Kalman_Filter(float angle_m, float gyro_m) // ƫ����
{
    float P[2][2] = {{1, 0},
                     {0, 1}};
    static float q_bias;
    static float K_0;
    static float K_1;
    static float angle_kalman;
    static uint8 first_angle;
    static float Q_angle = 0.000001;
    static float Q_gyro  = 0.05;
    static float R_angle = 8.0; // 8  10
    if (first_angle == 0) {
        angle_kalman = angle_m;
        q_bias       = 0;
        first_angle  = 1;
    }
    angle_kalman += (gyro_m - q_bias) * dt;
    P[0][0] += Q_angle - (P[0][1] + P[1][0]) * dt;
    P[0][1] -= P[1][1] * dt;
    P[1][0] -= P[1][1] * dt;
    P[1][1] += Q_gyro;
    K_0          = P[0][0] / (P[0][0] + R_angle);
    K_1          = P[1][0] / (P[1][0] + R_angle);
    angle_kalman = angle_kalman + K_0 * (angle_m - angle_kalman);
    q_bias       = q_bias + K_1 * (angle_m - angle_kalman);
    P[0][0] -= K_0 * P[0][0];
    P[0][1] -= K_0 * P[0][1];
    P[1][0] -= K_1 * P[0][0];
    P[1][1] -= K_1 * P[0][1];
    return angle_kalman;
}

// 俯仰角卡尔曼滤波
float gy_Kalman_Filter(float angle_m, float gyro_m) // ������
{
    float P[2][2] = {{1, 0},
                     {0, 1}};
    static float q_bias;
    static float K_0;
    static float K_1;
    static float angle_kalman;
    static uint8 first_angle = 0;
    static float Q_angle     = 0.00003;
    static float Q_gyro      = 0.001;
    static float R_angle     = 1;
    if (first_angle == 0) {
        angle_kalman = angle_m;
        q_bias       = 0;
        first_angle  = 1;
    }
    angle_kalman += (gyro_m - q_bias) * dt;
    P[0][0] += Q_angle - (P[0][1] + P[1][0]) * dt;
    P[0][1] -= P[1][1] * dt;
    P[1][0] -= P[1][1] * dt;
    P[1][1] += Q_gyro;
    K_0          = P[0][0] / (P[0][0] + R_angle);
    K_1          = P[1][0] / (P[1][0] + R_angle);
    angle_kalman = angle_kalman + K_0 * (angle_m - angle_kalman);
    q_bias       = q_bias + K_1 * (angle_m - angle_kalman);
    P[0][0] -= K_0 * P[0][0];
    P[0][1] -= K_0 * P[0][1];
    P[1][0] -= K_1 * P[0][0];
    P[1][1] -= K_1 * P[0][1];
    return angle_kalman;
}

/*****************************************四元数解算**********************************/

// quater_param_t Q_info = {1, 0, 0, 0}; // 四元数初始化
// float I_ex, I_ey, I_ez;               // 误差积分
// euler_param_t eulerAngle;
// gyro_param_t GyroOffsets;
// icm_param_t icm_data;

/*
 * 将采集的数值转化为实际物理值, 并对陀螺仪进行去零漂处理
 * 加速度计初始化配置 -> 测量范围: ±8g        对应灵敏度: 4096 LSB/g
 * 陀螺仪初始化配置   -> 测量范围: ±2000 dps  对应灵敏度: 16.4 LSB/dps   (degree per second)
 * gyro = (gyro_val / 16.4) °/s = ((gyro_val / 16.4) * PI / 180) rad/s
 */

// gyro_param_t gyroOffsetInit(void)
// {
//     gyro_param_t GyroOffset; // 陀螺仪校准值
//     GyroOffset.Xdata = 0;
//     GyroOffset.Ydata = 0;
//     GyroOffset.Zdata = 0;

//     for (int i = 0; i < 100; ++i) {
//         imu660ra_get_gyro(); // 获取陀螺仪角速度
//         GyroOffset.Xdata += imu660ra_gyro_x;
//         GyroOffset.Ydata += imu660ra_gyro_y;
//         GyroOffset.Zdata += imu660ra_gyro_z;
//         system_delay_ms(5);
//     }

//     GyroOffset.Xdata /= 100;
//     GyroOffset.Ydata /= 100;
//     GyroOffset.Zdata /= 100;

//     return GyroOffset;
// }

// icm_param_t icmGetValues(gyro_param_t GyroOffset)
// {

//     float alpha = 0.3;
//     imu660ra_get_acc();
//     imu660ra_get_gyro(); // 获取陀螺仪角速度
//     icm_param_t icm_data;
//     // 一阶低通滤波，单位g
//     icm_data.acc_x = (((float)imu660ra_acc_x) * alpha) / 4096 + icm_data.acc_x * (1 - alpha);
//     icm_data.acc_y = (((float)imu660ra_acc_y) * alpha) / 4096 + icm_data.acc_y * (1 - alpha);
//     icm_data.acc_z = (((float)imu660ra_acc_z) * alpha) / 4096 + icm_data.acc_z * (1 - alpha);

//     // 陀螺仪角速度必须转换为弧度制角速度: deg/s -> rad/s
//     icm_data.gyro_x = ((float)imu660ra_gyro_x - GyroOffset.Xdata) * PI / 180 / 16.4f;
//     icm_data.gyro_y = ((float)imu660ra_gyro_y - GyroOffset.Ydata) * PI / 180 / 16.4f;
//     icm_data.gyro_z = ((float)imu660ra_gyro_z - GyroOffset.Zdata) * PI / 180 / 16.4f;

//     return icm_data;
// }

// void icmAHRSupdate(icm_param_t *icm)
// {

//     float icm_kp = 100; // !加速度计的收敛速率比例增益
//     float icm_ki = 1;   // !陀螺仪收敛速率的积分增益

//     float halfT = 0.5 * dt; // 采样周期一半
//     float vx, vy, vz;       // 当前姿态计算得来的重力在三轴上的分量
//     float ex, ey, ez;       // 当前加速计测得的重力加速度在三轴上的分量与用当前姿态计算得来的重力在三轴上的分量的误差

//     float q0 = Q_info.q0; // 四元数
//     float q1 = Q_info.q1;
//     float q2 = Q_info.q2;
//     float q3 = Q_info.q3;

//     float q0q0 = q0 * q0; // 先相乘，方便后续计算
//     float q0q1 = q0 * q1;
//     float q0q2 = q0 * q2;
//     //    float q0q3 = q0 * q3;
//     float q1q1 = q1 * q1;
//     //    float q1q2 = q1 * q2;
//     float q1q3 = q1 * q3;
//     float q2q2 = q2 * q2;
//     float q2q3 = q2 * q3;
//     float q3q3 = q3 * q3;

//     // 正常静止状态为-g 反作用力。
//     if (icm->acc_x * icm->acc_y * icm->acc_z == 0) // 加计处于自由落体状态时(此时g = 0)不进行姿态解算，因为会产生分母无穷大的情况
//         return;

//     // 对加速度数据进行归一化 得到单位加速度 (a^b -> 载体坐标系下的加速度)
//     float norm = FSqrt(icm->acc_x * icm->acc_x + icm->acc_y * icm->acc_y + icm->acc_z * icm->acc_z);
//     icm->acc_x = icm->acc_x / norm;
//     icm->acc_y = icm->acc_y / norm;
//     icm->acc_z = icm->acc_z / norm;

//     // 载体坐标系下重力在三个轴上的分量
//     vx = 2 * (q1q3 - q0q2);
//     vy = 2 * (q0q1 + q2q3);
//     vz = q0q0 - q1q1 - q2q2 + q3q3;

//     // g^b 与 a^b 做向量叉乘，得到陀螺仪的校正补偿向量e的系数
//     ex = icm->acc_y * vz - icm->acc_z * vy;
//     ey = icm->acc_z * vx - icm->acc_x * vz;
//     ez = icm->acc_x * vy - icm->acc_y * vx;

//     // 误差累加
//     I_ex += halfT * ex;
//     I_ey += halfT * ey;
//     I_ez += halfT * ez;

//     // 使用PI控制器消除向量积误差(陀螺仪漂移误差)
//     icm->gyro_x = icm->gyro_x + icm_kp * ex + icm_ki * I_ex;
//     icm->gyro_y = icm->gyro_y + icm_kp * ey + icm_ki * I_ey;
//     icm->gyro_z = icm->gyro_z + icm_kp * ez + icm_ki * I_ez;

//     // 一阶龙格库塔法求解四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为b系陀螺仪角速度。
//     q0 = q0 + (-q1 * icm->gyro_x - q2 * icm->gyro_y - q3 * icm->gyro_z) * halfT;
//     q1 = q1 + (q0 * icm->gyro_x + q2 * icm->gyro_z - q3 * icm->gyro_y) * halfT;
//     q2 = q2 + (q0 * icm->gyro_y - q1 * icm->gyro_z + q3 * icm->gyro_x) * halfT;
//     q3 = q3 + (q0 * icm->gyro_z + q1 * icm->gyro_y - q2 * icm->gyro_x) * halfT;

//     // 单位化四元数在空间旋转时不会拉伸，仅有旋转角度，下面算法类似线性代数里的正交变换
//     norm      = FSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
//     Q_info.q0 = q0 / norm;
//     Q_info.q1 = q1 / norm;
//     Q_info.q2 = q2 / norm;
//     Q_info.q3 = q3 / norm; // 用全局变量记录上一次计算的四元数值

//     eulerAngle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180 / PI;                                 // pitch
//     eulerAngle.roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / PI; // roll
//     eulerAngle.yaw   = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / PI; // yaw、
// }

// // GyroOffsets = gyroOffsetInit();
// // icm_data    = icmGetValues(GyroOffsets);
// // icmAHRSupdate(&icm_data);