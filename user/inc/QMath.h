#ifndef _QMATH_H_
#define _QMATH_H_

#include "zf_common_headfile.h"
#include "headfile.h"

/*开平方*/
float32 FSqrt(float32 x);

/*指数计算*/
float32 FExp(float32 x);

/*反正切，用于梯度方向计算*/
uint8 Atan2(float32 y, float32 x);

/*快速排序*/
void Quicksort(float32 array[], uint8 maxlen, uint8 begin, uint8 end);

/*数值交换*/
void Swap(float32 *a, float32 *b);

/*点交换*/
void SwapPoint(FLOAT_POINT_INFO *a, FLOAT_POINT_INFO *b);

/*指数计算*/
float32 QPow(float32 base, int8 exp);

/*绝对值*/
uint16 Kabs(int16 num);

/*浮点数绝对值函数*/
float32 Fabs(float32 num);

/*范围限制*/
int16 clip(int16 x, int16 low, int16 high);

/*浮点范围限制*/
float32 fclip(float32 x, float32 low, float32 high);

/*遍历寻找数组最大值*/
float32 fFindABSMax(float32 *f, int16 len);

/*三点计算曲率*/
float32 ThreePointsCurvature(INT_POINT_INFO t_P1, INT_POINT_INFO t_P2, INT_POINT_INFO t_P3);
float32 process_curvity(int16 x1, int16 y1, int16 x2, int16 y2, int16 x3, int16 y3);
float64 Fprocess_curvity(float32 x1, float32 y1, float32 x2, float32 y2, float32 x3, float32 y3);

/*线性回归*/
void Regression(INT_POINT_INFO *Point, float64 *Slope, float64 *Intersect, int16 startLine, int16 endLine);
void FRegression(FLOAT_POINT_INFO *Point, float64 *Slope, float64 *Intersect, int16 startLine, int16 endLine);

/*赛道方差绝对值计算*/
void Variance(INT_POINT_INFO *Point, float64 Slope, float64 Intersect, float64 *Variance, int16 startLine, int16 endLine);
void FVariance(FLOAT_POINT_INFO *Point, float64 Slope, float64 Intersect, float64 *Variance, int16 startLine, int16 endLine);

void LinearRegression(INT_POINT_INFO *Point, int16 startLine1, int16 endLine1, int16 startLine2, int16 endLine2);

#endif /* CODE_QMATH_H_ */
