#include "QMath.h"
#include "math.h"

/*开平方函数*/
float32 FSqrt(float32 x)
{
    float xhalf = 0.5f * x;
    int i       = *(int *)&x; // evil floating point bit level hacking
    // i = 0x5f3759df - (i >> 1);  // what the fuck?
    i = 0X5F3504F3 - (i >> 1); // 精度更高
    x = *(float *)&i;
    x = x * (1.5f - (xhalf * x * x));
    return 1 / x;
}

/*指数函数*/
float32 FExp(float32 x)
{
    x = 1.0 + x / 4096;

    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    x *= x;
    return x;
}

/*简易atan函数，判断4个方向*/
uint8 Atan2(float32 y, float32 x)
{
    /*--------------------------------------------
     *    角度范围        |         弧度范围
     *--------------------------------------------
     * 0     ~ 22.5  ------> 0         ~ 0.3926990
     * 22.5  ~ 67.5  ------> 0.3926990 ~ 1.1780972
     * 67.5  ~ 112.5 ------> 1.1780972 ~ 1.9634954
     * 112.5 ~ 157.5 ------> 1.9634954 ~ 2.7488935
     * 157.5 ~ 180   ------> 2.7488935 ~ 3.1415926
     *--------------------------------------------
     *         y/x值对应弧度
     *  0          ----  0.41421356  水平方向
     *  0.41421347 ----  2.41421356  右上、左下
     *  2.41421326 ---- -2.41421356  竖直方向
     * -2.41421362 ---- -0.41421356  左上、右下
     * -0.41421365 ----  0           水平方向
     ********************************************/

    float32 f32_tanNum;
    uint8 u8_Alpha; // 返回角度
    f32_tanNum = y / x;
    if (f32_tanNum > -0.41421365 && f32_tanNum < 0.41421356)
        u8_Alpha = 0; // 水平方向
    else if (f32_tanNum >= 0.41421356 && f32_tanNum < 2.41421356)
        u8_Alpha = 1; // 右上、左下
    else if (f32_tanNum <= -0.41421356 && f32_tanNum > -2.41421362)
        u8_Alpha = 3; // 左上、右下
    else
        u8_Alpha = 2; // 竖直方向

    return u8_Alpha; // 方向
}

/*快速排序*/
void Quicksort(float32 array[], uint8 maxlen, uint8 begin, uint8 end)
{
    uint8 i, j;

    if (begin < end) {
        i = begin + 1; // 将array[begin]作为基准数，因此从array[begin+1]开始与基准数比较！
        j = end;       // array[end]是数组的最后一位

        while (i < j) {
            if (array[i] > array[begin]) // 如果比较的数组元素大于基准数，则交换位置。
            {
                Swap(&array[i], &array[j]); // 交换两个数
                j--;
            } else
                i++; // 将数组向后移一位，继续与基准数比较。
        }

        if (array[i] >= array[begin]) // 这里必须要取等“>=”，否则数组元素由相同的值时，会出现错误！
            i--;

        Swap(&array[begin], &array[i]); // 交换array[i]与array[begin]

        Quicksort(array, maxlen, begin, i);
        Quicksort(array, maxlen, j, end);
    }
}

/*交换数值*/
void Swap(float32 *a, float32 *b)
{
    float temp;

    temp = *a;
    *a   = *b;
    *b   = temp;
}

/*交换点*/
void SwapPoint(FLOAT_POINT_INFO *a, FLOAT_POINT_INFO *b)
{
    float temp;

    temp      = a->m_i16x;
    a->m_i16x = b->m_i16x;
    b->m_i16x = temp;

    temp      = a->m_i16y;
    a->m_i16y = b->m_i16y;
    b->m_i16y = temp;
}

/*指数计算*/
float32 QPow(float32 base, int8 exp)
{
    float32 f32_ans = 1;
    while (exp) {
        if (exp & 1) f32_ans *= base;
        base *= base;
        exp >>= 1;
    }
    return f32_ans;
}

/*绝对值函数*/
uint16 Kabs(int16 num)
{
    if (num >= 0) return num;
    return -num;
}

/*浮点数绝对值函数*/
float32 Fabs(float32 num)
{
    if (num >= 0) return num;
    return -num;
}

/*范围限制函数*/
int16 clip(int16 x, int16 low, int16 high)
{
    return x > high ? high : x < low ? low
                                     : x;
}

/*浮点范围限制*/
float32 fclip(float32 x, float32 low, float32 high)
{
    return x > high ? high : x < low ? low
                                     : x;
}

/*遍历寻找数组最大值*/
float32 fFindABSMax(float32 *f, int16 len)
{
    float32 f32_Max   = 0;
    int16 int16_loopi = -1;

    while (++int16_loopi < len) {
        if (fabs(*(f + int16_loopi)) > fabs(f32_Max)) {
            f32_Max = *(f + int16_loopi);
        }
    }

    return f32_Max;
}

/*三点计算曲率*/
// 这个曲率计算的函数有点bug
// float32 ThreePointsCurvature(INT_POINT_INFO t_P1, INT_POINT_INFO t_P2, INT_POINT_INFO t_P3)
//{
//     float32 f32_K;
//     int16 f32_AB = (t_P2.m_i16x - t_P1.m_i16x) * (t_P2.m_i16x - t_P1.m_i16x) + (t_P2.m_i16y - t_P1.m_i16y) * (t_P2.m_i16y - t_P1.m_i16y);
//     int16 f32_AC = (t_P3.m_i16x - t_P1.m_i16x) * (t_P3.m_i16x - t_P1.m_i16x) + (t_P3.m_i16y - t_P1.m_i16y) * (t_P3.m_i16y - t_P1.m_i16y);
//     int16 f32_BC = (t_P2.m_i16x - t_P3.m_i16x) * (t_P2.m_i16x - t_P3.m_i16x) + (t_P2.m_i16y - t_P3.m_i16y) * (t_P2.m_i16y - t_P3.m_i16y);
//
//     /*弯道向右拐面积为负,否则为正*/
//     int16 f32_S = ((t_P2.m_i16x - t_P1.m_i16x) * (t_P3.m_i16y - t_P1.m_i16y) - (t_P3.m_i16x - t_P1.m_i16x) * (t_P2.m_i16y - t_P1.m_i16y));
//
//
//     if(f32_AB * f32_AC * f32_BC == 0) f32_K = 0;
//     else f32_K = 2 * f32_S /FSqrt( (float32) (f32_AB * f32_AC * f32_BC) );
//
//
//     return f32_K;
// }

float32 process_curvity(int16 x1, int16 y1, int16 x2, int16 y2, int16 x3, int16 y3)
{
    float32 K;
    int32 S_of_ABC = ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2;
    // 面积的符号表示方向
    int32 q1 = (int16)((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    int32 AB = (int32)sqrt((float64)q1);
    q1       = (int32)((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
    int32 BC = (int32)sqrt((float64)q1);
    q1       = (int32)((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
    int32 AC = (int32)sqrt((float64)q1);
    if (AB * BC * AC == 0) {
        K = 0;
    } else
        K = (float64)4 * S_of_ABC / (AB * BC * AC);
    return K;
}

// 这里之前使用float32似乎会溢出
float64 Fprocess_curvity(float32 x1, float32 y1, float32 x2, float32 y2, float32 x3, float32 y3)
{
    float64 K;
    float64 S_of_ABC = ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2;
    // 面积的符号表示方向
    float64 q1 = ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    float64 AB = sqrt(q1);
    q1         = ((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
    float64 BC = sqrt(q1);
    q1         = ((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
    float64 AC = sqrt(q1);
    if (AB * BC * AC == 0) {
        K = 0;
    } else
        K = 4 * S_of_ABC / (AB * BC * AC);
    return K;
}

/*线性回归，选取一段数据进行拟合*/
void Regression(INT_POINT_INFO *Point, float64 *Slope, float64 *Intersect, int16 startLine, int16 endLine)
{
    int16 int16_LineNum = endLine - startLine;

    int32 int16_sumX = 0;
    int32 int16_sumY = 0;
    float32 f32_aveX = 0;
    float32 f32_aveY = 0;

    float32 f32_sumUP = 0;
    ;
    float32 f32_sumDOWN = 0;

    *Slope     = 0;
    *Intersect = 0;
    float32 f32_k;
    float32 f32_b;

    if (int16_LineNum != 0) {
        /*拟合左边线*/
        for (int16 i = startLine; i < endLine; i++) {
            int16_sumX += Point[i].m_i16x;
            int16_sumY += Point[i].m_i16y;
        }

        f32_aveX = (float32)int16_sumX / int16_LineNum;
        f32_aveY = (float32)int16_sumY / int16_LineNum;

        for (int16 i = startLine; i < endLine; i++) {
            f32_sumUP += (Point[i].m_i16x - f32_aveX) * (Point[i].m_i16y - f32_aveY);
            f32_sumDOWN += (Point[i].m_i16x - f32_aveX) * (Point[i].m_i16x - f32_aveX);
        }

        /*直线斜率*/
        if (f32_sumDOWN == 0)
            f32_k = 0;
        else
            f32_k = f32_sumUP / f32_sumDOWN;

        /*截距*/
        f32_b = f32_aveY - f32_aveX * f32_k;
    }

    *Slope     = f32_k;
    *Intersect = f32_b;
}

/*线性回归,选取两端数据进行拟合*/
void LinearRegression(INT_POINT_INFO *Point, int16 startLine1, int16 endLine1, int16 startLine2, int16 endLine2)
{
    int16 int16_LineNum1 = endLine1 - startLine1;
    int16 int16_LineNum2 = endLine2 - startLine2;
    int16 int16_LineNum  = int16_LineNum1 + int16_LineNum2;

    int32 int16_sumX = 0;
    int32 int16_sumY = 0;
    float32 f32_aveX = 0;
    float32 f32_aveY = 0;

    float32 f32_sumUP = 0;
    ;
    float32 f32_sumDOWN = 0;

    float32 f32_k;
    float32 f32_b;

    if (int16_LineNum != 0) {
        for (int16 i = startLine1; i < endLine1; i++) {
            int16_sumX += Point[i].m_i16x;
            int16_sumY += Point[i].m_i16y;
        }

        for (int16 i = startLine2; i < endLine2; i++) {
            int16_sumX += Point[i].m_i16x;
            int16_sumY += Point[i].m_i16y;
        }
        f32_aveX = (float32)int16_sumX / int16_LineNum;
        f32_aveY = (float32)int16_sumY / int16_LineNum;

        for (int16 i = startLine1; i < endLine1; i++) {
            f32_sumUP += (Point[i].m_i16x - f32_aveX) * (Point[i].m_i16y - f32_aveY);
            f32_sumDOWN += (Point[i].m_i16x - f32_aveX) * (Point[i].m_i16x - f32_aveX);
        }

        for (int16 i = startLine2; i < endLine2; i++) {
            f32_sumUP += (Point[i].m_i16x - f32_aveX) * (Point[i].m_i16y - f32_aveY);
            f32_sumDOWN += (Point[i].m_i16x - f32_aveX) * (Point[i].m_i16x - f32_aveX);
        }
        /*直线斜率*/
        if (f32_sumDOWN == 0)
            f32_k = 0;
        else
            f32_k = f32_sumUP / f32_sumDOWN;

        /*截距*/
        f32_b = f32_aveY - f32_aveX * f32_k;
    }
}

/*线性回归，选取一段数据进行拟合*/
void FRegression(FLOAT_POINT_INFO *Point, float64 *Slope, float64 *Intersect, int16 startLine, int16 endLine)
{
    int16 int16_LineNum = endLine - startLine;

    float64 int16_sumX = 0;
    float64 int16_sumY = 0;
    float64 f32_aveX   = 0;
    float64 f32_aveY   = 0;

    float64 f32_sumUP = 0;
    ;
    float64 f32_sumDOWN = 0;

    *Slope        = 0;
    *Intersect    = 0;
    float64 f32_k = 0;
    float64 f32_b = 0;

    if (int16_LineNum > 0) {
        for (int16 i = startLine; i < endLine; i++) {
            int16_sumX += Point[i].m_i16x;
            int16_sumY += Point[i].m_i16y;
        }

        f32_aveX = (float64)int16_sumX / int16_LineNum;
        f32_aveY = (float64)int16_sumY / int16_LineNum;

        for (int16 i = startLine; i < endLine; i++) {
            f32_sumUP += (Point[i].m_i16x - f32_aveX) * (Point[i].m_i16y - f32_aveY);
            f32_sumDOWN += (Point[i].m_i16x - f32_aveX) * (Point[i].m_i16x - f32_aveX);
        }

        /*斜率*/
        if (f32_sumDOWN == 0)
            f32_k = 0;
        else
            f32_k = f32_sumUP / f32_sumDOWN;

        /*截距*/
        f32_b = f32_aveY - f32_aveX * f32_k;
    }

    *Slope     = f32_k;
    *Intersect = f32_b;
}

void Variance(INT_POINT_INFO *Point, float64 Slope, float64 Intersect, float64 *Variance, int16 startLine, int16 endLine)
{
    int16 int16_LineNum      = endLine - startLine;
    float64 AbsoluteVariance = 0;
    *Variance                = 0;

    if (int16_LineNum > 0 && Slope != 0) {
        for (int16 i = startLine; i < endLine; i++) {
            AbsoluteVariance += Fabs((Point[i].m_i16y - Intersect) / Slope - Point[i].m_i16x);
        }
    }

    *Variance = AbsoluteVariance;
}

void FVariance(FLOAT_POINT_INFO *Point, float64 Slope, float64 Intersect, float64 *Variance, int16 startLine, int16 endLine)
{
    int16 int16_LineNum      = endLine - startLine;
    float64 AbsoluteVariance = 0;
    *Variance                = 0;

    if (int16_LineNum > 0 && Slope != 0) {
        for (int16 i = startLine; i < endLine; i++) {
            AbsoluteVariance += Fabs((Point[i].m_i16y - Intersect) / Slope - Point[i].m_i16x);
        }
    }

    *Variance = AbsoluteVariance;
}
