#ifndef _CROSS_H_
#define _CROSS_H_

#include "zf_common_headfile.h"
#include "headfile.h"
#include "imgStruct.h"
// #define CROSS_NONE 0
// #define CROSS_FAR 1
// #define CROSS_NEAR 2

typedef enum {
    CROSS_NONE,
    CROSS_FAR,
    CROSS_NEAR
} CrossStatus;
void DrawRemoteLine(TRACK_BORDER_INFO *p_Border);

void Check_Cross(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_Type);

void FindRemoteLine(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border);

/*根据边线角度变化率搜索远端角点*/
void FindRemoteCorner(TRACK_BORDER_INFO *p_Border);

/*中入十字检查
 * 判断条件
 * 1. 近端线全丢
 * 2. 找到左上右上L型拐点
 */
void Check_MIDCross(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_Type);

/*三拐点右斜入十字
 * 判断条件
 * 1. 左边线全丢
 * 2. 右边L型拐点
 *
 * 成立条件
 * 1. 找到右上L型拐点
 * 2. 找到左上L型拐点
 */
void RightThreeCornerCross(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_Type);

/*三拐点左斜入十字
 * 判断条件
 * 1. 右边线全丢
 * 2. 左边L型拐点
 *
 * 成立条件
 * 1. 找到左上L型拐点
 * 2. 找到右上L型拐点
 */
void LeftThreeCornerCross(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_Type);

/*二拐点右斜入十字
 * 两点斜入十字与P字特征相同，在两点斜入十字后，置右边两点斜入十字记忆标志位，后遇到十字后需要检查检查是否为出P字并刷掉记忆标志位，或遇到其他特殊元素也刷掉标志位。
 * 判断条件
 * 1. 左边线全丢
 * 2. 右边L型拐点
 *
 * 成立条件
 * 1. 找到右上L型拐点
 */

void SideCrossFindRemoteLine(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border);

// void RightTwoCornerCross(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_Type);

/*二拐点左斜入十字
 * 两点斜入十字与P字特征相同，在两点斜入十字后，置右边两点斜入十字记忆标志位，后遇到十字后需要检查检查是否为出P字并刷掉记忆标志位，或遇到其他特殊元素也刷掉标志位。
 * 判断条件
 * 1. 右边线全丢
 * 2. 左边L型拐点
 *
 * 成立条件
 * 1. 找到左上L型拐点
//  */
// void LeftTwoCornerCross(uint8 (*InImg)[IMGW], TRACK_BORDER_INFO *p_Border, TRACK_TYPE_INFO *p_Type);

#endif /* CODE_CROSS_H_ */
