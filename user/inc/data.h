#ifndef CODE_DATA_H_
#define CODE_DATA_H_

#include "zf_common_headfile.h"
#include "headfile.h"
#include "imgStruct.h"

/*-----------------------变量声明------------------------------------*/
#define RIGHT 1
#define LEFT  2
#define Pi    3.1415926

extern TRACK_BORDER_INFO g_Border;  /*全局图像提取的大部分信息都保存在这个结构体里面*/
extern TRACK_TYPE_INFO g_TrackType; /*赛道类型*/
extern LINE_ERROR_INFO g_LineError; /*存放图像与边线相关的偏差信息*/

extern uint16 Protect_Frame;

extern uint32 tim1;
extern uint32 tim2;

extern int16 BlackBlock_Monitor;

extern float32 Dist_Monitor;

extern int16 Pnt;

extern int16 GrowCount;
extern int16 GrowCount2;
extern int16 GrowCount3;
extern int16 GrowCount4;

extern uint8 BinaryImg[90][188];

/*-------------------------------函数声明---------------------------------*/
void data_init(void);
void Img_data_init(void);

#endif /* CODE_DATA_H_ */
