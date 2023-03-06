#ifndef _CAMERAPARAMS_H_
#define _CAMERAPARAMS_H_

#include "zf_common_headfile.h"
#include "headfile.h"
#include "imgStruct.h"

/*����ÿ��*/
extern const float32 PixelperMeter;

/*��͸�Ӿ���*/
extern const float32 INV_PERS[3][3];

/*��͸�Ӵ���任*/
extern uint8 Inv_x[22560];
extern uint8 Inv_y[22560];

#endif /* CODE_CAMERAPARAMS_H_ */
