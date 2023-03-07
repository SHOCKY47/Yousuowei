#ifndef _CAMERAPARAMS_H_
#define _CAMERAPARAMS_H_

#include "zf_common_headfile.h"
#include "headfile.h"
#include "imgStruct.h"

/*����ÿ��*/
extern const float32 PixelperMeter;
// extern const float32 PixelperMeter1;
extern const float32 PixelperMeter2;

/*��͸�Ӿ���*/
// extern const float32 INV_PERS[3][3];

/*��͸�Ӵ���任*/
extern uint8 FInv_x[22560];
extern uint8 FInv_y[22560];
extern uint8 Inv_x2[22560];
extern uint8 Inv_y2[22560];

// extern uint8 Inv_x2[22560];
// extern uint8 Inv_y2[22560];
// extern uint8 Inv_x_imge2[22560];
// extern uint8 Inv_y_imge2[22560];

#endif /* CODE_CAMERAPARAMS_H_ */
