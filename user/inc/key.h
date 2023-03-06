#ifndef _KEY_H_
#define _KEY_H_

#include "zf_common_headfile.h"
#include "headfile.h"

#define KEY1        (G0)
#define KEY2        (G1)
#define KEY3        (G2)
#define KEY4        (G3)

#define SWITCH1     (D14)
#define SWITCH2     (D15)

#define ROWMIN      0
#define ROW0MAX     3
#define ROW1MAX     4
#define ROW2MAX     1
#define ROW3MAX     2
#define ROW4MAX     2
#define ROW5MAX     2

#define BALANCE     1
#define VELOCITY    2
#define KALMAN      3
#define SYSTEM      4
#define DATA_OPTION 5

extern uint16 i;
extern uint8 key1_status, key2_status, key3_status, key4_status;
extern uint8 key1_last_status, key2_last_status, key3_last_status, key4_last_status;
extern uint8 key1_flag, key2_flag, key3_flag, key4_flag;

void Key_Switch(void);

#endif
