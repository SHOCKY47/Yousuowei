#ifndef __ANO_H__
#define __ANO_H__

#include "zf_common_headfile.h"
#include "headfile.h"

extern unsigned char data_to_send[50];

void ANO_DT_send_int16byte16(short data1, short data2, short data3, short data4, short data5, short data6, short data7, short data8);

#endif