#ifndef __SDCARD_H__
#define __SDCARD_H__

#include "zf_common_headfile.h"
#include "headfile.h"

extern uint8 image_read_buffer[MT9V03X_H][MT9V03X_W];

void sdcardinit();
void sdcard_read();
void sdcard_write(int16 *image_write_buffer, uint16 size);

#endif /* CODE_SDCARD_H_ */