#include "key.h"

uint8 key1_status    = 1;
uint8 key2_status    = 1;
uint8 key3_status    = 1;
uint8 key4_status    = 1;
uint8 SWITCH1_status = 1;
uint8 SWITCH2_status = 1;

uint8 key1_last_status, key2_last_status, key3_last_status, key4_last_status;
uint8 SWITCH1_last_status, SWITCH2_last_status;

uint8 key1_flag, key2_flag, key3_flag, key4_flag;
uint8 SWITCH1_flag, SWITCH2_flag;

uint16 i = 744;

void Key_Switch(void)
{

    key1_last_status    = key1_status;
    key2_last_status    = key2_status;
    key3_last_status    = key3_status;
    key4_last_status    = key4_status;
    SWITCH1_last_status = SWITCH1_status;
    SWITCH2_last_status = SWITCH1_status;

    key1_status = gpio_get_level(KEY1);
    key2_status = gpio_get_level(KEY2);
    key3_status = gpio_get_level(KEY3);
    key4_status = gpio_get_level(KEY4);

    SWITCH1_status = gpio_get_level(SWITCH1);
    SWITCH2_status = gpio_get_level(SWITCH2);

    if (key1_status && !key1_last_status)
        key1_flag = 1;
    if (key2_status && !key2_last_status)
        key2_flag = 1;
    if (key3_status && !key3_last_status)
        key3_flag = 1;
    if (key4_status && !key4_last_status)
        key4_flag = 1;
    if (!SWITCH1_status && SWITCH1_last_status)
        SWITCH1_flag = 1;
    if (!SWITCH2_status && SWITCH2_last_status)
        SWITCH2_flag = 1;

    if (key1_flag) {
        key1_flag = 0;

        i++;
    }
    if (key2_flag) {
        key2_flag = 0;

        i--;
    }
    if (key3_flag) {
        key3_flag = 0;
    }
    if (key4_flag) {
        key4_flag = 0;
    }
    if (SWITCH1_flag) {
        SWITCH1_flag = 1;
    }
    if (SWITCH2_flag) {
        SWITCH2_flag = 1;
    }
}
