#include "ANO.h"

unsigned char data_to_send[50];

void ANO_DT_send_int16byte16(short data1, short data2, short data3, short data4, short data5, short data6, short data7, short data8)
{
    unsigned char _cnt = 0;
    unsigned char sum = 0, i = 0;

    data_to_send[_cnt++] = 0xAA; // 匿名协议帧头  0xAAAA
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xF1; // 使用用户协议帧0xF1
    data_to_send[_cnt++] = 16;   // 8个int16_t 长度 16个字节

    data_to_send[_cnt++] = (unsigned short)(data1 >> 8);
    data_to_send[_cnt++] = (unsigned char)(data1);

    data_to_send[_cnt++] = (unsigned short)(data2 >> 8);
    data_to_send[_cnt++] = (unsigned char)(data2);

    data_to_send[_cnt++] = (unsigned short)(data3 >> 8);
    data_to_send[_cnt++] = (unsigned char)(data3);

    data_to_send[_cnt++] = (unsigned short)(data4 >> 8);
    data_to_send[_cnt++] = (unsigned char)(data4);

    data_to_send[_cnt++] = (unsigned short)(data5 >> 8);
    data_to_send[_cnt++] = (unsigned char)(data5);

    data_to_send[_cnt++] = (unsigned short)(data6 >> 8);
    data_to_send[_cnt++] = (unsigned char)(data6);

    data_to_send[_cnt++] = (unsigned short)(data7 >> 8);
    data_to_send[_cnt++] = (unsigned char)(data7);

    data_to_send[_cnt++] = (unsigned short)(data8 >> 8);
    data_to_send[_cnt++] = (unsigned char)(data8);

    sum = 0;
    for (i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

    // UART_PutBuff(UART1, data_to_send, _cnt); // 可以修改不同的串口发送数据;
}
