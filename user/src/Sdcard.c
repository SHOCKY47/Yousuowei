#include "Sdcard.h"
UINT bytesWritten;
UINT bytesRead;
UINT times;

FATFS fs;                     // 文件系统结构体
const TCHAR fs_drv[] = "1:/"; // 文件系统根目录 默认是两个部分 SD 卡在 1 部分

FRESULT error;             // 文件系统操作状态
DIR directory;             // 目录结构体变量
static FIL g_infileObject; // 文件结构体变量
static FIL g_outfileObject;
FILINFO fileInformation; // 文件信息结构体
uint8 image_read_buffer[MT9V03X_H][MT9V03X_W];

void sdcardinit()
{

    SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n Log -> fatfs_mkfs_test() example.");
    SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n Log -> f_mount() start.");
#if (FF_FS_RPATH >= 2U)                           // 默认两个部分
    error = f_chdrive((char const *)&fs_drv[0U]); // 切换目录
    if (error) {
        SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n ERROR -> Failed to change drive.");
        while (1)
            ;
    }
#endif
    while (1) {
        if (!f_mount(&fs, fs_drv, 1)) // 挂载文件系统
            break;
        SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n Log -> f_mount() error, try again after one second."); // 如果失败了就输出错误信息
        system_delay_ms(1000);                                                                                // 延时一秒后再试
    }
    SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n Log -> f_mount() success.");
}

void sdcard_read() // 读出sd卡的数组和大小
{

    error = f_open(&g_infileObject, "input.dat", (FA_READ)); // 读写权限创建文件

    error = f_read(&g_infileObject, image_read_buffer, sizeof(image_read_buffer), &bytesRead); // 从文件中读取出数据

    if ((error) || (bytesRead != sizeof(image_read_buffer))) // 确认是否出错 读取大小是否跟预期大小一致
    {
        SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n Warning -> Failed to read file.");

        f_close(&g_infileObject);
    }
}

void sdcard_write(int16 *image_write_buffer, uint16 size) // 写入sd卡的数组及大小
{
    error = f_open(&g_outfileObject, "output.dat", (FA_WRITE | FA_CREATE_ALWAYS)); // 读写权限创建文件

    if (error) {
        SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\r\n Warning -> Failed to open file.ERROR: %d", error);
    }

    error = f_write(&g_outfileObject, image_write_buffer, size, &bytesWritten);

    if (error) {
        SEGGER_RTT_printf(0, RTT_CTRL_TEXT_RED "\r\n Warning -> Failed to write file. ERROR: %d", error);
    }

    SEGGER_RTT_printf(0, RTT_CTRL_TEXT_GREEN "\r\n Log -> image write success.");

    f_close(&g_outfileObject);
}