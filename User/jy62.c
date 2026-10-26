#include "jy62.h"
#include <string.h>

float yaw = 0;

uint8_t uart_rx_buf[256];

/* ========== CRC-16/MODBUS ========== */

static uint16_t crc16_modbus(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* ========== 发送命令帧 ========== */

/**
 * @brief  向 CY-Z 发送下行命令帧
 * @param  cmd   命令号
 * @param  param 命令参数
 * @note   帧格式: A5 5A Cmd Param Seq CRC[2] 5A
 */
static void cyz_send_cmd(uint8_t cmd, uint8_t param)
{
    static uint8_t cmd_seq = 0;
    uint8_t tx_buf[8];

    tx_buf[0] = 0xA5;
    tx_buf[1] = 0x5A;
    tx_buf[2] = cmd;
    tx_buf[3] = param;
    tx_buf[4] = cmd_seq++;

    // CRC 计算范围：Cmd + Param + Seq = 3 字节
    uint16_t crc = crc16_modbus(&tx_buf[2], 3);
    tx_buf[5] = (uint8_t)(crc & 0xFF);        // CRC 低字节
    tx_buf[6] = (uint8_t)((crc >> 8) & 0xFF);  // CRC 高字节
    tx_buf[7] = 0x5A;

    HAL_UART_Transmit(&huart1, tx_buf, 8, 10);
}

/* ========== 初始化和数据解析 ========== */

/**
 * @brief  启动 DMA 空闲接收 + 发送读取数据命令
 * @note   CY-Z 必须收到 Cmd=0x04 才会开始输出数据帧
 */
void jy62_init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buf, sizeof(uart_rx_buf));

    // 短暂延迟让 DMA 就绪
    HAL_Delay(10);

    // 发送读取命令：Cmd=0x04 读取陀螺仪角度和角速度
    cyz_send_cmd(0x04, 0x00);
}

/**
 * @brief  处理收到的 CY-Z 原始数据，扫描合法帧并更新 yaw
 * @param  data  接收缓冲区指针
 * @param  size  本次接收到的字节数
 * @note   在 HAL_UARTEx_RxEventCallback 中调用
 */
void cyz_process_data(uint8_t *data, uint16_t size)
{
    // 清除可能存在的 ORE 溢出标志（不清除会锁死 UART），但不丢弃数据
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
    }

    // 同样清除帧错误/噪声标志
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE))
    {
        __HAL_UART_CLEAR_FEFLAG(&huart1);
    }

    // 至少需要一个完整帧 (16 字节)
    if (size < 16) return;

    // 从后往前扫描，使用最新的一帧
    for (int16_t i = (int16_t)size - 16; i >= 0; i--)
    {
        // 检查帧头 AA 55
        if (data[i] != 0xAA || data[i + 1] != 0x55) continue;

        // 检查帧尾 55 AA
        if (data[i + 14] != 0x55 || data[i + 15] != 0xAA) continue;

        // 计算 CRC：Seq[2] + AngleDeg[4] + GyroDps[4] = 10 字节
        uint16_t crc_calc = crc16_modbus(&data[i + 2], 10);

        // 提取接收到的 CRC（little-endian）
        uint16_t crc_recv = (uint16_t)data[i + 12]
                          | ((uint16_t)data[i + 13] << 8);

        if (crc_calc != crc_recv) continue;

        // CRC 通过：提取 AngleDeg (float32 little-endian，偏移 4)
        uint32_t raw = (uint32_t)data[i + 4]
                     | ((uint32_t)data[i + 5] << 8)
                     | ((uint32_t)data[i + 6] << 16)
                     | ((uint32_t)data[i + 7] << 24);

        float angle;
        memcpy(&angle, &raw, 4);
        yaw = angle;
        break;  // 使用最新一帧，退出循环
    }
}

/**
 * @brief  角度解析（兼容旧接口，在主循环中调用）
 * @note   yaw 已在中断回调中更新，此函数保留作为备用/兼容
 */
void yaw_Calculate(void)
{
    // yaw 在 cyz_process_data() 中已更新
    // 这里不需要额外处理
}