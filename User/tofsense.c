#include "tofsense.h"
#include <string.h>
#include <stdio.h>

/* ======================================================================
 * NLink TOFSense Frame0（UART 输出/查询应答帧，定长 16 字节）
 * ----------------------------------------------------------------------
 * 偏移  长度  内容
 * 0     2     帧头 0x57 0x00
 * 2     1     保留 0xFF
 * 3     1     id（模块 ID）
 * 4     4     system_time（小端 uint32，模块上电时间 ms）
 * 8     3     dis（小端 24bit 有符号整数，除以 1000 得米）
 * 11    1     dis_status（0=测量有效，255=未检测到目标）
 * 12    2     signal_strength（小端 uint16）
 * 14    1     range_precision（cm，仅 TOFSense-F 有效）
 * 15    1     SumCheck（前面所有字节求和后的最低字节）
 *
 * 查询帧（NLINK_TOFSENSE_READ_FRAME0，8 字节）：
 * 57 10 FF FF [id] FF FF [SumCheck]
 * ====================================================================== */

#define TOFSENSE_FRAME_SIZE           16
#define TOFSENSE_FRAME_HEADER         0x57
#define TOFSENSE_FRAME_MARK           0x00
#define TOFSENSE_FRAME_ID_OFFSET      3
#define TOFSENSE_FRAME_TIME_OFFSET    4
#define TOFSENSE_FRAME_DIS_OFFSET     8
#define TOFSENSE_FRAME_STATUS_OFFSET  11
#define TOFSENSE_FRAME_SIGNAL_OFFSET  12
#define TOFSENSE_FRAME_PRECISION_OFFSET 14
#define TOFSENSE_FRAME_CHECK_OFFSET   15

/* ========== 槽位配置 ========== */
/* 三个模块的 ID 用 NAssistant 分别设为 0、1、2；
 * 槽位 0/1/2 对应安装位置：左前/中间/右前（以车头方向为准） */
static const uint8_t tofsense_slot_id[TOFSENSE_NUM_SENSORS] = {0, 1, 2};

/* ========== 全局数据 ========== */
uint8_t tofsense_rx_buf[TOFSENSE_RX_BUF_SIZE];

#if TOFSENSE_ENABLE_UART6_BUS
uint8_t tofsense_uart6_rx_buf[TOFSENSE_RX_BUF_SIZE];
#endif

tofsense_data_t tofsense[TOFSENSE_NUM_SENSORS];

/* ========== 调试用全局变量（tofsense[] 的扁平镜像，解析时同步更新） ========== */
float   tofsense_distance_cm[TOFSENSE_NUM_SENSORS];
float   tofsense_distance_m[TOFSENSE_NUM_SENSORS];
uint8_t tofsense_valid[TOFSENSE_NUM_SENSORS];
uint32_t tofsense_last_update[TOFSENSE_NUM_SENSORS];

/* ========== 诊断统计 ========== */
uint32_t tofsense_query_count;
uint32_t tofsense_rx_callback_count;
uint16_t tofsense_last_rx_size;
uint32_t tofsense_valid_frame_count[TOFSENSE_NUM_SENSORS];
uint32_t tofsense_bad_frame_count;

/* ========== 查询调度状态 ========== */
static uint8_t  query_idx       = 0;
static uint32_t query_last_tick = 0;

/* ========== 工具函数 ========== */

/**
 * @brief  计算并校验一帧 NLink 数据的 SumCheck
 * @param  data     帧起始指针
 * @param  size     帧长度
 * @return 1=校验通过，0=失败
 */
static uint8_t tofsense_check_sum(const uint8_t *data, uint16_t size)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < size - 1; i++)
    {
        sum += data[i];
    }
    return (sum == data[size - 1]);
}

/* ========== 公开函数 ========== */

/**
 * @brief  初始化：重配级联串口为 921600/8N1，启动 DMA 空闲接收
 */
void tofsense_init(void)
{
    /* 初始化槽位（ID 预填，valid 清零） */
    for (uint8_t s = 0; s < TOFSENSE_NUM_SENSORS; s++)
    {
        tofsense[s].id          = tofsense_slot_id[s];
        tofsense[s].valid       = 0;
        tofsense[s].last_update = 0;
    }

    /* 把 USART3 重配为 TOFSense 默认波特率 921600/8N1
     * （GPIO/DMA/NVIC 已由 CubeMX 配好，HAL 直接改写寄存器） */
    TOFSENSE_CASCADE_UART.Init.BaudRate   = TOFSENSE_CASCADE_BAUD;
    TOFSENSE_CASCADE_UART.Init.WordLength = UART_WORDLENGTH_8B;
    TOFSENSE_CASCADE_UART.Init.StopBits   = UART_STOPBITS_1;
    TOFSENSE_CASCADE_UART.Init.Parity     = UART_PARITY_NONE;
    HAL_UART_Init(&TOFSENSE_CASCADE_UART);

    /* 启动 DMA 空闲接收（同 jy62/yaokong） */
    HAL_UARTEx_ReceiveToIdle_DMA(&TOFSENSE_CASCADE_UART,
                                 tofsense_rx_buf,
                                 sizeof(tofsense_rx_buf));

#if TOFSENSE_ENABLE_UART6_BUS
    /* 第二路总线：USART6 也按 921600 配置并启动接收 */
    huart6.Init.BaudRate   = TOFSENSE_CASCADE_BAUD;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits   = UART_STOPBITS_1;
    huart6.Init.Parity     = UART_PARITY_NONE;
    HAL_UART_Init(&huart6);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6,
                                 tofsense_uart6_rx_buf,
                                 sizeof(tofsense_uart6_rx_buf));
#endif

#if TOFSENSE_QUERY_MODE
    query_idx       = 0;
    query_last_tick = HAL_GetTick() - TOFSENSE_QUERY_INTERVAL_MS; /* 立即查询 */
#endif
}

/**
 * @brief  处理收到的 NLink 原始数据，扫描合法 Frame0 并更新对应槽位
 * @note   从后往前扫描，使用最新的一帧（同 jy62 的 cyz_process_data）
 */
void tofsense_process_data(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
    /* 诊断统计：DMA 回调次数 / 本次字节数 */
    tofsense_rx_callback_count++;
    tofsense_last_rx_size = size;

    /* 清除可能存在的 ORE 溢出标志（不清除会锁死 UART），但不丢弃数据 */
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
    }

    /* 同样清除帧错误/噪声标志 */
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))
    {
        __HAL_UART_CLEAR_FEFLAG(huart);
    }

    /* 至少需要一个完整帧 (16 字节) */
    if (size < TOFSENSE_FRAME_SIZE) return;

    /* 从后往前扫描，使用最新的一帧 */
    for (int16_t i = (int16_t)size - TOFSENSE_FRAME_SIZE; i >= 0; i--)
    {
        /* 检查帧头 57 00 */
        if (data[i] != TOFSENSE_FRAME_HEADER ||
            data[i + 1] != TOFSENSE_FRAME_MARK) continue;

        /* 校验和：前面所有字节求和的最低字节 */
        if (!tofsense_check_sum(&data[i], TOFSENSE_FRAME_SIZE))
        {
            tofsense_bad_frame_count++;   /* 有数据但校验失败：冲突/波特率/干扰 */
            continue;
        }

        /* 解析：id */
        uint8_t id = data[i + TOFSENSE_FRAME_ID_OFFSET];

        /* 按 ID 找槽位；未配置的 ID 若小于数量则直接落到同号槽位 */
        int8_t slot = -1;
        for (uint8_t s = 0; s < TOFSENSE_NUM_SENSORS; s++)
        {
            if (tofsense_slot_id[s] == id)
            {
                slot = (int8_t)s;
                break;
            }
        }
        if (slot < 0 && id < TOFSENSE_NUM_SENSORS)
        {
            slot = (int8_t)id;
        }
        if (slot < 0) continue; /* 非本车传感器，忽略并继续找更早的帧 */

        tofsense_data_t *ts = &tofsense[slot];

        /* 解析：system_time（小端 uint32） */
        ts->system_time = (uint32_t)data[i + TOFSENSE_FRAME_TIME_OFFSET]
                        | ((uint32_t)data[i + TOFSENSE_FRAME_TIME_OFFSET + 1] << 8)
                        | ((uint32_t)data[i + TOFSENSE_FRAME_TIME_OFFSET + 2] << 16)
                        | ((uint32_t)data[i + TOFSENSE_FRAME_TIME_OFFSET + 3] << 24);

        /* 解析：dis（小端 24bit 有符号，除以 1000 得米） */
        int32_t raw = (int32_t)data[i + TOFSENSE_FRAME_DIS_OFFSET]
                    | ((int32_t)data[i + TOFSENSE_FRAME_DIS_OFFSET + 1] << 8)
                    | ((int32_t)data[i + TOFSENSE_FRAME_DIS_OFFSET + 2] << 16);
        if (raw & 0x800000) raw |= 0xFF000000;   /* 符号扩展（无目标时为负） */

        ts->distance_m  = (float)raw / 1000.0f;
        ts->distance_cm = ts->distance_m * 100.0f;

        /* 解析：dis_status / signal_strength / range_precision */
        ts->dis_status      = data[i + TOFSENSE_FRAME_STATUS_OFFSET];
        ts->signal_strength = (uint16_t)data[i + TOFSENSE_FRAME_SIGNAL_OFFSET]
                            | ((uint16_t)data[i + TOFSENSE_FRAME_SIGNAL_OFFSET + 1] << 8);
        ts->range_precision = data[i + TOFSENSE_FRAME_PRECISION_OFFSET];

        /* 更新有效标志与时间戳 */
        ts->id          = id;
        ts->valid       = (ts->dis_status == 0);
        ts->last_update = HAL_GetTick();

        /* 同步镜像到调试用全局变量 */
        tofsense_distance_cm[slot]   = ts->distance_cm;
        tofsense_distance_m[slot]    = ts->distance_m;
        tofsense_valid[slot]         = ts->valid;
        tofsense_last_update[slot]   = ts->last_update;
        tofsense_valid_frame_count[slot]++;   /* 该槽位确实收到了合法帧 */

        break;  /* 使用最新一帧，退出循环 */
    }
}

/**
 * @brief  向指定 ID 的模块发送查询帧
 * @note   帧格式：57 10 FF FF [id] FF FF [SumCheck]
 *         SumCheck = 前 7 字节求和的最低字节
 */
void tofsense_send_read(uint8_t id)
{
    uint8_t buf[8];
    uint8_t sum = 0;

    buf[0] = 0x57;
    buf[1] = 0x10;
    buf[2] = 0xFF;
    buf[3] = 0xFF;
    buf[4] = id;
    buf[5] = 0xFF;
    buf[6] = 0xFF;

    for (uint8_t i = 0; i < 7; i++) sum += buf[i];
    buf[7] = sum;

    tofsense_query_count++;   /* 诊断统计：查询发送次数 */

    HAL_UART_Transmit(&TOFSENSE_CASCADE_UART, buf, 8, 10);
}

/**
 * @brief  查询调度：每隔 TOFSENSE_QUERY_INTERVAL_MS 查询下一个传感器
 */
void tofsense_poll(void)
{
#if TOFSENSE_QUERY_MODE
    if ((HAL_GetTick() - query_last_tick) >= TOFSENSE_QUERY_INTERVAL_MS)
    {
        query_last_tick = HAL_GetTick();
        tofsense_send_read(tofsense_slot_id[query_idx]);
        query_idx = (query_idx + 1) % TOFSENSE_NUM_SENSORS;
    }
#else
    /* 主动输出模式下模块自行 30Hz 输出，无需查询 */
#endif
}

/**
 * @brief  按槽位获取传感器数据指针
 */
tofsense_data_t *tofsense_get(uint8_t slot)
{
    if (slot >= TOFSENSE_NUM_SENSORS) return NULL;
    return &tofsense[slot];
}

/**
 * @brief  读取指定槽位的当前距离（单位：cm）
 * @param  slot  0 ~ TOFSENSE_NUM_SENSORS-1
 * @return 当前距离(cm)；slot 越界时返回 -1
 */
float tofsense_get_distance_cm(uint8_t slot)
{
    if (slot >= TOFSENSE_NUM_SENSORS) return -1.0f;
    return tofsense_distance_cm[slot];
}

/* ========== 诊断打印 ========== */

#if TOFSENSE_DEBUG_ENABLE
static void tofsense_debug_send(const char *str)
{
    HAL_UART_Transmit(TOFSENSE_DEBUG_UART, (const uint8_t *)str,
                      (uint16_t)strlen(str), 20);
}

/**
 * @brief  诊断打印（每 500ms 一次，自动限频）
 * @note   输出格式：
 *   [TOF] q=查询数 cb=DMA回调数 last=最近一次回调字节数 bad=坏帧数
 *   S0: OK/NO  dist=xx.xcm  sig=xxx  age=xxms  frm=xxx
 *   S1: ...
 *   S2: ...
 *   判读方法：
 *   - Sx 的 frm 不涨 且 bad 不涨     → 该模块没应答：ID 没配好 / 收不到查询 / RX 没接
 *   - bad 持续上涨                  → 总线上有数据但校验失败：多模块同 ID 冲突 /
 *                                     波特率不一致 / 有模块处于主动输出模式
 *   - 只有 S0 有 frm                → 其余两个模块没配 ID（默认都是 0）或没通电
 */
void tofsense_debug_task(void)
{
    static uint32_t last_print = 0;
    uint32_t now = HAL_GetTick();
    char line[128];

    if ((now - last_print) < 500) return;    /* 每 500ms 打印一次 */
    last_print = now;

    (void)snprintf(line, sizeof(line),
        "[TOF] q=%lu cb=%lu last=%u bad=%lu\r\n",
        (unsigned long)tofsense_query_count,
        (unsigned long)tofsense_rx_callback_count,
        tofsense_last_rx_size,
        (unsigned long)tofsense_bad_frame_count);
    tofsense_debug_send(line);

    for (uint8_t s = 0; s < TOFSENSE_NUM_SENSORS; s++)
    {
        uint32_t age = (now >= tofsense_last_update[s]) ?
                       (now - tofsense_last_update[s]) : 0xFFFFFFFFu;
        /* 距离用定点整数打印（mm），nano printf 不支持 %f */
        int32_t mm = (int32_t)(tofsense_distance_cm[s] * 10.0f);
        (void)snprintf(line, sizeof(line),
            "  S%u: %s dist=%ld.%ldcm sig=%4u age=%lu frm=%lu\r\n",
            s, tofsense_valid[s] ? "OK " : "NO ",
            (long)(mm / 10), (long)((mm < 0 ? -mm : mm) % 10),
            tofsense[s].signal_strength,
            (unsigned long)age,
            (unsigned long)tofsense_valid_frame_count[s]);
        tofsense_debug_send(line);
    }
}
#else
void tofsense_debug_task(void)
{
}
#endif
