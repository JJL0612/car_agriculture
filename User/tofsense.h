#ifndef __TOFSENSE_H
#define __TOFSENSE_H

#include "usart.h"

/* ======================================================================
 * TOFSense 激光测距驱动（NLink 协议）
 * ----------------------------------------------------------------------
 * 支持两种接线方式：
 *  1) 级联查询模式（推荐，本工程默认）：三个模块的 TX 并到一起接
 *     USART3_RX(PB11)，模块 TX 与 MCU RX 共地；MCU 通过 USART3_TX(PC10)
 *     发查询帧，按 ID 逐个读取。UART 级联最多支持 8 个。
 *  2) 主动输出模式：每个模块单独接一路串口（USART3/USART6 等），
 *     模块上电后自动 30Hz 输出，驱动按帧内 ID 自动归档。
 * 三个模块的 ID 需用 NAssistant 上位机分别设为 0、1、2。
 * ====================================================================== */

/* ========== 配置 ========== */
#define TOFSENSE_NUM_SENSORS        3        /* 传感器数量 */
#define TOFSENSE_CASCADE_UART       huart3   /* 级联/查询串口：USART3 */
#define TOFSENSE_CASCADE_BAUD       921600   /* TOFSense 默认波特率 */
#define TOFSENSE_QUERY_MODE         1        /* 1=查询模式(级联)；0=主动输出模式 */
#define TOFSENSE_QUERY_INTERVAL_MS  30       /* 每 30ms 查询一个传感器 */
#define TOFSENSE_RX_BUF_SIZE        64       /* DMA 空闲接收缓冲 */
#define TOFSENSE_ENABLE_UART6_BUS   0        /* 1=同时启用 USART6 作第二路总线 */

/* ========== 调试打印（排查级联问题时打开，联调完成后改成 0） ========== */
#define TOFSENSE_DEBUG_ENABLE       1
#define TOFSENSE_DEBUG_UART         (&huart1)   /* 与 yaokong 共用一个调试口：USART1 PA9，115200 */

/* ========== 数据结构 ========== */
typedef struct
{
    uint8_t  id;               /* 模块 ID（帧内自带，级联时用于区分） */
    float    distance_m;       /* 距离，单位 m（无目标时可能为负） */
    float    distance_cm;      /* 距离，单位 cm */
    uint8_t  dis_status;       /* 距离状态：0=有效，255=无目标，其余见表5 */
    uint16_t signal_strength;  /* 信号强度，越大信号越强 */
    uint8_t  range_precision;  /* 距离精度(cm)，仅 TOFSense-F 有效 */
    uint32_t system_time;      /* 模块上电时间，单位 ms */
    uint32_t last_update;      /* 最近一次收到该模块数据的时间(HAL_GetTick) */
    uint8_t  valid;            /* 1=校验通过且 dis_status==0（测量有效） */
} tofsense_data_t;

/* ========== 全局数据 ========== */
extern tofsense_data_t tofsense[TOFSENSE_NUM_SENSORS];
extern uint8_t tofsense_rx_buf[TOFSENSE_RX_BUF_SIZE];
#if TOFSENSE_ENABLE_UART6_BUS
extern uint8_t tofsense_uart6_rx_buf[TOFSENSE_RX_BUF_SIZE];
#endif

/* ========== 调试用全局变量 ==========
 * 解析出最新一帧时同步镜像到下面这些扁平数组，
 * 调试器（CLion/Keil Watch 窗口）里直接查看即可，无需展开结构体。
 * 下标 = 槽位 0/1/2，对应 ID 0/1/2（安装位置左/中/右，自行约定）。 */
extern float   tofsense_distance_cm[TOFSENSE_NUM_SENSORS]; /* 各槽位当前距离(cm)，无目标时为负 */
extern float   tofsense_distance_m[TOFSENSE_NUM_SENSORS];  /* 各槽位当前距离(m) */
extern uint8_t tofsense_valid[TOFSENSE_NUM_SENSORS];       /* 1=校验通过且测量有效(dis_status==0) */
extern uint32_t tofsense_last_update[TOFSENSE_NUM_SENSORS];/* 最近一次更新时刻(HAL_GetTick) */

/* ========== 诊断统计（排查"只有某一路有数据"时用） ==========
 * valid_frame_count[s] 持续增长 = 该槽位确实收到了合法帧；
 * bad_frame_count 增长但 valid 不增长 = 收到数据但校验失败（总线冲突/波特率不匹配）；
 * 两者都不增长 = 模块根本没应答（ID 没配好 / 没收到查询帧 / RX 没接）。 */
extern uint32_t tofsense_query_count;          /* 已发送查询帧总数 */
extern uint32_t tofsense_rx_callback_count;    /* DMA 空闲中断回调次数 */
extern uint16_t tofsense_last_rx_size;         /* 最近一次回调收到的字节数 */
extern uint32_t tofsense_valid_frame_count[TOFSENSE_NUM_SENSORS]; /* 每槽位校验通过的帧数 */
extern uint32_t tofsense_bad_frame_count;      /* 帧头正确但校验失败的帧数 */

/* ========== 函数 ========== */

/**
 * @brief  初始化：重配级联串口为 921600/8N1，启动 DMA 空闲接收，
 *         查询模式下立即开始按 ID 轮询
 * @note   在 init() 中调用；USART3 的 GPIO/DMA/NVIC 已由 CubeMX 配好
 */
void tofsense_init(void);

/**
 * @brief  处理串口收到的 NLink 原始数据，扫描合法 Frame0 并更新对应槽位
 * @param  huart 数据来源串口（用于清除错误标志）
 * @param  data  接收缓冲区指针
 * @param  size  本次接收到的字节数
 * @note   在 HAL_UARTEx_RxEventCallback 中调用（同 jy62/yaokong 风格）
 */
void tofsense_process_data(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);

/**
 * @brief  向指定 ID 的模块发送查询帧（57 10 FF FF id FF FF sum）
 * @param  id  目标模块 ID
 */
void tofsense_send_read(uint8_t id);

/**
 * @brief  查询调度：每隔 TOFSENSE_QUERY_INTERVAL_MS 查询下一个传感器
 * @note   查询模式下在主循环中周期调用；主动输出模式下为空操作
 */
void tofsense_poll(void);

/**
 * @brief  按槽位获取传感器数据指针
 * @param  slot  0 ~ TOFSENSE_NUM_SENSORS-1
 * @return 数据指针；slot 越界时返回 NULL
 */
tofsense_data_t *tofsense_get(uint8_t slot);

/**
 * @brief  读取指定槽位的当前距离（单位：cm）
 * @param  slot  0 ~ TOFSENSE_NUM_SENSORS-1
 * @return 当前距离(cm)；slot 越界时返回 -1
 * @note   直接读全局镜像 tofsense_distance_cm[slot]，非阻塞；
 *         数据新鲜度可用 tofsense_last_update[slot] 与 HAL_GetTick() 对比判断
 */
float tofsense_get_distance_cm(uint8_t slot);

/**
 * @brief  诊断打印（每 500ms 一次，自动限频）
 * @note   输出每个槽位的 有效性/距离/信号强度/数据年龄/累计帧数，以及
 *         查询数、DMA 回调数、坏帧数 —— 用于排查"某一路没数据"：
 *         - 该槽 valid 帧数不涨 & 坏帧数不涨  → 模块没应答（查 ID/接线）
 *         - 坏帧数持续涨                       → 总线冲突或波特率不一致
 * @note   在 main 循环中调用（与 tofsense_poll 并列）；关闭
 *         TOFSENSE_DEBUG_ENABLE 即停
 */
void tofsense_debug_task(void);

#endif /* __TOFSENSE_H */
