#ifndef __YAOKONG_H
#define __YAOKONG_H

#include "usart.h"

/* ========== 调试开关（联调完成后改成 0 即可关闭打印） ========== */
#define YAOKONG_DEBUG_ENABLE    1
/* 调试输出串口：默认 USART1 的 TX（PA9，115200 8N1）
 * 把 PA9 接到 USB转TTL 的 RX，电脑串口助手开 115200 查看。
 * 如果板子的调试口是别的 UART，改这个宏即可。 */
#define YAOKONG_DEBUG_UART      (&huart1)

/* ========== 遥控通道配置 ========== */
/**
 * 当前遥控器实测通道映射：
 *   CH1 副翼(右摇杆左右)  CH2 升降(右摇杆上下)
 *   CH3 左摇杆左右  CH4 左摇杆上下
 * 本程序映射：速度(油门) = 右摇杆上下(CH2)，转向 = 右摇杆左右(CH1)
 * 注意：sbus_ch[0] 是 CH1，所以宏里填的是数组下标 = 通道号 - 1
 */
#define SBUS_CH_THROTTLE   1   /* 右摇杆上下 = CH2 升降（油门） */
#define SBUS_CH_STEER      0   /* 右摇杆左右 = CH1 副翼（方向） */
#define SBUS_CH_SERVO      2U  /* VOFA I8 = CH3 左摇杆左右，同时控制两个舵机 */
#define SBUS_CH_AVOIDANCE  7U  /* CH8 / VOFA I13: exactly 1792 enables avoidance */
#define SBUS_AVOIDANCE_ON_VALUE 1792U

/* 精确匹配实测原始值：CH5 / VOFA I10 控制开关1，
 * CH6 / VOFA I11 控制开关2，CH7 / VOFA I12 控制开关3；
 * 其他值关闭对应开关。开关4～5保持关闭。
 */
#define SBUS_CH_SWITCH_1        4U
#define SBUS_CH_SWITCH_2        5U
#define SBUS_CH_SWITCH_3        6U
#define SBUS_SWITCH_ON_VALUE   1792U

#define SBUS_CENTER        992 // 通道中位值（992 = 1500us）
#define SBUS_DEADZONE      40  // 中位死区，防止摇杆抖动
#define SBUS_FULL_RANGE    819 // 标准行程满偏值（992-172≈819），可改大兼容扩展行程
#define SBUS_DIR_BRAKE_MS        500U /* PWM清零后等待，再切F/R；需实车确认已停转 */
#define SBUS_DIR_SETTLE_MS       50U  /* F/R切换后继续保持PWM为0 */
#define MOTOR_SPEED_MAX    999// 电机最大 PWM（TIM1/TIM5 ARR=999，最大约 999）

/* ========== 六轮映射 ========== */
/**
 * speed1~3 = 右三个轮，speed4~6 = 左三个轮（沿用原 motor_forward 约定）
 * 若实际接线左右相反，把 MOTOR_L1~L3 与 MOTOR_R1~R3 两组宏对调即可
 */
#define MOTOR_R1   0
#define MOTOR_R2   1
#define MOTOR_R3   2
#define MOTOR_L1   3
#define MOTOR_L2   4
#define MOTOR_L3   5

/* ========== 全局数据 ========== */
extern uint8_t sbus_rx_buf[256];     // DMA 接收缓冲
extern volatile uint8_t  sbus_ready;     // 是否收到过合法 SBUS 帧
extern volatile uint8_t  sbus_failsafe;  // 失控/丢帧标志（1 = 不安全，停车）
extern volatile uint16_t sbus_ch[16];    // 16 通道原始值（0~2047，中位 992）
extern volatile uint32_t sbus_frame_count; // 成功解析帧数（调试）
extern volatile uint32_t sbus_chunk_count; // DMA 回调次数（调试）
extern volatile uint16_t sbus_last_size;   // 最近一次 DMA 收到字节数（调试）
extern volatile uint8_t  sbus_last_flags;  // 最近一帧标志字节（调试）

/* 调试用中间变量（yaokong_control 计算过程值，方便调试器查看） */
extern volatile int speeds[6];   // 六轮最终速度（speed1~3=右轮，speed4~6=左轮）
extern volatile int throttle;    // 右摇杆油门差值（中位0，满偏±819）
extern volatile int steer;       // 右摇杆转向差值（中位0，满偏±819）
extern volatile int left;        // 左侧混控速度（油门+转向，含方向）
extern volatile int right;       // 右侧混控速度（油门-转向，含方向）

/* ========== 函数 ========== */

/**
 * @brief  遥控初始化（USART2 + SBUS + DMA）
 * @note   在 init() 中调用；会把 USART2 重配为 100000 波特、8E2
 */
void yaokong_init(void);

/**
 * @brief  处理 DMA 收到的 SBUS 数据（带跨段拼接，任意偏移都能找帧）
 * @param  data 接收缓冲指针
 * @param  size 本次收到的字节数
 * @note   在 HAL_UARTEx_RxEventCallback 中调用（同 jy62 风格）
 */
void yaokong_process_data(uint8_t *data, uint16_t size);

/**
 * @brief  遥控控制主函数（主循环调用）
 * @note   右摇杆控制六轮差速，CH3 控制两个舵机，CH5/CH6/CH7 控制开关1/2/3；
 *         CH8 / VOFA I13 严格等于1792时，由避障逻辑接管车轮；其他值恢复原遥控；
 *         输入200/1000/1800：PE5输出1100/1500/1900us，PE6输出1900/1500/1100us；
 *         失联时停车、关开关并回中
 */
void yaokong_control(void);
/* Valid SBUS received, no failsafe, most recent frame within 200ms. */
uint8_t yaokong_link_ok(void);

/**
 * @brief  调试打印（每 500ms 一次，自动限频）
 * @note   在 yaokong_control() 中自动调用；关闭 YAOKONG_DEBUG_ENABLE 即停
 */
void yaokong_debug_task(void);

#endif /* __YAOKONG_H */
