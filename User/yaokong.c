#include "yaokong.h"
#include "motor.h"
#include "tofsense.h"
#include "rc_switch.h"
#include "avoidance.h"
#include <string.h>
#include <stdio.h>

/* ========== SBUS 帧定义 ========== */
#define SBUS_FRAME_LEN       25    /* 0x0F + 22字节通道数据 + 1字节标志 + 0x00 */
#define SBUS_PENDING_MAX     (SBUS_FRAME_LEN - 1)  /* 跨段拼接最多缓存 24 字节 */

/* ========== SBUS 接收缓冲 ========== */

uint8_t sbus_rx_buf[256];          // DMA 接收缓冲（风格同 jy62）
volatile uint8_t  sbus_ready     = 0;
volatile uint8_t  sbus_failsafe  = 0;
volatile uint16_t sbus_ch[16]    = {0};

static volatile uint32_t sbus_last_tick = 0;  // 最近一次完整帧的时间戳

/* 跨段拼接缓冲：DMA 一次收到的数据可能刚好把一帧从中间断开，
 * 把上次的"尾巴"留到本次数据开头，保证任何字节偏移下都能拼出完整帧 */
static uint8_t sbus_pending[SBUS_PENDING_MAX];
static uint8_t sbus_pending_len = 0;

/* ========== 调试统计 ========== */

volatile uint32_t sbus_frame_count = 0;   // 成功解析的帧数
volatile uint32_t sbus_chunk_count = 0;   // DMA 回调次数
volatile uint16_t sbus_last_size   = 0;   // 最近一次 DMA 收到的字节数
volatile uint8_t  sbus_last_flags  = 0;   // 最近一帧的标志字节

static uint8_t sbus_last_head[8];         // 最近一次 DMA 数据前 8 字节（诊断协议类型）
static uint8_t sbus_last_head_len = 0;

/* ========== 调试用中间变量（全局，方便在调试器里查看） ========== */
/* volatile 防止被优化掉，保证调试器暂停时能看到最新值 */
volatile int speeds[6];
volatile int throttle, steer;
volatile int left, right;

/* ========== 工具函数 ========== */

static int clamp_i(int value, int min, int max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief  摇杆偏移量 -> 速度 的递进映射
 * @param  value 混控后的差值（正负代表方向）
 * @return 速度大小 0 ~ MOTOR_SPEED_MAX（方向由调用处的方向引脚决定）
 * @note   摇杆推得越小速度越小，推得越大速度越大；
 *         中位死区内输出 0；满行程正好输出 MOTOR_SPEED_MAX
 */
static int speed_map(int value)
{
    int v = value;

    /* 中位死区：摇杆轻微抖动不出速度 */
    if (v > -SBUS_DEADZONE && v < SBUS_DEADZONE)
    {
        return 0;
    }

    /* 满行程限幅（防止扩展行程把速度顶过头） */
    v = clamp_i(v, -SBUS_FULL_RANGE, SBUS_FULL_RANGE);

    /* 线性递进：满杆 = MOTOR_SPEED_MAX */
    v = v * MOTOR_SPEED_MAX / SBUS_FULL_RANGE;

    /* 可选指数曲线：低速段更细腻，手感更"递进"
     * 想要线性手感，把下面 4 行注释掉即可 */
    int sign = (v < 0) ? -1 : 1;
    v = sign * (v * sign) * (v * sign) / MOTOR_SPEED_MAX;

    return v < 0 ? -v : v;
}

/* ========== 电机方向控制 ========== */

/**
 * @brief  设置单个电机的转向（F/R 引脚）
 * @note   JGB37-520 五线电机只有一根方向线（F/R），高低电平切换正反转
 */
static void motor_set_dir(uint16_t fr_pin, GPIO_TypeDef *fr_port, uint8_t forward)
{
    HAL_GPIO_WritePin(fr_port, fr_pin,
                      forward ? MOTOR_FORWARD_LEVEL : MOTOR_REVERSE_LEVEL);
}

/* 右轮组：A/B/C 电机，F/R = AIN1/BIN1/CIN1，对应 speed1~3 */
static void set_right_direction(uint8_t forward)
{
    motor_set_dir(AIN1_Pin, AIN1_GPIO_Port, forward);
    motor_set_dir(BIN1_Pin, BIN1_GPIO_Port, forward);
    motor_set_dir(CIN1_Pin, CIN1_GPIO_Port, forward);
}

/* 左轮组：D/E/F 电机，F/R = DIN1/EIN1/FIN1，对应 speed4~6 */
static void set_left_direction(uint8_t forward)
{
    motor_set_dir(DIN1_Pin, DIN1_GPIO_Port, !forward); /* 4号电机(D)接线反相，遥控换向时取反 */
    motor_set_dir(EIN1_Pin, EIN1_GPIO_Port, forward);
    motor_set_dir(FIN1_Pin, FIN1_GPIO_Port, forward);
}

/* 记录当前两侧方向，用于安全换向检测（1 = 前进） */
static uint8_t left_dir_state  = 1;
static uint8_t right_dir_state = 1;

/* Both remote and automatic driving pass through this nonblocking guard.
 * Direction pins may change only after PWM has remained zero for BRAKE_MS.
 * A changing request restarts braking; after pin changes PWM stays zero again.
 */
static uint8_t drive_direction_ready(uint8_t left_fwd, uint8_t right_fwd)
{
    static uint8_t phase; /* 0=running, 1=braking, 2=direction settling */
    static uint8_t pending_left, pending_right;
    static uint32_t phase_start;
    uint32_t now = HAL_GetTick();

    if (phase == 2U) {
        motor_forward(0, 0, 0, 0, 0, 0);
        if (now - phase_start < SBUS_DIR_SETTLE_MS) return 0;
        phase = 0;
    }
    if (phase == 0U) {
        if (left_fwd == left_dir_state && right_fwd == right_dir_state) return 1;
        motor_forward(0, 0, 0, 0, 0, 0);
        pending_left = left_fwd;
        pending_right = right_fwd;
        phase_start = now;
        phase = 1;
        return 0;
    }

    motor_forward(0, 0, 0, 0, 0, 0);
    if (left_fwd != pending_left || right_fwd != pending_right) {
        pending_left = left_fwd;
        pending_right = right_fwd;
        phase_start = now;
        return 0;
    }
    if (now - phase_start < SBUS_DIR_BRAKE_MS) return 0;

    set_left_direction(pending_left);
    set_right_direction(pending_right);
    left_dir_state = pending_left;
    right_dir_state = pending_right;
    phase_start = now;
    phase = 2;
    return 0;
}

/* ========== 初始化 ========== */

/**
 * @brief  初始化遥控接收（USART2 + SBUS）
 * @note   USART2 在 CubeMX 里是 1M/8N1，这里重配为 SBUS 参数：
 *         100000 波特率、8E2（偶校验 + 2 停止位）
 *         注意：8E2 在 HAL 里必须配 WordLength=9B（8数据位+1校验位），
 *         配 8B + 偶校验 实际是 7数据位+1校验位，收到的字节会全部错位！
 * @warning STM32F407 的 USART 没有软件反相位（RXINV 只有 F3/F7/L4 有），
 *          SBUS 反相信号必须在硬件上先反相（反相器/三极管/74HC04）再接 PD6
 */
void yaokong_init(void)
{
    /* 重新配置 USART2（GPIO 已由 CubeMX 配好，HAL 会直接改写寄存器） */
    huart2.Init.BaudRate   = 100000;
    huart2.Init.WordLength = UART_WORDLENGTH_9B;   // 8E2：8数据位 + 1偶校验
    huart2.Init.StopBits   = UART_STOPBITS_2;
    huart2.Init.Parity     = UART_PARITY_EVEN;
    HAL_UART_Init(&huart2);

    /* 中断已在 CubeMX 中使能（USART2 + DMA1_Stream5），这里再确保一次 */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

    /* 复位拼接缓冲与统计 */
    sbus_pending_len = 0;
    sbus_ready       = 0;
    sbus_failsafe    = 0;
    sbus_last_tick   = 0;
    sbus_frame_count = 0;
    sbus_chunk_count = 0;
    sbus_last_size   = 0;
    sbus_last_flags  = 0;

    /* 启动 DMA 空闲接收（同 jy62）：IDLE 事件触发后由回调重新启动 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, sbus_rx_buf, sizeof(sbus_rx_buf));
}

/* ========== SBUS 帧解析（带跨段拼接） ========== */

/**
 * @brief  处理 DMA 收到的 SBUS 数据
 * @param  data 接收缓冲指针
 * @param  size 本次收到的字节数
 * @note   帧格式：0x0F + 22字节通道数据 + 1字节标志 + 0x00
 *         拼接逻辑：上次未凑齐的"尾巴"和本次数据拼成连续流，
 *         从后往前找最新一帧，保证帧被 DMA 切断/错位也能解析
 */
void yaokong_process_data(uint8_t *data, uint16_t size)
{
    static uint8_t buf[SBUS_PENDING_MAX + 256];   /* 拼接缓冲（24+256） */
    uint16_t total;
    int16_t  found = -1;

    sbus_chunk_count++;
    sbus_last_size = size;

    /* 保存本次数据前 8 字节，供调试诊断协议类型（0F=SBUS，C8/EE=CRSF） */
    sbus_last_head_len = (size > sizeof(sbus_last_head)) ?
                         (uint8_t)sizeof(sbus_last_head) : (uint8_t)size;
    memcpy(sbus_last_head, data, sbus_last_head_len);

    /* 清除可能存在的 ORE 溢出标志（不清除会锁死 UART），不丢弃数据 */
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
    }

    /* 同样清除帧错误标志 */
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_FE))
    {
        __HAL_UART_CLEAR_FEFLAG(&huart2);
    }

    /* 上次尾巴 + 本次数据 拼接成连续字节流 */
    total = (uint16_t)(sbus_pending_len + size);
    if (total > sizeof(buf))
    {
        total = (uint16_t)sizeof(buf);
    }
    memcpy(buf, sbus_pending, sbus_pending_len);
    memcpy(buf + sbus_pending_len, data, size);

    /* 从后往前找最新一帧：0x0F 开头 + 0x00 结尾（偏移 24） */
    if (total >= SBUS_FRAME_LEN)
    {
        for (int16_t pos = (int16_t)total - SBUS_FRAME_LEN; pos >= 0; pos--)
        {
            if (buf[pos] != 0x0F || buf[pos + SBUS_FRAME_LEN - 1] != 0x00)
            {
                continue;
            }
            found = pos;
            break;    /* 从后往前扫描，第一个命中即最新帧 */
        }
    }

    if (found >= 0)
    {
        uint16_t tail;

        /* 16 通道 × 11 bit 连续打包（小端位序），数据从 found+1 开始 */
        uint8_t  bits = 0;
        uint8_t  idx  = (uint8_t)found + 1;
        uint32_t val  = 0;

        for (uint8_t n = 0; n < 16; n++)
        {
            while (bits < 11)
            {
                val |= (uint32_t)buf[idx++] << bits;
                bits += 8;
            }
            sbus_ch[n] = (uint16_t)(val & 0x07FF);
            val >>= 11;
            bits -= 11;
        }

        /* 标志字节（偏移 23），兼容两种约定：
         * Futaba/Betaflight 约定：bit2=丢帧(0x04)  bit3=失控(0x08)
         * 部分资料约定：        bit4=失控(0x10)  bit5=丢帧(0x20)
         * 只有"失控"标志(0x08/0x10)才停车；"丢帧"(0x04/0x20)只是这一帧
         * 没赶上新的 RF 包，属正常现象（ELRS 常每隔一帧置位），
         * 真正的断链由 yaokong_control 里的 200ms 超时兜底 */
        sbus_last_flags = buf[found + 23];
        sbus_failsafe   = (sbus_last_flags & 0x18) ? 1 : 0;

        sbus_ready     = 1;
        sbus_last_tick = HAL_GetTick();
        sbus_frame_count++;

        /* 帧结束之后的字节可能是下一帧的开头，保留末尾最多 24 字节 */
        tail = (uint16_t)(total - (found + SBUS_FRAME_LEN));
        if (tail > SBUS_PENDING_MAX) tail = SBUS_PENDING_MAX;
        sbus_pending_len = (uint8_t)tail;
        memcpy(sbus_pending, buf + found + SBUS_FRAME_LEN, sbus_pending_len);
    }
    else
    {
        /* 本次没有完整帧：把末尾最多 24 字节留给下次拼接 */
        sbus_pending_len = (total > SBUS_PENDING_MAX) ?
                           SBUS_PENDING_MAX : (uint8_t)total;
        memcpy(sbus_pending, buf + total - sbus_pending_len, sbus_pending_len);
    }
}

/* ========== 调试打印 ========== */

#if YAOKONG_DEBUG_ENABLE
static void yaokong_debug_send(const char *str)
{
    HAL_UART_Transmit(YAOKONG_DEBUG_UART, (const uint8_t *)str,
                      (uint16_t)strlen(str), 20);
}

/**
 * @brief  调试打印（每 500ms 一次，自动限频）
 * @note   输出内容：
 *         [SBUS] 回调次数 / 每次字节数 / 帧数 / 标志字节 / 链路状态
 *         head: 最近一次 DMA 数据的前 8 字节
 *               -> 0F开头 = SBUS 正常；C8/EE = 还是 CRSF；乱码 = 反相/波特率/校验问题
 *         ch:   CH1~CH8 原始值（中位 992）
 */
void yaokong_debug_task(void)
{
    static uint32_t last_print = 0;
    static uint32_t last_frm   = 0;
    uint32_t now = HAL_GetTick();
    char line[96];
    uint8_t link;

    if ((now - last_print) < 500) return;    /* 每 500ms 打印一次 */
    last_print = now;

    link = (sbus_ready && !sbus_failsafe && (now - sbus_last_tick) <= 200) ? 1 : 0;

    (void)snprintf(line, sizeof(line),
        "[SBUS] chunk=%lu size=%u frm=%lu flags=0x%02X %s\r\n",
        (unsigned long)sbus_chunk_count, sbus_last_size,
        (unsigned long)sbus_frame_count, sbus_last_flags,
        link ? "LINK-OK" : (sbus_failsafe ? "FAILSAFE" : "LINK-LOST"));
    yaokong_debug_send(line);

    (void)snprintf(line, sizeof(line),
        "  head: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
        sbus_last_head[0], sbus_last_head[1], sbus_last_head[2], sbus_last_head[3],
        sbus_last_head[4], sbus_last_head[5], sbus_last_head[6], sbus_last_head[7]);
    yaokong_debug_send(line);

    if (sbus_frame_count != last_frm)        /* 有新帧才打印通道值 */
    {
        last_frm = sbus_frame_count;
        (void)snprintf(line, sizeof(line),
            "  ch: %u %u %u %u | %u %u %u %u\r\n",
            sbus_ch[0], sbus_ch[1], sbus_ch[2], sbus_ch[3],
            sbus_ch[4], sbus_ch[5], sbus_ch[6], sbus_ch[7]);
        yaokong_debug_send(line);
    }
}
#else
void yaokong_debug_task(void)
{
}
#endif

/* ========== 遥控控制 ========== */

_Static_assert(SBUS_CH_SWITCH_1 < 16U, "Invalid switch 1 channel");
_Static_assert(SBUS_CH_SERVO < 16U, "Invalid servo channel");
_Static_assert(SBUS_CH_AVOIDANCE < 16U, "Invalid avoidance channel");
_Static_assert(SBUS_CH_SWITCH_2 < 16U, "Invalid switch 2 channel");
_Static_assert(SBUS_CH_SWITCH_3 < 16U, "Invalid switch 3 channel");
_Static_assert(SBUS_SWITCH_ON_VALUE <= 2047U, "Invalid SBUS switch value");

uint8_t yaokong_link_ok(void)
{
    return sbus_ready && !sbus_failsafe &&
           (HAL_GetTick() - sbus_last_tick) <= 200U;
}

/**
 * @brief  遥控控制主函数（在主循环中调用）
 * @note   右摇杆：上推=前进，下推=后退，左推=左转，右推=右转，中位=静止
 *         差速控制：左轮 = 油门 + 转向，右轮 = 油门 - 转向
 *         转向量超过油门量时，对应侧电机反转，实现原地转向
 */
void yaokong_control(void)
{
    yaokong_debug_task();    /* 联调阶段的调试打印（可关） */

    /* 未收到信号 / 失控 / 超过 200ms 无新帧：停车、关闭开关、舵机回中 */
    if (!yaokong_link_ok())
    {
        avoidance_reset();
        RC_Switch_AllOff();
        motor_forward(0, 0, 0, 0, 0, 0);
        servo_set_pulse_us(SERVO_CENTER_US);
        return;
    }

    /* CH3 / VOFA I8: PE5 = 1100/1500/1900, PE6 = 1900/1500/1100 us. */
    servo_set_remote_value(sbus_ch[SBUS_CH_SERVO]);

    /* Exact 1792 match, independent commands; switches 4..5 stay OFF. */
    uint8_t switch_mask = 0U;
    if (sbus_ch[SBUS_CH_SWITCH_1] == SBUS_SWITCH_ON_VALUE) {
        switch_mask |= 0x01U;
    }
    if (sbus_ch[SBUS_CH_SWITCH_2] == SBUS_SWITCH_ON_VALUE) {
        switch_mask |= 0x02U;
    }
    if (sbus_ch[SBUS_CH_SWITCH_3] == SBUS_SWITCH_ON_VALUE) {
        switch_mask |= 0x04U;
    }
    RC_Switch_SetMask(switch_mask);

    uint8_t auto_mode = sbus_ch[SBUS_CH_AVOIDANCE] == SBUS_AVOIDANCE_ON_VALUE;
    if (auto_mode) {
        AvoidanceInput input;
        /* Distances/status/timestamps must belong to the same received samples. */
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        for (unsigned i = 0; i < 3U; ++i) {
            input.cm[i] = tofsense_distance_cm[i];
            input.valid[i] = tofsense_valid[i];
            input.updated[i] = tofsense_last_update[i];
        }
        __set_PRIMASK(primask);
        int auto_left, auto_right;
        avoidance_step(&input, HAL_GetTick(), &auto_left, &auto_right);
        left = auto_left;
        right = auto_right;
    } else {
        avoidance_reset();
        /* 通道值减中位：-992 ~ +1055（标准行程约 -819 ~ +819） */
        throttle = (int)sbus_ch[SBUS_CH_THROTTLE] - SBUS_CENTER;
        steer    = (int)sbus_ch[SBUS_CH_STEER] - SBUS_CENTER;

        /* 中位死区，防摇杆抖动 */
        if (throttle > -SBUS_DEADZONE && throttle < SBUS_DEADZONE) throttle = 0;
        if (steer    > -SBUS_DEADZONE && steer    < SBUS_DEADZONE) steer    = 0;

        /* 差速：右推转向时右轮减速（甚至反转），小车右转 */
        left  = clamp_i(throttle + steer, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
        right = clamp_i(throttle - steer, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
    }

    /* Negative commands reverse; stopped wheels keep their current direction. */
    uint8_t left_fwd  = left == 0 ? left_dir_state : (left > 0);
    uint8_t right_fwd = right == 0 ? right_dir_state : (right > 0);

    if (!drive_direction_ready(left_fwd, right_fwd)) {
        for (unsigned i = 0; i < 6U; ++i) speeds[i] = 0;
        return;
    }

    /* F/R carries the sign; timer compare registers receive magnitude only. */
    int left_pwm = auto_mode ? (left < 0 ? -left : left) : speed_map(left);
    int right_pwm = auto_mode ? (right < 0 ? -right : right) : speed_map(right);
    speeds[MOTOR_L1] = speeds[MOTOR_L2] = speeds[MOTOR_L3] = left_pwm;
    speeds[MOTOR_R1] = speeds[MOTOR_R2] = speeds[MOTOR_R3] = right_pwm;

    motor_forward(speeds[0], speeds[1], speeds[2],
                  speeds[3], speeds[4], speeds[5]);
}

/* ========== 中断与回调 ========== */

/**
 * @brief  串口错误回调（偶校验/帧错误等，自动重新启动接收）
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, sbus_rx_buf, sizeof(sbus_rx_buf));
    }
    else if (huart->Instance == USART3)
    {
        /* TOFSense 级联串口出错时同样重新启动接收 */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, tofsense_rx_buf, sizeof(tofsense_rx_buf));
    }
#if TOFSENSE_ENABLE_UART6_BUS
    else if (huart->Instance == USART6)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, tofsense_uart6_rx_buf, sizeof(tofsense_uart6_rx_buf));
    }
#endif
}
