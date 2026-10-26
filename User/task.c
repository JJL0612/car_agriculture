#include "task.h"
#include "encoder.h"
#include "yaokong.h"
#include "tofsense.h"

/* ========== 计时器内部变量 ========== */
static uint32_t timer_start_tick = 0;   // 起始时刻 (HAL_GetTick)
static uint8_t  timer_stopped_val = 0;  // 锁存的秒数
static uint8_t  timer_frozen      = 0;  // 是否已冻结
static uint8_t  timer_started     = 0;  // 计时器是否已启动

/* ========== 任务状态机 ========== */
typedef enum {
    TASK_IDLE = 0,      // 空闲，等待按键
    TASK_WAIT_2S = 1,   // 等待 2 秒（屏幕显示 00）
    TASK_RUNNING = 2    // 任务运行中
} TaskState;

static TaskState task_state     = TASK_IDLE;
static uint8_t   current_task   = 0;       // 当前选中的任务编号 (1/2/3)
static uint32_t  wait_start_tick = 0;      // 等待 2 秒的起始时刻

int KEY = 0;

/* ========== 公开函数 ========== */

void init(void)
{
    motor_init();

    yaokong_init();   // 遥控初始化（USART2 + SBUS）

    tofsense_init();  // TOFSense 测距初始化（USART3 + 级联查询）

    OLED_Init();      // OLED 显示初始化（SPI2 + SSD1306，用于实时显示测距距离）

}

/**
 * @brief  启动计时
 */
void Timer_Start(void)
{
    timer_start_tick  = HAL_GetTick();
    timer_stopped_val = 0;
    timer_frozen      = 0;
    timer_started     = 1;
}

/**
 * @brief  获取计时秒数 (0~99)
 *         计时器未启动时返回 0
 *         首次检测到 n==1 时锁存当前秒数，之后不再更新
 * @return 从 Timer_Start 开始经过的秒数
 */
uint8_t Timer_GetSeconds(void)
{
    uint32_t elapsed;
    uint8_t  seconds;

    // 计时器未启动，返回 0（屏幕显示 00）
    if (!timer_started) {
        return 0;
    }

    // 已冻结则直接返回锁存值
    if (timer_frozen) {
        return timer_stopped_val;
    }

    // 计算已过秒数
    elapsed = (HAL_GetTick() - timer_start_tick) / 1000;
    if (elapsed > 99) {
        seconds = 99;
    } else {
        seconds = (uint8_t)elapsed;
    }

    // 检测停止条件 (n 变为 1)
    if (n == 1) {
        timer_stopped_val = seconds;
        timer_frozen = 1;
    }

    return seconds;
}

/**
 * @brief  检测按键（仅在 IDLE 状态下响应）
 */
void task_check()
{
    // 只在空闲状态检测按键，防止运行中被其他按键打断
    if (task_state != TASK_IDLE) {
        return;
    }

    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
    {
        KEY = 1;
    }
    else if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
    {
        KEY = 2;
    }

}

/**
 * @brief  任务调度主函数（在 main 循环中调用）
 *
 *         状态机流程：
 *         IDLE      → 检测按键 → WAIT_2S
 *         WAIT_2S   → 等待 2 秒（非阻塞）→ RUNNING（启动计时器）
 *         RUNNING   → 执行当前任务
 */
void task_run()
{
    switch (task_state)
    {
        case TASK_IDLE:
            task_check();
            if (KEY != 0)
            {
                // 按键按下：重置状态，准备进入等待
                current_task = KEY;
                n = 0;
                i = 0;
                timer_started = 0;
                timer_frozen  = 0;
                wait_start_tick = HAL_GetTick();
                task_state = TASK_WAIT_2S;
            }
            break;

        case TASK_WAIT_2S:
            // 非阻塞等待 2 秒（期间屏幕由 main 循环持续刷新，显示 00）
            if ((HAL_GetTick() - wait_start_tick) >= 2000)
            {
                // 2 秒到：重置编码器距离，启动计时器，进入运行状态，小车开始启动
                Encoder_ResetDistance();
                Timer_Start();
                task_state = TASK_RUNNING;
            }
            break;

        case TASK_RUNNING:
            // 执行对应任务
            if (current_task == 1)
            {
                task_1();
            }
            else if (current_task == 2)
            {
                task_2();
            }
            else if (current_task == 3)
            {
                task_3();
            }
            break;
    }
}

void task_1()
{
    if (n == 0)
    {
        car_run_track();
    }
    else
    {
        motor_forward(0, 0, 0, 0, 0, 0);
    }
}

void task_2()
{
    if (n == 0)
    {
        float dist  = -Encoder_GetDistance();      // 获取当前平均路程 (cm)

        if (dist < 120.0f)
        {
            car_run_track_30s(135.0f);
        }
        else if (dist >= 120.0f && dist < 300.0f)
        {
            car_run_track_30s(180.0f);
        }
        else if (dist >= 300.0f && dist < 450.0f)
        {
            car_run_track_30s(210.0f);
        }
        else if (dist >= 450.0f && dist < 2000.0f)
        {
            car_run_track_30s(240.0f);
        }
        else if (dist >= 2000.0f && (yaw <= 10 && yaw > -45.0f))
        {
            car_run_track_30s(260.0f);
        }
        else if (dist >= 2000.0f && (yaw <= -45.0f && yaw > -90.0f))
        {
            car_run_track_30s(275.0f);
        }
        else if (dist >= 2000.0f && ((yaw <= 180.0f && yaw >= 155.0f) || (yaw <= -155.0f && yaw >= -180.0f)))
        {
            car_run_track_30s(285.0f);
        }
        else if(yaw < 155.0f && yaw >= 120.0f)
        {
            car_run_track_30s(270.0f);
        }
        else if(yaw < 120.0f && yaw >= 95.0f)
        {
            car_run_track_30s(240.0f);
        }
        else if(yaw < 95.0f && yaw >= 80.0f)
        {
            car_run_track_30s(210.0f);
        }
        else if(yaw < 80.0f && yaw >= 60.0f)
        {
            car_run_track_30s(170.0f);
        }
        else if(yaw < 60.0f && yaw >= 40.0f)
        {
            car_run_track_30s(130.0f);
        }
        else if(yaw < 40.0f && yaw >= 20.0f)
        {
            car_run_track_30s(110.0f);
        }

    }
    else
    {
        motor_forward(0, 0, 0, 0, 0, 0);
    }
}

void task_3()
{
    float dist  = -Encoder_GetDistance();      // 获取当前平均路程 (cm)
    //float left  = Encoder_GetLeftDistance();  // 获取左轮路程
    //float right = Encoder_GetRightDistance(); // 获取右轮路程

    if (dist < 100.0f)
    {
        car_run_track_8s(130.0f);
    }
    else if (dist < 200.0f && dist >= 100.0f)
    {
        car_run_track_8s(160.0f);
    }
    else if (dist < 300.0f && dist >= 200.0f)
    {
        car_run_track_8s(190.0f);
    }
    else if (dist < 400.0f && dist >= 300.0f)
    {
        car_run_track_8s(220.0f);
    }
    else if (dist < 500.0f && dist >= 400.0f)
    {
        car_run_track_8s(250.0f);
    }
    else if (dist < 600.0f && dist >= 500.0f)
    {
        car_run_track_8s(270.0f);
    }
    else if (dist < 800.0f && dist >= 600.0f)
    {
        car_run_track_8s(285.0f);
    }
    else if (dist < 1600.0f && dist >= 800.0f)
    {
        car_run_track_8s(310.0f);
    }
    else if (dist < 1700.0f && dist >= 1600.0f)
    {
        car_run_track_8s(280.0f);
    }
    else if (dist < 1800.0f && dist >= 1700.0f)
    {
        car_run_track_8s(250.0f);
    }
    else if (dist < 1900.0f && dist >= 1800.0f)
    {
        car_run_track_8s(200.0f);
    }
    else if (dist < 1950.0f && dist >= 1900.0f)
    {
        car_run_track_8s(170.0f);
    }
    else if (dist < 2050.0f && dist >= 1950.0f)
    {
        car_run_track_8s(140.0f);
    }
    else if (dist < 2200.0f && dist >= 2050.0f)
    {
        car_run_track_8s(110.0f);
    }

    else
    {
        motor_forward(0, 0, 0, 0, 0, 0);
        n = 1;
    }
}
