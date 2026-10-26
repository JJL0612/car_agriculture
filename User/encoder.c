//
// Created by 36138 on 26-7-30.
//

#include "encoder.h"

/* ========== 内部变量 ========== */

static int32_t  left_total_counts    = 0;    // 左轮累计编码器计数
static int32_t  right_total_counts   = 0;    // 右轮累计编码器计数

static uint16_t left_prev_cnt        = 0;    // 左轮上一次 TIM->CNT 值
static uint16_t right_prev_cnt       = 0;    // 右轮上一次 TIM->CNT 值

/* ========== 函数实现 ========== */

/**
 * @brief  编码器初始化
 *         启动 TIM3/TIM4 编码器模式，启动 TIM2 10ms 定时中断，清零累计变量
 *         注意：TIM2 的 NVIC 配置已由 CubeMX 在 HAL_TIM_Base_MspInit 中自动生成
 */
void Encoder_Init(void)
{
    /* 启动编码器 */
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);     // 左轮 (TIM4, AB相)
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);     // 右轮 (TIM3, 单相)

    /* 读取初始计数值 */
    left_prev_cnt  = __HAL_TIM_GET_COUNTER(&htim4);
    right_prev_cnt = __HAL_TIM_GET_COUNTER(&htim3);

    /* 清零累计计数 */
    left_total_counts  = 0;
    right_total_counts = 0;

    /* 启动 TIM2 定时中断 (10ms 周期)，NVIC 已由 CubeMX 配置 */
    HAL_TIM_Base_Start_IT(&htim2);
}

/**
 * @brief  编码器更新（供 HAL_TIM_PeriodElapsedCallback 中调用）
 *         读取当前编码器计数值并与上次比较，累加差值
 *         注意：此函数应在 TIM2 中断上下文中调用（每 10ms）
 */
void Encoder_TIM2_PeriodElapsedCallback(void)
{
    uint16_t left_now, right_now;
    int16_t  left_delta, right_delta;

    /* ---- 左轮 (TIM4) ---- */
    left_now = __HAL_TIM_GET_COUNTER(&htim4);
    left_delta = (int16_t)(left_now - left_prev_cnt);
    left_total_counts += left_delta;
    left_prev_cnt = left_now;

    /* ---- 右轮 (TIM3) ---- */
    right_now = __HAL_TIM_GET_COUNTER(&htim3);
    right_delta = (int16_t)(right_now - right_prev_cnt);
    right_total_counts += right_delta;
    right_prev_cnt = right_now;
}

/**
 * @brief  获取左轮累计路程
 * @return 左轮路程 (cm)
 */
float Encoder_GetLeftDistance(void)
{
    return (float)left_total_counts * LEFT_CM_PER_COUNT;
}

/**
 * @brief  获取右轮累计路程
 * @return 右轮路程 (cm)
 */
float Encoder_GetRightDistance(void)
{
    return (float)right_total_counts * RIGHT_CM_PER_COUNT;
}

/**
 * @brief  获取小车平均累计路程
 * @return 左右轮平均路程 (cm)
 */
float Encoder_GetDistance(void)
{
    return (Encoder_GetLeftDistance() + Encoder_GetRightDistance()) / 2.0f;
}

/**
 * @brief  重置累计路程（清零所有计数）
 */
void Encoder_ResetDistance(void)
{
    left_total_counts  = 0;
    right_total_counts = 0;
    left_prev_cnt  = __HAL_TIM_GET_COUNTER(&htim4);
    right_prev_cnt = __HAL_TIM_GET_COUNTER(&htim3);
}