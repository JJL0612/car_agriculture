//
// Created by 36138 on 26-7-30.
//

#ifndef CAR_CODE_7_21_ENCODER_H
#define CAR_CODE_7_21_ENCODER_H

#include "tim.h"

/* ========== 编码器参数配置 ========== */

#define WHEEL_RADIUS_CM         3.25f           // 轮子半径 (cm)
#define ENCODER_PPR             13              // 编码器线数 (脉冲/圈)，请根据实际编码器修改

/* 轮子周长 = 2 * PI * r */
#define WHEEL_PERIMETER_CM      (2.0f * 3.1415926f * WHEEL_RADIUS_CM)

/*
 * 编码器倍频系数：
 *   TIM4 使用 TIM_ENCODERMODE_TI12 (AB相4倍频) → 每圈计数 = ENCODER_PPR * 4
 *   TIM3 使用 TIM_ENCODERMODE_TI1  (单相1倍频) → 每圈计数 = ENCODER_PPR * 1
 */
#define LEFT_COUNTS_PER_ROUND    (ENCODER_PPR * 4)   // TIM4 (左轮) 每圈计数值
#define RIGHT_COUNTS_PER_ROUND   (ENCODER_PPR * 1)   // TIM3 (右轮) 每圈计数值

/*
 * 每个编码器计数对应的路程 (cm/count)
 */
#define LEFT_CM_PER_COUNT       (WHEEL_PERIMETER_CM / LEFT_COUNTS_PER_ROUND)
#define RIGHT_CM_PER_COUNT      (WHEEL_PERIMETER_CM / RIGHT_COUNTS_PER_ROUND)

/* ========== 函数声明 ========== */

/**
 * @brief  编码器初始化
 *         启动 TIM3/TIM4 编码器模式，启动 TIM2 10ms 定时中断，清零累计路程
 */
void Encoder_Init(void);

/**
 * @brief  TIM2 定时中断回调（每 10ms 自动触发，在 HAL_TIM_PeriodElapsedCallback 中调用）
 *         读取当前编码器计数值并与上次比较，累加差值
 */
void Encoder_TIM2_PeriodElapsedCallback(void);

/**
 * @brief  获取左轮累计路程
 * @return 左轮路程 (cm)
 */
float Encoder_GetLeftDistance(void);

/**
 * @brief  获取右轮累计路程
 * @return 右轮路程 (cm)
 */
float Encoder_GetRightDistance(void);

/**
 * @brief  获取小车平均累计路程
 * @return 左右轮平均路程 (cm)
 */
float Encoder_GetDistance(void);

/**
 * @brief  重置累计路程（清零所有计数）
 */
void Encoder_ResetDistance(void);

#endif //CAR_CODE_7_21_ENCODER_H