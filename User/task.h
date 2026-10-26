#ifndef __TASK_H
#define __TASK_H

#include "sensor.h"
#include "motor.h"
#include "oled.h"
#include "gpio.h"
#include "jy62.h"


extern int KEY;

void init(void);

/* ========== 计时器 ========== */

/**
 * @brief  启动计时（在 init 中自动调用）
 *         记录当前 HAL_GetTick() 作为起始时刻
 */
void Timer_Start(void);

/**
 * @brief  获取计时秒数（0 ~ 99）
 *         当 n==1 时自动锁存停止时间，后续调用始终返回停止时刻的值
 * @return 从 Timer_Start 开始经过的秒数
 */
uint8_t Timer_GetSeconds(void);

void task_check();
void task_run();
void task_1();
void task_2();
void task_3();




#endif

