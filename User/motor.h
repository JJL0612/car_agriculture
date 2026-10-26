#ifndef __MOTOR_H
#define __MOTOR_H

#include "gpio.h"
#include "tim.h"

/* JGB37-520 5 线电机：F/R 引脚电平控制正反转
 * 若实测发现方向反了，把这两个宏对调即可 */
#define MOTOR_FORWARD_LEVEL  GPIO_PIN_SET
#define MOTOR_REVERSE_LEVEL  GPIO_PIN_RESET

void motor_init(void);

/* Call from the main loop to sweep both servos for testing. */
void servo_test_task(void);

void motor_forward(int speed1, int speed2, int speed3, int speed4, int speed5, int speed6);

#endif
