#ifndef __MOTOR_H
#define __MOTOR_H

#include "gpio.h"
#include "tim.h"

/* JGB37-520 5 线电机：F/R 引脚电平控制正反转
 * 若实测发现方向反了，把这两个宏对调即可 */
#define MOTOR_FORWARD_LEVEL  GPIO_PIN_SET
#define MOTOR_REVERSE_LEVEL  GPIO_PIN_RESET

void motor_init(void);

#define SERVO_MIN_US       1100U
#define SERVO_CENTER_US    1500U
#define SERVO_MAX_US       1900U

/* PE5 uses pulse_us clamped to 1100..1900 us; PE6 uses 3000 - pulse_us. */
void servo_set_pulse_us(uint16_t pulse_us);
/* Raw 200/1000/1800: PE5 = 1100/1500/1900, PE6 = 1900/1500/1100 us. */
void servo_set_remote_value(uint16_t value);

/* Call from the main loop to sweep both servos for testing. */
void servo_test_task(void);

void motor_forward(int speed1, int speed2, int speed3, int speed4, int speed5, int speed6);

#endif
