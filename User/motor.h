#ifndef __MOTOR_H
#define __MOTOR_H

#include "gpio.h"
#include "tim.h"

/* JGB37-520 5 线电机：F/R 引脚电平控制正反转
 * 若实测发现方向反了，把这两个宏对调即可 */
#define MOTOR_FORWARD_LEVEL  GPIO_PIN_SET
#define MOTOR_REVERSE_LEVEL  GPIO_PIN_RESET

void motor_init(void);

/* Independent calibrated positions: servo 1=right, servo 2=left.
 * Pulse targets describe commands; physical angles require measurement.
 */
#define SERVO_RIGHT_CENTER_US 1400U  /* PE5 / TIM9 CH1 */
#define SERVO_RIGHT_UP_US     1900U
#define SERVO_RIGHT_DOWN_US   970U
#define SERVO_LEFT_CENTER_US  1500U  /* PE6 / TIM9 CH2 */
#define SERVO_LEFT_UP_US      900U
#define SERVO_LEFT_DOWN_US    2100U

/* CH3: low < 742, center 742..1242, high > 1242 from center.
 * Returning from an outer position needs 40 counts of hysteresis.
 */
#define SERVO_RC_CENTER       992U
#define SERVO_RC_CENTER_BAND  250U
#define SERVO_RC_HYSTERESIS   40U

/* Shared trajectory speed in units of the larger pulse travel.
 * Higher logical position (down -> center -> up) uses slower lift settings. */
#define SERVO_UPDATE_MS        20U
#define SERVO_MAX_STEP_US      8U
#define SERVO_SMOOTH_DIVISOR   8U
#define SERVO_ACCEL_Q8         128U /* +0.5 us/frame per update */
#define SERVO_LIFT_MAX_STEP_US 3U   /* 150 us/s after ramp-up */
#define SERVO_LIFT_DIVISOR     16U
#define SERVO_LIFT_ACCEL_Q8    32U  /* +0.125 us/frame per update */

/* Immediately center both servos and clear selection/trajectory state. */
void servo_center(void);
/* Call once per SERVO_UPDATE_MS. Select low/center/high and approach smoothly. */
void servo_set_remote_value(uint16_t value);

/* Call from the main loop to sweep both servos for testing. */
void servo_test_task(void);

void motor_forward(int speed1, int speed2, int speed3, int speed4, int speed5, int speed6);

#endif
