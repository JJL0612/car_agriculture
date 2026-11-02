#include "motor.h"

#define SERVO_TEST_HALF_MS      2000U
#define SERVO_TEST_UPDATE_MS    SERVO_UPDATE_MS

#define ASSERT_SERVO_PULSE(pulse) _Static_assert((pulse) > 0U && (pulse) < 20000U, "Invalid servo pulse")
ASSERT_SERVO_PULSE(SERVO_RIGHT_CENTER_US);
ASSERT_SERVO_PULSE(SERVO_RIGHT_UP_US);
ASSERT_SERVO_PULSE(SERVO_RIGHT_DOWN_US);
ASSERT_SERVO_PULSE(SERVO_LEFT_CENTER_US);
ASSERT_SERVO_PULSE(SERVO_LEFT_UP_US);
ASSERT_SERVO_PULSE(SERVO_LEFT_DOWN_US);
_Static_assert(SERVO_RC_HYSTERESIS < SERVO_RC_CENTER_BAND, "Invalid servo hysteresis");
_Static_assert(SERVO_LIFT_MAX_STEP_US > 0U && SERVO_MAX_STEP_US > 0U, "Invalid speed");
_Static_assert(SERVO_LIFT_DIVISOR > 0U && SERVO_SMOOTH_DIVISOR > 0U, "Invalid easing");
_Static_assert(SERVO_LIFT_ACCEL_Q8 > 0U && SERVO_ACCEL_Q8 > 0U, "Invalid acceleration");

static uint16_t servo_right_us = SERVO_RIGHT_CENTER_US;
static uint16_t servo_left_us = SERVO_LEFT_CENTER_US;
static uint16_t servo_start_right_us, servo_start_left_us;
static uint16_t servo_target_right_us = SERVO_RIGHT_CENTER_US;
static uint16_t servo_target_left_us = SERVO_LEFT_CENTER_US;
static uint32_t servo_progress, servo_distance;
static uint32_t servo_speed_q8, servo_fraction_q8;
static uint8_t servo_lifting;
static int8_t servo_target_position, servo_remote_position; /* -1=down, 0=center, +1=up */

static uint32_t servo_test_last_tick;
static uint32_t servo_test_phase_ms;

/**
 * @brief  电机初始化
 * @note   启动 TIM1/TIM5 的 PWM 输出（接各电机 BRK 调速脚），
 *         并把所有电机 F/R 方向引脚（AIN1~FIN1）设为前进
 *         JGB37-520 5 线电机：AIN2~FIN2 未使用
 */
void motor_init(void)
{

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

    /* F/R 方向引脚：全部设为前进 */
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, MOTOR_FORWARD_LEVEL);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, MOTOR_FORWARD_LEVEL);
    HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, MOTOR_FORWARD_LEVEL);
    HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, MOTOR_REVERSE_LEVEL); /* 4号电机(D)接线反相，初始化即取反 */
    HAL_GPIO_WritePin(EIN1_GPIO_Port, EIN1_Pin, MOTOR_FORWARD_LEVEL);
    HAL_GPIO_WritePin(FIN1_GPIO_Port, FIN1_Pin, MOTOR_FORWARD_LEVEL);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);

    /* 初始中位：右舵机 PE5=1400μs，左舵机 PE6=1500μs */
    servo_center();

    /* 启动后，硬件持续自动输出 PWM */
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

    servo_test_last_tick = HAL_GetTick();
    servo_test_phase_ms = SERVO_TEST_HALF_MS / 2U;
}

static void servo_write_pair(uint16_t right_us, uint16_t left_us)
{
    servo_right_us = right_us;
    servo_left_us = left_us;
    /* Both preload values are written for the next TIM9 PWM update. */
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, right_us);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, left_us);
}

void servo_center(void)
{
    servo_remote_position = 0;
    servo_target_position = 0;
    servo_progress = servo_distance = 0U;
    servo_speed_q8 = servo_fraction_q8 = 0U;
    servo_target_right_us = SERVO_RIGHT_CENTER_US;
    servo_target_left_us = SERVO_LEFT_CENTER_US;
    servo_write_pair(SERVO_RIGHT_CENTER_US, SERVO_LEFT_CENTER_US);
}

static uint32_t servo_delta(uint16_t a, uint16_t b)
{
    return a > b ? a - b : b - a;
}

static uint16_t servo_interpolate(uint16_t start, uint16_t target,
                                  uint32_t progress, uint32_t distance)
{
    /* Signed delta supports independent travel sizes and pulse directions.
     * Truncation keeps each nonzero channel short of its endpoint until the
     * shared progress reaches 100%; both channels finish on the same update.
     */
    return (uint16_t)((int32_t)start +
                     ((int32_t)target - start) * (int32_t)progress / (int32_t)distance);
}

void servo_set_remote_value(uint16_t value)
{
    const unsigned low = SERVO_RC_CENTER - SERVO_RC_CENTER_BAND;
    const unsigned high = SERVO_RC_CENTER + SERVO_RC_CENTER_BAND;

    /* A direct reversal selects the opposite target immediately. Hysteresis
     * prevents repeated center/outer switching when the stick jitters.
     */
    if (value < low) {
        servo_remote_position = -1;
    } else if (value > high) {
        servo_remote_position = 1;
    } else if ((servo_remote_position < 0 &&
                value < low + SERVO_RC_HYSTERESIS) ||
               (servo_remote_position > 0 &&
                value > high - SERVO_RC_HYSTERESIS)) {
        /* Hold the previous outer position inside its hysteresis band. */
    } else {
        servo_remote_position = 0;
    }

    uint16_t right_target = servo_remote_position > 0 ? SERVO_RIGHT_UP_US :
                            servo_remote_position < 0 ? SERVO_RIGHT_DOWN_US : SERVO_RIGHT_CENTER_US;
    uint16_t left_target = servo_remote_position > 0 ? SERVO_LEFT_UP_US :
                           servo_remote_position < 0 ? SERVO_LEFT_DOWN_US : SERVO_LEFT_CENTER_US;
    if (servo_remote_position != servo_target_position ||
        right_target != servo_target_right_us || left_target != servo_target_left_us) {
        /* Start both channels from their latest commanded positions. Each
         * uses the same completion fraction, including interrupted returns.
         */
        servo_lifting = servo_remote_position > servo_target_position;
        servo_target_position = servo_remote_position;
        servo_start_right_us = servo_right_us;
        servo_start_left_us = servo_left_us;
        servo_target_right_us = right_target;
        servo_target_left_us = left_target;
        uint32_t right_distance = servo_delta(right_target, servo_right_us);
        uint32_t left_distance = servo_delta(left_target, servo_left_us);
        servo_distance = right_distance > left_distance ? right_distance : left_distance;
        servo_progress = servo_speed_q8 = servo_fraction_q8 = 0U;
    }
    if (servo_progress == servo_distance) return;

    uint32_t remaining = servo_distance - servo_progress;
    uint32_t max_step = servo_lifting ? SERVO_LIFT_MAX_STEP_US : SERVO_MAX_STEP_US;
    uint32_t divisor = servo_lifting ? SERVO_LIFT_DIVISOR : SERVO_SMOOTH_DIVISOR;
    uint32_t acceleration = servo_lifting ? SERVO_LIFT_ACCEL_Q8 : SERVO_ACCEL_Q8;
    uint32_t desired_q8 = remaining * 256U / divisor;
    if (desired_q8 < 256U) desired_q8 = 256U;
    if (desired_q8 > max_step * 256U) desired_q8 = max_step * 256U;
    servo_speed_q8 += acceleration;
    if (servo_speed_q8 > desired_q8) servo_speed_q8 = desired_q8;

    servo_fraction_q8 += servo_speed_q8;
    uint32_t step = servo_fraction_q8 / 256U;
    servo_fraction_q8 %= 256U;
    if (step > remaining) step = remaining;
    servo_progress += step;
    servo_write_pair(servo_interpolate(servo_start_right_us, servo_target_right_us,
                                       servo_progress, servo_distance),
                     servo_interpolate(servo_start_left_us, servo_target_left_us,
                                       servo_progress, servo_distance));
}

/* Optional diagnostic sweep, not called during remote control. */
void servo_test_task(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - servo_test_last_tick;
    if (elapsed < SERVO_TEST_UPDATE_MS) return;
    servo_test_last_tick = now;
    servo_test_phase_ms = (servo_test_phase_ms + elapsed % (2U * SERVO_TEST_HALF_MS))
                         % (2U * SERVO_TEST_HALF_MS);
    uint32_t ramp_ms = servo_test_phase_ms <= SERVO_TEST_HALF_MS ? servo_test_phase_ms :
                       2U * SERVO_TEST_HALF_MS - servo_test_phase_ms;
    servo_center();
    servo_write_pair(servo_interpolate(SERVO_RIGHT_DOWN_US, SERVO_RIGHT_UP_US,
                                       ramp_ms, SERVO_TEST_HALF_MS),
                     servo_interpolate(SERVO_LEFT_DOWN_US, SERVO_LEFT_UP_US,
                                       ramp_ms, SERVO_TEST_HALF_MS));
    /* Ensure returning to remote center also starts from these test outputs. */
    servo_target_right_us = servo_right_us;
    servo_target_left_us = servo_left_us;
}

void motor_forward(int speed1, int speed2, int speed3, int speed4, int speed5, int speed6)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed3);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed4);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, speed5);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, speed6);


}
