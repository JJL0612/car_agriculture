#include "motor.h"

/* Servo test: 1000..2000 us, 2 s each way, update every PWM frame. */
#define SERVO_TEST_MIN_US       1000U
#define SERVO_TEST_MAX_US       2000U
#define SERVO_TEST_HALF_MS      2000U
#define SERVO_TEST_UPDATE_MS    20U

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

    /* 设置两个舵机的初始脉宽为 1500μs */
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 1800);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 1800);

    /* 启动后，硬件持续自动输出 PWM */
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

    servo_test_last_tick = HAL_GetTick();
    servo_test_phase_ms = SERVO_TEST_HALF_MS / 2U;
}

void servo_test_task(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - servo_test_last_tick;
    uint32_t ramp_ms;
    uint32_t pulse_us;

    if (elapsed < SERVO_TEST_UPDATE_MS) {
        return;
    }
    servo_test_last_tick = now;
    servo_test_phase_ms = (servo_test_phase_ms
                          + elapsed % (2U * SERVO_TEST_HALF_MS))
                         % (2U * SERVO_TEST_HALF_MS);
    ramp_ms = servo_test_phase_ms <= SERVO_TEST_HALF_MS
              ? servo_test_phase_ms
              : 2U * SERVO_TEST_HALF_MS - servo_test_phase_ms;
    pulse_us = SERVO_TEST_MIN_US
               + (SERVO_TEST_MAX_US - SERVO_TEST_MIN_US) * ramp_ms
                 / SERVO_TEST_HALF_MS;

    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, pulse_us);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, pulse_us);
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
