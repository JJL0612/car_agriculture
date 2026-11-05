"""Run the actual remote controller against host HAL/motor/switch stubs."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as directory:
    temp = Path(directory)
    (temp / "usart.h").write_text(r"""
#pragma once
#include <stdint.h>
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
#define HAL_ERROR 1
typedef struct { unsigned BaudRate, WordLength, StopBits, Parity; } UART_InitTypeDef;
typedef struct { void *Instance; UART_InitTypeDef Init; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart1, huart2, huart3;
uint32_t HAL_GetTick(void);
#define USART2 ((void *)2)
#define USART3 ((void *)3)
#define UART_WORDLENGTH_9B 9
#define UART_STOPBITS_2 2
#define UART_PARITY_EVEN 1
#define USART2_IRQn 2
#define DMA1_Stream5_IRQn 5
#define HAL_UART_Init(...) ((void)0)
#define HAL_NVIC_SetPriority(...) ((void)0)
#define HAL_NVIC_EnableIRQ(...) ((void)0)
#define HAL_UARTEx_ReceiveToIdle_DMA(...) ((void)0)
#define __HAL_UART_GET_FLAG(...) 0
#define __HAL_UART_CLEAR_OREFLAG(...) ((void)0)
#define __HAL_UART_CLEAR_FEFLAG(...) ((void)0)
#define HAL_UART_Transmit(uart, data, size, timeout) ((void)(uart), (void)(data), (void)(size), (void)(timeout))
""")
    (temp / "gpio.h").write_text(r"""
#pragma once
#include "usart.h"
typedef int GPIO_TypeDef;
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define AIN1_Pin 1
#define BIN1_Pin 2
#define CIN1_Pin 3
#define DIN1_Pin 4
#define EIN1_Pin 5
#define FIN1_Pin 6
#define AIN1_GPIO_Port ((GPIO_TypeDef *)0)
#define BIN1_GPIO_Port ((GPIO_TypeDef *)0)
#define CIN1_GPIO_Port ((GPIO_TypeDef *)0)
#define DIN1_GPIO_Port ((GPIO_TypeDef *)0)
#define EIN1_GPIO_Port ((GPIO_TypeDef *)0)
#define FIN1_GPIO_Port ((GPIO_TypeDef *)0)
static inline void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, uint8_t value) {
    (void)port; (void)pin; (void)value;
}
""")
    (temp / "tim.h").write_text('#pragma once\n#include "usart.h"\n')
    (temp / "main.h").write_text('#pragma once\n#include "gpio.h"\n')
    source = r"""
#include <assert.h>
#include <stdio.h>
#include "@CONTROLLER@"
UART_HandleTypeDef huart1, huart2, huart3;
static uint32_t now = 1000;
static uint8_t switch_outputs;
static uint8_t aux_servo_90;
static int pwm[6];
uint32_t HAL_GetTick(void) { return now; }
HAL_StatusTypeDef RC_Switch_SetMask(uint8_t mask) { assert(mask < 8); switch_outputs = mask; return HAL_OK; }
void Aux_Servo_Set90(uint8_t enabled) { aux_servo_90 = enabled != 0; }
void RC_Switch_AllOff(void) { switch_outputs = 0; }
void motor_forward(int a, int b, int c, int d, int e, int f) {
    pwm[0]=a; pwm[1]=b; pwm[2]=c; pwm[3]=d; pwm[4]=e; pwm[5]=f;
}
void servo_center(void) {}
void servo_set_remote_value(uint16_t value) { (void)value; }
static void connected(void) {
    sbus_ready = 1;
    sbus_failsafe = 0;
    sbus_last_tick = now;
}
static void expect_stopped(void) {
    assert(switch_outputs == 0);
    assert(aux_servo_90 == 0);
    for (unsigned i=0; i<6; ++i) assert(pwm[i] == 0);
}
int main(void) {
    sbus_ch[SBUS_CH_THROTTLE] = SBUS_CENTER;
    sbus_ch[SBUS_CH_STEER] = SBUS_CENTER;
    connected();
    for (unsigned combination=0; combination<16; ++combination) {
        for (unsigned channel=0; channel<4; ++channel)
            sbus_ch[4+channel] = combination & (1U << channel) ? 1792 : 992;
        yaokong_control();
        unsigned expected = combination & 7U;
        assert(aux_servo_90 == ((combination & 8U) != 0));
        assert(switch_outputs == expected);
        for (unsigned i=0; i<6; ++i) assert(pwm[i] == 0); /* CH8 cannot start avoidance. */
    }
    for (unsigned raw=0; raw<=2047; ++raw) {
        for (unsigned i=4; i<7; ++i) sbus_ch[i] = 992;
        sbus_ch[7] = raw;
        yaokong_control();
        assert(switch_outputs == 0);
        assert(aux_servo_90 == (raw == 1792));
    }
    for (unsigned channel=4; channel<8; ++channel) {
        for (unsigned raw=1791; raw<=1793; ++raw) {
            for (unsigned i=4; i<8; ++i) sbus_ch[i] = 992;
            sbus_ch[channel] = raw;
            yaokong_control();
            unsigned mask = channel == 7 ? 0 : 1U << (channel-4);
            assert(aux_servo_90 == (channel == 7 && raw == 1792));
            assert(switch_outputs == (raw == 1792 ? mask : 0));
        }
    }
    sbus_ch[SBUS_CH_THROTTLE] = 1400;
    sbus_ch[7] = 992;
    yaokong_control();
    int expected_pwm = pwm[0];
    assert(expected_pwm > 0);
    sbus_ch[7] = 1792;
    yaokong_control();
    for (unsigned i=0; i<6; ++i) assert(pwm[i] == expected_pwm);
    assert(switch_outputs == 0);
    assert(aux_servo_90 == 1);
    for (unsigned i=4; i<8; ++i) sbus_ch[i] = 1792;
    yaokong_control();
    assert(switch_outputs == 0x07);
    assert(aux_servo_90 == 1);
    sbus_ready = 0;
    yaokong_control();
    expect_stopped();
    connected();
    yaokong_control();
    assert(switch_outputs == 0x07);
    assert(aux_servo_90 == 1);
    sbus_failsafe = 1;
    yaokong_control();
    expect_stopped();
    connected();
    now += 200;
    yaokong_control();
    assert(switch_outputs == 0x07);
    assert(aux_servo_90 == 1);
    now += 1;
    yaokong_control();
    expect_stopped();
    connected();
    yaokong_control();
    assert(switch_outputs == 0x07);
    assert(aux_servo_90 == 1);
    /* SG90s also track CH8 / loss from the 20 ms SysTick path. */
    servo_control_ready = 1;
    for (unsigned i=0; i<20; ++i) yaokong_servo_tick();
    assert(aux_servo_90 == 1);
    sbus_failsafe = 1;
    for (unsigned i=0; i<20; ++i) yaokong_servo_tick();
    assert(aux_servo_90 == 0);
    connected();
    sbus_ch[7] = 997;
    for (unsigned i=0; i<20; ++i) yaokong_servo_tick();
    assert(aux_servo_90 == 0);
    sbus_ch[7] = 1792;
    for (unsigned i=0; i<20; ++i) yaokong_servo_tick();
    assert(aux_servo_90 == 1);
    now += 201;
    for (unsigned i=0; i<20; ++i) yaokong_servo_tick();
    assert(aux_servo_90 == 0);
    puts("PASS: 3 switches + SG90 pair, all CH8 values, manual drive preserved, no-frame/failsafe/timeout, SysTick updates and recovery");
}
""".replace("@CONTROLLER@", str(ROOT / "User/yaokong.c"))
    (temp / "test.c").write_text(source)
    executable = temp / "remote_switch_test"
    subprocess.run([
        "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections",
        f"-I{temp}", f"-I{ROOT / 'User'}", str(temp / "test.c"), "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)
