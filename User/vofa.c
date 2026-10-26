#include "vofa.h"
#include "usart.h"
#include "tim.h"
#include "yaokong.h"
#include "rc_switch.h"
#include "tofsense.h"
#include <string.h>

#if TOFSENSE_ENABLE_UART6_BUS
#error "USART6 is reserved for VOFA; disable the TOFSense UART6 bus"
#endif

#define VOFA_PERIOD_MS     50U

_Static_assert(sizeof(float) == 4U, "JustFloat requires 32-bit floats");

void vofa_task(void)
{
    /* Static DMA buffer: never rewrite until UART transmission completes. */
    static uint8_t frame[VOFA_CHANNEL_COUNT * sizeof(float) + 4U];
    static uint32_t last_send;
    uint32_t now = HAL_GetTick();
    float values[VOFA_CHANNEL_COUNT];
    uint16_t channels[16];
    uint32_t motor_pwm[6];
    uint8_t on_mask;
    uint8_t link_ok;
    uint32_t chunk_count;
    uint32_t frame_count;
    uint16_t last_size;
    uint8_t last_flags;
    uint32_t primask;
    const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};

    if ((now - last_send) < VOFA_PERIOD_MS ||
        huart6.gState != HAL_UART_STATE_READY) {
        return;
    }

    /* Capture one coherent SBUS frame without doing float work in the ISR. */
    primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0U; i < 16U; ++i) {
        channels[i] = sbus_ch[i];
    }
    link_ok = yaokong_link_ok();
    on_mask = RC_Switch_GetMask();
    chunk_count = sbus_chunk_count;
    frame_count = sbus_frame_count;
    last_size = sbus_last_size;
    last_flags = sbus_last_flags;
    /* Read the programmed PWM compare values, including stop/failsafe writes.
     * speeds[] can retain the previous command after motor_forward(0,...).
     */
    motor_pwm[0] = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
    motor_pwm[1] = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
    motor_pwm[2] = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
    motor_pwm[3] = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
    motor_pwm[4] = __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_3);
    motor_pwm[5] = __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_4);
    __set_PRIMASK(primask);

    /* Keep the five switch commands next to the link indicator. */
    values[VOFA_LINK_OK] = (float)link_ok;
    for (uint8_t i = 0U; i < 16U; ++i) {
        values[VOFA_RC_FIRST + i] = (float)channels[i];
    }
    for (uint8_t i = 0U; i < RC_SWITCH_COUNT; ++i) {
        values[VOFA_SWITCH_FIRST + i] = (float)((on_mask >> i) & 1U);
    }
    values[VOFA_RX_CALLBACKS] = (float)chunk_count;
    values[VOFA_RX_FRAMES] = (float)frame_count;
    values[VOFA_RX_BYTES] = (float)last_size;
    values[VOFA_RX_FLAGS] = (float)last_flags;
    for (uint8_t i = 0U; i < 6U; ++i) {
        values[VOFA_MOTOR_FIRST + i] = (float)motor_pwm[i];
    }
    /* STM32F407 uses little-endian IEEE754 floats, as JustFloat requires. */
    memcpy(frame, values, sizeof(values));
    memcpy(frame + sizeof(values), tail, sizeof(tail));
    if (HAL_UART_Transmit_DMA(&huart6, frame, sizeof(frame)) == HAL_OK) {
        last_send = now;
    }
}
