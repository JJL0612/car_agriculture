#include "rc_switch.h"
#include "tim.h"

_Static_assert(RC_SWITCH_SHORT_US > 0U, "Pulse must be positive");
_Static_assert(RC_SWITCH_SHORT_US < RC_SWITCH_LONG_US, "Pulse order");
_Static_assert(RC_SWITCH_LONG_US < RC_SWITCH_FRAME_US, "Pulse exceeds frame");
_Static_assert(RC_SWITCH_FRAME_US <= 65536U, "TIM6 is a 16-bit timer");
_Static_assert(RC_SWITCH_COUNT == 3U, "Update pin mapping when adding switches");
_Static_assert(RC_SWITCH_INVERT_MASK < (1U << RC_SWITCH_COUNT), "Invalid invert mask");
#define ASSERT_AUX_PULSE(pulse) _Static_assert((pulse) > 0U && (pulse) < RC_SWITCH_FRAME_US, "Invalid SG90 pulse")
ASSERT_AUX_PULSE(AUX_SERVO1_ZERO_US);
ASSERT_AUX_PULSE(AUX_SERVO1_90_US);
ASSERT_AUX_PULSE(AUX_SERVO2_ZERO_US);
ASSERT_AUX_PULSE(AUX_SERVO2_90_US);

#define RC_OUTPUT_COUNT 5U
#define RC_OUTPUT_PINS (RC_SW1_Pin | RC_SW2_Pin | RC_SW3_Pin | AUX_SERVO1_Pin | AUX_SERVO2_Pin)
static const uint16_t channel_pins[RC_OUTPUT_COUNT] = {
    RC_SW1_Pin, RC_SW2_Pin, RC_SW3_Pin, AUX_SERVO1_Pin, AUX_SERVO2_Pin
};
static volatile uint8_t requested_on_mask;
static volatile uint8_t requested_servo_90;
static uint16_t frame_pulse_us[RC_OUTPUT_COUNT];
static uint16_t pending_pins;
static uint16_t next_edge_us;

void RC_Switch_Init(void)
{
    requested_on_mask = 0U;
    requested_servo_90 = 0U;
    pending_pins = 0U;
    GPIOE->BSRR = (uint32_t)RC_OUTPUT_PINS << 16U;
    __HAL_TIM_SET_AUTORELOAD(&htim6, RC_SWITCH_FRAME_US - 1U);
    __HAL_TIM_SET_COUNTER(&htim6, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    HAL_NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) Error_Handler();
}

HAL_StatusTypeDef RC_Switch_Set(uint8_t channel, uint8_t on)
{
    if (channel < 1U || channel > RC_SWITCH_COUNT || on > 1U) return HAL_ERROR;
    uint8_t bit = (uint8_t)(1U << (channel - 1U));
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (on != 0U) requested_on_mask |= bit;
    else requested_on_mask &= (uint8_t)~bit;
    __set_PRIMASK(primask);
    return HAL_OK;
}

HAL_StatusTypeDef RC_Switch_SetMask(uint8_t on_mask)
{
    if (on_mask >= (1U << RC_SWITCH_COUNT)) return HAL_ERROR;
    requested_on_mask = on_mask;
    return HAL_OK;
}

void RC_Switch_AllOff(void) { requested_on_mask = 0U; }
uint8_t RC_Switch_GetMask(void) { return requested_on_mask; }
void Aux_Servo_Set90(uint8_t enabled) { requested_servo_90 = enabled != 0U; }
uint8_t Aux_Servo_Get90(void) { return requested_servo_90; }

void RC_Switch_TimerElapsed(void)
{
    uint16_t elapsed_us = 0U;
    if (pending_pins == 0U) {
        /* Snapshot all widths once per frame: command changes cannot truncate
         * active pulses. The two SG90s share one position snapshot.
         */
        uint8_t long_mask = requested_on_mask ^ RC_SWITCH_INVERT_MASK;
        uint8_t position_90 = requested_servo_90;
        for (unsigned i = 0U; i < RC_SWITCH_COUNT; ++i) {
            frame_pulse_us[i] = long_mask & (1U << i) ? RC_SWITCH_LONG_US : RC_SWITCH_SHORT_US;
        }
        frame_pulse_us[3] = position_90 ? AUX_SERVO1_90_US : AUX_SERVO1_ZERO_US;
        frame_pulse_us[4] = position_90 ? AUX_SERVO2_ZERO_US : AUX_SERVO2_90_US;
        pending_pins = RC_OUTPUT_PINS;
        GPIOE->BSRR = RC_OUTPUT_PINS;
    } else {
        uint16_t falling_pins = 0U;
        elapsed_us = next_edge_us;
        for (unsigned i = 0U; i < RC_OUTPUT_COUNT; ++i) {
            if ((pending_pins & channel_pins[i]) != 0U && frame_pulse_us[i] == elapsed_us)
                falling_pins |= channel_pins[i];
        }
        GPIOE->BSRR = (uint32_t)falling_pins << 16U;
        pending_pins &= (uint16_t)~falling_pins;
    }

    /* Find the earliest remaining falling edge, or wait for the next frame.
     * Handles independently calibrated SG90s as well as 1/2 ms RC switches.
     */
    next_edge_us = RC_SWITCH_FRAME_US;
    for (unsigned i = 0U; i < RC_OUTPUT_COUNT; ++i) {
        if ((pending_pins & channel_pins[i]) != 0U && frame_pulse_us[i] < next_edge_us)
            next_edge_us = frame_pulse_us[i];
    }
    __HAL_TIM_SET_AUTORELOAD(&htim6, next_edge_us - elapsed_us - 1U);
}
