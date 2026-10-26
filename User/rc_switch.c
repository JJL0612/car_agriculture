#include "rc_switch.h"
#include "tim.h"

_Static_assert(RC_SWITCH_SHORT_US > 0U, "Pulse must be positive");
_Static_assert(RC_SWITCH_SHORT_US < RC_SWITCH_LONG_US, "Pulse order");
_Static_assert(RC_SWITCH_LONG_US < RC_SWITCH_FRAME_US, "Pulse exceeds frame");
_Static_assert(RC_SWITCH_FRAME_US <= 65536U, "TIM6 is a 16-bit timer");
_Static_assert(RC_SWITCH_COUNT == 5U, "Update pin mapping when adding channels");
_Static_assert(RC_SWITCH_INVERT_MASK < (1U << RC_SWITCH_COUNT), "Invalid invert mask");

#define RC_SWITCH_PINS (RC_SW1_Pin | RC_SW2_Pin | RC_SW3_Pin | RC_SW4_Pin | RC_SW5_Pin)

static const uint16_t channel_pins[RC_SWITCH_COUNT] = {
    RC_SW1_Pin, RC_SW2_Pin, RC_SW3_Pin, RC_SW4_Pin, RC_SW5_Pin
};
static volatile uint8_t requested_on_mask;
static uint16_t frame_long_pins;
static uint8_t edge_phase;

void RC_Switch_Init(void)
{
    /* MX_TIM6_Init configures 84 MHz / 84 = 1 MHz, no ARR preload.
     * First frame begins after 20 ms; pins remain low until then.
     */
    requested_on_mask = 0U;
    frame_long_pins = 0U;
    edge_phase = 0U;
    GPIOE->BSRR = (uint32_t)RC_SWITCH_PINS << 16U;
    __HAL_TIM_SET_AUTORELOAD(&htim6, RC_SWITCH_FRAME_US - 1U);
    __HAL_TIM_SET_COUNTER(&htim6, 0U);
    /* Base_Init generates an update event: discard its pending flag. */
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    HAL_NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        Error_Handler();
    }
}

HAL_StatusTypeDef RC_Switch_Set(uint8_t channel, uint8_t on)
{
    uint32_t primask;
    uint8_t bit;

    if (channel < 1U || channel > RC_SWITCH_COUNT || on > 1U) {
        return HAL_ERROR;
    }
    bit = (uint8_t)(1U << (channel - 1U));
    /* Protect read-modify-write from commands issued by other ISRs. */
    primask = __get_PRIMASK();
    __disable_irq();
    if (on != 0U) {
        requested_on_mask |= bit;
    } else {
        requested_on_mask &= (uint8_t)~bit;
    }
    __set_PRIMASK(primask);
    return HAL_OK;
}

HAL_StatusTypeDef RC_Switch_SetMask(uint8_t on_mask)
{
    if (on_mask >= (1U << RC_SWITCH_COUNT)) {
        return HAL_ERROR;
    }
    requested_on_mask = on_mask;
    return HAL_OK;
}

void RC_Switch_AllOff(void)
{
    requested_on_mask = 0U;
}

uint8_t RC_Switch_GetMask(void)
{
    return requested_on_mask;
}

void RC_Switch_TimerElapsed(void)
{
    if (edge_phase == 0U) {
        uint8_t long_mask = requested_on_mask ^ RC_SWITCH_INVERT_MASK;

        /* Snapshot commands once per frame, so mid-pulse changes cannot
         * truncate a pulse. All five rising edges share one GPIO write.
         */
        GPIOE->BSRR = RC_SWITCH_PINS;
        frame_long_pins = 0U;
        for (uint8_t i = 0U; i < RC_SWITCH_COUNT; ++i) {
            if ((long_mask & (1U << i)) != 0U) {
                frame_long_pins |= channel_pins[i];
            }
        }
        __HAL_TIM_SET_AUTORELOAD(&htim6, RC_SWITCH_SHORT_US - 1U);
        edge_phase = 1U;
    } else if (edge_phase == 1U) {
        GPIOE->BSRR = (uint32_t)(RC_SWITCH_PINS & ~frame_long_pins) << 16U;
        __HAL_TIM_SET_AUTORELOAD(&htim6,
                               RC_SWITCH_LONG_US - RC_SWITCH_SHORT_US - 1U);
        edge_phase = 2U;
    } else {
        GPIOE->BSRR = (uint32_t)RC_SWITCH_PINS << 16U;
        __HAL_TIM_SET_AUTORELOAD(&htim6,
                               RC_SWITCH_FRAME_US - RC_SWITCH_LONG_US - 1U);
        edge_phase = 0U;
    }
}
