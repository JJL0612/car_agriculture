#ifndef RC_SWITCH_H
#define RC_SWITCH_H

#include "main.h"

/* Five independent RC switches, numbered 1..5: PE0/PE7/PE8/PE10/PE12.
 * TIM6 schedules GPIO edges; these pins are NOT timer alternate functions.
 * Confirm the module's pulse direction before connecting the final loads.
 */
#define RC_SWITCH_COUNT             5U
#define RC_SWITCH_FRAME_US          20000U
#define RC_SWITCH_SHORT_US          1000U
#define RC_SWITCH_LONG_US           2000U

/* Default: 1000 us = OFF, 2000 us = ON (requires hardware verification).
 * Set bit (channel - 1) if that module instead turns ON at 1000 us.
 * Example: 0x04 reverses channel 3; 0x1F reverses all five.
 */
#ifndef RC_SWITCH_INVERT_MASK
#define RC_SWITCH_INVERT_MASK       0x00U
#endif

/* Call once after MX_GPIO_Init / MX_TIM6_Init. Starts OFF commands. */
void RC_Switch_Init(void);

/* Returns HAL_ERROR for channel outside 1..5 or on outside 0..1.
 * New commands take effect together at the next frame (within 20 ms).
 * Callable from the main loop or an ISR; does not block.
 */
HAL_StatusTypeDef RC_Switch_Set(uint8_t channel, uint8_t on);
/* Atomically replace all five commands; bit0..bit4 = switches 1..5. */
HAL_StatusTypeDef RC_Switch_SetMask(uint8_t on_mask);
/* Requested logical ON states, before pulse polarity inversion. */
uint8_t RC_Switch_GetMask(void);
void RC_Switch_AllOff(void);

/* Called only from the TIM6 update callback. */
void RC_Switch_TimerElapsed(void);

#endif
