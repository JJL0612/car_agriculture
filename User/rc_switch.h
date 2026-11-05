#ifndef RC_SWITCH_H
#define RC_SWITCH_H

#include "main.h"

/* TIM6 drives three RC switches (PE0/PE7/PE8) and two SG90 servos
 * (PE10/PE12). GPIO edges avoid sharing the wheel motor PWM timers.
 */
#define RC_SWITCH_COUNT             3U
#define RC_SWITCH_FRAME_US          20000U
#define RC_SWITCH_SHORT_US          1000U
#define RC_SWITCH_LONG_US           2000U

/* Only the three electronic switches use pulse polarity inversion. */
#ifndef RC_SWITCH_INVERT_MASK
#define RC_SWITCH_INVERT_MASK       0x00U
#endif

/* Initial 0/90 degree calibration, independently adjustable per SG90.
 * Nominal 180-degree SG90 mapping: 500 us=0, 1500 us=90.
 * Measure the actual horn angles and adjust for the installed servo variant.
 */
#ifndef AUX_SERVO1_ZERO_US
#define AUX_SERVO1_ZERO_US 500U /* PE10 */
#endif
#ifndef AUX_SERVO1_90_US
#define AUX_SERVO1_90_US 1500U
#endif
#ifndef AUX_SERVO2_ZERO_US
#define AUX_SERVO2_ZERO_US 500U /* PE12 */
#endif
#ifndef AUX_SERVO2_90_US
#define AUX_SERVO2_90_US 1500U
#endif

/* Starts TIM6: switches OFF, PE10 at 0 degrees, PE12 at 90 degrees. */
void RC_Switch_Init(void);

/* Electronic switch channels 1..3, logical on=0/1. */
HAL_StatusTypeDef RC_Switch_Set(uint8_t channel, uint8_t on);
/* bit0..bit2 = switches 1..3. Other bits are rejected. */
HAL_StatusTypeDef RC_Switch_SetMask(uint8_t on_mask);
uint8_t RC_Switch_GetMask(void);
void RC_Switch_AllOff(void);

/* Requested CH8-high state: 0 -> PE10=0deg, PE12=90deg;
 * 1 -> PE10=90deg, PE12=0deg. Getter reports this state (PE10 direction).
 * Each servo retains its own angle calibration. Next 20 ms frame applies it.
 */
void Aux_Servo_Set90(uint8_t enabled);
uint8_t Aux_Servo_Get90(void);

/* Called only from the TIM6 update callback. No blocking work. */
void RC_Switch_TimerElapsed(void);

#endif
