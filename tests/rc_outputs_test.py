"""Verify real TIM6 edge scheduling for 3 RC switches and 2 SG90s."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
# Take pin assignments from the actual board header, rather than a second mapping.
pins = re.findall(r'^#define (RC_SW[1-3]_Pin|AUX_SERVO[12]_Pin) GPIO_PIN_(\d+)$',
                  (ROOT / 'Core/Inc/main.h').read_text(), re.M)
assert len(pins) == 5
with tempfile.TemporaryDirectory() as directory:
    temp = Path(directory)
    (temp / 'main.h').write_text('''#pragma once
#include <stdint.h>
#include <stdlib.h>
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
#define HAL_ERROR 1
typedef struct { uint32_t BSRR; } GPIO_TypeDef;
extern GPIO_TypeDef gpio;
#define GPIOE (&gpio)
#define TIM6_DAC_IRQn 6
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(value) ((void)(value))
#define HAL_NVIC_ClearPendingIRQ(...) ((void)0)
#define Error_Handler() abort()
''' + ''.join(f'#define {name} (1U << {number})\n' for name, number in pins))
    (temp / 'tim.h').write_text('''#pragma once
#include "main.h"
typedef struct { uint32_t ARR, CNT; } TIM_HandleTypeDef;
extern TIM_HandleTypeDef htim6;
#define TIM_FLAG_UPDATE 1
#define __HAL_TIM_SET_AUTORELOAD(t, value) ((t)->ARR=(value))
#define __HAL_TIM_SET_COUNTER(t, value) ((t)->CNT=(value))
#define __HAL_TIM_CLEAR_FLAG(...) ((void)0)
#define HAL_TIM_Base_Start_IT(t) ((void)(t), HAL_OK)
''')
    (temp / 'test.c').write_text(r'''
#include <assert.h>
#include <stdio.h>
#include "@DRIVER@"
GPIO_TypeDef gpio;
TIM_HandleTypeDef htim6;
static uint32_t levels;
static void apply(void) {
    levels |= gpio.BSRR & 65535U;
    levels &= ~(gpio.BSRR >> 16U);
}
static void frame(unsigned mask, unsigned position, unsigned change_mid_frame) {
    assert(RC_Switch_SetMask(mask) == HAL_OK);
    Aux_Servo_Set90(position);
    assert(RC_Switch_GetMask() == mask && Aux_Servo_Get90() == position);
    RC_Switch_TimerElapsed(); apply();
    assert((levels & RC_OUTPUT_PINS) == RC_OUTPUT_PINS);
    unsigned expected[5], widths[5] = {0};
    for (unsigned i = 0; i < 3; ++i)
        expected[i] = (mask ^ RC_SWITCH_INVERT_MASK) & (1U<<i) ? 2000 : 1000;
    expected[3] = position ? AUX_SERVO1_90_US : AUX_SERVO1_ZERO_US;
    expected[4] = position ? AUX_SERVO2_ZERO_US : AUX_SERVO2_90_US;
    if (change_mid_frame) {
        RC_Switch_SetMask(mask ^ 7U);
        Aux_Servo_Set90(!position);
    }
    unsigned elapsed = 0, edges = 0;
    while (levels & RC_OUTPUT_PINS) {
        assert(htim6.ARR < 20000);
        elapsed += htim6.ARR + 1;
        assert(elapsed < 20000);
        unsigned previous = levels;
        RC_Switch_TimerElapsed(); apply();
        for (unsigned i=0; i<5; ++i)
            if ((previous & channel_pins[i]) && !(levels & channel_pins[i])) widths[i]=elapsed;
        assert(++edges <= 5);
    }
    for (unsigned i=0; i<5; ++i) assert(widths[i] == expected[i]);
    assert(elapsed + htim6.ARR + 1 == 20000);
}
int main(void) {
    levels = 65535;
    RC_Switch_Init(); apply();
    assert((levels & RC_OUTPUT_PINS) == 0);
    assert(htim6.ARR == 19999);
    assert(RC_Switch_GetMask() == 0 && Aux_Servo_Get90() == 0);
    frame(0, 0, 0); /* First powered frame: PE10=0deg, PE12=90deg. */
    for (unsigned mask=0; mask<8; ++mask)
        for (unsigned pos=0; pos<2; ++pos) {
            frame(mask, pos, 0);
            frame(mask, pos, 1);
        }
    assert(RC_Switch_SetMask(8) == HAL_ERROR);
    assert(RC_Switch_Set(4, 1) == HAL_ERROR);
    assert(RC_Switch_Set(0, 1) == HAL_ERROR);
    assert(RC_Switch_Set(1, 2) == HAL_ERROR);
    assert(RC_Switch_SetMask(0) == HAL_OK);
    for (unsigned channel=1; channel<=3; ++channel) {
        assert(RC_Switch_Set(channel, 1) == HAL_OK);
        assert(RC_Switch_GetMask() == (1U << channel) - 1);
    }
    Aux_Servo_Set90(1);
    RC_Switch_AllOff();
    assert(RC_Switch_GetMask() == 0 && Aux_Servo_Get90() == 1);
    frame(0, 0, 0); /* Failsafe: PE10=0deg, PE12=90deg. */
    puts("PASS: physical pins, all 16 output combinations, opposed SG90 angles, exact pulse widths, 20 ms frames, mid-frame changes and switch/servo isolation");
}
'''.replace('@DRIVER@', str(ROOT / 'User/rc_switch.c')))
    # Include distinct per-servo widths to test the general edge scheduler.
    variants = [[], ['-DRC_SWITCH_INVERT_MASK=7'],
                ['-DAUX_SERVO2_ZERO_US=700', '-DAUX_SERVO2_90_US=1700']]
    for variant in variants:
        executable = temp / 'rc_outputs_test'
        subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', *variant,
                        f'-I{temp}', f'-I{ROOT / "User"}', str(temp / 'test.c'),
                        '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
