"""Host-side checks of the real motor.c; run with python3 tests/servo_smoothing_test.py."""
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as directory:
    temp = Path(directory)
    (temp / "gpio.h").write_text("""
#include <stdint.h>
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define HAL_GPIO_WritePin(...) ((void)0)
""")
    (temp / "tim.h").write_text("""
#include <stdint.h>
extern int htim1, htim5, htim9;
extern uint32_t pulses[4];
#define TIM_CHANNEL_1 0
#define TIM_CHANNEL_2 1
#define TIM_CHANNEL_3 2
#define TIM_CHANNEL_4 3
#define HAL_TIM_PWM_Start(...) ((void)0)
#define __HAL_TIM_SET_COMPARE(timer, channel, value) do { \
    if ((timer) == &htim9) pulses[channel] = (value); } while (0)
static inline uint32_t HAL_GetTick(void) { return 0; }
""")
    (temp / "test.c").write_text(r"""
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "motor.h"
int htim1, htim5, htim9;
uint32_t pulses[4];
static const unsigned right_targets[] = {970, 1400, 1900};
static const unsigned left_targets[] = {2100, 1500, 900};
static const unsigned raw_targets[] = {200, 992, 1800};

static unsigned settle(unsigned raw, unsigned position, unsigned max_step) {
    unsigned target_r = right_targets[position], target_l = left_targets[position];
    int start_r = pulses[0], start_l = pulses[1];
    int delta_r = (int)target_r - start_r, delta_l = (int)target_l - start_l;
    unsigned frames = 0;
    do {
        int prev_r = pulses[0], prev_l = pulses[1];
        servo_set_remote_value(raw);
        assert(abs((int)pulses[0] - prev_r) <= (int)max_step);
        assert(abs((int)pulses[1] - prev_l) <= (int)max_step);
        assert(pulses[0] >= 970 && pulses[0] <= 1900);
        assert(pulses[1] >= 900 && pulses[1] <= 2100);
        assert(abs((int)pulses[0] - (int)target_r) <= abs(prev_r - (int)target_r));
        assert(abs((int)pulses[1] - (int)target_l) <= abs(prev_l - (int)target_l));
        /* Same progress fraction to within integer PWM rounding. */
        int cross = ((int)pulses[0] - start_r) * delta_l -
                    ((int)pulses[1] - start_l) * delta_r;
        assert(abs(cross) <= abs(delta_r) + abs(delta_l));
        if (delta_r && delta_l) assert((pulses[0] == target_r) == (pulses[1] == target_l));
        assert(++frames < 600);
    } while (pulses[0] != target_r || pulses[1] != target_l);
    for (unsigned i = 0; i < 10; ++i) {
        servo_set_remote_value(raw);
        assert(pulses[0] == target_r && pulses[1] == target_l);
    }
    return frames * SERVO_UPDATE_MS;
}

int main(void) {
    motor_init();
    assert(pulses[0] == 1400 && pulses[1] == 1500);
    for (unsigned raw = 0; raw <= 2047; ++raw) {
        servo_center();
        unsigned position = raw < 742 ? 0 : raw > 1242 ? 2 : 1;
        settle(raw, position, position > 1 ? SERVO_LIFT_MAX_STEP_US : SERVO_MAX_STEP_US);
    }
    for (unsigned from = 0; from < 3; ++from) {
        for (unsigned to = 0; to < 3; ++to) {
            servo_center();
            settle(raw_targets[from], from, SERVO_MAX_STEP_US);
            unsigned elapsed = settle(raw_targets[to], to,
                                      to > from ? SERVO_LIFT_MAX_STEP_US : SERVO_MAX_STEP_US);
            if (from != to) printf("position %u -> %u: %u ms\n", from, to, elapsed);
        }
    }
    /* Boundary noise must not change the current outer target. */
    servo_center();
    settle(1243, 2, SERVO_LIFT_MAX_STEP_US);
    settle(1242, 2, SERVO_LIFT_MAX_STEP_US);
    settle(1203, 2, SERVO_LIFT_MAX_STEP_US);
    settle(1202, 1, SERVO_MAX_STEP_US);
    settle(741, 0, SERVO_MAX_STEP_US);
    settle(742, 0, SERVO_MAX_STEP_US);
    settle(781, 0, SERVO_MAX_STEP_US);
    settle(782, 1, SERVO_LIFT_MAX_STEP_US);

    /* Return or reverse at different points before finishing the backward move. */
    for (unsigned frames = 1; frames < 90; ++frames) {
        for (unsigned to = 1; to < 3; ++to) {
            servo_center();
            for (unsigned i = 0; i < frames; ++i) servo_set_remote_value(200);
            settle(raw_targets[to], to, SERVO_LIFT_MAX_STEP_US);
        }
    }
    /* Retarget while lifting, and resume after an immediate failsafe reset. */
    servo_center();
    for (unsigned i = 0; i < 70; ++i) servo_set_remote_value(1800);
    settle(992, 1, SERVO_MAX_STEP_US);
    settle(1800, 2, SERVO_LIFT_MAX_STEP_US);
    servo_center();
    assert(pulses[0] == 1400 && pulses[1] == 1500);
    settle(1242, 1, SERVO_MAX_STEP_US);
    for (unsigned i = 0; i < 3; ++i) {
        servo_set_remote_value(1800);
        assert(pulses[0] == 1400 && pulses[1] == 1500);
    }
    settle(1800, 2, SERVO_LIFT_MAX_STEP_US);
    puts("PASS: independent endpoints, all inputs/transitions, proportional progress, same-frame arrival, interrupted returns, hysteresis, gentle start, limits and reset");
}
""")
    executable = temp / "servo_test"
    subprocess.run([
        "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
        f"-I{temp}", f"-I{ROOT / 'User'}", str(ROOT / "User/motor.c"),
        str(temp / "test.c"), "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)
