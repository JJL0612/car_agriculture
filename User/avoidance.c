#include "avoidance.h"

/* -1 left, 0 straight/stopped, +1 right. Keep a turn until clearance recovers. */
static int turn;

void avoidance_reset(void)
{
    turn = 0;
}

void avoidance_step(const AvoidanceInput *input, uint32_t now,
                    int *left_pwm, int *right_pwm)
{
    *left_pwm = 0;
    *right_pwm = 0;
    for (unsigned i = 0; i < 3U; ++i) {
        if (!input->valid[i] || !(input->cm[i] >= 0.0f) ||
            now - input->updated[i] > AVOID_SENSOR_TIMEOUT_MS) {
            avoidance_reset();
            return;
        }
    }

    float left = input->cm[0];
    float front = input->cm[1];
    float right = input->cm[2];
    if (front <= AVOID_FRONT_STOP_CM ||
        (left <= AVOID_SIDE_ENTER_CM && right <= AVOID_SIDE_ENTER_CM)) {
        avoidance_reset();
        return;
    }

    if (front >= AVOID_FRONT_CLEAR_CM && left >= AVOID_SIDE_CLEAR_CM &&
        right >= AVOID_SIDE_CLEAR_CM) {
        turn = 0;
    }
    if (turn == 0 && (front <= AVOID_FRONT_ENTER_CM ||
                     left <= AVOID_SIDE_ENTER_CM || right <= AVOID_SIDE_ENTER_CM)) {
        /* Equal space: consistently choose left, rather than oscillating. */
        turn = left >= right ? -1 : 1;
    }
    /* If the chosen side closes, use the other side only if it has clearance. */
    if (turn == -1 && left <= AVOID_SIDE_ENTER_CM) turn = 1;
    else if (turn == 1 && right <= AVOID_SIDE_ENTER_CM) turn = -1;

    if (turn == -1) {
        *left_pwm = -AVOID_TURN_PWM;
        *right_pwm = AVOID_TURN_PWM;
    } else if (turn == 1) {
        *left_pwm = AVOID_TURN_PWM;
        *right_pwm = -AVOID_TURN_PWM;
    } else {
        *left_pwm = AVOID_FORWARD_PWM;
        *right_pwm = AVOID_FORWARD_PWM;
    }
}
