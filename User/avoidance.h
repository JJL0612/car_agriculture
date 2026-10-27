#ifndef AVOIDANCE_H
#define AVOIDANCE_H

#include <stdint.h>

/* Installation, viewed from the rear: ID0 left-front, ID1 center-front, ID2 right-front. */
#define AVOID_SENSOR_TIMEOUT_MS 300U
#define AVOID_FRONT_STOP_CM     20.0f
#define AVOID_FRONT_ENTER_CM    50.0f
#define AVOID_FRONT_CLEAR_CM    70.0f
#define AVOID_SIDE_ENTER_CM     30.0f
#define AVOID_SIDE_CLEAR_CM     40.0f
#define AVOID_FORWARD_PWM       300
#define AVOID_TURN_PWM          300

typedef struct {
    float cm[3];
    uint8_t valid[3];
    uint32_t updated[3];
} AvoidanceInput;

void avoidance_reset(void);
/* Signed drive commands -999..999: negative=reverse; magnitude=PWM.
 * Apply direction through the stop/brake guard, then write absolute PWM only.
 * Invalid/stale readings stop both sides.
 */
void avoidance_step(const AvoidanceInput *input, uint32_t now,
                    int *left_pwm, int *right_pwm);

#endif
