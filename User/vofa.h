#ifndef VOFA_H
#define VOFA_H

/* USART6 TX = PC6, 115200 8N1, JustFloat, up to 20 frames/s.
 * I0: link OK; I1..I5: switch 1..5 commands (not measured feedback).
 * I6..I21: SBUS CH1..CH16 (last received values, check I0).
 * I22..I27: motor 1..6 PWM compare values, not measured speed.
 * I28: RX callbacks; I29: decoded frames; I30: last RX bytes; I31: flags.
 * Full channel table: VOFA通道对照.csv and 五路电子开关接线说明.md.
 */
typedef enum {
    VOFA_LINK_OK = 0,
    VOFA_SWITCH_FIRST = 1,
    VOFA_RC_FIRST = 6,
    VOFA_MOTOR_FIRST = 22,
    VOFA_RX_CALLBACKS = 28,
    VOFA_RX_FRAMES = 29,
    VOFA_RX_BYTES = 30,
    VOFA_RX_FLAGS = 31,
    VOFA_CHANNEL_COUNT = 32
} VofaChannel;

void vofa_task(void);

#endif
