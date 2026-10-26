#ifndef __PID_H
#define __PID_H

#include "motor.h"

#define Kp 3.5
#define Ki 0
#define Kd 0

#define target_speed 100

typedef struct
{
    float kp;
    float ki;
    float kd;
    float kf;

    float p_out;
    float i_out;
    float d_out;
    float f_out;

    float error;

    float output;

    float max_out;
    float max_i_out;

    float target;
    float now;

    float pre_target;
    float pre_now;
    float pre_error;


} PID_Struct;

extern PID_Struct pid1;
extern PID_Struct pid2;
extern PID_Struct yaw_pid;

void PID_Init(PID_Struct *pid, float kp, float ki, float kd, float kf, float max_out, float max_i_out);

void PID_SetKp(PID_Struct *pid, float kp);
void PID_SetKi(PID_Struct *pid, float ki);
void PID_SetKd(PID_Struct *pid, float kd);
void PID_SetKf(PID_Struct *pid, float kf);

void PID_SetTarget(PID_Struct *pid, float target);
void PID_SetNow(PID_Struct *pid, float now);

void PID_Calculate(PID_Struct *pid);


#endif