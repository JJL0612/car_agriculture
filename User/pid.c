#include "pid.h"


float delta_speed = 0;

PID_Struct pid1;
PID_Struct pid2;
PID_Struct yaw_pid;


void PID_Init(PID_Struct *pid, float kp, float ki, float kd, float kf, float max_out, float max_i_out)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kf = kf;

    pid->max_out = max_out;
    pid->max_i_out = max_i_out;

    pid->pre_error = 0;
    pid->error = 0;
}

void PID_SetKp(PID_Struct *pid, float kp)
{
    pid->kp = kp;
}

void PID_SetKi(PID_Struct *pid, float ki)
{
    pid->ki = ki;
}

void PID_SetKd(PID_Struct *pid, float kd)
{
    pid->kd = kd;
}

void PID_SetKf(PID_Struct *pid, float kf)
{
    pid->kf = kf;
}

void PID_SetTarget(PID_Struct *pid, float target)
{
    pid->target = target;
}

void PID_SetNow(PID_Struct *pid, float now)
{
    pid->now = now;
}

void PID_Calculate(PID_Struct *pid)
{
    pid->error = pid->target - pid->now;

    pid->p_out = pid->kp * pid->error;
    pid->i_out = pid->ki * pid->error;
    pid->d_out = pid->kd * pid->error;
    pid->f_out = pid->kf * pid->error;

    pid->output = pid->p_out + pid->i_out + pid->d_out + pid->f_out;

    //输出限幅
    if(pid->output > pid->max_out)
    {
        pid->output = pid->max_out;
    }
    else if(pid->output < -pid->max_out)
    {
        pid->output = -pid->max_out;
    }

    //积分限幅
    if(pid->i_out > pid->max_i_out)
    {
        pid->i_out = pid->max_i_out;
    }
    else if(pid->i_out < -pid->max_i_out)
    {
        pid->i_out = -pid->max_i_out;
    }

    pid->pre_error = pid->error;
    pid->pre_target = pid->target;
}