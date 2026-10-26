#include "sensor.h"
#include "encoder.h"
#include "jy62.h"

#define STOP_MIN_DISTANCE_CM  150.0f   // 最小停车距离：小车必须行驶超过此距离后才允许触发停止

int n = 0;

int i = 0;


uint8_t track_data[8];

void Read_Track()
{
    track_data[0] = HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin);
    track_data[1] = HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin);
    track_data[2] = HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin);
    track_data[3] = HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin);
    track_data[4] = HAL_GPIO_ReadPin(S5_GPIO_Port, S5_Pin);
    track_data[5] = HAL_GPIO_ReadPin(S6_GPIO_Port, S6_Pin);
    track_data[6] = HAL_GPIO_ReadPin(S7_GPIO_Port, S7_Pin);
    track_data[7] = HAL_GPIO_ReadPin(S8_GPIO_Port, S8_Pin);
}

void car_run_track()
{

    Read_Track();

    // 设定目标速度
    float left_target = 400;
    float right_target = 400;





    if (yaw < 33.0f && yaw > 10.0)
    {
        n = 1;
    }


    // 根据循迹传感器调整目标速度
    else if (track_data[0] == 0 || track_data[1] == 0 || track_data[2] == 0 || track_data[3] == 0 || track_data[4] == 0 || track_data[5] == 0 || track_data[6] == 0 || track_data[7] == 0)
    {
        if (!track_data[0])
        {
            i = 0;
            left_target = 400;
            right_target = 170;
        }
        else if (!track_data[7])
        {
            i = 7;
            left_target = 170;
            right_target = 400;
        }
        else if (!track_data[1])
        {
            i = 1;
            left_target = 400;
            right_target = 220;
        }
        else if (!track_data[6])
        {
            i = 6;
            left_target = 220;
            right_target = 400;
        }
        else if (!track_data[2])
        {
            i = 2;
            left_target = 400;
            right_target = 300;
        }
        else if (!track_data[5])
        {
            i = 5;
            left_target = 300;
            right_target = 400;
        }
        else if (!track_data[3])
        {
            i = 3;
            left_target = 400;
            right_target = 400;
        }
        else if (!track_data[4])
        {
            i = 4;
            left_target = 400;
            right_target = 400;
        }



    }
    else
    {
        if (i == 3)
        {
            left_target = 330;
            right_target = 330;
        }
        else if (i == 4)
        {
            left_target = 330;
            right_target = 330;
        }
        else if (i == 2)
        {
            left_target = 330;
            right_target = 260;
        }
        else if (i == 5)
        {
            left_target = 260;
            right_target = 330;
        }
        else if (i == 1)
        {
            left_target = 330;
            right_target = 220;
        }
        else if (i == 6)
        {
            left_target = 220;
            right_target = 330;
        }
        else if (i == 0)
        {
            left_target = 330;
            right_target = 150;
        }
        else if (i == 7)
        {
            left_target = 150;
            right_target = 330;
        }
        else
        {
            left_target = 330;
            right_target = 330;
        }
    }

    motor_forward((int)right_target, (int)right_target, (int)right_target,
                  (int)left_target,  (int)left_target,  (int)left_target);
}

void car_run_track_30s(float target)
{

    Read_Track();

    // 设定目标速度

    float left_target = target;
    float right_target = target;


    if (yaw < 30.0f && yaw > 10.0)
    {
        n = 1;
    }


        // 根据循迹传感器调整目标速度
    else if (track_data[0] == 0 || track_data[1] == 0 || track_data[2] == 0 || track_data[3] == 0 || track_data[4] == 0 ||
             track_data[5] == 0 || track_data[6] == 0 || track_data[7] == 0) {
        if (!track_data[0]) {
            i = 0;
            left_target = target;
            right_target = target / 2.50f;
        } else if (!track_data[7]) {
            i = 7;
            left_target = target / 2.50f;
            right_target = target;
        } else if (!track_data[1]) {
            i = 1;
            left_target = target;
            right_target = target / 1.80f;
        } else if (!track_data[6]) {
            i = 6;
            left_target = target / 1.80f;
            right_target = target;
        } else if (!track_data[2]) {
            i = 2;
            left_target = target;
            right_target = target / 1.20f;
        } else if (!track_data[5]) {
            i = 5;
            left_target = target / 1.20f;
            right_target = target;
        } else if (!track_data[3]) {
            i = 3;
            left_target = target;
            right_target = target;
        } else if (!track_data[4]) {
            i = 4;
            left_target = target;
            right_target = target;
        }


    }

    else
    {
        if (i == 3)
        {
            left_target = 250;
            right_target = 250;
        }
        else if (i == 4)
        {
            left_target = 200;
            right_target = 200;
        }
        else if (i == 2)
        {
            left_target = 200;
            right_target = 150;
        }
        else if (i == 5)
        {
            left_target = 150;
            right_target = 200;
        }
        else if (i == 1)
        {
            left_target = 200;
            right_target = 165;
        }
        else if (i == 6)
        {
            left_target = 165;
            right_target = 200;
        }
        else if (i == 0)
        {
            left_target = 200;
            right_target = 0;
        }
        else if (i == 7)
        {
            left_target = 0;
            right_target = 200;
        }
        else
        {
            left_target = 200;
            right_target = 200;
        }
    }

    motor_forward((int)right_target, (int)right_target, (int)right_target,
                  (int)left_target,  (int)left_target,  (int)left_target);
}

void car_run_track_8s(float target)
{
    Read_Track();

    float left_target = target;
    float right_target = target;


    if (track_data[0] == 0 || track_data[1] == 0 || track_data[2] == 0 || track_data[3] == 0 || track_data[4] == 0 ||
        track_data[5] == 0 || track_data[6] == 0 || track_data[7] == 0) {
        if (!track_data[0]) {
            i = 0;
            left_target = target;
            right_target = target / 2.50f;
        } else if (!track_data[7]) {
            i = 7;
            left_target = target / 2.50f;
            right_target = target;
        } else if (!track_data[1]) {
            i = 1;
            left_target = target;
            right_target = target / 1.80f;
        } else if (!track_data[6]) {
            i = 6;
            left_target = target / 1.80f;
            right_target = target;
        } else if (!track_data[2]) {
            i = 2;
            left_target = target;
            right_target = target / 1.20f;
        } else if (!track_data[5]) {
            i = 5;
            left_target = target / 1.20f;
            right_target = target;
        } else if (!track_data[3]) {
            i = 3;
            left_target = target;
            right_target = target;
        } else if (!track_data[4]) {
            i = 4;
            left_target = target;
            right_target = target;
        }

        motor_forward((int)right_target, (int)right_target, (int)right_target,
                      (int)left_target,  (int)left_target,  (int)left_target);
    }
}
