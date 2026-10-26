#ifndef __SENSOR_H
#define __SENSOR_H

#include "motor.h"

extern int n;
extern int i;

void Read_Track();
void car_run_track();
void car_run_track_30s(float target);
void car_run_track_8s(float target);



#endif


