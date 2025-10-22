#ifndef INTERSECTION_ORIGIN_H
#define INTERSECTION_ORIGIN_H

#include <stdbool.h>
#include <math.h>
<<<<<<<< HEAD:EV3_RobotControl/intersection_origin.h
========
#include "calibration.h"
>>>>>>>> b5a77ce (update  it with clibration and move intersection part):intersection.h

int scan_intersection(int *tl, int *tr, int *br, int *bl);
int leftright_turn_degrees(int direction, double target_angle);
int detect_intersection(void);

int turn_right_90_degrees(void);
int turn_left_90_degrees(void);
int turn_back_180_degrees(void);

#endif