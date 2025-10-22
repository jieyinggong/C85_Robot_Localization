#ifndef INTERSECTION_H
#define INTERSECTION_H

#include <stdbool.h>
#include <math.h>
#include "calibration.h"

int scan_intersection(int *tl, int *tr, int *br, int *bl);
int leftright_turn_degrees(int direction, double target_angle);
int detect_intersection(void);

int turn_right_90_degrees(void);
int turn_left_90_degrees(void);
int turn_back_180_degrees(void);

#endif