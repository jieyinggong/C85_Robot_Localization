#ifndef INTERSECTION_ORIGIN_H
#define INTERSECTION_ORIGIN_H

#include <stdbool.h>
#include <math.h>

int scan_intersection(int *tl, int *tr, int *br, int *bl);
int leftright_turn_degrees(int direction, double target_angle);
#endif