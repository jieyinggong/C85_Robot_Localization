
#ifndef COLOR_H
#define COLOR_H


#include <stdbool.h>
#include <math.h>

int classify_color_hsv_from_values(int R, int G, int B, int A, bool color);
void rgba_to_hsv(int R, int G, int B, int A, double *H, double *S, double *V);

#endif