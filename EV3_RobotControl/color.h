
#ifndef COLOR_H
#define COLOR_H


#include <stdbool.h>
#include <math.h>

extern int colors[6][3]; // global array to store color calibration data

int classify_color_hsv_from_values(int R, int G, int B, int A, bool color);
void rgba_to_hsv(int R, int G, int B, int A, double *H, double *S, double *V);
int classify_color_euclidean(int R, int G, int B, int A);
void read_color_calibration();

#endif