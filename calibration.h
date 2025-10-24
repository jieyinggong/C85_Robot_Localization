#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <ctime>

// constants
#define EPS 1e-6
#define COLOR_SAMPLE_COUNT 6
#define COLOR_READ_COUNT 50
#define COLOR_COUNT 6

// data structures
typedef struct {
    double H_min, H_max, H_avg;
    double S_min, S_max, S_avg;
    double V_min, V_max, V_avg;
} HSVRange;

typedef struct {
    int color; 
    double probability;
} ColorProbability;

// global variables
extern HSVRange ranges[COLOR_COUNT]; // global variable to hold calibration data
extern ColorProbability color_probabilities[COLOR_COUNT]; // global variable to hold color probabilities

// color calibration functions
void color_calibration(void);
void adjust_hue_overlaps(HSVRange *ranges);
void adjust_black_white_thresholds(HSVRange *ranges);
void print_get_color_calibration(int i);

// color conversion
void rgba_to_hsv(int R, int G, int B, int A, double *H, double *S, double *V);

// color determination
void read_color_calibration(HSVRange *ranges);
int classify_color_hsv(int R, int G, int B, int A);

// color probability functions
void color_probability(void);
void read_color_probability(ColorProbability *color_probabilities); 


#endif
