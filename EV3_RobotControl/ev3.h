#ifndef __localization_header
#define __localization_header

#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<malloc.h>
#include "btcomm.h"
#include <stdbool.h>

#ifndef HEXKEY
	#define HEXKEY "00:16:53:55:D9:FC"	// <--- SET UP YOUR EV3's HEX ID here
#endif

int get_color_from_rgb(int R, int G, int B, int A);
void recorrect_to_black(void);
int find_street(void);
int classify_color_hsv_from_values(int R, int G, int B, int A, bool color);
void rgba_to_hsv(int R, int G, int B, int A, double *H, double *S, double *V);

#endif