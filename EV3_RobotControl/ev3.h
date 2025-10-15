#ifndef __localization_header
#define __localization_header

#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<malloc.h>
#include "btcomm.h"

#ifndef HEXKEY
	#define HEXKEY "00:16:53:56:55:D9"	// <--- SET UP YOUR EV3's HEX ID here
#endif

int get_color_from_rgb(int R, int G, int B, int A);
int find_street(void);

#endif