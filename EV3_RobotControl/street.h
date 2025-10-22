#ifndef STREET_H
#define STREET_H

#include <stdbool.h>
#include <math.h>

void recorrect_to_black(void);
void recorrect_to_black_internal(int depth);
void verify_and_recorrect_internal(int depth);
int find_street(void);
int drive_along_street(void);

#endif