#ifndef STREET_H
#define STREET_H

void recorrect_to_black(void);
void recorrect_to_black_internal(int depth);
void verify_and_recorrect_internal(int depth);
int find_street(void);
int classify_color_hsv_from_values(int R, int G, int B, int A, bool color);
void rgba_to_hsv(int R, int G, int B, int A, double *H, double *S, double *V);

#endif