#include <stdbool.h>
#include <math.h>
#include "color.h"
#include <stdio.h>
#include "btcomm.h"

int colors[6][3];

int classify_color_hsv_from_values(int R, int G, int B, int A, bool color)
{
    double env_light = (double)(A) / !((R + G + B)/3 + A); // estimated environmental light level
    // ---- thresholds (tune if needed) ----
    double V_BLACK = 0.2; // when using ambient mode, black is lighter due to ambient light
   double S_BLACK = 0.35;   // low saturation + low value => black
    double S_WHITE = 0.18;   // low saturation + high value => white
    double V_WHITE = 0.75;
    double S_MIN   = 0.15;   // if saturation below this, hue unreliable

    if (color) { V_BLACK = 0.15; } // when using color mode, black can be darker
    else        {V_BLACK = 0.25; } // when using ambient mode, black is lighter due to ambient light

    double H,S,V;
    rgba_to_hsv(R, G, B, A, &H, &S, &V);   // <-- use your function

    // Black / White guards
    if (V <= V_BLACK)                 return 5; // BLACK
   // if (S <= S_BLACK && V <= V_BLACK + 0.05) return 5; // BLACK
    if (S <= S_WHITE && V >= V_WHITE) return 4; // WHITE
    if (S < S_MIN)                    return 6; // too desaturated to trust hue

    // Hue-based buckets (degrees)
    if ((H >=   0.0 && H <  25.0) || (H >= 335.0 && H < 360.0)) return 0; // RED
    if (H >=  25.0 && H <  70.0) {
        if (V >= V_WHITE -0.2 && S <= 0.5 && env_light > 0.3) return 4; // WHITE (high value, low saturation)
         if (H >= 60.0 && H < 70.0) return 2; // Likely green in environmental light
        if (V <= S_BLACK && S <= S_BLACK && V <= 0.3) return 5; // BLACK (low value, low saturation)
        return 1; // YELLOW
    }
    if ( H >=  70.0 && H < 170.0)                               return 2; // GREEN
    if ( H >= 190.0 && H < 260.0)                               return 3; // BLUE

    // Resolve in-between regions by nearest primary (optional but helpful)
    if (H >= 170.0 && H < 190.0) { // between green & blue
        return (fabs(H-120.0) <= fabs(H-220.0)) ? 2 : 3;
    }
    if (H >= 260.0 && H < 335.0) { // between blue & red
        double dB = fabs(H-220.0);
        double dR = fmin(fabs(H-360.0), fabs(H-0.0));
        return (dB <= dR) ? 3 : 0;
    }

    return 6; // UNKNOWN
}


// Convert RGBA -> HSV (H in [0,360), S,V in [0,1])
// R,G,B,A are raw sensor ints. H,S,V are outputs.
// If chroma ~ 0 (gray), H is set to -1.0.
void rgba_to_hsv(int R, int G, int B, int A, double *H, double *S, double *V)
{
    const double EPS = 1e-6;

    // 1) Ambient correction + clamp to [0,1] using auto scale
    double Rc = (double)R + (double)A;
    double Gc = (double)G + (double)A;
    double Bc = (double)B + (double)A;

    double raw_max = fmax(Rc, fmax(Gc, Bc));
    double SCALE = (raw_max > 320.0) ? 1023.0 : 255.0;  // auto-detect range

    double r = Rc / SCALE; if (r < 0) r = 0; if (r > 1) r = 1;
    double g = Gc / SCALE; if (g < 0) g = 0; if (g > 1) g = 1;
    double b = Bc / SCALE; if (b < 0) b = 0; if (b > 1) b = 1;

    // 2) RGB -> HSV
    double v = fmax(r, fmax(g, b));
    double m = fmin(r, fmin(g, b));
    double c = v - m;                 // chroma
    double s = (v > EPS) ? (c / v) : 0.0;

    double h;
    if (c < EPS) {
        h = -1.0; // undefined hue for gray/black
    } else if (fabs(v - r) < EPS) {
        h = 60.0 * fmod(((g - b) / (c + EPS)), 6.0);
    } else if (fabs(v - g) < EPS) {
        h = 60.0 * (((b - r) / (c + EPS)) + 2.0);
    } else { // v == b
        h = 60.0 * (((r - g) / (c + EPS)) + 4.0);
    }
    if (h < 0.0) h += 360.0;

    // 3) Write outputs
    if (H) *H = h;
    if (S) *S = s;
    if (V) *V = v;
}

// write a function that use euclidean distance to find the nearest color in the colors array

// void read_color_calibration()
// {
//   FILE *fp = fopen("color_calibration.txt", "r");
//   if (fp == NULL) {
//     fprintf(stderr, "Failed to open file for reading color calibration.\n");
//     return;
//   }

//   char line[100];
//   int idx = 0;
//   while (fgets(line, sizeof(line), fp)) {
//     fprintf(stderr, "%s", line); // Print each line read from the file
//     sscanf(line, "Color %d average RGB = (%d, %d, %d)", &idx, &colors[idx][0], &colors[idx][1], &colors[idx][2]);
//   }

//   fclose(fp);
// }

// use read_color_calibration() to read the color calibration data from color_calibration.txt
// and store it in the global array colors[6][3]


void read_color_calibration()
{
  FILE *fp = fopen("color_calibration.txt", "r");
  if (fp == NULL) {
    fprintf(stderr, "Failed to open file for reading color calibration.\n");
    return;
  }

  char line[100];
  int idx = 0;
  while (fgets(line, sizeof(line), fp)) {
    // fprintf(stderr, "%s", line); // Print each line read from the file
    sscanf(line, "Color %d average RGB = (%d, %d, %d)", &idx, &colors[idx][0], &colors[idx][1], &colors[idx][2]);
    fprintf(stderr, "Loaded Color %d average RGB = (%d, %d, %d)\n", idx, colors[idx][0], colors[idx][1], colors[idx][2]);
    idx++;
  }

  fclose(fp);
}

int classify_color_euclidean(int R, int G, int B, int A)
{
    int adjR = R + A;
    int adjG = G + A;
    int adjB = B + A;

    int min_index = -1;
    double min_dist = 1e9;

    for (int i = 0; i < 6; i++) {
        double dR = adjR - colors[i][0];
        double dG = adjG - colors[i][1];
        double dB = adjB - colors[i][2];
        double dist = sqrt(dR * dR + dG * dG + dB * dB);

        if (dist < min_dist) {
            min_dist = dist;
            min_index = i;
        }
    }

    return min_index;
}

