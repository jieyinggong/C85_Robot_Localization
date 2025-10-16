#include "ev3.h"
#include <time.h>

int get_color_from_rgb(int R, int G, int B, int A) {
  // Thresholds for color detection
  const int RED_THRESHOLD = 200;
  const int GREEN_THRESHOLD = 200;
  const int BLUE_THRESHOLD = 200;
  const int BLACK_THRESHOLD = 50;
  const int WHITE_THRESHOLD = 600;

  int brightness = R + G + B;

  if (brightness < BLACK_THRESHOLD || A > 80) {
    return 4;  // Black
  } else if (brightness > WHITE_THRESHOLD || A < 10) {
    return 5;  // White
  } else if (R > RED_THRESHOLD && G < GREEN_THRESHOLD && B < BLUE_THRESHOLD) {
    return 0;  // Red
  } else if (R > RED_THRESHOLD && G > GREEN_THRESHOLD && B < BLUE_THRESHOLD) {
    return 1;  // Yellow
  } else if (G > GREEN_THRESHOLD && R < RED_THRESHOLD && B < BLUE_THRESHOLD) {
    return 2;  // Green
  } else if (B > BLUE_THRESHOLD && R < RED_THRESHOLD && G < GREEN_THRESHOLD) {
    return 3;  // Blue
  } else {
    return 6;  // Other
  }
}

int find_street(void)
{
    int color = -1;
    int outcome = 0;

    srand(time(NULL));  // random seed once

    int R, G, B, A;
    BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
    color = classify_color_hsv_from_values(R, G, B, A);
    printf("First Color detected: %d\n", color);
    sleep(1);
    if (color == 5) // Black
    {
        printf("Street found!\n");
        return 1;
    }

    while (1)
    {
        // Read color sensor
        int R, G, B, A;
        BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
        color = classify_color_hsv_from_values(R, G, B, A);
        printf("Color detected: %d\n", color);

        if (color == 5) // Black
        {
        BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
        printf("Street found!\n");
        outcome = 1;
        break;
        }

        // Detect red (border)
        if (color == 0)
        {
            printf("Border detected! Backing up...\n");
            BT_drive(MOTOR_A, MOTOR_C, -24, -20);
            sleep(3);
            BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

            // Turn away from border
            int angle = (rand() % 120) + 60; // 60°–180° turn
            int dir = (rand() % 2) ? 1 : -1; // random left/right
            BT_turn(MOTOR_A, 30 * dir, MOTOR_C, -30 * dir);
            sleep(3); // crude rotation timing
            BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
            continue;
        }

        // Keep moving forward in small steps
        BT_drive(MOTOR_A, MOTOR_C, 0, 10);
        sleep(1);
        BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
    }

    return outcome;
}

// Returns: 0=RED, 1=YELLOW, 2=GREEN, 3=BLUE, 4=WHITE, 5=BLACK, -1=UNKNOWN
int classify_color_hsv_from_values(int R, int G, int B, int A)
{
    // ---- thresholds (tune if needed) ----
    const double V_BLACK = 0.18;   // darker than this => black
    const double S_WHITE = 0.18;   // low saturation + high value => white
    const double V_WHITE = 0.75;
    const double S_MIN   = 0.22;   // if saturation below this, hue unreliable

    double H,S,V;
    rgba_to_hsv(R, G, B, A, &H, &S, &V);   // <-- use your function

    // Black / White guards
    if (V <= V_BLACK)                 return 5; // BLACK
    if (S <= S_WHITE && V >= V_WHITE) return 4; // WHITE
    if (S < S_MIN)                    return -1; // too desaturated to trust hue

    // Hue-based buckets (degrees)
    if ((H >=   0.0 && H <  25.0) || (H >= 335.0 && H < 360.0)) return 0; // RED
    if ( H >=  25.0 && H <  70.0)                               return 1; // YELLOW
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

    return -1; // UNKNOWN
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
