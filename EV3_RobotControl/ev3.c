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
    const int  BLACK_STABILITY   = 2;
    const int  NONBLACK_DEBOUNCE = 2; 

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
        while (1) {
        // 1) Drive forward while we remain on black (with a little stability to avoid flicker)
        int black_run_confirm = 0;
        BT_drive(MOTOR_A, MOTOR_C, 8, 7);

        while (1) {
            BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
            color = classify_color_hsv_from_values(R, G, B, A);

            if (color == 5) {
                // still black
                if (black_run_confirm < BLACK_STABILITY) black_run_confirm++;
            } else {
                // potential exit from black; require a few consecutive reads
                int nonblack_cnt = 0;
                for (int i = 0; i < NONBLACK_DEBOUNCE; i++) {
                    sleep(2);
                    BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
                    int c2 = classify_color_hsv_from_values(R, G, B, A);
                    if (c2 != 5) nonblack_cnt++;
                }
                if (nonblack_cnt >= NONBLACK_DEBOUNCE) {
                    break; // left black for sure
                }
            }
            sleep(2);
        }

        // We just left black; stop now
        BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

        // 2) Decide based on the new color
        BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
        color = classify_color_hsv_from_values(R, G, B, A);
        printf("Left black, color now: %d\n", color);

        if (color == 1) {
            // Yellow intersection found
            printf("Yellow tile detected — success.\n");
            outcome = 1;
            break;
        }

        if (color == 0) {
            // Red border — back up and turn away more aggressively
            printf("Red border — back & turn away.\n");
            BT_drive(MOTOR_A, MOTOR_C, 8, 7);
            sleep(4);
            BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

            int dir = (rand() % 2) ? 1 : -1;
            BT_turn(MOTOR_A, 12, MOTOR_C, -12);
            sleep(2);
            BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
            // Continue loop (re-enter forward-while-black on next pass)
            continue;
        }

        // Any other color (white/green/blue/unknown):
        // Back a little to return to the last black position, do a small corrective rotate,
        // and push forward again to re-capture the black street.
        printf("Off street (color=%d) — correcting.\n", color);

        // Back up
        BT_drive(MOTOR_A, MOTOR_C, 8, 7);
        sleep(4);
        BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

        // Small corrective rotate (alternate or random)
        int dir = (rand() % 2) ? 1 : -1;  // random left/right nudge
        BT_turn(MOTOR_A, 12 * dir, MOTOR_C, -12 * dir);
        sleep(2);
        BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

        // Short forward push to re-acquire black quickly
        BT_drive(MOTOR_A, MOTOR_C, 8, 7);
        sleep(2);
        BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

        // Now the loop repeats: we’ll drive while black, exit, evaluate, etc.
    }
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

// int find_street(void)
// {
//     // ---- Tunables for 1-second granularity ----
//     const int FWD_L_PWR = 10, FWD_R_PWR = 9;   // gentle forward
//     const int REV_L_PWR = -12, REV_R_PWR = -12; // consistent backup
//     const int TURN_PWR  = 14;                  // small corrective spin
//     const int STEP_FWD_S   = 1;                // 1s forward steps
//     const int POLL_S       = 1;                // sensor poll cadence
//     const int BACK_S       = 1;                // 1s back-up
//     const int ROTATE_S     = 1;                // ~1s rotate nudge
//     const int NONBLACK_DEBOUNCE = 2;           // need 2 consecutive non-black reads
//     const int BLACK_STABILITY   = 2;           // see black in 2 consecutive reads

//     int R=0, G=0, B=0, A=0;
//     int color = -1;
//     int outcome = 0;

//     static int seeded = 0; if (!seeded) { srand((unsigned)time(NULL)); seeded = 1; }

//     // ---- Initial read
//     BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
//     color = classify_color_hsv_from_values(R, G, B, A);
//     printf("First Color detected: %d\n", color);
//     sleep(1);
//     if (color == 5) {
//         printf("Street found!\n");
//         return 1;
//     }

//     // ---- SEARCH: step forward until we find black (avoid red)
//     printf("Searching for street...\n");
//     while (1) {
//         // step forward a bit
//         BT_drive(MOTOR_A, MOTOR_C, FWD_L_PWR, FWD_R_PWR);
//         sleep(STEP_FWD_S);
//         BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

//         // read color
//         BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
//         color = classify_color_hsv_from_values(R, G, B, A);
//         printf("Search color: %d\n", color);

//         if (color == 0) {
//             // red border: back up & turn away
//             printf("Border during search — back & turn.\n");
//             BT_drive(MOTOR_A, MOTOR_C, REV_L_PWR, REV_R_PWR);
//             sleep(BACK_S);
//             BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

//             int dir = (rand() % 2) ? 1 : -1;
//             BT_turn(MOTOR_A, TURN_PWR * dir, MOTOR_C, -TURN_PWR * dir);
//             sleep(ROTATE_S);
//             BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
//             continue;
//         }

//         if (color == 5) {
//             printf("Street found — starting follow loop.\n");
//             break;
//         }
//     }

//     // ---- FOLLOW: move along black until we reach yellow
//     while (1) {
//         // Drive forward while we keep seeing black (with stability)
//         int black_ok = 0;
//         BT_drive(MOTOR_A, MOTOR_C, FWD_L_PWR, FWD_R_PWR);

//         while (1) {
//             BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
//             color = classify_color_hsv_from_values(R, G, B, A);

//             if (color == 5) {
//                 if (black_ok < BLACK_STABILITY) black_ok++;
//             } else {
//                 // confirm we really left black
//                 int nonblack_cnt = 0;
//                 for (int i = 0; i < NONBLACK_DEBOUNCE; i++) {
//                     sleep(POLL_S);
//                     BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
//                     int c2 = classify_color_hsv_from_values(R, G, B, A);
//                     if (c2 != 5) nonblack_cnt++;
//                 }
//                 if (nonblack_cnt >= NONBLACK_DEBOUNCE) {
//                     break; // left black for sure
//                 }
//             }
//             sleep(POLL_S);
//         }

//         // left black — stop where we are
//         BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

//         // read the new tile color
//         BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
//         color = classify_color_hsv_from_values(R, G, B, A);
//         printf("Left black, color now: %d\n", color);

//         if (color == 1) {
//             // Yellow intersection found
//             printf("Yellow tile detected — success.\n");
//             outcome = 1;
//             break;
//         }

//         if (color == 0) {
//             // Red border — back & stronger turn
//             printf("Red border — back & turn away.\n");
//             BT_drive(MOTOR_A, MOTOR_C, REV_L_PWR, REV_R_PWR);
//             sleep(BACK_S);
//             BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

//             int dir = (rand() % 2) ? 1 : -1;
//             BT_turn(MOTOR_A, (TURN_PWR + 4) * dir, MOTOR_C, -(TURN_PWR + 4) * dir);
//             sleep(ROTATE_S);
//             BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
//             continue;
//         }

//         // Other color (white/green/blue/unknown): self-correct
//         printf("Off street (color=%d) — correcting.\n", color);

//         // Back to last black region
//         BT_drive(MOTOR_A, MOTOR_C, REV_L_PWR, REV_R_PWR);
//         sleep(BACK_S);
//         BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

//         // Small corrective rotate (random side)
//         int dir = (rand() % 2) ? 1 : -1;
//         BT_turn(MOTOR_A, TURN_PWR * dir, MOTOR_C, -TURN_PWR * dir);
//         sleep(ROTATE_S);
//         BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

//         // Short forward push to re-acquire black quickly
//         BT_drive(MOTOR_A, MOTOR_C, FWD_L_PWR, FWD_R_PWR);
//         sleep(STEP_FWD_S);
//         BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
//         // Loop will continue and re-enter the “drive while black” phase
//     }

//     return outcome;
// }

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
