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

// int find_street(void)
// {
//     int color = -1;
//     int outcome = 0;
//     const int  BLACK_STABILITY   = 2;
//     const int  NONBLACK_DEBOUNCE = 2; 

//     srand(time(NULL));  // random seed once

//     int R, G, B, A;
//     BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
//     color = classify_color_hsv_from_values(R, G, B, A);
//     printf("First Color detected: %d\n", color);
//     sleep(1);
//     if (color == 5) // Black
//     {
//         printf("Street found!\n");
//         return 1;
//     }

//     while (1)
//     {
//         // Read color sensor
//         int R, G, B, A;
//         BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
//         color = classify_color_hsv_from_values(R, G, B, A);
//         printf("Color detected: %d\n", color);

//         if (color == 5) // Black
//         {
//         BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
//         printf("Street found!\n");
//         while (1) {
//         // 1) Drive forward while we remain on black (with a little stability to avoid flicker)
//         int black_run_confirm = 0;
//         BT_drive(MOTOR_A, MOTOR_C, 8, 7);

//         while (1) {
//             BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
//             color = classify_color_hsv_from_values(R, G, B, A);

//             if (color == 5) {
//                 // still black
//                 if (black_run_confirm < BLACK_STABILITY) black_run_confirm++;
//             } else {
//                 // potential exit from black; require a few consecutive reads
//                 int nonblack_cnt = 0;
//                 for (int i = 0; i < NONBLACK_DEBOUNCE; i++) {
//                     sleep(2);
//                     BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
//                     int c2 = classify_color_hsv_from_values(R, G, B, A);
//                     if (c2 != 5) nonblack_cnt++;
//                 }
//                 if (nonblack_cnt >= NONBLACK_DEBOUNCE) {
//                     break; // left black for sure
//                 }
//             }
//             sleep(2);
//         }

//         // We just left black; stop now
//         BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

//         // 2) Decide based on the new color
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
//             // Red border — back up and turn away more aggressively
//             printf("Red border — back & turn away.\n");
//             BT_drive(MOTOR_A, MOTOR_C, 8, 7);
//             sleep(4);
//             BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

//             int dir = (rand() % 2) ? 1 : -1;
//             BT_turn(MOTOR_A, 12, MOTOR_C, -12);
//             sleep(2);
//             BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
//             // Continue loop (re-enter forward-while-black on next pass)
//             continue;
//         }

//         // Any other color (white/green/blue/unknown):
//         // Back a little to return to the last black position, do a small corrective rotate,
//         // and push forward again to re-capture the black street.
//         printf("Off street (color=%d) — correcting.\n", color);

//         // Back up
//         BT_drive(MOTOR_A, MOTOR_C, 8, 7);
//         sleep(4);
//         BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

//         // Small corrective rotate (alternate or random)
//         int dir = (rand() % 2) ? 1 : -1;  // random left/right nudge
//         BT_turn(MOTOR_A, 12 * dir, MOTOR_C, -12 * dir);
//         sleep(2);
//         BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

//         // Short forward push to re-acquire black quickly
//         BT_drive(MOTOR_A, MOTOR_C, 8, 7);
//         sleep(2);
//         BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

//         // Now the loop repeats: we’ll drive while black, exit, evaluate, etc.
//     }
//         outcome = 1;
//         break;
//         }

//         // Detect red (border)
//         if (color == 0)
//         {
//             printf("Border detected! Backing up...\n");
//             BT_drive(MOTOR_A, MOTOR_C, -24, -20);
//             sleep(3);
//             BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

//             // Turn away from border
//             int angle = (rand() % 120) + 60; // 60°–180° turn
//             int dir = (rand() % 2) ? 1 : -1; // random left/right
//             BT_turn(MOTOR_A, 30 * dir, MOTOR_C, -30 * dir);
//             sleep(3); // crude rotation timing
//             BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
//             continue;
//         }

//         // Keep moving forward in small steps
//         BT_drive(MOTOR_A, MOTOR_C, 0, 10);
//         sleep(1);
//         BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
//     }

//     return outcome;
// }

void recorrect_to_black_internal(int depth);

void verify_and_recorrect_internal(int depth)
{
    int R = 0, G = 0, B = 0, A = 0;
    int color_forward = -1, color_backward = -1;

    printf("[Verify #%d] Checking color consistency around final position...\n", depth);

    // forward
    BT_timed_motor_port_start(MOTOR_A, 7, 80, 1000, 80);
    BT_timed_motor_port_start(MOTOR_C, 6, 100, 1000, 100);
    sleep(2);
    BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
    color_forward = classify_color_hsv_from_values(R, G, B, A, false);
    printf("[Verify #%d] Forward color = %d (R=%d,G=%d,B=%d,A=%d)\n", depth, color_forward, R, G, B, A);

    // back
    BT_timed_motor_port_start(MOTOR_A, -7, 80, 1000, 80);
    BT_timed_motor_port_start(MOTOR_C, -6, 100, 1000, 100);
    sleep(2);

    BT_timed_motor_port_start(MOTOR_A, -7, 80, 1000, 80);
    BT_timed_motor_port_start(MOTOR_C, -6, 100, 1000, 100);
    sleep(2);
    BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
    color_backward = classify_color_hsv_from_values(R, G, B, A, false);
    printf("[Verify #%d] Backward color = %d (R=%d,G=%d,B=%d,A=%d)\n", depth, color_backward, R, G, B, A);
    BT_timed_motor_port_start(MOTOR_A, 7, 80, 1000, 80);
    BT_timed_motor_port_start(MOTOR_C, 6, 100, 1000, 100);
    sleep(2);

    // If either reading is not black/yellow, or they disagree, do another correction
    if (!(color_forward == 5 && color_backward == 5) &&
        !(color_forward == 1 && color_backward == 1) &&
        !(color_forward == 5 && color_backward == 1) &&
        !(color_forward == 1 && color_backward == 5))
    {
        printf("[Verify #%d] Not stable — re-correction triggered.\n", depth);
         BT_turn(MOTOR_A, 0, MOTOR_C, 10);
         sleep(1);
          BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

        if (depth < 5) // limit max recursion depth
        {
            recorrect_to_black_internal(depth + 1);
        }
        else
        {
            printf("[Verify] Max recursion depth reached, stopping further correction.\n");
        }
    }
    else
    {
        printf("[Verify #%d] Both sides confirmed on black — correction stable.\n", depth);
    }
}


void recorrect_to_black_internal(int depth)
{
    int R = 0, G = 0, B = 0, A = 0;
    int color = -1;
    int angle = 0, rate = 0;
    const int BASE_TIME = 1500;
    const double GYRO_SCALE = 1.0 / 60.0;

    printf("[Correction #%d] Starting adaptive correction loop...\n", depth);

    BT_read_gyro(PORT_2, 1, &angle, &rate);

    while (1)
    {
        BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
        color = classify_color_hsv_from_values(R, G, B, A, false);

        if (color == 5 || color == 1)
        {
            printf("[Correction #%d] Reacquired black line.\n", depth);
            BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
            break;
        }

        printf("[Correction #%d] Off street (color=%d) — correcting.\n", depth, color);

        // Backward (angle compensation)
        BT_read_gyro(PORT_2, 0, &angle, &rate);
        double theta = fabs(angle);
        double back_factor = 1.0 + theta * GYRO_SCALE;
        int back_time = (int)(BASE_TIME * back_factor);

        BT_timed_motor_port_start(MOTOR_A, -7, 80, back_time, 80);
        BT_timed_motor_port_start(MOTOR_C, -6, 100, back_time, 100);
        sleep(3);
        BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

        // Rotate
        BT_read_gyro(PORT_2, 1, &angle, &rate);
        int dir = 1;
        BT_turn(MOTOR_A, 0, MOTOR_C, 10 * dir);
        sleep(1);
        BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

        // Forward (angle compensation)
        BT_read_gyro(PORT_2, 0, &angle, &rate);
        double fwd_factor = 1.0 + fabs(angle) * (GYRO_SCALE * 0.8);
        int fwd_time = (int)(BASE_TIME * fwd_factor);
        BT_timed_motor_port_start(MOTOR_A, 7, 80, fwd_time, 80);
        BT_timed_motor_port_start(MOTOR_C, 6, 100, fwd_time, 100);
        sleep(3);
        BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

        sleep(1);
    }

    printf("[Correction #%d] Completed — back on street.\n", depth);
    verify_and_recorrect_internal(depth); 
  }

// External interface (unified entry point)
void recorrect_to_black(void)
{
    recorrect_to_black_internal(1);
}


int find_street(void)
{
    // ---- Tunables ----
    const int FWD_L_PWR = 10, FWD_R_PWR = 9;      // normal forward
    const int REV_L_PWR = -12, REV_R_PWR = -12;   // consistent backup
    const int TURN_PWR  = 14;                     // small corrective spin
    const int STEP_FWD_S   = 1;                   // 1s forward steps
    const int POLL_S       = 1;                   // sensor poll cadence
    const int BACK_S       = 1;                   // back-up duration
    const int ROTATE_S     = 1;                   // small rotate nudge
    const int NONBLACK_DEBOUNCE = 2;              // need 2 consecutive non-black reads
    const int BLACK_STABILITY   = 2;              // need 2 stable black reads

    int R=0, G=0, B=0, A=0;
    int color = -1;
    int outcome = 0;

    static int seeded = 0; 
    if (!seeded) { srand((unsigned)time(NULL)); seeded = 1; }

    // ---- Initial read ----
    BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
    color = classify_color_hsv_from_values(R, G, B, A, false);
    printf("First Color detected: %d\n", color);
    sleep(1);
    if (color == 5) {
        printf("Street found!\n");
        return 1;
    }

    // ---- SEARCH PHASE: move forward until we find black ----
    printf("Searching for street...\n");
    while (1) {
        // small forward step
        BT_drive(MOTOR_A, MOTOR_C, FWD_L_PWR, FWD_R_PWR);
        sleep(STEP_FWD_S);
        BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

        // check color
        BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
        color = classify_color_hsv_from_values(R, G, B, A, false);
        printf("Search color: %d\n", color);

        if (color == 0) {
            // red border — back up & turn away
            printf("Border during search — back & turn.\n");
            BT_drive(MOTOR_A, MOTOR_C, REV_L_PWR, REV_R_PWR);
            sleep(BACK_S);
            BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

            int dir = (rand() % 2) ? 1 : -1;
            BT_turn(MOTOR_A, TURN_PWR * dir, MOTOR_C, -TURN_PWR * dir);
            sleep(ROTATE_S);
            BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
            continue;
        }

        if (color == 5) {
            printf("Street found — starting follow loop.\n");
            break;
        }
    }

    // ---- FOLLOW PHASE: drive along black until yellow ----
    while (1) {
        int black_ok = 0;
        BT_drive(MOTOR_A, MOTOR_C, FWD_L_PWR, FWD_R_PWR);

        while (1) {
            BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
            color = classify_color_hsv_from_values(R, G, B, A, false);

            if (color == 5) {
                if (black_ok < BLACK_STABILITY) black_ok++;
            } else {
                // confirm we really left black
                int nonblack_cnt = 0;
                for (int i = 0; i < NONBLACK_DEBOUNCE; i++) {
                    sleep(POLL_S);
                    BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
                    int c2 = classify_color_hsv_from_values(R, G, B, A, false);
                    if (c2 != 5) nonblack_cnt++;
                }
                if (nonblack_cnt >= NONBLACK_DEBOUNCE)
                    break; // left black confirmed
            }
            sleep(POLL_S);
        }

        // left black — stop
        BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
        BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
        color = classify_color_hsv_from_values(R, G, B, A, false);
        printf("Left black, color now: %d\n", color);

        if (color == 1) {
            printf("Yellow tile detected — success.\n");
            outcome = 1;
            break;
        }

        if (color == 0) {
            // red border — back & stronger turn
            printf("Red border — back & turn away.\n");
            BT_drive(MOTOR_A, MOTOR_C, REV_L_PWR, REV_R_PWR);
            sleep(BACK_S);
            BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

            int dir = (rand() % 2) ? 1 : -1;
            BT_turn(MOTOR_A, (TURN_PWR + 4) * dir, MOTOR_C, -(TURN_PWR + 4) * dir);
            sleep(ROTATE_S);
            BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
            continue;
        }

        // Other color — perform small correction
        printf("Off street (color=%d) — correcting.\n", color);

        //Back up slightly
        BT_timed_motor_port_start(MOTOR_A, -7, 80, 1400, 80);
        BT_timed_motor_port_start(MOTOR_C, -6, 100, 1380, 100);
        sleep(3);

        //Small corrective rotate
        BT_turn(MOTOR_A, TURN_PWR, MOTOR_C, -TURN_PWR);
        sleep(ROTATE_S);
        BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

        //Fine forward nudge — replaced with smooth timed motion
        printf("Micro forward correction (timed).\n");
        BT_timed_motor_port_start(MOTOR_A, 7, 80, 1400, 80);
        BT_timed_motor_port_start(MOTOR_C, 6, 100, 1380, 100);
        sleep(3);
        BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
    }

    return outcome;
}


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
    else        {V_BLACK = 0.18; } // when using ambient mode, black is lighter due to ambient light

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
