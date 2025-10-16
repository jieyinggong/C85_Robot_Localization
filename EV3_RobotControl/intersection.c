
#include "btcomm.h"
#include <unistd.h>  // Required for usleep
#include <stdbool.h>
#include <math.h>   // Required for fmod


#define PORT_GYRO  PORT_2
#define PORT_COLOR PORT_3
#define MOTOR_LEFT  MOTOR_A
#define MOTOR_RIGHT MOTOR_C

#define DEG_STEP   15.0      
#define WINDOW     5.0     
#define N_SAMPLES  24     


//(-180,180]
static inline double wrap180(double x){
    x = fmod(x + 180.0, 360.0);
    if (x < 0) x += 360.0;
    return x - 180.0;
}

//  [0,360)
static inline double wrap360(double x){
    x = fmod(x, 360.0);
    if (x < 0) x += 360.0;
    return x;
}

int get_region(double angle) {
    if (angle < 90 && angle >= 0) return 0;
    else if (angle < 180) return 1;
    else if (angle < 270) return 2;
    else return 3;
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

// Returns: 0=RED, 1=YELLOW, 2=GREEN, 3=BLUE, 4=WHITE, 5=BLACK, -1=UNKNOWN
int classify_color_hsv_from_values(int R, int G, int B, int A, bool color)
{
    double env_light = (double)(A) / !((R + G + B)/3 + A); // estimated environmental light level
    // ---- thresholds (tune if needed) ----
    double V_BLACK = 0.2; // when using ambient mode, black is lighter due to ambient light
   double S_BLACK = 0.25;   // low saturation + low value => black
    double S_WHITE = 0.18;   // low saturation + high value => white
    double V_WHITE = 0.75;
    double S_MIN   = 0.15;   // if saturation below this, hue unreliable

    if (color) { V_BLACK = 0.15; } // when using color mode, black can be darker
    else        {V_BLACK = 0.25; } // when using ambient mode, black is lighter due to ambient light

    double H,S,V;
    rgba_to_hsv(R, G, B, A, &H, &S, &V);   // <-- use your function

    // Black / White guards
    if (V <= V_BLACK)                 return 5; // BLACK
    if (S <= S_BLACK && V <= V_BLACK + 0.2) return 5; // BLACK
    if (S <= S_WHITE && V >= V_WHITE) return 4; // WHITE
    if (S < S_MIN)                    return 6; // too desaturated to trust hue

    // Hue-based buckets (degrees)
    if ((H >=   0.0 && H <  25.0) || (H >= 335.0 && H < 360.0)) return 0; // RED
    if (H >=  25.0 && H <  70.0) {
        if (V >= V_WHITE -0.2 && S <= 0.5 && env_light > 0.3) return 4; // WHITE (high value, low saturation)
         if (H >= 60.0 && H < 70.0) return 2; // Likely green in environmental light
        if (V <= V_BLACK +0.1) return 5; // BLACK (low value, low saturation)
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

int scan_intersection(int *tl, int *tr, int *br, int *bl)
{
   // intialize gyro sensor
   int ref_angle = 0, rate = 0;
    if (BT_read_gyro(PORT_GYRO, 1, &ref_angle, &rate) != 1) {
        fprintf(stderr, "Failed to reset gyro sensor.\n");
        return -1;
    } else {
        fprintf(stderr, "Scan intersection: Gyro sensor reset to zero successfully.\n");
    }

    sleep(1); // Wait a moment for the gyro to stabilize
    // drive forwarde to the start point of scan
    BT_timed_motor_port_start(MOTOR_A, 7, 80, 1400, 80); // Start motor A with power 7, ramp up time 500ms, run time 1400ms, ramp down time 80ms
    BT_timed_motor_port_start(MOTOR_C, 6, 100, 1380, 100); // Start motor C with power 6, ramp up time 500ms
   // BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);  // Stop motors A and B with active brake
    fprintf(stderr, "Drive forward to start point of scan.\n");
    sleep(3); // Wait for the bot to reach the start point
    sleep(1); // wait for a while
    // ADD A ANGLE CHECK HERE TO ENSURE IT IS STRAIGHT
      int  current_angle = 0, current_rate = 0;
    if (BT_read_gyro(PORT_GYRO, 0, &current_angle, &current_rate) != 1) {
        fprintf(stderr, "Failed to read gyro sensor for angle check.\n");
        return -1;
    } else {
        fprintf(stderr, "Current angle before scan: %d°\n", current_angle);
        if (fabs(current_angle) > 5) { // Allowable deviation threshold
            fprintf(stderr, "Robot is not straight. \n");
        }
    }
    
    
    // start scanning
        double last_angle = 0.0;
        double theta_sum  = 0.0;      // accumulated angle
        double target_deg = DEG_STEP; // next sampling target
        double sampled_right[N_SAMPLES]; // record sampled angles for right turn
        double sampled_left[N_SAMPLES];  // record sampled angles for left turn
        for (int i = 0; i < N_SAMPLES; ++i) {
            sampled_right[i] = -1.0;
            sampled_left[i] = -1.0;
        }
        int color_samples_right[N_SAMPLES][4] = {0}; // record sampled color rgba for right turn
        int color_samples_left[N_SAMPLES][4] = {0};  // record sampled color rgba for left turn

        int start_color = 6;
        {
            int R, G, B, A;
            if (BT_read_colour_RGBraw_NXT(PORT_COLOR, &R, &G, &B, &A) == 1) {
                start_color = classify_color_hsv_from_values(R, G, B, A, false);
                fprintf(stderr, "Start color scan: RGB = (%d, %d, %d), A= %d and RGB adjusted = (%d, %d, %d), color index = %d\n",
                        R, G, B, A, R + A, G + A, B + A, start_color);
            } else {
                fprintf(stderr, "Failed to read NXT color sensor (RGB raw) at start.\n");
            }
        }
        sleep(1); // wait for a while

        fprintf(stderr, "Testing turn right with 12x30° scan...\n");
        // Start turning right --> Clockwise 360 degrees
        BT_turn(MOTOR_LEFT, 13, MOTOR_RIGHT, -9);

        // angle check and color scanning
        while (theta_sum < 360.0 + 0.5) {  
            int angle_raw = 0;
            if (BT_read_gyro(PORT_GYRO, 0, &angle_raw, &rate) != 1) {
                fprintf(stderr, "Failed to read gyro sensor.\n");
                break;
            }

            // software accumulate angle
            double dtheta = wrap180((double)angle_raw - last_angle);
            theta_sum += dtheta;
            if (theta_sum >= 360.0 + 0.5) break; // safety check to avoid overshoot
            last_angle = angle_raw;

            double theta_norm = wrap360(theta_sum);

            // check if reached sampling point
            for (int k = 0; k < N_SAMPLES; ++k) {
                double target = k * DEG_STEP;
                if (sampled_right[k] == -1.0 && fabs(wrap180(theta_norm - target)) < WINDOW) {
                    int R, G, B, A;
                    if (BT_read_colour_RGBraw_NXT(PORT_COLOR, &R, &G, &B, &A) == 1) {
                        color_samples_right[k][0] = R;
                        color_samples_right[k][1] = G;
                        color_samples_right[k][2] = B;
                        color_samples_right[k][3] = A;
                        sampled_right[k] = theta_norm;
                    } else {
                        fprintf(stderr, "Failed to read NXT color sensor (RGB raw) at sample #%d (%.1f°).\n",
                                k, theta_sum);
                    }
                }
            }

           // usleep(5000);  // 5 ms 
        }

        BT_motor_port_stop(MOTOR_LEFT | MOTOR_RIGHT, 1);  // stop while finish 360 degree turn
        fprintf(stderr, "Finished clockwise rotation at %.1f° total.\n", theta_sum);
        sleep(1); // wait for a while

        // Start counter-clockwise scanning
        last_angle = 0.0;
        theta_sum = 0.0;      // Reset accumulated angle
        target_deg = DEG_STEP; // Reset next sampling target

        fprintf(stderr, "Testing turn left counter-clockwise with 12x30° scan...\n");
        // Start turning left --> Counter-clockwise 360 degrees
        BT_turn(MOTOR_LEFT, -10, MOTOR_RIGHT, 12);

        // angle check and color scanning
        while (fabs(theta_sum) < 360.0 - 2) {
            int angle_raw = 0;
            if (BT_read_gyro(PORT_GYRO, 0, &angle_raw, &rate) != 1) {
                fprintf(stderr, "Failed to read gyro sensor.\n");
                break;
            }

            // Software accumulate angle
            double dtheta = wrap180((double)angle_raw - last_angle);
            theta_sum += dtheta;
            last_angle = angle_raw;
            if (fabs(theta_sum) >= 360.0 - 2) break; // safety check to avoid overshoot
            double theta_norm = wrap360(theta_sum);

            // Check if reached sampling point
            for (int k = 0; k < N_SAMPLES; ++k) {
                double target = -k * DEG_STEP;
                if (sampled_left[k] == -1.0 && fabs(wrap180(theta_norm - target)) < WINDOW) {
                    int R, G, B, A;
                    if (BT_read_colour_RGBraw_NXT(PORT_COLOR, &R, &G, &B, &A) == 1) {
                        color_samples_left[k][0] = R;
                        color_samples_left[k][1] = G;
                        color_samples_left[k][2] = B;
                        color_samples_left[k][3] = A;
                        sampled_left[k] = theta_norm;
                    } else {
                        fprintf(stderr, "Failed to read NXT color sensor (RGB raw) at sample #%d (%.1f°).\n", k, theta_sum);
                    }
                }
            }

           // usleep(5000);  // 5 ms
        }

        BT_motor_port_stop(MOTOR_LEFT | MOTOR_RIGHT, 1);  // Stop after completing 360-degree turn
        fprintf(stderr, "Finished counter-clockwise rotation at %.1f° total.\n", theta_sum);
        sleep(1); // Wait for a while
        // Arrays to store color indices
        int color_indices_right[N_SAMPLES] = {0};
        int color_indices_left[N_SAMPLES] = {0};

        // Print all collected samples
        fprintf(stderr, "Right turn samples:\n");
        for (int i = 0; i < N_SAMPLES; ++i) {
            color_indices_right[i] = classify_color_hsv_from_values(
            color_samples_right[i][0], color_samples_right[i][1],
            color_samples_right[i][2], color_samples_right[i][3], true);

            fprintf(stderr, "Sample #%d (%.1f°): RGB = (%d, %d, %d, A=%d) and adjusted RGB = (%d, %d, %d, A=%d) and color index = %d\n",
                i, sampled_right[i], color_samples_right[i][0], color_samples_right[i][1],
                color_samples_right[i][2], color_samples_right[i][3],
                color_samples_right[i][0] + color_samples_right[i][3], color_samples_right[i][1] + color_samples_right[i][3],
                color_samples_right[i][2] + color_samples_right[i][3], color_samples_right[i][3],
                color_indices_right[i]);
        }

        fprintf(stderr, "Left turn samples:\n");
        for (int i = 0; i < N_SAMPLES; ++i) {
            color_indices_left[N_SAMPLES - 1 - i] = classify_color_hsv_from_values(
            color_samples_left[i][0], color_samples_left[i][1],
            color_samples_left[i][2], color_samples_left[i][3], true);

            fprintf(stderr, "Sample #%d (%.1f°): RGB = (%d, %d, %d, A=%d) and adjusted RGB = (%d, %d, %d, A=%d) and color index = %d\n",
                i, sampled_left[i], color_samples_left[i][0], color_samples_left[i][1],
                color_samples_left[i][2], color_samples_left[i][3],
                color_samples_left[i][0] + color_samples_left[i][3], color_samples_left[i][1] + color_samples_left[i][3],
                color_samples_left[i][2] + color_samples_left[i][3], color_samples_left[i][3],
                color_indices_left[N_SAMPLES - 1 - i]);
        }

        // Print both arrays with their color indices and corresponding colors
        const char *color_names[] = {"RED", "YELLOW", "GREEN", "BLUE", "WHITE", "BLACK", "UNKNOWN"};
        fprintf(stderr, "Comparing Right and Left turn samples:\n");
        for (int i = 0; i < N_SAMPLES; ++i) {
            if (sampled_right[i] == -1.0) {
            color_indices_right[i] = 6; // UNKNOWN
            }
            if (sampled_left[N_SAMPLES - 1 - i] == -1.0) {
            color_indices_left[i] = 6; // UNKNOWN
            }

            const char *right_color = (color_indices_right[i] >= 0 && color_indices_right[i] <= 5) 
                  ? color_names[color_indices_right[i]] : color_names[6];
            const char *left_color = (color_indices_left[i] >= 0 && color_indices_left[i] <= 5) 
                 ? color_names[color_indices_left[i]] : color_names[6];

            fprintf(stderr, "Index #%d: Right = %s (%d), Left = %s (%d)\n",
            i, right_color, color_indices_right[i], left_color, color_indices_left[i]);
        }

        int region_color[4] = {0} ; // top-right, bottom-right, bottom-left, top-left
        double color_vote[4][7] = {0}; // count votes for each color in each region
        double right_weight[4] = {1.5, 1.2, 1, 0.8}; 
        double left_weight[4]  = {0.8, 1, 1.2, 1.5};

        for (int i = 0; i < N_SAMPLES; ++i) {
            // ---- Right turn ----
            if (sampled_right[i] >= 0.0) {
                double angle = sampled_right[i];
                int c = color_indices_right[i];

                int region = get_region(angle); // 0~3
                int center = region * 90 + 45; // center angle of the region
                double w_angle = exp(-pow((angle - center)/45.0, 2));
                double w_dir   = right_weight[region];
                color_vote[region][c] += w_angle * w_dir;
                // fprintf(stderr, "Right turn sample #%d: angle=%.1f°, region=%d, color index=%d, w_angle=%.2f, w_dir=%.2f, vote=%.2f\n",
                //     i, angle, region, c, w_angle, w_dir, w_angle * w_dir);
            }

            // ---- Left turn ----
            int j = N_SAMPLES - 1 - i;
            if (sampled_left[j] >= 0.0) {
                double angle = sampled_left[j];
                int c = color_indices_left[i];

                int region = get_region(angle);
                int center = 45.0 + 90.0 * region;
                double w_angle = exp(-pow((angle - center)/45.0, 2));
                double w_dir   = left_weight[region];
                color_vote[region][c] += w_angle * w_dir;
                // fprintf(stderr, "Left turn sample #%d: angle=%.1f°, region=%d, color index=%d, w_angle=%.2f, w_dir=%.2f, vote=%.2f\n",
                //     j, angle, region, c, w_angle, w_dir, w_angle * w_dir);
            }

        }
        // Print vote counts for each region
        for (int region = 0; region < 4; ++region) {
            fprintf(stderr, "Region %d vote counts: ", region);
            for (int color = 0; color < 7; ++color) {
                fprintf(stderr, "%s=%.2f ", color_names[color], color_vote[region][color]);
            }
            fprintf(stderr, "\n");
        }

        // Determine the color for each region based on majority vote
            for (int region = 0; region < 4; ++region) {
                double max_votes = 0;
                int best_color = 6; // Default to UNKNOWN
                for (int color = 0; color < 7; ++color) {
                // Only consider blue, green, or white
                if ((color == 3 || color == 2 || color == 4) && color_vote[region][color] > max_votes) {
                    max_votes = color_vote[region][color];
                    best_color = color;
                }
                }
                region_color[region] = best_color;
                fprintf(stderr, "Region %d: Color index = %d (%s) with %.3f votes\n",
                    region, best_color,
                    (best_color >= 0 && best_color <= 5) ? color_names[best_color] : color_names[6],
                    max_votes);

            }
            // for testing 

           // color_vote[1][6] += 15.0; // artificially increase unknown votes for region 0
            // Handle regions with unknown color or over 50% unknown votes
            for (int region = 0; region < 4; ++region) {
                double unknown_votes = color_vote[region][6];
                double total_votes = 0.0;
                for (int color = 0; color < 7; ++color) {
                    total_votes += color_vote[region][color];
                }

                if (region_color[region] == 6 || (unknown_votes / total_votes > 0.5)) {
                    fprintf(stderr, "Region %d has unknown color or over 50%% unknown votes. Performing specified check...\n", region);

                    // Calculate average angle for unknown color in this region
                    double avg_angle = 0.0;
                    int unknown_count = 0;
                    for (int i = 0; i < N_SAMPLES; ++i) {
                        if (sampled_right[i] >= 0.0 && get_region(sampled_right[i]) == region && color_indices_right[i] == 6) {
                            avg_angle += sampled_right[i];
                            ++unknown_count;
                        }
                        int j = N_SAMPLES - 1 - i;
                        if (sampled_left[j] >= 0.0 && get_region(sampled_left[j]) == region && color_indices_left[i] == 6) {
                            avg_angle += sampled_left[j];
                            ++unknown_count;
                        }
                    }

                    if (unknown_count > 0) {
                        avg_angle /= unknown_count;
                    } else {
                        avg_angle = 45.0 + 90.0 * region; // Default to region center if no unknown samples
                    }

                    fprintf(stderr, "Region %d: Average angle for unknown color = %.1f°\n", region, avg_angle);

                    // Check the average angle and the center of the region
                    double angles_to_check[] = {avg_angle, 45.0 + 90.0 * region};
                    for (int a = 0; a < 2; ++a) {
                        double angle_to_check = angles_to_check[a];
                        int current_angle = 0, current_rate = 0;
                        if (BT_read_gyro(PORT_GYRO, 0, &current_angle, &current_rate) == 1) {
                             if (fabs(wrap180(angle_to_check - current_angle)) <= 2.0){
                                break;
                             }
                             else if (angle_to_check - current_angle > 0){
                                    BT_turn(MOTOR_LEFT, 13, MOTOR_RIGHT, -9);
                                } else {
                                    BT_turn(MOTOR_LEFT, -10, MOTOR_RIGHT, 12);
                                }
                            while (fabs(wrap180(angle_to_check - current_angle)) > 2.0) { 
                                if (BT_read_gyro(PORT_GYRO, 0, &current_angle, &current_rate) != 1) {
                                    fprintf(stderr, "Failed to read gyro sensor during turn.\n");
                                    break;
                                }
                            }
                            BT_motor_port_stop(MOTOR_LEFT | MOTOR_RIGHT, 1); // Stop motors after turning
                        } else {
                            fprintf(stderr, "Failed to read gyro sensor for angle adjustment.\n");
                        } 
                        int R, G, B, A;
                        if (BT_read_colour_RGBraw_NXT(PORT_COLOR, &R, &G, &B, &A) == 1) {
                            int exact_color = classify_color_hsv_from_values(R, G, B, A, true);
                            double weight = 20.0; // Assign a high weight for this exact check
                            color_vote[region][exact_color] += weight;

                            fprintf(stderr, "Checked angle %.1f°: RGB = (%d, %d, %d, A=%d), exact color = %d (%s), added weight = %.2f\n",
                                    angle_to_check, R, G, B, A, exact_color,
                                    (exact_color >= 0 && exact_color <= 5) ? color_names[exact_color] : color_names[6], weight);
                        } else {
                            fprintf(stderr, "Failed to read NXT color sensor at angle %.1f° for region %d.\n", angle_to_check, region);
                        }
                    }

                    // Recalculate the final color for this region
                    double max_votes = 0;
                    int best_color = 6; // Default to UNKNOWN
                    for (int color = 0; color < 7; ++color) {
                        if ((color == 3 || color == 2 || color == 4) && color_vote[region][color] > max_votes) {
                            max_votes = color_vote[region][color];
                            best_color = color;
                        }
                    }
                    region_color[region] = best_color;
                    fprintf(stderr, "Region %d: Final color index = %d (%s) with %.3f votes\n",
                            region, best_color,
                            (best_color >= 0 && best_color <= 5) ? color_names[best_color] : color_names[6],
                            max_votes);
                }
            }

            // Rotate back to the initial angle (angle = 0)
            int final_angle = 0, final_rate = 0;
            if (BT_read_gyro(PORT_GYRO, 0, &final_angle, &final_rate) == 1) {
                double target_angle = wrap360(final_angle);
                 if (wrap180(target_angle) > 0) {
                        BT_turn(MOTOR_LEFT, -10, MOTOR_RIGHT, 12); // Turn left
                    } else {
                        BT_turn(MOTOR_LEFT, 13, MOTOR_RIGHT, -9); // Turn right
                    }
                while (fabs(wrap180(target_angle)) > 2.0) {
                    if (BT_read_gyro(PORT_GYRO, 0, &final_angle, &final_rate) != 1) {
                        fprintf(stderr, "Failed to read gyro sensor during return to initial angle.\n");
                        break;
                    }
                    target_angle = wrap360(final_angle);
                }
                BT_motor_port_stop(MOTOR_LEFT | MOTOR_RIGHT, 1); // Stop motors after adjustment
                fprintf(stderr, "Returned to initial angle (0°).\n");
            } else {
                fprintf(stderr, "Failed to read gyro sensor for final angle adjustment.\n");
            }

        BT_timed_motor_port_start(MOTOR_A, -7, 80, 1400, 100); 
        BT_timed_motor_port_start(MOTOR_C, -6, 100, 1380, 100);

        sleep(2); // Wait for the bot to return to original position
        fprintf(stderr, "Scan complete.\n");

    // Return invalid colour values, and a zero to indicate failure (you will replace this with your code)
    *(tl)=-1;
    *(tr)=-1;
    *(br)=-1;
    *(bl)=-1;

    return(0);
 
}