#include "./EV3_RobotControl/btcomm.h"
#include "motion.h"
#include <unistd.h>  // Required for usleep
#include <stdbool.h>
#include <math.h>   // Required for fmod
#include "const.h"

#define DEG_STEP   15.0      
#define WINDOW     5.0     
#define N_SAMPLES  24   

void recorrect_to_black_internal(int depth);
// void micro_swing_correction(int rotate_power);
int find_alternate_street(void);

int detect_intersection(void)
{
 /*
  * This function attempts to detect if the bot is currently over an intersection. You can implement this in any way
  * you like, but it should be reliable and robust.
  * 
  * The return value should be 1 if an intersection is detected, and 0 otherwise.
  */   
  int R, G, B, A;
  int color = 6;
  if (BT_read_colour_RGBraw_NXT(COLOR_PORT, &R, &G, &B, &A) == 1) {
   // fprintf(stderr, "RGB values: R=%d, G=%d, B=%d, A=%d\n", R, G, B, A);
    color = classify_color_hsv(R, G, B, A);
    if (color == C_YELLOW) { // Yellow
      fprintf(stderr, "Detected intersection (Yellow)\n");
      return 1; // meet intersection
    } else if (color == C_BLACK) {
     // fprintf(stderr, "Detected street (Black), not an intersection\n");
      return 0; // on the street 
    } else if (color ==C_RED){
        // RETUR N
        fprintf(stderr, "Detected border (Red), not an intersection\n");
        return 2; // on the street
    } 
    else {
    // the case for not on the street nor intersection
     // fprintf(stderr, "Not an intersection\n");
      return 3;
    }
  } else {
    fprintf(stderr, "Failed to read NXT color sensor (RGB raw).\n");
    return -1;
  }
}

int detect_intersection_or_street(void){
  int R, G, B, A;
  if (BT_read_colour_RGBraw_NXT(COLOR_PORT, &R, &G, &B, &A) == 1) {
    fprintf(stderr, "RGB values: R=%d, G=%d, B=%d, A=%d\n", R, G, B, A);
    int color = classify_color_hsv(R, G, B, A);
    if (color == C_BLACK || color == C_YELLOW) {
      fprintf(stderr, "Detected intersection (Yellow) or Street (Black)\n");
      return 1;
    } else {
      fprintf(stderr, "Not an intersection or street\n");
      return 0;
    }
  } else {
    fprintf(stderr, "Failed to read NXT color sensor (RGB raw).\n");
    return 0;
  }
}


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

// 0: Red, 1: Yellow, 2: Green, 3: Blue, 4: White, 5: Black, 6: Other
int scan_intersection(int *tl, int *tr, int *br, int *bl)
{
    sleep(1);
   // intialize gyro sensor
   int ref_angle = 0, rate = 0;
    if (BT_read_gyro(GYRO_PORT, 1, &ref_angle, &rate) != 1) {
        fprintf(stderr, "Failed to reset gyro sensor.\n");
        return -1;
    } else {
        fprintf(stderr, "Scan intersection: Gyro sensor reset to zero successfully.\n");
    }

    sleep(1); // Wait a moment for the gyro to stabilize
    // drive forwarde to the start point of scan
    BT_timed_motor_port_start(LEFT_MOTOR, 8, 100, 1500, 80); // Start motor A with power 7, ramp up time 500ms, run time 1400ms, ramp down time 80ms
    BT_timed_motor_port_start(RIGHT_MOTOR, 7, 120, 1500, 100); // Start motor C with power 6, ramp up time 500ms
   // BT_motor_port_stop(MOTOR_A | MOTOR_D, 1);  // Stop motors A and B with active brake
    fprintf(stderr, "Drive forward to start point of scan.\n");
    sleep(3); // Wait for the bot to reach the start point
    // ADD A ANGLE CHECK HERE TO ENSURE IT IS STRAIGHT
      int  current_angle = 0, current_rate = 0;
    if (BT_read_gyro(GYRO_PORT, 0, &current_angle, &current_rate) != 1) {
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
            if (BT_read_colour_RGBraw_NXT(COLOR_PORT, &R, &G, &B, &A) == 1) {
                start_color = classify_color_hsv(R, G, B, A);
                fprintf(stderr, "Start color scan: RGB = (%d, %d, %d), A= %d and RGB adjusted = (%d, %d, %d), color index = %d\n",
                        R, G, B, A, R + A, G + A, B + A, start_color);
            } else {
                fprintf(stderr, "Failed to read NXT color sensor (RGB raw) at start.\n");
            }
        }
        sleep(1); // wait for a while

        fprintf(stderr, "Testing turn right with 12x30° scan...\n");
        // Start turning right --> Clockwise 360 degrees
        BT_turn(LEFT_MOTOR, 13, RIGHT_MOTOR, -9);

        // angle check and color scanning
        while (theta_sum < 360.0 + 0.5) {  
            int angle_raw = 0;
            if (BT_read_gyro(GYRO_PORT, 0, &angle_raw, &rate) != 1) {
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
                    if (BT_read_colour_RGBraw_NXT(COLOR_PORT, &R, &G, &B, &A) == 1) {
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

        BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);  // stop while finish 360 degree turn
        fprintf(stderr, "Finished clockwise rotation at %.1f° total.\n", theta_sum);
        sleep(1); // wait for a while

        // Start counter-clockwise scanning
        last_angle = 0.0;
        theta_sum = 0.0;      // Reset accumulated angle
        target_deg = DEG_STEP; // Reset next sampling target

        fprintf(stderr, "Testing turn left counter-clockwise with 12x30° scan...\n");
        // Start turning left --> Counter-clockwise 360 degrees
        BT_turn(LEFT_MOTOR, -10, RIGHT_MOTOR, 12);

        // angle check and color scanning
        while (fabs(theta_sum) < 360.0 - 2) {
            int angle_raw = 0;
            if (BT_read_gyro(GYRO_PORT, 0, &angle_raw, &rate) != 1) {
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
                    if (BT_read_colour_RGBraw_NXT(COLOR_PORT, &R, &G, &B, &A) == 1) {
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

        BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);  // Stop after completing 360-degree turn
        fprintf(stderr, "Finished counter-clockwise rotation at %.1f° total.\n", theta_sum);
        sleep(1); // Wait for a while
        // Arrays to store color indices
        int color_indices_right[N_SAMPLES] = {0};
        int color_indices_left[N_SAMPLES] = {0};

        // Print all collected samples
        fprintf(stderr, "Right turn samples:\n");
        for (int i = 0; i < N_SAMPLES; ++i) {
            color_indices_right[i] = classify_color_hsv(
            color_samples_right[i][0], color_samples_right[i][1],
            color_samples_right[i][2], color_samples_right[i][3]);

            fprintf(stderr, "Sample #%d (%.1f°): RGB = (%d, %d, %d, A=%d) and adjusted RGB = (%d, %d, %d, A=%d) and color index = %d\n",
                i, sampled_right[i], color_samples_right[i][0], color_samples_right[i][1],
                color_samples_right[i][2], color_samples_right[i][3],
                color_samples_right[i][0] + color_samples_right[i][3], color_samples_right[i][1] + color_samples_right[i][3],
                color_samples_right[i][2] + color_samples_right[i][3], color_samples_right[i][3],
                color_indices_right[i]);
        }

        fprintf(stderr, "Left turn samples:\n");
        for (int i = 0; i < N_SAMPLES; ++i) {
            color_indices_left[N_SAMPLES - 1 - i] = classify_color_hsv(
            color_samples_left[i][0], color_samples_left[i][1],
            color_samples_left[i][2], color_samples_left[i][3]);

            fprintf(stderr, "Sample #%d (%.1f°): RGB = (%d, %d, %d, A=%d) and adjusted RGB = (%d, %d, %d, A=%d) and color index = %d\n",
                i, sampled_left[i], color_samples_left[i][0], color_samples_left[i][1],
                color_samples_left[i][2], color_samples_left[i][3],
                color_samples_left[i][0] + color_samples_left[i][3], color_samples_left[i][1] + color_samples_left[i][3],
                color_samples_left[i][2] + color_samples_left[i][3], color_samples_left[i][3],
                color_indices_left[N_SAMPLES - 1 - i]);
        }

        // Print both arrays with their color indices and corresponding colors
        const char *color_names[] = {"BLACK", "WHITE", "RED", "YELLOW", "GREEN", "BLUE", "UNKNOWN"};
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
                if ((color == C_BLUE || color == C_GREEN || color == C_WHITE) && color_vote[region][color] > max_votes) {
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

                if (region_color[region] == C_UNKNOWN || (unknown_votes / total_votes > 0.5)) {
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
                        if (BT_read_gyro(GYRO_PORT, 0, &current_angle, &current_rate) == 1) {
                             if (fabs(wrap180(angle_to_check - current_angle)) <= 2.0){
                                break;
                             }
                             else if (angle_to_check - current_angle > 0){
                                    BT_turn(LEFT_MOTOR, 13, RIGHT_MOTOR, -9);
                                } else {
                                    BT_turn(LEFT_MOTOR, -10, RIGHT_MOTOR, 12);
                                }
                            while (fabs(wrap180(angle_to_check - current_angle)) > 2.0) {
                                if (BT_read_gyro(GYRO_PORT, 0, &current_angle, &current_rate) != 1) {
                                    fprintf(stderr, "Failed to read gyro sensor during turn.\n");
                                    break;
                                }
                            }
                            BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1); // Stop motors after turning
                        } else {
                            fprintf(stderr, "Failed to read gyro sensor for angle adjustment.\n");
                        } 
                        int R, G, B, A;
                        if (BT_read_colour_RGBraw_NXT(COLOR_PORT, &R, &G, &B, &A) == 1) {
                            int exact_color = classify_color_hsv(R, G, B, A);
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
                    int best_color = C_UNKNOWN; // Default to UNKNOWN
                    for (int color = 0; color < 7; ++color) {
                        if ((color == C_BLUE || color == C_GREEN || color == C_WHITE) && color_vote[region][color] > max_votes) {
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
            if (BT_read_gyro(GYRO_PORT, 0, &final_angle, &final_rate) == 1) {
                double target_angle = wrap360(final_angle);
                 if (wrap180(target_angle) > 0) {
                        BT_turn(LEFT_MOTOR, -10, RIGHT_MOTOR, 12); // Turn left
                    } else {
                        BT_turn(LEFT_MOTOR, 13, RIGHT_MOTOR, -9); // Turn right
                    }
                while (fabs(wrap180(target_angle)) > 2.0) {
                    if (BT_read_gyro(GYRO_PORT, 0, &final_angle, &final_rate) != 1) {
                        fprintf(stderr, "Failed to read gyro sensor during return to initial angle.\n");
                        break;
                    }
                    target_angle = wrap360(final_angle);
                }
                BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1); // Stop motors after adjustment
                fprintf(stderr, "Returned to initial angle (0°).\n");
            } else {
                fprintf(stderr, "Failed to read gyro sensor for final angle adjustment.\n");
            }

        BT_timed_motor_port_start(LEFT_MOTOR, -7, 80, 1400, 100);
        BT_timed_motor_port_start(RIGHT_MOTOR, -6, 100, 1380, 100);

        sleep(2); // Wait for the bot to return to original position
        fprintf(stderr, "Scan complete.\n");

    // Return invalid colour values, and a zero to indicate failure (you will replace this with your code)
    *(tl)=region_color[3];
    *(tr)=region_color[0];
    *(br)=region_color[1];
    *(bl)=region_color[2];

   // BT_timed_motor_port_start(LEFT_MOTOR, 8, 100, 1500, 80); // Start motor A with power 7, ramp up time 500ms, run time 1400ms, ramp down time 80ms
  //BT_timed_motor_port_start(RIGHT_MOTOR, 7, 120, 1500, 100); // Start motor C with power 6, ramp up time 500ms
   // BT_motor_port_stop(MOTOR_A | MOTOR_D, 1);  // Stop motors A and B with active brake
//fprintf(stderr, "Drive forward to start point of scan.\n");
   // sleep(3);

    return 1; // success
 
}

int leftright_turn_degrees(int direction, double target_angle){
    int angle = 0, rate = 0;

    /* Reset gyro to zero */
    if (BT_read_gyro(GYRO_PORT, 1, &angle, &rate) != 1) {
        fprintf(stderr, "Failed to reset gyro sensor.\n");
        return -1;
    }

    BT_timed_motor_port_start(LEFT_MOTOR, 7, 80, 600, 80);
    BT_timed_motor_port_start(RIGHT_MOTOR, 6, 80, 600, 80);
    sleep(1); // Wait a moment for the gyro to stabilize

    if (direction == 1) {
        /* Turn right */
         BT_turn(LEFT_MOTOR, 14, RIGHT_MOTOR, -10);
        /* Monitor until angle reaches about +90 degrees */
        while (angle < target_angle - 0.5) {
            if (BT_read_gyro(GYRO_PORT, 0, &angle, &rate) != 1) {
                fprintf(stderr, "Failed to read gyro sensor.\n");
                break;
            }
         //   fprintf(stderr, "Turning right: Current angle = %d\n", angle);
        }
    } else {
        BT_turn(LEFT_MOTOR, -10, RIGHT_MOTOR, 13);

        // Monitor the angle until it reaches 90 degrees
        while (angle > -target_angle + 0.5) {
            if (BT_read_gyro(GYRO_PORT, 0, &angle, &rate) != 1) {
                fprintf(stderr, "Failed to read gyro sensor.\n");
                break;
            }
        }
    }
    // Stop the motors
    BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);  // Stop with active brake
    usleep(10000);

    return 1;
}

int turn_right_90_degrees(){
    return leftright_turn_degrees(1,90.0);
}

int turn_back_180_degrees(){
    
    leftright_turn_degrees(1,90.0);

    BT_timed_motor_port_start(LEFT_MOTOR, -7, 80, 1200, 80);
    BT_timed_motor_port_start(RIGHT_MOTOR, -6, 80, 1200, 80);
    sleep(1); // Wait a moment for the gyro to stabilize

    leftright_turn_degrees(1,90.0);

    return 1;
    
}

int turn_left_90_degrees(){
    return leftright_turn_degrees(-1,90.0);
}

int drive_along_street(int dir, int* border_flag)
{
 /*
  * This function drives your bot along a street, making sure it stays on the street without straying to other pars of
  * the map. It stops at an intersection.
  * 
  * You can implement this in many ways, including a controlled (PID for example), a neural network trained to track and
  * follow streets, or a carefully coded process of scanning and moving. It's up to you, feel free to consult your TA
  * or the course instructor for help carrying out your plan.
  * 
  * You can use the return value to indicate success or failure, or to inform the rest of your code of the state of your
  * bot after calling this function.
  */   

  // Test driving forward
  fprintf(stderr, "Starting drive_along_street (dir=%d)\n", dir);

  int started = 0;

  while (1) {
    int detected = detect_intersection();

    if (detected == 0) {
        if (!started) {
            fprintf(stderr, "On the street, starting to drive forward...\n");
            started = 1;
            BT_drive(LEFT_MOTOR, RIGHT_MOTOR, dir * 6, dir * 5);
        }
      continue;
    }

    BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);
      // Double-check: small back, return, small forward, return.
      int R = 0, G = 0, B = 0, A = 0;
      int c_back = -1, c_forward = -1;

      // Step back a little
      BT_timed_motor_port_start(LEFT_MOTOR, -8, 80, 500, 80);
      BT_timed_motor_port_start(RIGHT_MOTOR, -7, 100, 500, 100);
      sleep(1);
      BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);
      BT_read_colour_RGBraw_NXT(COLOR_PORT, &R, &G, &B, &A);
      c_back = classify_color_hsv(R, G, B, A);

      // Return to original position
      BT_timed_motor_port_start(LEFT_MOTOR, 8, 80, 500, 80);
      BT_timed_motor_port_start(RIGHT_MOTOR, 7, 100, 500, 100);
      sleep(1);
      BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);

      // Step forward a little
      BT_timed_motor_port_start(LEFT_MOTOR, 8, 80, 500, 80);
      BT_timed_motor_port_start(RIGHT_MOTOR, 7, 100, 500, 100);
      sleep(1);
      BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);
      BT_read_colour_RGBraw_NXT(COLOR_PORT, &R, &G, &B, &A);
      c_forward = classify_color_hsv(R, G, B, A);

      // Return to original position again
      BT_timed_motor_port_start(LEFT_MOTOR, -8, 80, 500, 80);
      BT_timed_motor_port_start(RIGHT_MOTOR, -7, 100, 500, 100);
      sleep(1);
      BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);

      fprintf(stderr, "Double-check: back=%d forward=%d\n", c_back, c_forward);

      // If both checks indicate street (treat YELLOW as street too), resume as if on black
      if ((c_back == C_BLACK || c_back == C_YELLOW || c_back == C_RED) &&
          (c_forward == C_BLACK || c_forward == C_YELLOW || c_forward == C_RED)) {
          fprintf(stderr, "Double-check confirms street — resuming drive.\n");
          if (!started) {
              started = 1;
              BT_drive(LEFT_MOTOR, RIGHT_MOTOR, dir * 6, dir * 5);
          }
          continue;
      }

    if (detected == 1) {
      fprintf(stderr, "Intersection detected. Stopping.\n");
      return 1;
    }

    if (detected == 2) {
        if (border_flag != NULL && *border_flag <= 0){
            dir = -dir;
        }
      (*border_flag)++;
      fprintf(stderr, "Border detected! Backing up. (count=%d)\n", *border_flag);

      BT_drive(LEFT_MOTOR, RIGHT_MOTOR, dir * 8, dir * 7);
      sleep(1);
      BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);
      started = 0;
      continue;  // retry main loop
    }

    if (detected == 3) {
      fprintf(stderr, "Minor deviation detected. Correcting...\n");
      micro_swing_correction(8);
      started = 0;
      continue;
    }
    usleep(10000); // 10 ms
  }

  return 1;
}


void verify_and_recorrect_internal(int depth)
{
    int R = 0, G = 0, B = 0, A = 0;
    int color_forward = -1, color_backward = -1;

    printf("[Verify #%d] Checking color consistency around final position...\n", depth);

    // forward
    BT_timed_motor_port_start(LEFT_MOTOR, 7, 80, 1000, 80);
    BT_timed_motor_port_start(RIGHT_MOTOR, 6, 100, 1000, 100);
    sleep(2);
    BT_read_colour_RGBraw_NXT(COLOR_PORT, &R, &G, &B, &A);
    color_forward = classify_color_hsv(R, G, B, A);
    printf("[Verify #%d] Forward color = %d (R=%d,G=%d,B=%d,A=%d)\n", depth, color_forward, R, G, B, A);

    // back
    BT_timed_motor_port_start(LEFT_MOTOR, -7, 80, 1000, 80);
    BT_timed_motor_port_start(RIGHT_MOTOR, -6, 100, 1000, 100);
    sleep(2);

    BT_timed_motor_port_start(LEFT_MOTOR, -7, 80, 1000, 80);
    BT_timed_motor_port_start(RIGHT_MOTOR, -6, 100, 1000, 100);
    sleep(2);

    BT_timed_motor_port_start(LEFT_MOTOR, -7, 80, 1000, 80);
    BT_timed_motor_port_start(RIGHT_MOTOR, -6, 100, 1000, 100);
    sleep(2);
    
    BT_read_colour_RGBraw_NXT(COLOR_PORT, &R, &G, &B, &A);
    color_backward = classify_color_hsv(R, G, B, A);
    printf("[Verify #%d] Backward color = %d (R=%d,G=%d,B=%d,A=%d)\n", depth, color_backward, R, G, B, A);
    BT_timed_motor_port_start(LEFT_MOTOR, 7, 80, 1000, 80);
    BT_timed_motor_port_start(RIGHT_MOTOR, 6, 100, 1000, 100);
    sleep(2);

    // If either reading is not black/yellow, or they disagree, do another correction
    if (!(color_forward == C_BLACK && color_backward == C_BLACK) &&
        !(color_forward == C_YELLOW && color_backward == C_YELLOW) &&
        !(color_forward == C_BLACK && color_backward == C_YELLOW) &&
        !(color_forward == C_YELLOW && color_backward == C_BLACK))
    {
        printf("[Verify #%d] Not stable — re-correction triggered.\n", depth);
         BT_turn(LEFT_MOTOR, 0, RIGHT_MOTOR, 10);
         sleep(1);
          BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);

        if (depth < 5) // limit max recursion depth
        {
            recorrect_to_black_internal(depth + 1);
        }
        else
        {
            printf("[Verify] Max recursion depth reached, stopping further correction.\n");
            if (find_alternate_street()) {
                recorrect_to_black();
            }    
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

    BT_read_gyro(GYRO_PORT, 1, &angle, &rate);

    while (1)
    {
        BT_read_colour_RGBraw_NXT(COLOR_PORT, &R, &G, &B, &A);
        color =classify_color_hsv(R, G, B, A);

        if (color == C_BLACK || color == C_YELLOW)
        {
            printf("[Correction #%d] Reacquired black line.\n", depth);
            BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);
            break;
        }

        printf("[Correction #%d] Off street (color=%d) — correcting.\n", depth, color);

        // Backward (angle compensation)
        BT_read_gyro(GYRO_PORT, 0, &angle, &rate);
        double theta_b = fabs(angle);
        if (theta_b > 180) theta_b = 360 - theta_b;  // wrap to [0,180]
        double theta_rad_b = theta_b * M_PI / 180.0;
        double back_factor = 1.0 + sin(theta_rad_b) * GYRO_SCALE;
        int back_time = (int)(BASE_TIME * back_factor);

        BT_timed_motor_port_start(LEFT_MOTOR, -7, 80, back_time, 80);
        BT_timed_motor_port_start(RIGHT_MOTOR, -6, 100, back_time, 100);
        sleep(3);
        BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);

        // Rotate
        BT_read_gyro(GYRO_PORT, 1, &angle, &rate);
        int dir = 1;
        BT_turn(LEFT_MOTOR, 0, RIGHT_MOTOR, 10 * dir);
        sleep(1);
        BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);

        // Forward (angle compensation)
        BT_read_gyro(GYRO_PORT, 0, &angle, &rate);
        double theta_f = fabs(angle);
        if (theta_f > 180) theta_f = 360 - theta_f;  // wrap to [0,180]
        double theta_rad_f = theta_f * M_PI / 180.0;
        double fwd_factor = 1.0 + sin(theta_rad_f) * GYRO_SCALE;
        int fwd_time = (int)(BASE_TIME * fwd_factor);
        BT_timed_motor_port_start(LEFT_MOTOR, 7, 80, fwd_time, 80);
        BT_timed_motor_port_start(RIGHT_MOTOR, 6, 100, fwd_time, 100);
        sleep(3);
        BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);

        sleep(1);
    }

    printf("[Correction #%d] Completed — back on street.\n", depth);
    verify_and_recorrect_internal(depth); 
  }
 
void recorrect_to_black(void)
{
    recorrect_to_black_internal(1);
}


int find_street(int dir)
{
    int color = -1;
    int outcome = 0;

    srand(time(NULL));  // random seed once

    int R, G, B, A;
    BT_read_colour_RGBraw_NXT(COLOR_PORT, &R, &G, &B, &A);
    color =classify_color_hsv(R, G, B, A);
    printf("First Color detected with RGB(%d, %d, %d, %d): %d\n", R, G, B, A, color);
    sleep(1);
    if (color == C_BLACK) // Black
    {
        printf("Street found!\n");
        return 1;
    }

    while (1)
    {
        // Read color sensor
        int R, G, B, A;
        BT_read_colour_RGBraw_NXT(COLOR_PORT, &R, &G, &B, &A);
        color =classify_color_hsv(R, G, B, A);
        printf("Color detected: %d\n", color);

        if (color == C_BLACK) // Black
        {
        BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);
        printf("Street found!\n");
        outcome = 1;
        break;
        }

        // // Detect red (border)
        // if (color == C_RED)
        // {
        //     printf("Border detected! Backing up...\n");
        //     BT_drive(LEFT_MOTOR, RIGHT_MOTOR, -24, -20);
        //     sleep(3);
        //     BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);

        //     // Turn away from border
        //     int angle = (rand() % 120) + 60; // 60°–180° turn
        //     int dir = (rand() % 2) ? 1 : -1; // random left/right
        //     BT_turn(LEFT_MOTOR, 30 * dir, RIGHT_MOTOR, -30 * dir);
        //     sleep(3); // crude rotation timing
        //     BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);
        //     continue;
        // }

        // Keep moving forward in small steps
        BT_drive(LEFT_MOTOR, RIGHT_MOTOR, dir * 10, 0);
        sleep(1);
        BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);
    }

    return outcome;
}

int find_alternate_street(void)
{

    leftright_turn_degrees(1, 120);   
    BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);
    sleep(1);

    int result = find_street(-1);
    if (result)
        printf("[Reacquire] Found new street successfully!\n");
    else
        printf("[Reacquire] No street found after rotation.\n");

    return result;
}


void micro_swing_correction(int rotate_power)
{
    int R = 0, G = 0, B = 0, A = 0;
    int color = -1;
    int angle = 0, rate = 0;
    const int MAX_SWING = 4;       
    const int STEP_TIME = 500;     

    printf("[MicroCorrection] Starting small fan-sweep correction...\n");

    int dir = 1;           
    int swing_count = 0;   

    BT_read_gyro(GYRO_PORT, 0, &angle, &rate);

    while (swing_count < MAX_SWING * 2)
    {   
        int time = (int)(STEP_TIME * (1.0 + 0.6 * swing_count));

        if (dir == 1){
            BT_timed_motor_port_start(RIGHT_MOTOR, 10, 100, time, 100);
            BT_timed_motor_port_start(LEFT_MOTOR, -7, 100, time, 100);
        }
        else{
             BT_timed_motor_port_start(LEFT_MOTOR,  11, 100, time, 100);
              BT_timed_motor_port_start(RIGHT_MOTOR, -7, 100, time, 100);
        }
        usleep(1010*(time+500));

        BT_read_colour_RGBraw_NXT(COLOR_PORT, &R, &G, &B, &A);
        color = classify_color_hsv(R, G, B, A);
        printf("[MicroCorrection] Swing #%d dir=%d color=%d with time %d\n", swing_count, dir, color, time);

        if (color == C_BLACK || color == C_YELLOW)
        {
            printf("[MicroCorrection] Street line reacquired — stopping.\n");
                BT_drive(LEFT_MOTOR, RIGHT_MOTOR, 8, 7);
                sleep(1);
                BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);  // Stop motors A and B with active brake
            BT_motor_port_stop(LEFT_MOTOR | RIGHT_MOTOR, 1);
            break;
        }

         if (dir == 1){
            BT_timed_motor_port_start(RIGHT_MOTOR,  -10, 100, time * 0.9, 100);
             BT_timed_motor_port_start(LEFT_MOTOR,  7, 100, time * 0.9, 100);
            
        }
        else{
             BT_timed_motor_port_start(LEFT_MOTOR,  -11, 100, time * 0.9, 100);
             BT_timed_motor_port_start(RIGHT_MOTOR,  7, 100, time * 0.9, 100);
        }
        usleep(1010*(time+500));
        

        dir = -dir;
        swing_count++;

       usleep(100000);
    }

    printf("[MicroCorrection] Completed small fan-sweep correction.\n");
}


int correct_to_intersection(){
    // Check if the bot is on an intersection
  if (!detect_intersection_or_street()) {
    fprintf(stderr, "Not on an intersection, adjusting position...\n");

    // Adjust position until the intersection is detected
    int adjustment_attempts = 0;
    while (!detect_intersection_or_street() && adjustment_attempts < 10) {

    double time = 800+adjustment_attempts*100; // increase time for each attempt
      // back
      BT_timed_motor_port_start(LEFT_MOTOR, -7, 80, time, 80);
      BT_timed_motor_port_start(RIGHT_MOTOR, -6, 100, time, 100);
      sleep(2);

      if (detect_intersection_or_street()) {
        fprintf(stderr, "Intersection or street found after backward adjustment.\n");
        break;
      };
      // forward
      BT_timed_motor_port_start(LEFT_MOTOR, 7, 80, time, 80);
      BT_timed_motor_port_start(RIGHT_MOTOR, 6, 100, time, 100);
      sleep(2);
      if (detect_intersection_or_street()) {
        fprintf(stderr, "Intersection or street found after forward adjustment.\n");
        break;
      }

      adjustment_attempts++;
    }
    if (adjustment_attempts >= 10) {
      fprintf(stderr, "Failed to locate intersection after multiple adjustments.\n");
    }
  } else {
    fprintf(stderr, "Already on an intersection.\n");
  }
  return 1;
}