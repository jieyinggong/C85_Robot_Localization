
#include "btcomm.h"
#include <unistd.h>  // Required for usleep
#include <stdbool.h>
#include <math.h>   // Required for fmod


#define PORT_GYRO  PORT_2
#define PORT_COLOR PORT_3
#define MOTOR_LEFT  MOTOR_A
#define MOTOR_RIGHT MOTOR_C

#define DEG_STEP   30.0      
#define WINDOW     6.0     
#define N_SAMPLES  12     


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

int scan_intersection(int *tl, int *tr, int *br, int *bl)
{
 /*
  * This function carries out the intersection scan - the bot should (obviously) be placed at an intersection for this,
  * and the specific set of actions will depend on how you designed your bot and its sensor. Whatever the process, you
  * should make sure the intersection scan is reliable - i.e. the positioning of the sensor is reliably over the buildings
  * it needs to read, repeatably, and as the robot moves over the map.
  * 
  * Use the APIs sensor reading calls to poll the sensors. You need to remember that sensor readings are noisy and 
  * unreliable so * YOU HAVE TO IMPLEMENT SOME KIND OF SENSOR / SIGNAL MANAGEMENT * to obtain reliable measurements.
  * 
  * Recall your lectures on sensor and noise management, and implement a strategy that makes sense. Document your process
  * in the code below so your TA can quickly understand how it works.
  * 
  * Once your bot has read the colours at the intersection, it must return them using the provided pointers to 4 integer
  * variables:
  * 
  * tl - top left building colour
  * tr - top right building colour
  * br - bottom right building colour
  * bl - bottom left building colour
  * 
  * The function's return value can be used to indicate success or failure, or to notify your code of the bot's state
  * after this call.
  */
 
  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/
   // intialize gyro sensor
   int ref_angle = 0, rate = 0;
    if (BT_read_gyro(PORT_GYRO, 1, &ref_angle, &rate) != 1) {
        fprintf(stderr, "Failed to reset gyro sensor.\n");
        return -1;
    } else {
        fprintf(stderr, "Gyro sensor reset to zero successfully.\n");
    }

    sleep(2); // Wait a moment for the gyro to stabilize

    // drive forwarde to the start point of scan
    BT_timed_motor_port_start(MOTOR_A, 7, 80, 1400, 80); // Start motor A with power 7, ramp up time 80ms, run time 1400ms, ramp down time 80ms
    BT_timed_motor_port_start(MOTOR_C, 6, 100, 1400, 100); // Start motor C with power 7, ramp up time 80ms
    BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);  // Stop motors A and B with active brake

    sleep(1);

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

        fprintf(stderr, "Testing turn right with 12x30° scan...\n");
        // Start turning right --> Clockwise 360 degrees
        BT_turn(MOTOR_LEFT, 13, MOTOR_RIGHT, -9);

        // angle check and color scanning
        while (theta_sum < 360.0 - 1) {  
            int angle_raw = 0;
            if (BT_read_gyro(PORT_GYRO, 0, &angle_raw, &rate) != 1) {
                fprintf(stderr, "Failed to read gyro sensor.\n");
                break;
            }

            // software accumulate angle
            double dtheta = wrap180((double)angle_raw - last_angle);
            theta_sum += dtheta;
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

            usleep(5000);  // 5 ms 
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
        BT_turn(MOTOR_LEFT, -11, MOTOR_RIGHT, 11);

        // angle check and color scanning
        while (fabs(theta_sum) < 360.0 - 1) {
            int angle_raw = 0;
            if (BT_read_gyro(PORT_GYRO, 0, &angle_raw, &rate) != 1) {
                fprintf(stderr, "Failed to read gyro sensor.\n");
                break;
            }

            // Software accumulate angle
            double dtheta = wrap180((double)angle_raw - last_angle);
            theta_sum += dtheta;
            last_angle = angle_raw;

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
                        fprintf(stderr, "Failed to read NXT color sensor (RGB raw) at sample #%d (%.1f°).\n",
                                k, theta_sum);
                    }
                }
            }

            usleep(5000);  // 5 ms
        }

        BT_motor_port_stop(MOTOR_LEFT | MOTOR_RIGHT, 1);  // Stop after completing 360-degree turn
        fprintf(stderr, "Finished counter-clockwise rotation at %.1f° total.\n", theta_sum);
        sleep(1); // Wait for a while

        // Print all collected samples
        fprintf(stderr, "Right turn samples:\n");
        for (int i = 0; i < N_SAMPLES; ++i) {
            fprintf(stderr, "Sample #%d (%.1f°): RGB = (%d, %d, %d, A=%d) and adjusted RGB = (%d, %d, %d, A=%d)\n",
                    i, sampled_right[i], color_samples_right[i][0], color_samples_right[i][1],
                    color_samples_right[i][2], color_samples_right[i][3],
                    color_samples_right[i][0] + color_samples_right[i][3], color_samples_right[i][1] + color_samples_right[i][3],
                    color_samples_right[i][2] + color_samples_right[i][3], color_samples_right[i][3]);
        }

        fprintf(stderr, "Left turn samples:\n");
        for (int i = 0; i < N_SAMPLES; ++i) {
            fprintf(stderr, "Sample #%d (%.1f°): RGB = (%d, %d, %d, A=%d) and adjusted RGB = (%d, %d, %d, A=%d)\n",
                    i, sampled_left[i], color_samples_left[i][0], color_samples_left[i][1],
                    color_samples_left[i][2], color_samples_left[i][3],
                    color_samples_left[i][0] + color_samples_left[i][3], color_samples_left[i][1] + color_samples_left[i][3],
                    color_samples_left[i][2] + color_samples_left[i][3], color_samples_left[i][3]);
        }
    // Return invalid colour values, and a zero to indicate failure (you will replace this with your code)
    *(tl)=-1;
    *(tr)=-1;
    *(br)=-1;
    *(bl)=-1;

    return(0);
 
}