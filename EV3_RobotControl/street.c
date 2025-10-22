#include "street.h"
#include "btcomm.h"
#include <time.h>
#include <stdbool.h>
#include <math.h>

void verify_and_recorrect_internal(int depth)
{
    int R = 0, G = 0, B = 0, A = 0;
    int color_forward = -1, color_backward = -1;

    printf("[Verify #%d] Checking color consistency around final position...\n", depth);

    // forward
    BT_timed_motor_port_start(MOTOR_A, 7, 80, 1000, 80);
    BT_timed_motor_port_start(MOTOR_D, 6, 100, 1000, 100);
    sleep(2);
    BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
    color_forward =classify_color_hsv(R, G, B, A);
    printf("[Verify #%d] Forward color = %d (R=%d,G=%d,B=%d,A=%d)\n", depth, color_forward, R, G, B, A);

    // back
    BT_timed_motor_port_start(MOTOR_A, -7, 80, 1000, 80);
    BT_timed_motor_port_start(MOTOR_D, -6, 100, 1000, 100);
    sleep(2);

    BT_timed_motor_port_start(MOTOR_A, -7, 80, 1000, 80);
    BT_timed_motor_port_start(MOTOR_D, -6, 100, 1000, 100);
    sleep(2);
    BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
    color_backward = classify_color_hsv(R, G, B, A);
    printf("[Verify #%d] Backward color = %d (R=%d,G=%d,B=%d,A=%d)\n", depth, color_backward, R, G, B, A);
    BT_timed_motor_port_start(MOTOR_A, 7, 80, 1000, 80);
    BT_timed_motor_port_start(MOTOR_D, 6, 100, 1000, 100);
    sleep(2);

    // If either reading is not black/yellow, or they disagree, do another correction
    if (!(color_forward == 0 && color_backward == 0) &&
        !(color_forward == 3 && color_backward == 3) &&
        !(color_forward == 0 && color_backward == 3) &&
        !(color_forward == 3 && color_backward == 0))
    {
        printf("[Verify #%d] Not stable — re-correction triggered.\n", depth);
         BT_turn(MOTOR_A, 0, MOTOR_D, 10);
         sleep(1);
          BT_motor_port_stop(MOTOR_A | MOTOR_D, 1);

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
        color =classify_color_hsv(R, G, B, A);

        if (color == 0 || color == 3)
        {
            printf("[Correction #%d] Reacquired black line.\n", depth);
            BT_motor_port_stop(MOTOR_A | MOTOR_D, 1);
            break;
        }

        printf("[Correction #%d] Off street (color=%d) — correcting.\n", depth, color);

        // Backward (angle compensation)
        BT_read_gyro(PORT_2, 0, &angle, &rate);
        double theta_b = fabs(angle);
        if (theta_b > 180) theta_b = 360 - theta_b;  // wrap to [0,180]
        double theta_rad_b = theta_b * M_PI / 180.0;
        double back_factor = 1.0 + sin(theta_rad_b) * GYRO_SCALE;
        int back_time = (int)(BASE_TIME * back_factor);

        BT_timed_motor_port_start(MOTOR_A, -7, 80, back_time, 80);
        BT_timed_motor_port_start(MOTOR_D, -6, 100, back_time, 100);
        sleep(3);
        BT_motor_port_stop(MOTOR_A | MOTOR_D, 1);

        // Rotate
        BT_read_gyro(PORT_2, 1, &angle, &rate);
        int dir = 1;
        BT_turn(MOTOR_A, 0, MOTOR_D, 10 * dir);
        sleep(1);
        BT_motor_port_stop(MOTOR_A | MOTOR_D, 1);

        // Forward (angle compensation)
        BT_read_gyro(PORT_2, 0, &angle, &rate);
        double theta_f = fabs(angle);
        if (theta_f > 180) theta_f = 360 - theta_f;  // wrap to [0,180]
        double theta_rad_f = theta_f * M_PI / 180.0;
        double fwd_factor = 1.0 + sin(theta_rad_f) * GYRO_SCALE;
        int fwd_time = (int)(BASE_TIME * fwd_factor);
        BT_timed_motor_port_start(MOTOR_A, 7, 80, fwd_time, 80);
        BT_timed_motor_port_start(MOTOR_D, 6, 100, fwd_time, 100);
        sleep(3);
        BT_motor_port_stop(MOTOR_A | MOTOR_D, 1);

        sleep(1);
    }

    printf("[Correction #%d] Completed — back on street.\n", depth);
    verify_and_recorrect_internal(depth); 
  }
 
void recorrect_to_black(void)
{
    recorrect_to_black_internal(1);
}


int find_street(void)
{
    int color = -1;
    int outcome = 0;

    srand(time(NULL));  // random seed once

    int R, G, B, A;
    BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
    color =classify_color_hsv(R, G, B, A);
    printf("First Color detected with RGB(%d, %d, %d, %d): %d\n", R, G, B, A, color);
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
        color =classify_color_hsv(R, G, B, A);
        printf("Color detected: %d\n", color);

        if (color == 0) // Black
        {
        BT_motor_port_stop(MOTOR_A | MOTOR_D, 1);
        printf("Street found!\n");
        outcome = 1;
        break;
        }

        // Detect red (border)
        if (color == 2)
        {
            printf("Border detected! Backing up...\n");
            BT_drive(MOTOR_A, MOTOR_D, -24, -20);
            sleep(3);
            BT_motor_port_stop(MOTOR_A | MOTOR_D, 1);

            // Turn away from border
            int angle = (rand() % 120) + 60; // 60°–180° turn
            int dir = (rand() % 2) ? 1 : -1; // random left/right
            BT_turn(MOTOR_A, 30 * dir, MOTOR_D, -30 * dir);
            sleep(3); // crude rotation timing
            BT_motor_port_stop(MOTOR_A | MOTOR_D, 1);
            continue;
        }

        // Keep moving forward in small steps
        BT_drive(MOTOR_A, MOTOR_D, 0, 10);
        sleep(1);
        BT_motor_port_stop(MOTOR_A | MOTOR_D, 1);
    }

    return outcome;
}

int drive_along_street(void)
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
  fprintf(stderr, "Testing drive forward...\n");
  BT_drive(MOTOR_A, MOTOR_D, 12, 10); // pretty straight forward, will implement PID (use gyro) if have time

  // Test stopping with brake mode
  // stop when detect intersection
  if (detect_intersection()) {
    fprintf(stderr, "Detected intersection, stopping...\n");
    BT_motor_port_stop(MOTOR_A | MOTOR_D, 1);  // Stop motors A and B with active brake
    sleep(1);
    return 1; // Successfully reached an intersection
  }

  fprintf(stderr, "Detected intersection, stopping...\n");
  BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);  // Stop motors A and B with active brake
  sleep(1);
  return 1;

  //return(0);
}

int detect_intersectionc1(void)
{
 /*
  * This function attempts to detect if the bot is currently over an intersection. You can implement this in any way
  * you like, but it should be reliable and robust.
  * 
  * The return value should be 1 if an intersection is detected, and 0 otherwise.
  */   
  int R, G, B, A;
  if (BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A) == 1) {
    fprintf(stderr, "RGB values: R=%d, G=%d, B=%d, A=%d\n", R, G, B, A);
    int color = classify_color_hsv(R, G, B, A);
    if (color == 3) { // Yellow
      fprintf(stderr, "Detected intersection (Yellow)\n");
      return 1;
    } else {
      fprintf(stderr, "Not an intersection\n");
      return 0;
    }
  } else {
    fprintf(stderr, "Failed to read NXT color sensor (RGB raw).\n");
    return 0;
  }
}

int detect_intersection_or_street1(void){
  int R, G, B, A;
  if (BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A) == 1) {
    fprintf(stderr, "RGB values: R=%d, G=%d, B=%d, A=%d\n", R, G, B, A);
    int color = classify_color_hsv(R, G, B, A);
    if (color == 0 || color == 3) { // Yellow
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
