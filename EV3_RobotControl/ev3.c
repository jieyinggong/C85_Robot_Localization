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
    int R, G, B, A;
    int outcome = 0;

    srand(time(NULL));  // random seed once

    while (1)
    {
        // Read color sensor
        BT_read_colour_RGBraw_NXT(PORT_1, &R, &G, &B, &A);
        int color = get_color_from_rgb(R, G, B, A);

        // Detect black (street)
        if (color == 4)
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
            BT_drive(MOTOR_B, MOTOR_C, -24, -20);
            sleep(3);
            BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);

            // Turn away from border
            int angle = (rand() % 120) + 60; // 60°–180° turn
            int dir = (rand() % 2) ? 1 : -1; // random left/right
            BT_turn(MOTOR_B, 30 * dir, MOTOR_C, -30 * dir);
            sleep(3); // crude rotation timing
            BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
            continue;
        }

        // Keep moving forward in small steps
        BT_drive(MOTOR_B, MOTOR_C, 12, 10);
        sleep(1);
        BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
    }

    return outcome;
}
