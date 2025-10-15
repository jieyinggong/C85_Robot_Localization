#include "ev3.h"

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
  /*
   * Simple but safe strategy:
   * - Drive forward while polling the colour sensor in short intervals so red/black are detected during motion.
   * - If red detected -> back up and turn ~180 degrees to avoid leaving the map.
   * - If black detected -> stop and return success.
   * - If nothing after MAX_ATTEMPTS, return failure.
   */
  const int MAX_ATTEMPTS = 40;
  const int FORWARD_TOTAL_US = 200000; // total forward probe time
  const int POLL_US = 40000;           // poll interval while moving
  const int BACKUP_US = 150000;        // backup duration when seeing red
  const int TURN180_US = 700000;       // approximate 180deg spin duration (adjust on robot)
  const int LEFT_DRIVE_POWER = 24;
  const int RIGHT_DRIVE_POWER = 20;
  const int TURN_POWER = 40;

  int R=0,G=0,B=0,A=0;
  int col;
  int attempt;
  int p, q;
  int loops;
  int rotate_loops;
  int restart_attempt;

  // Quick check: already on a street?
  if (BT_read_colour_RGBraw_NXT(PORT_1, &R, &G, &B, &A) == 1) {
    col = get_color_from_rgb(R,G,B,A);
    if (col == 4) { // black
      BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
      return 1;
    }
  }

  for (attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
    restart_attempt = 0;

    // Start driving forward and poll frequently during the motion
    BT_drive(MOTOR_A, MOTOR_C, LEFT_DRIVE_POWER, RIGHT_DRIVE_POWER);
    loops = FORWARD_TOTAL_US / POLL_US;
    for (p = 0; p < loops; p++) {
      usleep(POLL_US);
      if (BT_read_colour_RGBraw_NXT(PORT_1, &R, &G, &B, &A) == 1) {
        col = get_color_from_rgb(R,G,B,A);
        if (col == 0) {
          // RED border detected: immediately stop, back up, and turn around
          BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
          usleep(50000);
          BT_drive(MOTOR_A, MOTOR_C, -LEFT_DRIVE_POWER/2, -RIGHT_DRIVE_POWER/2);
          usleep(BACKUP_US);
          BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
          usleep(80000);
          // Spin ~180 degrees to return into map
          BT_turn(MOTOR_A, TURN_POWER, MOTOR_C, -TURN_POWER);
          usleep(TURN180_US);
          BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
          usleep(100000);
          restart_attempt = 1;
          break; // break forward polling
        }
        if (col == 4) {
          // BLACK (street) detected while moving
          BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
          usleep(50000);
          return 1;
        }
      }
    }
    // stop after forward probe if nothing found or if red handled
    BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
    usleep(40000);

    if (restart_attempt) {
      continue; // start next attempt with new heading
    }

    // Rotate in place while polling (alternate left/right)
    if ((attempt & 1) == 0) {
      BT_turn(MOTOR_A, -TURN_POWER, MOTOR_C, TURN_POWER); // left
    } else {
      BT_turn(MOTOR_A, TURN_POWER, MOTOR_C, -TURN_POWER); // right
    }
    rotate_loops = FORWARD_TOTAL_US / POLL_US; // reuse similar chunk for rotation
    for (q = 0; q < rotate_loops; q++) {
      usleep(POLL_US);
      if (BT_read_colour_RGBraw_NXT(PORT_1, &R, &G, &B, &A) == 1) {
        col = get_color_from_rgb(R,G,B,A);
        if (col == 0) {
          // RED detected during rotation -> stop, back up and turn 180
          BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
          usleep(50000);
          BT_drive(MOTOR_A, MOTOR_C, -LEFT_DRIVE_POWER/2, -RIGHT_DRIVE_POWER/2);
          usleep(BACKUP_US);
          BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
          usleep(80000);
          BT_turn(MOTOR_A, TURN_POWER, MOTOR_C, -TURN_POWER);
          usleep(TURN180_US);
          BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
          usleep(100000);
          restart_attempt = 1;
          break; // break rotation polling and continue attempts
        }
        if (col == 4) {
          BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
          usleep(50000);
          return 1;
        }
      }
    }
    BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
    usleep(40000);

    if (restart_attempt) {
      continue;
    }

    // otherwise loop to next attempt automatically
  }

  // Failed to find a street within attempt limit
  return 0;
}
