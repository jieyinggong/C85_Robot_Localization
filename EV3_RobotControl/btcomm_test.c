/* EV3 API
 *  Copyright (C) 2018-2019 Francisco Estrada and Lioudmila Tishkina
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

// Initial testing of a bare-bones BlueTooth communication
// library for the EV3 - Thank you Lego for changing everything
// from the NXT to the EV3!

#include "btcomm.h"

int main(int argc, char *argv[]) {
  char test_msg[8] = {0x06, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x01};
  char reply[1024];
  int tone_data[50][3];

  // Reset tone data information
  for (int i = 0; i < 50; i++) {
    tone_data[i][0] = -1;
    tone_data[i][1] = -1;
    tone_data[i][2] = -1;
  }

  tone_data[0][0] = 262;
  tone_data[0][1] = 250;
  tone_data[0][2] = 1;
  tone_data[1][0] = 330;
  tone_data[1][1] = 250;
  tone_data[1][2] = 25;
  tone_data[2][0] = 392;
  tone_data[2][1] = 250;
  tone_data[2][2] = 50;
  tone_data[3][0] = 523;
  tone_data[3][1] = 250;
  tone_data[3][2] = 63;

  memset(&reply[0], 0, 1024);

// just uncomment your bot's hex key to compile for your bot, and comment the
// other ones out.
#ifndef HEXKEY
#define HEXKEY "00:16:53:55:D9:FC"  // <--- SET UP YOUR EV3's HEX ID here
#endif

  BT_open(HEXKEY);

  // name must not contain spaces or special characters
  // max name length is 12 characters
  //BT_setEV3name("R2D2");

  BT_play_tone_sequence(tone_data);

  // Test driving forward
  fprintf(stderr, "Testing drive forward...\n");
  BT_drive(MOTOR_A, MOTOR_C, 12, 10);
  sleep(8);  // Drive for 4 seconds

  // Test stopping with brake mode
  fprintf(stderr, "Testing stop with brake mode...\n");
  BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);  // Stop motors A and B with active brake
  sleep(1);

  // 2. find street (assume robot is on random place on map)
  success = find_street();
  if (success) {
    fprintf(stderr, "Street found, now correcting...\n");
  }

  sleep(1);
  // correct to black line
  recorrect_to_black();
  
  // sleep(1);
  // // Initialize gyro sensor and set it to zero
  // if (BT_read_gyro(PORT_2, 1, &angle, &rate) == 1) {
  //   fprintf(stderr, "Gyro initialized. Current angle: %d, rate: %d\n", angle, rate);
  // } else {
  //   fprintf(stderr, "Failed to initialize gyro sensor.\n");
  // }

  // 3. drive along street until intersection
  success = drive_along_street();
  if (success) {
    fprintf(stderr, "Reached intersection!\n");
    BT_play_tone_sequence(TONE_INTERSECTION);
    sleep(1);
  }else {
    fprintf(stderr, "Failed to reach intersection.\n");
    BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);
    BT_play_tone_sequence(TONE_FAILURE);
    sleep(20);
  }

  // 4. scan intersection
  int tl, tr, br, bl;
  success = scan_intersection(&tl, &tr, &br, &bl);
  if (success) {
    fprintf(stderr, "Scan complete! Colours: TL=%d, TR=%d, BR=%d, BL=%d\n", tl, tr, br, bl);
    // play tones in sequence for tl, tr, br, bl
    // play tone for tl
    if (tl == 2) BT_play_tone_sequence(TONE_GREEN);
    else if (tl == 3) BT_play_tone_sequence(TONE_BLUE);
    else if (tl == 5) BT_play_tone_sequence(TONE_WHITE);
    else BT_play_tone_sequence(tone_data);
    sleep(1);
    // play tone for tr
    if (tr == 2) BT_play_tone_sequence(TONE_GREEN);
    else if (tr == 3) BT_play_tone_sequence(TONE_BLUE);
    else if (tr == 5) BT_play_tone_sequence(TONE_WHITE);
    else BT_play_tone_sequence(tone_data);
    sleep(1);
    // play tone for br
    if (br == 2) BT_play_tone_sequence(TONE_GREEN);
    else if (br == 3) BT_play_tone_sequence(TONE_BLUE); 
    else if (br == 5) BT_play_tone_sequence(TONE_WHITE);
    else BT_play_tone_sequence(tone_data);
    sleep(1);
    // play tone for bl
    if (bl == 2) BT_play_tone_sequence(TONE_GREEN);
    else if (bl == 3) BT_play_tone_sequence(TONE_BLUE);
    else if (bl == 5) BT_play_tone_sequence(TONE_WHITE);
    else BT_play_tone_sequence(tone_data);
    sleep(1);
  }

  // Check if the bot is on an intersection
  // if (!detect_intersection_or_street()) {
  //   fprintf(stderr, "Not on an intersection, adjusting position...\n");

  //   // Adjust position until the intersection is detected
  //   int adjustment_attempts = 0;
  //   while (!detect_intersection_or_street() && adjustment_attempts < 10) {

    double time = 800+adjustment_attempts*100; // increase time for each attempt
      // back
      BT_timed_motor_port_start(MOTOR_A, -7, 80, time, 80);
      BT_timed_motor_port_start(MOTOR_C, -6, 100, time, 100);
      sleep(2);

      if (detect_intersection_or_street()) {
        fprintf(stderr, "Intersection or street found after backward adjustment.\n");
        break;
      }
      // forward
      BT_timed_motor_port_start(MOTOR_A, 7, 80, time, 80);
      BT_timed_motor_port_start(MOTOR_C, 6, 100, time, 100);
      sleep(2);
      if (detect_intersection_or_street()) {
        fprintf(stderr, "Intersection or street found after forward adjustment.\n");
        break;
      }

  //     adjustment_attempts++;
  //   }
  //   if (adjustment_attempts >= 10) {
  //     fprintf(stderr, "Failed to locate intersection after multiple adjustments.\n");
  //   }
  // } else {
  //   fprintf(stderr, "Already on an intersection.\n");
  // }

  // sleep(1);

  // 5. turn right at intersection
  // Reset gyro sensor to zero
  if (BT_read_gyro(PORT_2, 1, &angle, &rate) != 1) {
    fprintf(stderr, "Failed to reset gyro sensor.\n");
  } else {
    // Start turning right
    BT_turn(MOTOR_A, 12, MOTOR_C, -10);  // Turn right

  //   // Monitor the angle until it reaches 90 degrees
  //   while (angle < 90) {
  //     if (BT_read_gyro(PORT_2, 0, &angle, &rate) != 1) {
  //       fprintf(stderr, "Failed to read gyro sensor.\n");
  //       break;
  //     }
  //     fprintf(stderr, "Current angle: %d\n", angle);
  //   }

    // Stop the motors
    BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);  // Stop with active brake
  }
  sleep(1);
  // 6. keep going... (drive along street until next intersection)
  success = drive_along_street();
  if (success) {
    fprintf(stderr, "Reached intersection again!\n");
    BT_play_tone_sequence(TONE_INTERSECTION);
    sleep(1);
  }

  // // 7. turn left at intersection

    if (BT_read_gyro(PORT_2, 1, &angle, &rate) != 1) {
    fprintf(stderr, "Failed to reset gyro sensor.\n");
  } else {
    // Start turning right
    BT_turn(MOTOR_A, -12, MOTOR_C, 10);  // Turn right

    // Monitor the angle until it reaches 90 degrees
    while (angle > -90.5) {
      if (BT_read_gyro(PORT_2, 0, &angle, &rate) != 1) {
        fprintf(stderr, "Failed to read gyro sensor.\n");
        break;
      }
    //  fprintf(stderr, "Current angle: %d\n", angle);
    }

    // Stop the motors
    BT_motor_port_stop(MOTOR_A | MOTOR_D, 1);  // Stop with active brake
  }

  BT_close();
  fprintf(stderr, "Done!\n");
}