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
#include "ev3.h"

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
  BT_setEV3name("R2D2");

  BT_play_tone_sequence(tone_data);

  // test find_street function
  if (find_street()) 
  {
    fprintf(stderr, "Street found!\n");
  }

  // // Test driving forward
  // fprintf(stderr, "Testing drive forward...\n");
  // BT_drive(MOTOR_A, MOTOR_C, 12, 10);
  // sleep(8);  // Drive for 4 seconds

  // // Test stopping with brake mode
  // fprintf(stderr, "Testing stop with brake mode...\n");
  // BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);  // Stop motors A and B with active brake
  // sleep(1);

  // // Test turning right
  // fprintf(stderr, "Testing turn right...\n");
  // BT_turn(MOTOR_A, 50, MOTOR_C, -50);  // Turn left by running motor A forward and motor B backward
  // sleep(2);

  // // Test turning left
  // fprintf(stderr, "Testing turn left...\n");
  // BT_turn(MOTOR_A, -50, MOTOR_C, 50);  // Turn right by running motor A backward and motor C forward
  // sleep(2);

  // // Test stopping without brake mode
  // fprintf(stderr, "Testing stop without brake mode...\n");
  // BT_motor_port_stop(MOTOR_A | MOTOR_C, 0);  // Stop motors A and B without active brake
  // sleep(1);

  // // Test reading RGB color sensor
  // fprintf(stderr, "Testing NXT color sensor (RGB raw)...\n");
  // int R, G, B, A;
  // if (BT_read_colour_RGBraw_NXT(PORT_1, &R, &G, &B, &A) == 1) {
  //   fprintf(stderr, "RGB values: R=%d, G=%d, B=%d, A=%d\n", R, G, B, A);
  //   int color = get_color_from_rgb(R, G, B, A);
  //   switch (color) {
  //     case 0:
  //       fprintf(stderr, "Detected color: Red\n");
  //       break;
  //     case 1:
  //       fprintf(stderr, "Detected color: Yellow\n");
  //       break;
  //     case 2:
  //       fprintf(stderr, "Detected color: Green\n");
  //       break;
  //     case 3:
  //       fprintf(stderr, "Detected color: Blue\n");
  //       break;
  //     case 4:
  //       fprintf(stderr, "Detected color: Black\n");
  //       break;
  //     case 5:
  //       fprintf(stderr, "Detected color: White\n");
  //       break;
  //     default:
  //       fprintf(stderr, "Detected color: Other\n");
  //       break;
  //   }
  // } else {
  //   fprintf(stderr, "Failed to read NXT color sensor (RGB raw).\n");
  // }

  // // Test reading gyro sensor and turning right 90 degrees
  // fprintf(stderr, "Testing gyro sensor for 90-degree turn...\n");
  // int angle = 0, rate = 0;

  // // Reset gyro sensor to zero
  // if (BT_read_gyro(PORT_2, 1, &angle, &rate) != 1) {
  //   fprintf(stderr, "Failed to reset gyro sensor.\n");
  // } else {
  //   // Start turning right
  //   BT_turn(MOTOR_A, 50, MOTOR_C, -50);  // Turn right

  //   // Monitor the angle until it reaches 90 degrees
  //   while (angle < 90) {
  //     if (BT_read_gyro(PORT_2, 0, &angle, &rate) != 1) {
  //       fprintf(stderr, "Failed to read gyro sensor.\n");
  //       break;
  //     }
  //     fprintf(stderr, "Current angle: %d\n", angle);
  //   }

  //   // Stop the motors
  //   BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);  // Stop with active brake
  // }

  BT_close();
  fprintf(stderr, "Done!\n");
}
