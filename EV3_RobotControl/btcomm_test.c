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

int get_color_from_rgb(int R, int G, int B, int A);
void color_calibration_rgb(); 

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

  int R, G, B, A;
  color_calibration_rgb();
  
  return 0; 

  // Test driving forward
  fprintf(stderr, "Testing drive forward...\n");
  BT_drive(MOTOR_A, MOTOR_C, 12, 10);
  sleep(8);  // Drive for 4 seconds

  // Test stopping with brake mode
  fprintf(stderr, "Testing stop with brake mode...\n");
  BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);  // Stop motors A and B with active brake
  sleep(1);

  // Test turning right
  fprintf(stderr, "Testing turn right...\n");
  BT_turn(MOTOR_A, 50, MOTOR_C, -50);  // Turn left by running motor A forward and motor B backward
  sleep(2);

  // Test turning left
  fprintf(stderr, "Testing turn left...\n");
  BT_turn(MOTOR_A, -50, MOTOR_C, 50);  // Turn right by running motor A backward and motor C forward
  sleep(2);

  // Test stopping without brake mode
  fprintf(stderr, "Testing stop without brake mode...\n");
  BT_motor_port_stop(MOTOR_A | MOTOR_C, 0);  // Stop motors A and B without active brake
  sleep(1);

  // Test reading RGB color sensor
  fprintf(stderr, "Testing NXT color sensor (RGB raw)...\n");
  // int R, G, B, A;
  if (BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A) == 1) {
    fprintf(stderr, "RGB values: R=%d, G=%d, B=%d, A=%d\n", R, G, B, A);
    int color = get_color_from_rgb(R, G, B, A);
    switch (color) {
      case 0:
        fprintf(stderr, "Detected color: Red\n");
        break;
      case 1:
        fprintf(stderr, "Detected color: Yellow\n");
        break;
      case 2:
        fprintf(stderr, "Detected color: Green\n");
        break;
      case 3:
        fprintf(stderr, "Detected color: Blue\n");
        break;
      case 4:
        fprintf(stderr, "Detected color: Black\n");
        break;
      case 5:
        fprintf(stderr, "Detected color: White\n");
        break;
      default:
        fprintf(stderr, "Detected color: Other\n");
        break;
    }
  } else {
    fprintf(stderr, "Failed to read NXT color sensor (RGB raw).\n");
  }

  // Test reading gyro sensor and turning right 90 degrees
  fprintf(stderr, "Testing gyro sensor for 90-degree turn...\n");
  int angle = 0, rate = 0;

  // Reset gyro sensor to zero
  if (BT_read_gyro(PORT_2, 1, &angle, &rate) != 1) {
    fprintf(stderr, "Failed to reset gyro sensor.\n");
  } else {
    // Start turning right
    BT_turn(MOTOR_A, 50, MOTOR_C, -50);  // Turn right

    // Monitor the angle until it reaches 90 degrees
    while (angle < 90) {
      if (BT_read_gyro(PORT_2, 0, &angle, &rate) != 1) {
        fprintf(stderr, "Failed to read gyro sensor.\n");
        break;
      }
      fprintf(stderr, "Current angle: %d\n", angle);
    }

    // Stop the motors
    BT_motor_port_stop(MOTOR_A | MOTOR_C, 1);  // Stop with active brake
  }

  // Test color calibration
  //  R,G,B,A;
  // BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
  // color_calibration_rgb();

  BT_close();
  fprintf(stderr, "Done!\n");
}

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

void color_calibration_rgb(){
  // provide middling values first for calibration
  int R, G, B, A;
  int max_light = 0;  // detect white
  int min_light = 700;  // detect black
  int max_r = 0, max_g = 0, max_b = 0; // detect red, green, blue
  int max_y[3] = {128,128,0}; // detect yellow. Yellow is when red ~= green
  int err = 10; // error tolerance
  double color_probability[7] = {1,1,1,1,1,1,1}; // probability for correct prediction of every color

  // color sensing
  for (int i = 0; i < 10; i++) {
    BT_drive(MOTOR_A, MOTOR_C, 12, 10); // Drive forward
    sleep(0.5);  // Drive for 0.5 seconds
    BT_motor_port_stop(MOTOR_A | MOTOR_C, 1); // Stop with active brake
    BT_read_colour_RGBraw_NXT(PORT_3, &R, &G, &B, &A);
    printf("Reading %d: R=%d, G=%d, B=%d, A=%d\n", i, R, G, B, A);

    printf("Max Light=%d, Min Light=%d, Max R=%d, Max G=%d, Max B=%d, Max Y0=%d, Max Y1=%d, Max Y2=%d\n", max_light, min_light, max_r, max_g, max_b, max_y[0], max_y[1], max_y[2]);
    A *= 1.0; // adjust for ambient light
    G = (G + A)*1.3;
    B = (B + A)*1.3;
    R = (R + A)*1.3;

    int brightness = R + G + B;
    
    // calibrate colors 
    if (brightness > max_light) { // white. white and black first makes sure that color > max_color isn't because its white so the value is high 
      max_light = brightness;
    }
    else if (brightness < min_light) { // black
      min_light = brightness;
    }
    else if (abs(R - G) < 20) {  // yellow. yellow goes before rgb so that r and g aren't big cuz yellow
      if (R + G > max_y[0] + max_y[1]) {
        max_y[0] = R;
        max_y[1] = G;
        max_y[2] = B;
      }
    }
    else if (R > max_r) {  // red
      max_r = R;
    }
    else if (G > max_g) {  // green
      max_g = G;
    }
    else if (B > max_b) {  // blue
      max_b = B;
    }

    int color = -1;
    // detect color
    if (brightness < min_light + err) {
      color = 4;  // Black
    } else if (brightness > max_light - err) {
      color = 5;  // White
    } else if (R > max_y[0] - err && G > max_y[1] - err && B < max_y[2] + err) {
      color = 1;  // Yellow
    } else if (R > max_r - err && G < max_g - err && B < max_b - err) {
      color = 0;  // Red
    } else if (G > max_g - err && R < max_r - err && B < max_b - err) {
      color = 2;  // Green
    } else if (B > max_b - err && R < max_r - err && G < max_g - err) {
      color = 3;  // Blue
    } else {
      color = 6;  // Other
    }
    printf("R=%d, G=%d, B=%d, Color=%d\n", R, G, B, color);

    if (color == 0){ // don't get it get past the border
      BT_turn(MOTOR_A, -50, MOTOR_C, 50); // Turn left
    }
  }

  // color probability? 

  return; 
}