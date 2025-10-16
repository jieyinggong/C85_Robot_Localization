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
#include "street.h"
#include "color.h"
#include "intersection.h"

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

  int TONE_GREEN[50][3] = {
  {330,150,30},  // E4
  {392,150,40},  // G4
  {523,220,50},  // C5 (land)
  {-1,-1,-1}     // terminator
  };

  int TONE_BLUE[50][3] = {
  {523,130,30},  // C5
  {392,130,30},  // G4
  {330,200,30},  // E4
  {-1,-1,-1}
  };

  int TONE_WHITE[50][3] = {
  {523,90,18},   // C5
  {523,90,18},   // C5 (repeat)
  {523,90,18},   // C5 (repeat)
  {-1,-1,-1}
  };

  int TONE_INTERSECTION[50][3] = {
  {880,100,40},   // A5
  {440,100,40},   // A4
  {880,100,45},   // A5 again (echo)
  {660,250,50},   // E5 (hold)
  {-1,-1,-1}
  };

  int TONE_LOCALIZATION_DONE[50][3] = {
  {392,180,45},   // G4
  {523,180,50},   // C5
  {659,180,55},   // E5
  {784,220,60},   // G5 (octave)
  {1047,400,63},  // C6 (bright finale)
  {-1,-1,-1}
  };

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

  // 1. starting sound
  BT_play_tone_sequence(tone_data);
  sleep(1);

  // 2. find street (assume robot is on random place on map)
  int success = find_street();
  if (success) {
    fprintf(stderr, "Street found, now correcting...\n");
  }

  // correct to black line
  recorrect_to_black();

  // 3. drive along street until intersection
  success = drive_along_street();
  if (success) {
    fprintf(stderr, "Reached intersection!\n");
    BT_play_tone_sequence(TONE_INTERSECTION);
    sleep(1);
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
    sleep(2);
    // play tone for tr
    if (tr == 2) BT_play_tone_sequence(TONE_GREEN);
    else if (tr == 3) BT_play_tone_sequence(TONE_BLUE);
    else if (tr == 5) BT_play_tone_sequence(TONE_WHITE);
    else BT_play_tone_sequence(tone_data);
    sleep(2);
    // play tone for br
    if (br == 2) BT_play_tone_sequence(TONE_GREEN);
    else if (br == 3) BT_play_tone_sequence(TONE_BLUE); 
    else if (br == 5) BT_play_tone_sequence(TONE_WHITE);
    else BT_play_tone_sequence(tone_data);
    sleep(2);
    // play tone for bl
    if (bl == 2) BT_play_tone_sequence(TONE_GREEN);
    else if (bl == 3) BT_play_tone_sequence(TONE_BLUE);
    else if (bl == 5) BT_play_tone_sequence(TONE_WHITE);
    else BT_play_tone_sequence(tone_data);
    sleep(2);
  }

  // 5. turn right at intersection

  // 6. keep going... (drive along street until next intersection)
  success = drive_along_street();
  if (success) {
    fprintf(stderr, "Reached intersection again!\n");
    BT_play_tone_sequence(TONE_INTERSECTION);
    sleep(1);
  }

  // 7. turn left at intersection

  // 7. done
  BT_close();
  fprintf(stderr, "Done!\n");
}
