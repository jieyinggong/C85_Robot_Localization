#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include "calibration.h"
#include "EV3_RobotControl/btcomm.h"
#include "const.h"

void color_calibration()
{
    // possible changes:
    // use gaussian for color hue instead of just black and white
    // instead of using gaussian, get rid of outliers storing all the values, sorting, then stripping the higher and lower 5% of values

    // open file for writing 
    FILE *fp = fopen("color_calibration.txt", "w");
    if (fp == NULL) {
        fprintf(stderr, "Failed to open file for color calibration.\n");
        return;
    }

    HSVRange ranges[COLOR_COUNT];

    for (int i=0; i<COLOR_COUNT; i++){    // Color order: BLACK, BLUE, GREEN, YELLOW, RED, WHITE
        double H_min = 1e9, S_min = 1e9, V_min = 1e9;
        double H_max = -1e9, S_max = -1e9, V_max = -1e9;
        double H_sum = 0.0, S_sum = 0.0, V_sum = 0.0;
        double H_avg = 0.0, S_avg = 0.0, V_avg = 0.0;
        int valid_count = 0;

        for (int k=0; k<COLOR_SAMPLE_COUNT; k++){    // get 6 different color samples for each color
            // prompt user to place color under sensor
            print_get_color_calibration(i);

            for (int j=0; j<COLOR_READ_COUNT; j++){
                int R, G, B, A;
                if (BT_read_colour_RGBraw_NXT(PORT_1, &R, &G, &B, &A) != 1)
                    continue;

                valid_count++;
                double H, S, V;
                rgba_to_hsv(R, G, B, A, &H, &S, &V);

                if (i == C_RED){    // red case, can wrap around
                    if (H < 180){
                        H += 360.0;
                    }
                }

                if (H >= 0) { //  check valid hue
                    if (H < H_min) H_min = H;
                    if (H > H_max) H_max = H;
                    H_sum += H;
                }
                if (S < S_min) S_min = S;
                if (S > S_max) S_max = S;
                S_sum += S;
                if (V < V_min) V_min = V;
                if (V > V_max) V_max = V;
                V_sum += V;

                // usleep(100000); // 100 ms between samples
            }
        }

        // calculate average
        H_avg = H_sum / (valid_count);
        S_avg = S_sum / (valid_count);
        V_avg = V_sum / (valid_count);

        if (i == C_RED){ 
            if (H_max >= 360.0) H_max -= 360.0;
            if (H_avg >= 360.0) H_avg -= 360.0;
            if (H_min >= 360.0) H_min -= 360.0;
        }

        ranges[i] = (HSVRange){H_min, H_max, H_avg, S_min, S_max, S_avg, V_min, V_max, V_avg};

        // print non adjusted values
        printf("Color %d max HSV = (%.2f, %.2f, %.2f)\n", i+1, H_max, S_max, V_max);
        printf("Color %d min HSV = (%.2f, %.2f, %.2f)\n", i+1, H_min, S_min, V_min);
        printf("Color %d average HSV = (%.2f, %.2f, %.2f)\n", i+1, H_avg, S_avg, V_avg);
    }

    adjust_hue_overlaps(ranges); // adjust overlaps starting from RED
    adjust_black_white_thresholds(ranges); // readjust black and white V thresholds

    
    for (int i = 0; i < COLOR_COUNT; i++) {
        // print to file
        fprintf(fp, "%d: H[%.1f, %.1f, %.1f], S[%.2f, %.2f, %.2f], V[%.2f, %.2f, %.2f]\n",  
                i+1, ranges[i].H_min, ranges[i].H_max, ranges[i].H_avg,
                ranges[i].S_min, ranges[i].S_max, ranges[i].S_avg, ranges[i].V_min, ranges[i].V_max, ranges[i].V_avg);

        // print to console
        printf("%d: H[%.1f, %.1f, %.1f], S[%.2f, %.2f, %.2f], V[%.2f, %.2f, %.2f]\n",
                i+1, ranges[i].H_min, ranges[i].H_max, ranges[i].H_avg,
                ranges[i].S_min, ranges[i].S_max, ranges[i].S_avg, ranges[i].V_min, ranges[i].V_max, ranges[i].V_avg);
    }

    // close i/o 
    fclose(fp);
}


void adjust_hue_overlaps(HSVRange *ranges) {
    // adjust hue overlaps for the colors (not black and white cuz we don't care about its hue)
    // if ranges overlap, set the boundary to the midpoint of the overlap minus/plus delta

    const double delta = 1.0;   // can adjust
    for (int i = 1; i < COLOR_COUNT - 2; i++) {
        for (int j = i + 1; j < COLOR_COUNT - 1; j++) {
            if (ranges[i].H_max > ranges[j].H_min && ranges[i].H_min < ranges[j].H_max) {
                double mid = (ranges[i].H_max + ranges[j].H_min) / 2.0;
                ranges[i].H_max = mid - delta;
                ranges[j].H_min = mid + delta;

                // clamp to [0,360)
                if (ranges[i].H_max > 360.0) ranges[i].H_max -= 360.0;
                if (ranges[j].H_min < 0.0)   ranges[j].H_min += 360.0;

                if (ranges[i].H_max == ranges[i].H_min) {
                    ranges[i].H_max = ranges[i].H_max + 1;
                    ranges[i].H_min = ranges[i].H_min - 1; 
                }
            }
        }
    }
}


void adjust_black_white_thresholds(HSVRange *ranges) {
    // readjust black and white V thresholds based on gaussian assumption
    // and we compute standard deviation from V_min/V_max as an approximation

    // possible changes: adjust sigma and confidence level

    for (int i = 0; i <= 1; i++) { // 0 = black, 1 = white
        double mean = ranges[i].V_avg;

        // approximate standard deviation from min/max (assuming ~4 sigma range) -- adjustable
        double sigma = (ranges[i].V_max - ranges[i].V_min) / 4.0;

        // 95% confidence threshold (~2 sigma) -- can adjust from 2.0 to other value to control tightness of black and white bound
        if (i == 0) { // black: max V threshold = mean + 2*sigma
            ranges[i].V_max = mean + 2.0 * sigma;
            if (ranges[i].V_max > 1.0) ranges[i].V_max = 1.0;
        } else { // white: min V threshold = mean - 2*sigma
            ranges[i].V_min = mean - 2.0 * sigma;
            if (ranges[i].V_min < 0.0) ranges[i].V_min = 0.0;
        }

        // printf("Adjusted %s V threshold: min=%.3f, max=%.3f\n",
        //       (i==0) ? "BLACK" : "WHITE", ranges[i].V_min, ranges[i].V_max);
    }
}


void print_get_color_calibration(int i)
{
    // for command line ui

    const char *names[] = {"BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE"};
    printf("Place %s under the sensor and press Enter...\n", names[i]);
    getchar();
}

// Convert RGBA -> HSV (H in [0,360), S,V in [0,1])
// R,G,B,A are raw sensor ints. H,S,V are outputs.
// If chroma ~ 0 (gray), H is set to -1.0.
void rgba_to_hsv(int R, int G, int B, int A, double *H, double *S, double *V)
{
    // recheck this to make sure its correct
    // 1) Ambient correction + clamp to [0,1] using auto scale
    double Rc = (double)R + (double)A;
    double Gc = (double)G + (double)A;
    double Bc = (double)B + (double)A;

    double SCALE = 255.0;  

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
    
    return; 
}

void read_color_calibration(HSVRange *ranges)
{
    FILE *fp = fopen("color_calibration.txt", "r");
    if (fp == NULL) {
        fprintf(stderr, "Failed to open file for reading color calibration.\n");
        return;
    }

    char line[200];
    int idx = 0;
    while (fgets(line, sizeof(line), fp)) {
        double h_min, h_max, s_min, s_max, v_min, v_max;
        sscanf(line, "%*d: H[%lf, %lf, %*f], S[%lf, %lf, %*f], V[%lf, %lf, %*f]",
       &h_min, &h_max, &s_min, &s_max, &v_min, &v_max);

        ranges[idx].H_min = h_min;
        ranges[idx].H_max = h_max;
        ranges[idx].S_min = s_min;
        ranges[idx].S_max = s_max;
        ranges[idx].V_min = v_min;
        ranges[idx].V_max = v_max;

        idx++;
    }

    fclose(fp);
}


int classify_color_hsv(int R, int G, int B, int A)
{
    double H, S, V;
    rgba_to_hsv(R, G, B, A, &H, &S, &V);

    // check black and white first
    //printf("H: %.2f, S: %.2f, V: %.2f, V_max: %.2f, S_max: %.2f, V_min: %.2f, S_max: %.2f\n", H, S, V, ranges[C_BLACK-1].V_max, ranges[C_BLACK-1].S_max, ranges[C_WHITE-1].V_min, ranges[C_WHITE-1].S_max);
    if (V <= ranges[C_BLACK-1].V_max && S <= ranges[C_BLACK-1].S_max) {
        return C_BLACK; // black
    }
    if (V >= ranges[C_WHITE-1].V_min && S <= ranges[C_WHITE-1].S_max) {
        return C_WHITE; // white
    }
    // not black or white, check other colors
    // printf("H: %.2f, S: %.2f, V: %.2f, V_min: %.2f, V_max: %.2f\n", H, S, V, ranges[2].V_min, ranges[2].V_max);
    if (ranges[C_RED-1].H_min <= ranges[C_RED-1].H_max) {
        // Normal (non-wrapping) range
        if (H >= ranges[C_RED-1].H_min && H <= ranges[C_RED-1].H_max)
            return C_RED;
    } else {
        // Wrapping range (e.g. 300 to 60)
        if (H >= ranges[C_RED-1].H_min || H <= ranges[C_RED-1].H_max)
            return C_RED;
    }

    for (int i = 1; i < 4; i++) {
        if (H >= ranges[i].H_min && H <= ranges[i].H_max) { // hue matters more than saturation in color detection so you could take it out ig
            return i+1; // return color index
        }
    }
    return C_UNKNOWN;
}

void color_probability(){
    // possible changes: 
    // increase sample count per color. Even when writing the real code, sample a color multiple times to determine color
    // take more samples per scan (add another inner loop   )

    FILE *fp = fopen("color_probability.txt", "w");
    if (fp == NULL) {
        fprintf(stderr, "Failed to open file for color probability.\n");
        return;
    }
    const char *names[] = {"BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE"};

    for (int i = 0; i < 6; i++) {
        int success_count = 0;
        int total_count = COLOR_SAMPLE_COUNT; 
        
        for (int j = 0; j < COLOR_SAMPLE_COUNT; j++) {
            // press enter when you are ready to measure
            printf("Place %s under the sensor and press Enter to measure probability...\n", names[i]);
            getchar();

            int R, G, B, A;
            if (BT_read_colour_RGBraw_NXT(PORT_1, &R, &G, &B, &A) != 1)
                continue;
            int color = classify_color_hsv(R, G, B, A);
            printf("Measured color index: %d\n", color);

            // if classified color matches expected color success count + 1
            if (color == i+1) {
                success_count++;
            }
        }

        double probability = (double)success_count / (total_count);
        printf("Color %d probability: %.2f\n", i+1, probability);
        fprintf(fp, "Color %d probability: %.2f\n", i+1, probability);
    }

    fclose(fp);
}

void read_color_probability(ColorProbability *color_probabilities)
{
    FILE *fp = fopen("color_probability.txt", "r");
    if (fp == NULL) {
        fprintf(stderr, "Failed to open file for reading color probability.\n");
        return;
    }

    char line[200];
    int i = 0;
    while (fgets(line, sizeof(line), fp)) {
        sscanf(line, "Color %*d probability: %lf", &color_probabilities[i].probability);
        printf("Read color probability %d: %.2f\n", i+1, color_probabilities[i].probability);
        i++;
    }

    fclose(fp);
}

// int main(int argc, char *argv[]) {
//     if (argc < 2) {
//         printf("Usage: %s [0] [1]\n", argv[0]);
//         return 1;
//     }
// 
//     // connect to robot 
//     // char test_msg[8] = {0x06, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x01};
//     char reply[1024];
// 
//     memset(&reply[0], 0, 1024);
// 
//     #ifndef HEXKEY
//     #define HEXKEY "00:16:53:55:D9:FC"  // <--- SET UP YOUR EV3's HEX ID here
//     #endif
// 
//     BT_open(HEXKEY);
// 
//     // name must not contain spaces or special characters
//     // max name length is 12 characters
//     BT_setEV3name("R2D2");
// 
//     for (int i = 1; i < argc; i++) {
//         int arg = atoi(argv[i]);
// 
//         if (arg == 0) {
//             color_calibration();
//         } else if (arg == 1) {
//             read_color_calibration(ranges);
//             color_probability();
//         } else {
//             printf("Unknown argument: %s\n", argv[i]);
//         }
//     }
//     BT_close();
//     return 0;
// }