/*

  CSC C85 - Embedded Systems - Project # 1 - EV3 Robot Localization
  
 This file provides the implementation of all the functionality required for the EV3
 robot localization project. Please read through this file carefully, and note the
 sections where you must implement functionality for your bot. 
 
 You are allowed to change *any part of this file*, not only the sections marked
 ** TO DO **. You are also allowed to add functions as needed (which must also
 be added to the header file). However, *you must clearly document* where you 
 made changes so your work can be properly evaluated by the TA.

 NOTES on your implementation:

 * It should be free of unreasonable compiler warnings - if you choose to ignore
   a compiler warning, you must have a good reason for doing so and be ready to
   defend your rationale with your TA.
 * It must be free of memory management errors and memory leaks - you are expected
   to develop high wuality, clean code. Test your code extensively with valgrind,
   and make sure its memory management is clean.
 
 In a nutshell, the starter code provides:
 
 * Reading a map from an input image (in .ppm format). The map is bordered with red, 
   must have black streets with yellow intersections, and buildings must be either
   blue, green, or be left white (no building).
   
 * Setting up an array with map information which contains, for each intersection,
   the colours of the buildings around it in ** CLOCKWISE ** order from the top-left.
   
 * Initialization of the EV3 robot (opening a socket and setting up the communication
   between your laptop and your bot)
   
 What you must implement:
 
 * All aspects of robot control:
   - Finding and then following a street
   - Recognizing intersections
   - Scanning building colours around intersections
   - Detecting the map boundary and turning around or going back - the robot must not
     wander outside the map (though of course it's possible parts of the robot will
     leave the map while turning at the boundary)

 * The histogram-based localization algorithm that the robot will use to determine its
   location in the map - this is as discussed in lecture.

 * Basic robot exploration strategy so the robot can scan different intersections in
   a sequence that allows it to achieve reliable localization
   
 * Basic path planning - once the robot has found its location, it must drive toward a 
   user-specified position somewhere in the map.

 --- OPTIONALLY but strongly recommended ---
 
  The starter code provides a skeleton for implementing a sensor calibration routine,
 it is called when the code receives -1  -1 as target coordinates. The goal of this
 function should be to gather informatin about what the sensor reads for different
 colours under the particular map/room illumination/battery level conditions you are
 working on - it's entirely up to you how you want to do this, but note that careful
 calibration would make your work much easier, by allowing your robot to more
 robustly (and with fewer mistakes) interpret the sensor data into colours. 
 
   --> The code will exit after calibration without running localization (no target!)
       SO - your calibration code must *save* the calibration information into a
            file, and you have to add code to main() to read and use this
            calibration data yourselves.
   
 What you need to understand thoroughly in order to complete this project:
 
 * The histogram localization method as discussed in lecture. The general steps of
   probabilistic robot localization.

 * Sensors and signal management - your colour readings will be noisy and unreliable,
   you have to handle this smartly
   
 * Robot control with feedback - your robot does not perform exact motions, you can
   assume there will be error and drift, your code has to handle this.
   
 * The robot control API you will use to get your robot to move, and to acquire 
   sensor data. Please see the API directory and read through the header files and
   attached documentation
   
 Starter code:
 F. Estrada, 2018 - for CSC C85 
 
*/

#include "EV3_Localization.h"
#include "calibration.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>
// Added headers to avoid implicit declaration warnings for standard functions
#include <stdio.h>
#include <string.h>
#include "EV3_RobotControl/btcomm.h"
#include "const.h"

// #define DIR_UP    0
// #define DIR_RIGHT 1
// #define DIR_DOWN  2
// #define DIR_LEFT  3


// #define C_BLACK  0
// #define C_WHITE  1
// #define C_RED    2
// #define C_YELLOW 3
// #define C_GREEN  4
// #define C_BLUE   5

// #define COLOR_PORT PORT_1
// #define GYRO_PORT PORT_2
// #define LEFT_MOTOR MOTOR_A
// #define RIGHT_MOTOR MOTOR_D

#define MIN_SCANS_TO_CONFIRM 3
#define CERTAINTY_THRESHOLD  0.80
#define MASS_EPS             1e-9
#define FLOOR_EPS            1e-3
#define NAV_CERTAINTY_THRESHOLD  0.65   // 导航阶段的“仍然定位”阈值，可以稍低于定位完成阈值
#define NAV_SCAN_EVERY_STEPS     1      // 每经过几个路口做一次扫描；设为1表示每个路口都扫描

int map[400][4];            // This holds the representation of the map, up to 20x20
                            // intersections, raster ordered, 4 building colours per
                            // intersection.
int sx, sy;                 // Size of the map (number of intersections along x and y)
double beliefs[400][4];     // Beliefs for each location and motion direction

HSVRange ranges[COLOR_COUNT]; // global variable to hold calibration data
ColorProbability color_probabilities[COLOR_COUNT]; // global variable to hold color probabilities

// ColorProbability color_probabilities[COLOR_COUNT];  // holds color probabilities read from file
// HSVRange ranges[COLOR_COUNT]; // holds color calibration read from file

// ---- Local tiny helpers in this translation unit ----
static inline int idx_to_x(int idx){ return idx % sx; }
static inline int idx_to_y(int idx){ return idx / sx; }
static inline int xy_to_idx(int x,int y){ return x + y*sx; }

// sum of all beliefs (for normalization)
static double sum_all_beliefs(void){
  double s = 0.0;
  for(int i=0;i<sx*sy;i++)
    for(int d=0; d<4; d++) s += beliefs[i][d];
  return s;
}

static void normalize_beliefs(void){
  double s = sum_all_beliefs();
  if (s < MASS_EPS) return; // avoid divide-by-zero; if this happens, your update produced degenerate mass
  double inv = 1.0 / s;
  for(int i=0;i<sx*sy;i++)
    for(int d=0; d<4; d++) beliefs[i][d] *= inv;
}

static void current_argmax(int* bestIdx, int* bestDir, double* bestVal){
  *bestIdx = 0; *bestDir = 0; *bestVal = -1.0;
  for(int i=0;i<sx*sy;i++){
    for(int d=0; d<4; d++){
      if (beliefs[i][d] > *bestVal){
        *bestVal = beliefs[i][d];
        *bestIdx = i;
        *bestDir = d;
      }
    }
  }
}

// compute which absolute move directions are *globally feasible* given current belief mass:
// a direction is feasible if moving that way doesn't exit the grid for a non-trivial portion of belief.
static int build_feasible_directions(int outDirs[4]){
  double mass[4] = {0,0,0,0};
  for(int i=0;i<sx*sy;i++){
    // total mass at intersection i across all headings
    double mi = beliefs[i][0]+beliefs[i][1]+beliefs[i][2]+beliefs[i][3];
    if (mi < MASS_EPS) continue;
    int x = idx_to_x(i), y = idx_to_y(i);
    if (y > 0)          mass[DIR_UP]    += mi; // can move up
    if (x < sx-1)       mass[DIR_RIGHT] += mi; // can move right
    if (y < sy-1)       mass[DIR_DOWN]  += mi; // can move down
    if (x > 0)          mass[DIR_LEFT]  += mi; // can move left
  }
  int n=0;
  for(int d=0; d<4; d++){
    if (mass[d] > MASS_EPS) outDirs[n++] = d;
  }
  // Fallback: if somehow empty, allow all 4 (should rarely happen)
  if (n == 0){
    outDirs[0]=DIR_UP; outDirs[1]=DIR_RIGHT; outDirs[2]=DIR_DOWN; outDirs[3]=DIR_LEFT;
    n = 4;
  }
  return n;
}

// execute the chosen move: turn to absolute dir, leave the intersection, and follow street to next one
static void execute_move(int absDir){
  turn_at_intersection(absDir);  // your helper aligns & points to desired absolute direction
  sleep(3);
  drive_along_street();          // cross intersection + follow the black line to the next intersection
}

// 将地图四角(TL,TR,BR,BL)按朝向 dir 顺时针旋转 dir 步，得到“机器人视角下应看到的顺序”
static inline void expected_corners_for_dir(int idx, int dir, int out4[4]){
  // map[idx] = [TL, TR, BR, BL] (clockwise)
  // 面向UP:   期望 = [TL, TR, BR, BL]
  // 面向RIGHT:期望 = [TR, BR, BL, TL]   (顺时针移动1位)
  // 面向DOWN: 期望 = [BR, BL, TL, TR]   (顺时针移动2位)
  // 面向LEFT: 期望 = [BL, TL, TR, BR]   (顺时针移动3位)
  int base[4] = { map[idx][0], map[idx][1], map[idx][2], map[idx][3] };
  for(int k=0;k<4;k++){
    out4[k] = base[(k + dir) & 3]; // 等价于 (k+dir)%4
  }
}

// 从 color_probabilities[] 取命中率；若未加载或异常，给一个保守默认值
static inline double get_color_hit_prob(int c){
  // 仅对 WHITE/GREEN/BLUE 用到（交叉口），其余颜色用不到也返回个默认
  if (c < 0 || c > 5) return 0.85;
  double p = color_probabilities[c].probability;
  if (!(p > 0.0 && p < 1.0)) {
    // 合理默认：传感器分类在“期待为该色”时的命中率
    // 你也可以给不同颜色不同默认，比如白 0.92、绿 0.88、蓝 0.90
    p = 0.90;
  }
  // 稍作夹取，避免 0 或 1 导致数值问题
  if (p < 0.55) p = 0.55;
  if (p > 0.99) p = 0.99;
  return p;
}

static inline int is_allowed_corner_colour(int c){
  return (c == C_WHITE || c == C_GREEN || c == C_BLUE);
}

// --------------------------------- 似然模型（简单稳健版）---------------------------------
// 读数 readings: [tl, tr, br, bl]
// 期望 expect:    [tl, tr, br, bl]（已按朝向对齐）
// 返回 P(z|s)：逐角相乘（匹配给高权，不匹配给低权；允许“未知”时给中等权）
static inline double sensor_likelihood_4(const int readings[4], const int expect[4]){
  // 对未知读数的保守概率（可调）：既不过分惩罚，也不鼓励
  const double p_unknown = 0.50;

  double p_total = 1.0;

  for (int k = 0; k < 4; k++){
    const int e = expect[k];         // 期望颜色（一定应在 {W,G,B}）
    const int z = readings[k];       // 传感器读到的颜色（可能是 -1 未知）

    // 防御：若地图标注不在允许集，给个很小概率，避免把分布完全冲没
    if (!is_allowed_corner_colour(e)) {
      p_total *= 0.05;
      continue;
    }

    // 取“期望色 e 的命中率”作为 p_hit(e)
    const double p_hit_e = get_color_hit_prob(e);

    // 将 (1 - p_hit_e) 分给 “被误判为另外两种允许颜色”：
    // 这里采取对称分配（也可按你后续测得的非对称混淆率再细化）
    const double p_confuse_to_other_allowed = (1.0 - p_hit_e) / 2.0;

    // 对“读到不在 {W,G,B} 的颜色”（黑/黄/红 或 未知），给更小的“离群”概率：
    // 让它是“把 (1-p_hit_e) 中的一个很小比例分出去”，以体现少见但可能的错判
    const double p_outlier = fmax(0.01, 0.15 * (1.0 - p_hit_e));  // 可调：0.15 表示(1-p_hit)中拿15%给离群

    double pk;  // 单角似然

    if (z < 0) {
      // 未识别：保守值
      pk = p_unknown;
    } else if (z == e){
      // 命中期望色
      pk = p_hit_e;
    } else if (is_allowed_corner_colour(z)){
      // 被误判为另一个允许色（W/G/B 之间混淆）
      pk = p_confuse_to_other_allowed;
    } else {
      // 被误判为不可能出现在角楼的颜色（黑/红/黄等）
      pk = p_outlier;
    }

    // 夹取，避免出现 0
    if (pk < 1e-6) pk = 1e-6;

    p_total *= pk;
  }

  return p_total;
}

// ==============================================================================================
//                                  测量更新：updateBelief
// ==============================================================================================
//
// 参数：
//   moveDir   - 上一次或下一次运动方向（如果你的测量模型需要用到“将要去的方向”，可以使用；
//               若不用，可忽略）
//   readings  - 当前路口观测到的四角颜色，顺序 [tl, tr, br, bl]
//
// 逻辑：
//   对每个状态 s=(idx,dir)
//     1) 取该 idx 的地图四角，并按 dir 旋转对齐到“机器人视角”
//     2) 计算 P(z|s) 并做 Bel[s] *= P(z|s)
//   然后全局归一化
//
void updateBelief(int moveDir, int readings[4])
{
  (void)moveDir; // 当前实现未使用，如需将来引入“朝向偏好”可利用该参数

  // 一次性把所有状态都按观测更新
  for(int idx=0; idx < sx*sy; idx++){
    for(int dir=0; dir<4; dir++){
      int expect[4];
      expected_corners_for_dir(idx, dir, expect);
      double pz = sensor_likelihood_4(readings, expect);
      beliefs[idx][dir] *= pz;
    }
  }

  // 归一化，保持概率分布合法
  normalize_beliefs();
}

void actionModel(int moveDir)
{
  static double newBeliefs[400][4];
  static unsigned char hasIncoming[400][4];

  // 清零累加数组与“是否有来源进入”的标记
  memset(newBeliefs, 0, sizeof(newBeliefs));
  memset(hasIncoming, 0, sizeof(hasIncoming));

  // 把每个状态沿 moveDir 平移一格；越界的来源直接丢弃（不加回）
  for (int idx = 0; idx < sx * sy; idx++) {
    int x = idx_to_x(idx);
    int y = idx_to_y(idx);

    for (int dir = 0; dir < 4; dir++) {
      double mass = beliefs[idx][dir];
      if (mass <= 0.0) continue; // 无质量，跳过

      int nx = x, ny = y;
      if      (moveDir == DIR_UP)    ny--;
      else if (moveDir == DIR_RIGHT) nx++;
      else if (moveDir == DIR_DOWN)  ny++;
      else if (moveDir == DIR_LEFT)  nx--;

      // invalid
      if (nx < 0 || nx >= sx || ny < 0 || ny >= sy) {
        continue;
      }

      int j = xy_to_idx(nx, ny);
      newBeliefs[j][dir] += mass;         // 方向不变，只平移位置
      hasIncoming[j][dir] = 1;            // 标记：这个状态有来源质量进入
    }
  }

  // 给所有“没有任何来源进入”的状态加一个极小地板值
  for (int j = 0; j < sx * sy; j++) {
    for (int d = 0; d < 4; d++) {
      if (!hasIncoming[j][d]) {
        newBeliefs[j][d] += FLOOR_EPS;
      }
    }
  }

  // 覆盖并归一化
  memcpy(beliefs, newBeliefs, sizeof(newBeliefs));
  normalize_beliefs();
}

int main(int argc, char *argv[])
{
 char mapname[1024];
 int dest_x, dest_y, rx, ry;
 unsigned char *map_image;
 
 memset(&map[0][0],0,400*4*sizeof(int));
 sx=0;
 sy=0;
 
 if (argc<4)
 {
  fprintf(stderr,"Usage: EV3_Localization map_name dest_x dest_y\n");
  fprintf(stderr,"    map_name - should correspond to a properly formatted .ppm map image\n");
  fprintf(stderr,"    dest_x, dest_y - target location for the bot within the map, -1 -1 calls calibration routine\n");
  exit(1);
 }
 strcpy(&mapname[0],argv[1]);
 dest_x=atoi(argv[2]);
 dest_y=atoi(argv[3]);

  // Open a socket to the EV3 for remote controlling the bot.
 if (BT_open(HEXKEY)!=0)
 {
  fprintf(stderr,"Unable to open comm socket to the EV3, make sure the EV3 kit is powered on, and that the\n");
  fprintf(stderr," hex key for the EV3 matches the one in EV3_Localization.h\n");
  free(map_image);
  exit(1);
 }

 if (dest_x==-1&&dest_y==-1)
 {
  calibrate_sensor();
  exit(1);
 }

 /******************************************************************************************************************
  * OPTIONAL TO DO: If you added code for sensor calibration, add just below this comment block any code needed to
  *   read your calibration data for use in your localization code. Skip this if you are not using calibration
  * ****************************************************************************************************************/
  read_color_calibration(ranges);
  read_color_probability(color_probabilities);
  
 // Your code for reading any calibration information should not go below this line //
 
 map_image=readPPMimage(&mapname[0],&rx,&ry);
 if (map_image==NULL)
 {
  fprintf(stderr,"Unable to open specified map image\n");
  exit(1);
 }
 
 if (parse_map(map_image, rx, ry)==0)
 { 
  fprintf(stderr,"Unable to parse input image map. Make sure the image is properly formatted\n");
  free(map_image);
  exit(1);
 }

 if (dest_x<0||dest_x>=sx||dest_y<0||dest_y>=sy)
 {
  fprintf(stderr,"Destination location is outside of the map\n");
  free(map_image);
  exit(1);
 }

 // Initialize beliefs - uniform probability for each location and direction
 for (int j=0; j<sy; j++)
  for (int i=0; i<sx; i++)
  {
   beliefs[i+(j*sx)][0]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][1]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][2]=1.0/(double)(sx*sy*4);
   beliefs[i+(j*sx)][3]=1.0/(double)(sx*sy*4);
  }

 fprintf(stderr,"All set, ready to go!\n");
 
/*******************************************************************************************************************************
 *
 *  TO DO - Implement the main localization loop, this loop will have the robot explore the map, scanning intersections and
 *          updating beliefs in the beliefs array until a single location/direction is determined to be the correct one.
 * 
 *          The beliefs array contains one row per intersection (recall that the number of intersections in the map_image
 *          is given by sx, sy, and that the map[][] array contains the colour indices of buildings around each intersection.
 *          Indexing into the map[][] and beliefs[][] arrays is by raster order, so for an intersection at i,j (with 0<=i<=sx-1
 *          and 0<=j<=sy-1), index=i+(j*sx)
 *  
 *          In the beliefs[][] array, you need to keep track of 4 values per intersection, these correspond to the belief the
 *          robot is at that specific intersection, moving in one of the 4 possible directions as follows:
 * 
 *          beliefs[i][0] <---- belief the robot is at intersection with index i, facing UP
 *          beliefs[i][1] <---- belief the robot is at intersection with index i, facing RIGHT
 *          beliefs[i][2] <---- belief the robot is at intersection with index i, facing DOWN
 *          beliefs[i][3] <---- belief the robot is at intersection with index i, facing LEFT
 * 
 *          Initially, all of these beliefs have uniform, equal probability. Your robot must scan intersections and update
 *          belief values based on agreement between what the robot sensed, and the colours in the map. 
 * 
 *          You have two main tasks these are organized into two major functions:
 * 
 *          robot_localization()    <---- Runs the localization loop until the robot's location is found
 *          go_to_target()          <---- After localization is achieved, takes the bot to the specified map location
 * 
 *          The target location, read from the command line, is left in dest_x, dest_y
 * 
 *          Here in main(), you have to call these two functions as appropriate. But keep in mind that it is always possible
 *          that even if your bot managed to find its location, it can become lost again while driving to the target
 *          location, or it may be the initial localization was wrong and the robot ends up in an unexpected place - 
 *          a very solid implementation should give your robot the ability to determine it's lost and needs to 
 *          run localization again.
 *
 *******************************************************************************************************************************/  

 // HERE - write code to call robot_localization() and go_to_target() as needed, any additional logic required to get the
 //        robot to complete its task should be here.
 int tl, tr, br, bl;
 if (detect_intersection() == 0) {
   fprintf(stderr, "Not at an intersection to start with, exiting...\n");
   free(map_image);
   BT_close();
   exit(0);
 }

  free(map_image);
  BT_close();
 exit(0); 
 
 int robot_x = -1;
 int robot_y = -1;
 int direction = 0;
 robot_localization(&robot_x, &robot_y, &direction);
 fprintf(stderr, "Localization complete! Robot at (%d, %d) facing %d\n", robot_x, robot_y, direction);
 go_to_target(robot_x, robot_y, direction, dest_x,  dest_y);

 // Cleanup and exit - DO NOT WRITE ANY CODE BELOW THIS LINE
 BT_close();
 free(map_image);
 exit(0);
}

int find_street(void)   
{
  return 0;
}

// int drive_along_street(void)
// {
//  /*
//   * This function drives your bot along a street, making sure it stays on the street without straying to other pars of
//   * the map. It stops at an intersection.
//   * 
//   * You can implement this in many ways, including a controlled (PID for example), a neural network trained to track and
//   * follow streets, or a carefully coded process of scanning and moving. It's up to you, feel free to consult your TA
//   * or the course instructor for help carrying out your plan.
//   * 
//   * You can use the return value to indicate success or failure, or to inform the rest of your code of the state of your
//   * bot after calling this function.
//   */   

//   return(0);
// }

// int detect_intersection(void)
// {
//  /*
//   * This function attempts to detect if the bot is currently over an intersection. You can implement this in any way
//   * you like, but it should be reliable and robust.
//   * 
//   * The return value should be 1 if an intersection is detected, and 0 otherwise.
//   */   
//   // use this function: int BT_read_colour_RGBraw_NXT(char sensor_port, int *R, int *G, int *B, int *A);
//   return(0);
// }

int detect_intersection(void)
{
 /*
  * This function attempts to detect if the bot is currently over an intersection. You can implement this in any way
  * you like, but it should be reliable and robust.
  * 
  * The return value should be 1 if an intersection is detected, and 0 otherwise.
  */   
  // use this function: int BT_read_colour_RGBraw_NXT(char sensor_port, int *R, int *G, int *B, int *A);
  return(0);
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
 
//   /************************************************************************************************************************
//    *   TO DO  -   Complete this function
//    ***********************************************************************************************************************/
//   /************************************************************************************************************************
//    *   TO DO  -   Complete this function
//    ***********************************************************************************************************************/

//  // Return invalid colour values, and a zero to indicate failure (you will replace this with your code)
//  *(tl)=-1;
//  *(tr)=-1;
//  *(br)=-1;
//  *(bl)=-1;
//  return(0);
//  // Return invalid colour values, and a zero to indicate failure (you will replace this with your code)
//  *(tl)=-1;
//  *(tr)=-1;
//  *(br)=-1;
//  *(bl)=-1;
//  return(0);
 
// }
// }

int turn_at_intersection(int turn_direction)
{
 /*
  * This function is used to have the robot turn either left or right at an intersection (obviously your bot can not just
  * drive forward!). 
  * 
  * If turn_direction=0, turn right, else if turn_direction=1, turn left.
  * 
  * You're free to implement this in any way you like, but it should reliably leave your bot facing the correct direction
  * and on a street it can follow. 
  * 
  * You can use the return value to indicate success or failure, or to inform your code of the state of the bot
  */
 // if turn direction = 0 up, 1 right, 2 down, 3 left
 if (turn_direction == DIR_RIGHT){
    turn_right_90_degrees();
 }
 else if (turn_direction == DIR_DOWN){
    turn_back_180_degrees();
 }
 else if (turn_direction == DIR_LEFT){
    turn_left_90_degrees();
 }
  // do nothing, already facing up
  return(0);
}

int robot_localization(int *robot_x, int *robot_y, int *direction)
{
 /*  This function implements the main robot localization process. You have to write all code that will control the robot
  *  and get it to carry out the actions required to achieve localization.
  *
  *  Localization process:
  *
  *  - Find the street, and drive along the street toward an intersection
  *  - Scan the colours of buildings around the intersection
  *  - Update the beliefs in the beliefs[][] array according to the sensor measurements and the map data
  *  - Repeat the process until a single intersection/facing direction is distintly more likely than all the rest
  * 
  *  * We have provided headers for the following functions:
  * 
  *  find_street()
  *  drive_along_street()
  *  scan_intersection()
  *  turn_at_intersection()
  *  void updateBelief(int moveDir, int readings[4]); // sensor update (uses last/next moveDir)
  *  void actionModel(int moveDir);                   // motion update: shift beliefs deterministically one cell
  * 
  *  You *do not* have to use them, and can write your own to organize your robot's work as you like, they are
  *  provided as a suggestion.
  * 
  *  Note that *your bot must explore* the map to achieve reliable localization, this means your intersection
  *  scanning strategy should not rely exclusively on moving forward, but should include turning and exploring
  *  other streets than the one your bot was initially placed on.
  * 
  *  For each of the control functions, however, you will need to use the EV3 API, so be sure to become familiar with
  *  it.
  * 
  *  In terms of sensor management - the API allows you to read colours either as indexed values or RGB, it's up to
  *  you which one to use, and how to interpret the noisy, unreliable data you're likely to get from the sensor
  *  in order to update beliefs.
  * 
  *  HOWEVER: *** YOU must document clearly both in comments within this function, and in your report, how the
  *               sensor is used to read colour data, and how the beliefs are updated based on the sensor readings.
  * 
  *  DO NOT FORGET - Beliefs should always remain normalized to be a probability distribution, that means the
  *                  sum of beliefs over all intersections and facing directions must be 1 at all times.
  * 
  *  The function receives as input pointers to three integer values, these will be used to store the estimated
  *   robot's location and facing direction. The direction is specified as:
  *   0 - UP
  *   1 - RIGHT
  *   2 - BOTTOM
  *   3 - LEFT
  * 
  *  The function's return value is 1 if localization was successful, and 0 otherwise.
  */
 
  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/
  // If not already on street, acquire it first
  // *Minty - find street and drive along it to reach an intersection
  // find_street();
  // drive_along_street();
  // Minty*

  // Random seed for action selection
  srand((unsigned int)time(NULL));

  int scans = 0;
  int lastMoveDir = DIR_UP; // arbitrary init; only used by updateBelief if your design needs it
  
  // test: assume start at a intersection

  while (1)
  {
    // 1) Sense: scan 4 corners and do sensor (measurement) update
    // *Minty
    int tl,tr,br,bl;
    scan_intersection(&tl,&tr,&br,&bl);
    int z[4] = {tl,tr,br,bl};
    // Minty*

    updateBelief(lastMoveDir, z);   // Bayesian sensor update with your internal likelihood model
    scans++;

    // 2) Check convergence
    int bestIdx, bestDir;
    double bestVal;
    current_argmax(&bestIdx, &bestDir, &bestVal);

    if (scans >= MIN_SCANS_TO_CONFIRM && bestVal >= CERTAINTY_THRESHOLD){
      // Found our location
      *(robot_x) = idx_to_x(bestIdx);
      *(robot_y) = idx_to_y(bestIdx);
      *(direction) = bestDir;
      break;
    }

    // 3) Choose a random feasible absolute direction, then execute the move
    int dirs[4]; 
    int n = build_feasible_directions(dirs);
    int moveDir = dirs[rand() % n];

    // *Minty
    execute_move(moveDir);
    // Minty*

    // 4) Motion update (deterministic straight-line model provided by your helper)
    actionModel(moveDir);

    lastMoveDir = moveDir;
  }

 // Localization succeeded (we broke out of the loop when confident)
 return 1;
}

static int localization_still_ok(void){
  int bi, bd; double bv;
  current_argmax(&bi,&bd,&bv);
  return (bv >= NAV_CERTAINTY_THRESHOLD);
}

static int perform_step_and_check(int moveDir, int *pCurX, int *pCurY, int *pCurDir, int *pStepCounter)
{
  // Execute physical movement
  execute_move(moveDir);

  // Motion (mathematical) update
  actionModel(moveDir);

  (*pStepCounter)++;

  // Possibly scan at this intersection according to NAV_SCAN_EVERY_STEPS
  if ((*pStepCounter) % NAV_SCAN_EVERY_STEPS == 0){
    int tl,tr,br,bl;
    if (scan_intersection(&tl,&tr,&br,&bl)){
      int z[4] = {tl,tr,br,bl};
      updateBelief(moveDir, z);
    } else {
      // If scan failed, be conservative but continue; still check localization
      int dummy_tl=-1,dummy_tr=-1,dummy_br=-1,dummy_bl=-1;
      int z[4] = {dummy_tl,dummy_tr,dummy_br,dummy_bl};
      updateBelief(moveDir, z);
    }

    if (!localization_still_ok()){
      return 0; // LOST
    }
  }

  // Update current best estimate from beliefs
  {
    int bi, bd; double bv;
    current_argmax(&bi,&bd,&bv);
    *pCurX = idx_to_x(bi);
    *pCurY = idx_to_y(bi);
    *pCurDir = bd;
  }

  return 1;
}

int go_to_target(int robot_x, int robot_y, int direction, int target_x, int target_y)
{
 /*
  * This function is called once localization has been successful, it performs the actions required to take the robot
  * from its current location to the specified target location. 
  *
  * You have to write the code required to carry out this task - once again, you can use the function headers provided, or
  * write your own code to control the bot, but document your process carefully in the comments below so your TA can easily
  * understand how everything works.
  *
  * Your code should be able to determine if the robot has gotten lost (or if localization was incorrect), and your bot
  * should be able to recover.
  * 
  * Inputs - The robot's current location x,y (the intersection coordinates, not image pixel coordinates)
  *          The target's intersection location
  * 
  * Return values: 1 if successful (the bot reached its target destination), 0 otherwise
  */   

  /************************************************************************************************************************
   *   TO DO  -   Complete this function
   ***********************************************************************************************************************/
  // 进入街道（以防起始时没在黑线上）
  find_street();

  // 这里用简单的曼哈顿路径：先水平，再垂直（你可以换成A*或别的策略）
  int cur_x = robot_x;
  int cur_y = robot_y;
  int cur_dir = direction;

  int stepCounter = 0;

  // 1) 先走 X 方向
  while (cur_x != target_x){
    int moveDir = (target_x > cur_x) ? DIR_RIGHT : DIR_LEFT;
    if (!perform_step_and_check(moveDir, &cur_x, &cur_y, &cur_dir, &stepCounter)) return 0;  // LOST → 让 main 触发重定位
  }

  // 2) 再走 Y 方向
  while (cur_y != target_y){
    int moveDir = (target_y > cur_y) ? DIR_DOWN : DIR_UP;
    if (!perform_step_and_check(moveDir, &cur_x, &cur_y, &cur_dir, &stepCounter)) return 0;  // LOST → 让 main 触发重定位
  }
 
  // 到达目标格后再做一次确认扫描（可选，增强鲁棒性）
  {
    int tl,tr,br,bl; scan_intersection(&tl,&tr,&br,&bl);
    int z[4] = {tl,tr,br,bl};
    updateBelief(cur_dir, z);
    if (!localization_still_ok()){
      return 0; // 目标附近定位不稳，也让 main 走重定位流程
    }
  }

  return 1; // 成功到达
 }
 
 void calibrate_sensor(void)
{
 /*
  * This function is called when the program is started with -1  -1 for the target location. 
  *
  * You DO NOT NEED TO IMPLEMENT ANYTHING HERE - but it is strongly recommended as good calibration will make sensor
  * readings more reliable and will make your code more resistent to changes in illumination, map quality, or battery
  * level.
  * 
  * The principle is - Your code should allow you to sample the different colours in the map, and store representative
  * values that will help you figure out what colours the sensor is reading given the current conditions.
  * 
  * Inputs - None
  * Return values - None - your code has to save the calibration information to a file, for later use (see in main())
  * 
  * How to do this part is up to you, but feel free to talk with your TA and instructor about it!
  */   

  /************************************************************************************************************************
   *   OIPTIONAL TO DO  -   Complete this function
   ***********************************************************************************************************************/
  // color_calibration();
  // 
  // printf("Calibration complete, now measuring colour probabilities...\n");
  // getchar();

  read_color_calibration(ranges);
  color_probability(); 
  return;
}

int parse_map(unsigned char *map_img, int rx, int ry)
{
 /*
   This function takes an input image map array, and two integers that specify the image size.
   It attempts to parse this image into a representation of the map in the image. The size
   and resolution of the map image should not affect the parsing (i.e. you can make your own
   maps without worrying about the exact position of intersections, roads, buildings, etc.).

   However, this function requires:
   
   * White background for the image  [255 255 255]
   * Red borders around the map  [255 0 0]
   * Black roads  [0 0 0]
   * Yellow intersections  [255 255 0]
   * Buildings that are pure green [0 255 0], pure blue [0 0 255], or white [255 255 255]
   (any other colour values are ignored - so you can add markings if you like, those 
    will not affect parsing)

   The image must be a properly formated .ppm image, see readPPMimage below for details of
   the format. The GIMP image editor saves properly formatted .ppm images, as does the
   imagemagick image processing suite.
   
   The map representation is read into the map array, with each row in the array corrsponding
   to one intersection, in raster order, that is, for a map with k intersections along its width:
   
    (row index for the intersection)
    
    0     1     2    3 ......   k-1
    
    k    k+1   k+2  ........    
    
    Each row will then contain the colour values for buildings around the intersection 
    clockwise from top-left, that is
    
    
    top-left               top-right
            
            intersection
    
    bottom-left           bottom-right
    
    So, for the first intersection (at row 0 in the map array)
    map[0][0] <---- colour for the top-left building
    map[0][1] <---- colour for the top-right building
    map[0][2] <---- colour for the bottom-right building
    map[0][3] <---- colour for the bottom-left building
    
    Color values for map locations are defined as follows (this agrees with what the
    EV3 sensor returns in indexed-colour-reading mode):
    
    1 -  Black
    2 -  Blue
    3 -  Green
    4 -  Yellow
    5 -  Red
    6 -  White
    
    If you find a 0, that means you're trying to access an intersection that is not on the
    map! Also note that in practice, because of how the map is defined, you should find
    only Green, Blue, or White around a given intersection.
    
    The map size (the number of intersections along the horizontal and vertical directions) is
    updated and left in the global variables sx and sy.

    Feel free to create your own maps for testing (you'll have to print them to a reasonable
    size to use with your bot).
    
 */    
 
 int last3[3];
 int x,y;
 unsigned char R,G,B;
 int ix,iy;
 int bx,by,dx,dy,wx,wy;         // Intersection geometry parameters
 int tgl;
 int idx;
 
 ix=iy=0;       // Index to identify the current intersection
 
 // Determine the spacing and size of intersections in the map
 tgl=0;
 for (int i=0; i<rx; i++)
 {
  for (int j=0; j<ry; j++)
  {
   R=*(map_img+((i+(j*rx))*3));
   G=*(map_img+((i+(j*rx))*3)+1);
   B=*(map_img+((i+(j*rx))*3)+2);
   if (R==255&&G==255&&B==0)
   {
    // First intersection, top-left pixel. Scan right to find width and spacing
    bx=i;           // Anchor for intersection locations
    by=j;
    for (int k=i; k<rx; k++)        // Find width and horizontal distance to next intersection
    {
     R=*(map_img+((k+(by*rx))*3));
     G=*(map_img+((k+(by*rx))*3)+1);
     B=*(map_img+((k+(by*rx))*3)+2);
     if (tgl==0&&(R!=255||G!=255||B!=0))
     {
      tgl=1;
      wx=k-i;
     }
     if (tgl==1&&R==255&&G==255&&B==0)
     {
      tgl=2;
      dx=k-i;
     }
    }
    for (int k=j; k<ry; k++)        // Find height and vertical distance to next intersection
    {
     R=*(map_img+((bx+(k*rx))*3));
     G=*(map_img+((bx+(k*rx))*3)+1);
     B=*(map_img+((bx+(k*rx))*3)+2);
     if (tgl==2&&(R!=255||G!=255||B!=0))
     {
      tgl=3;
      wy=k-j;
     }
     if (tgl==3&&R==255&&G==255&&B==0)
     {
      tgl=4;
      dy=k-j;
     }
    }
    
    if (tgl!=4)
    {
     fprintf(stderr,"Unable to determine intersection geometry!\n");
     return(0);
    }
    else break;
   }
  }
  if (tgl==4) break;
 }
  fprintf(stderr,"Intersection parameters: base_x=%d, base_y=%d, width=%d, height=%d, horiz_distance=%d, vertical_distance=%d\n",bx,by,wx,wy,dx,dy);

  sx=0;
  for (int i=bx+(wx/2);i<rx;i+=dx)
  {
   R=*(map_img+((i+(by*rx))*3));
   G=*(map_img+((i+(by*rx))*3)+1);
   B=*(map_img+((i+(by*rx))*3)+2);
   if (R==255&&G==255&&B==0) sx++;
  }

  sy=0;
  for (int j=by+(wy/2);j<ry;j+=dy)
  {
   R=*(map_img+((bx+(j*rx))*3));
   G=*(map_img+((bx+(j*rx))*3)+1);
   B=*(map_img+((bx+(j*rx))*3)+2);
   if (R==255&&G==255&&B==0) sy++;
  }
  
  fprintf(stderr,"Map size: Number of horizontal intersections=%d, number of vertical intersections=%d\n",sx,sy);

  // Scan for building colours around each intersection
  idx=0;
  for (int j=0; j<sy; j++)
   for (int i=0; i<sx; i++)
   {
    x=bx+(i*dx)+(wx/2);
    y=by+(j*dy)+(wy/2);
    
    fprintf(stderr,"Intersection location: %d, %d\n",x,y);
    // Top-left
    x-=wx;
    y-=wy;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][0]=3;
    else if (R==0&&G==0&&B==255) map[idx][0]=2;
    else if (R==255&&G==255&&B==255) map[idx][0]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Top-Left RGB=%d,%d,%d\n",i,j,R,G,B);

    // Top-right
    x+=2*wx;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][1]=3;
    else if (R==0&&G==0&&B==255) map[idx][1]=2;
    else if (R==255&&G==255&&B==255) map[idx][1]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Top-Right RGB=%d,%d,%d\n",i,j,R,G,B);

    // Bottom-right
    y+=2*wy;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][2]=3;
    else if (R==0&&G==0&&B==255) map[idx][2]=2;
    else if (R==255&&G==255&&B==255) map[idx][2]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Bottom-Right RGB=%d,%d,%d\n",i,j,R,G,B);
    
    // Bottom-left
    x-=2*wx;
    R=*(map_img+((x+(y*rx))*3));
    G=*(map_img+((x+(y*rx))*3)+1);
    B=*(map_img+((x+(y*rx))*3)+2);
    if (R==0&&G==255&&B==0) map[idx][3]=3;
    else if (R==0&&G==0&&B==255) map[idx][3]=2;
    else if (R==255&&G==255&&B==255) map[idx][3]=6;
    else fprintf(stderr,"Colour is not valid for intersection %d,%d, Bottom-Left RGB=%d,%d,%d\n",i,j,R,G,B);
    
    fprintf(stderr,"Colours for this intersection: %d, %d, %d, %d\n",map[idx][0],map[idx][1],map[idx][2],map[idx][3]);
    
    idx++;
   }

 return(1);  
}

unsigned char *readPPMimage(const char *filename, int *rx, int *ry)
{
 // Reads an image from a .ppm file. A .ppm file is a very simple image representation
 // format with a text header followed by the binary RGB data at 24bits per pixel.
 // The header has the following form:
 //
 // P6
 // # One or more comment lines preceded by '#'
 // 340 200
 // 255
 //
 // The first line 'P6' is the .ppm format identifier, this is followed by one or more
 // lines with comments, typically used to inidicate which program generated the
 // .ppm file.
 // After the comments, a line with two integer values specifies the image resolution
 // as number of pixels in x and number of pixels in y.
 // The final line of the header stores the maximum value for pixels in the image,
 // usually 255.
 // After this last header line, binary data stores the RGB values for each pixel
 // in row-major order. Each pixel requires 3 bytes ordered R, G, and B.
 //
 // NOTE: Windows file handling is rather crotchetty. You may have to change the
 //       way this file is accessed if the images are being corrupted on read
 //       on Windows.
 //

 FILE *f;
 unsigned char *im;
 char line[1024];
 int i;
 unsigned char *tmp;
 double *fRGB;

 im=NULL;
 f=fopen(filename,"rb+");
 if (f==NULL)
 {
  fprintf(stderr,"Unable to open file %s for reading, please check name and path\n",filename);
  return(NULL);
 }
 fgets(&line[0],1000,f);
 if (strcmp(&line[0],"P6\n")!=0)
 {
  fprintf(stderr,"Wrong file format, not a .ppm file or header end-of-line characters missing\n");
  fclose(f);
  return(NULL);
 }
 fprintf(stderr,"%s\n",line);
 // Skip over comments
 fgets(&line[0],511,f);
 while (line[0]=='#')
 {
  fprintf(stderr,"%s",line);
  fgets(&line[0],511,f);
 }
 sscanf(&line[0],"%d %d\n",rx,ry);                  // Read image size
 fprintf(stderr,"nx=%d, ny=%d\n\n",*rx,*ry);

 fgets(&line[0],9,f);  	                // Read the remaining header line
 fprintf(stderr,"%s\n",line);
 im=(unsigned char *)calloc((*rx)*(*ry)*3,sizeof(unsigned char));
 if (im==NULL)
 {
  fprintf(stderr,"Out of memory allocating space for image\n");
  fclose(f);
  return(NULL);
 }
 fread(im,(*rx)*(*ry)*3*sizeof(unsigned char),1,f);
 fclose(f);

 return(im);    
}