#include "MarlinFirmware.h"

extern float delta_radius;
extern float delta_tower1_x; // front left tower
extern float delta_tower1_y;
extern float delta_tower2_x; // front right tower
extern float delta_tower2_y;
extern float delta_tower3_x; // back middle tower
extern float delta_tower3_y;
extern float delta_radius_trim_tower_1;
extern float delta_radius_trim_tower_2;
extern float delta_radius_trim_tower_3;
extern float delta_diagonal_rod;
extern float delta_diagonal_rod_trim_tower_1;
extern float delta_diagonal_rod_trim_tower_2;
extern float delta_diagonal_rod_trim_tower_3;
extern float delta_diagonal_rod_2_tower_1;
extern float delta_diagonal_rod_2_tower_2;
extern float delta_diagonal_rod_2_tower_3;
#ifndef MESH_NUM_X_POINTS
  extern float bed_level[1][1];
#else
  extern float bed_level[MESH_NUM_X_POINTS][MESH_NUM_Y_POINTS];
#endif
