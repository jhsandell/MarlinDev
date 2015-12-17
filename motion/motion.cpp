#include "MarlinFirmware.h"
#include "motion/motion.h"
#include "debug_only_routines.h"

#ifndef XY_TRAVEL_SPEED
  #define XY_TRAVEL_SPEED 1000   // So slow that we will recoginize it as a problem
#endif

#if ENABLED(AUTO_BED_LEVELING_FEATURE)
  int xy_travel_speed = XY_TRAVEL_SPEED;
#endif

/**
 *  Plan a move to (X, Y, Z) and set the current_position
 *  The final current_position may not be the one that was requested
 */

void do_blocking_move_to(float x, float y, float z) {
  float oldFeedRate = feedrate;
#if ENABLED(ROXY_ONLY)
  static int recursion_cnt = -1; // Roxy debug stuff
  recursion_cnt++; // Roxy debug stuff
#endif
#if ENABLED(DEBUG_LEVELING_FEATURE)
  if (marlin_debug_flags & DEBUG_LEVELING) {
#if ENABLED(ROXY_ONLY)
    int i;  // Roxy debug stuff
    for (i = 0; i < recursion_cnt; i++) // Roxy debug stuff
      SERIAL_PROTOCOLPGM("  ");  // Roxy debug stuff
#endif
    print_xyz("do_blocking_move_to", x, y, z);
  }
#endif
#if ENABLED(DELTA)
  feedrate = xy_travel_speed;
#if ENABLED(ROXY_ONLY)
  //
  // Should we break up the move because it is too big ???
  //
#define MAX_MOVEMENT_WITHOUT_BREAKING_UP_MOVE	25
  if (abs(x - current_position[X_AXIS]) > MAX_MOVEMENT_WITHOUT_BREAKING_UP_MOVE	||
      abs(y - current_position[Y_AXIS]) > MAX_MOVEMENT_WITHOUT_BREAKING_UP_MOVE	||
      abs(z - current_position[Z_AXIS]) > MAX_MOVEMENT_WITHOUT_BREAKING_UP_MOVE) {
    do_blocking_move_to(current_position[X_AXIS] + (x - current_position[X_AXIS]) / 2.0,
                        current_position[Y_AXIS] + (y - current_position[Y_AXIS]) / 2.0,
                        current_position[Z_AXIS] + (z - current_position[Z_AXIS]) / 2.0);
    //
    // At this point, we are halfway to where we want to be.  Do the other half of the move.
    //
    do_blocking_move_to(x, y, z);
    feedrate = oldFeedRate;
    recursion_cnt--; // Roxy debug stuff
    return;
  } else
#endif
  {		// No...  It is small enough to do in one piece.
    destination[X_AXIS] = x;
    destination[Y_AXIS] = y;
    destination[Z_AXIS] = z;
    prepare_move_raw(); // this will also set_current_to_destination
    st_synchronize();
  }
#else
  feedrate = homing_feedrate[Z_AXIS];
  current_position[Z_AXIS] = z;
  line_to_current_position();
  st_synchronize();
  feedrate = xy_travel_speed;
  current_position[X_AXIS] = x;
  current_position[Y_AXIS] = y;
  line_to_current_position();
  st_synchronize();
#endif
  feedrate = oldFeedRate;
#if ENABLED(ROXY_ONLY)
  recursion_cnt--; // Roxy debug stuff
#endif
}

