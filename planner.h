/*
  planner.h - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

// This module is to be considered a sub-module of stepper.c. Please don't include
// this file from any other module.

#ifndef PLANNER_H
#define PLANNER_H

#include "MarlinFirmware.h"
#include "planner_queue.h"

// Initialize the motion plan subsystem
void plan_init();

void check_axes_activity();

#if ENABLED(AUTO_BED_LEVELING_FEATURE) || ENABLED(MESH_BED_LEVELING)

  #if ENABLED(AUTO_BED_LEVELING_FEATURE)
    #include "vector_3.h"

    // Transform required to compensate for bed level
    extern matrix_3x3 plan_bed_level_matrix;

    /**
     * Get the position applying the bed level matrix
     */
    vector_3 plan_get_position();
  #endif  // AUTO_BED_LEVELING_FEATURE

  /**
   * Add a new linear movement to the buffer. x, y, z are the signed, absolute target position in
   * millimeters. Feed rate specifies the (target) speed of the motion.
   */
  void plan_buffer_line(float x, float y, float z, const float& e, float feed_rate, const uint8_t extruder);

  /**
   * Set the planner positions. Used for G92 instructions.
   * Multiplies by axis_steps_per_unit[] to set stepper positions.
   * Clears previous speed values.
   */
  void plan_set_position(float x, float y, float z, const float& e);

#else

  void plan_buffer_line(const float& x, const float& y, const float& z, const float& e, float feed_rate, const uint8_t extruder);
  void plan_set_position(const float& x, const float& y, const float& z, const float& e);

#endif // AUTO_BED_LEVELING_FEATURE || MESH_BED_LEVELING

void plan_set_e_position(const float& e);

//===========================================================================
//============================= public variables ============================
//===========================================================================

extern millis_t minsegmenttime;
extern float max_feedrate[NUM_AXIS]; // Max speeds in mm per minute
extern unsigned long max_acceleration_units_per_sq_second[NUM_AXIS]; // Use M201 to override by software
extern float minimumfeedrate;
extern float acceleration;         // Normal acceleration mm/s^2  DEFAULT ACCELERATION for all printing moves. M204 SXXXX
extern float retract_acceleration; // Retract acceleration mm/s^2 filament pull-back and push-forward while standing still in the other axes M204 TXXXX
extern float travel_acceleration;  // Travel acceleration mm/s^2  DEFAULT ACCELERATION for all NON printing moves. M204 MXXXX
extern float max_xy_jerk;          // The largest speed change requiring no acceleration
extern float max_z_jerk;
extern float max_e_jerk;
extern float mintravelfeedrate;
extern unsigned long axis_steps_per_sqr_second[NUM_AXIS];

#if ENABLED(AUTOTEMP)
  extern bool autotemp_enabled;
  extern float autotemp_max;
  extern float autotemp_min;
  extern float autotemp_factor;
#endif

void reset_acceleration_rates();

#endif // PLANNER_H
