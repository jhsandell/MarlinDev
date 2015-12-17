extern float feedrate, saved_feedrate;
extern float current_position[NUM_AXIS];
extern float destination[NUM_AXIS];
extern const float homing_feedrate[];
extern uint8_t active_extruder;
extern int xy_travel_speed;

extern void do_blocking_move_to(float x, float y, float z);

void setup_for_endstop_move();
void clean_up_after_endstop_move();

inline void do_blocking_move_to_z(float z) { do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], z); }
inline void do_blocking_move_to_xy(float x, float y) { do_blocking_move_to(x, y, current_position[Z_AXIS]); }
inline void do_blocking_move_to_x(float x) { do_blocking_move_to(x, current_position[Y_AXIS], current_position[Z_AXIS]); }
#ifndef Z_RAISE_AFTER_PROBING
  #define Z_RAISE_AFTER_PROBING 0
#endif
inline void raise_z_after_probing() { do_blocking_move_to_z(current_position[Z_AXIS] + Z_RAISE_AFTER_PROBING); }
inline void set_current_to_destination() { memcpy(current_position, destination, sizeof(current_position)); }
inline void set_destination_to_current() { memcpy(destination, current_position, sizeof(destination)); }


// These should be moved into the movement section

extern void prepare_move_raw();
extern void run_z_probe();

// Block until all buffered steps are executed
void st_synchronize();
// Get current position in steps
long st_get_position(uint8_t axis);
void endstops_hit_on_purpose(); //avoid creation of the message, i.e. after homing and before a routine call of checkHitEndstops();


#if ENABLED(AUTO_BED_LEVELING_FEATURE) || ENABLED(MESH_BED_LEVELING)
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

inline void sync_plan_position() {
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}
#if ENABLED(DELTA) || ENABLED(SCARA)
  inline void sync_plan_position_delta() {
    calculate_delta(current_position);
    plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
  }
#endif

inline void line_to_current_position() {
  plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate / 60, active_extruder);
}

