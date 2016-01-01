// movement.h

// Initialize and start the stepper motor subsystem
void st_init();


// Set current position in steps
void st_set_position(const long& x, const long& y, const long& z, const long& e);
void st_set_e_position(const long& e);

// Get current position in steps
long st_get_position(uint8_t axis);

// Get current position in mm
float st_get_position_mm(AxisEnum axis);

void endstops_hit_on_purpose(); //avoid creation of the message, i.e. after homing and before a routine call of checkHitEndstops();

void enable_endstops(bool check); // Enable/disable endstop checking

// planner motion
#if ENABLED(HAS_LEVELING)
  void plan_set_position(float x, float y, float z, const float& e);
#else
  void plan_set_position(const float& x, const float& y, const float& z, const float& e);
#endif // HAS_LEVELING

#if ENABLED(DELTA) || ENABLED(SCARA)
  inline void sync_plan_position_delta() {
    calculate_delta(current_position);
    plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
  }
#endif

