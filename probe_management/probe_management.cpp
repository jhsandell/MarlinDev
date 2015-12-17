#include "MarlinFirmware.h"
#include "probe_management/probe_management.h"

bool z_probe_is_active = false;

#if ENABLED(USE_PROBE)

#include "motion/motion.h"
#include "movement.h"
#include "debug_only_routines.h"
#include "display/display.h"
#include "unit_conversion.h"
#include "thermal/heater_management.h"
#include "host_interface/host_io.h"
#include "planner.h"

#if ENABLED(MANUAL_ALLEN_KEY_DEPLOYMENT)
  #include "display/panel.h"
#endif

#if HAS_SERVOS
  #include "servo.h"
#endif
#if HAS_SERVO_ENDSTOPS
  const int servo_endstop_id[] = SERVO_ENDSTOP_IDS;
  const int servo_endstop_angle[][2] = SERVO_ENDSTOP_ANGLES;
#endif

#if ENABLED(AUTO_BED_LEVELING_FEATURE)
  bool z_probe_is_stowed = true;
#endif

#ifndef X_PROBE_OFFSET_FROM_EXTRUDER
  #define X_PROBE_OFFSET_FROM_EXTRUDER 0
#endif
#ifndef Y_PROBE_OFFSET_FROM_EXTRUDER
  #define Y_PROBE_OFFSET_FROM_EXTRUDER 0
#endif

#define DEFINE_PGM_READ_ANY(type, reader)       \
  static inline type pgm_read_any(const type *p)  \
  { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
  static const PROGMEM type array##_P[3] =        \
      { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
  static inline type array(int axis)          \
  { return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, home_bump_mm,   HOME_BUMP_MM);

void deploy_z_probe() {
  bool z_probe_endstop;

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (marlin_debug_flags & DEBUG_LEVELING) {
      print_xyz("deploy_z_probe > current_position", current_position);
    }
  #endif

  #if HAS_SERVO_ENDSTOPS

    // Engage Z Servo endstop if enabled
    if (servo_endstop_id[Z_AXIS] >= 0) servo[servo_endstop_id[Z_AXIS]].move(servo_endstop_angle[Z_AXIS][0]);

  #elif ENABLED(Z_PROBE_ALLEN_KEY)

  #ifndef MANUAL_ALLEN_KEY_DEPLOYMENT
    feedrate = Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE;

    // If endstop is already false, the Z probe is deployed
    #if ENABLED(Z_MIN_PROBE_ENDSTOP)
      z_probe_endstop = (READ(Z_MIN_PROBE_PIN) != Z_MIN_PROBE_ENDSTOP_INVERTING);
    #else
      z_probe_endstop = (READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING);
    #endif
    if (z_probe_endstop)
      {
        // Move to the start position to initiate deployment
        destination[X_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_1_X;
        destination[Y_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_1_Y;
        destination[Z_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_1_Z;
        prepare_move_raw(); // this will also set_current_to_destination

        // Move to engage deployment
        if (Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE != Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE)
          feedrate = Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE;
        if (Z_PROBE_ALLEN_KEY_DEPLOY_2_X != Z_PROBE_ALLEN_KEY_DEPLOY_1_X)
          destination[X_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_2_X;
        if (Z_PROBE_ALLEN_KEY_DEPLOY_2_Y != Z_PROBE_ALLEN_KEY_DEPLOY_1_Y)
          destination[Y_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_2_Y;
        if (Z_PROBE_ALLEN_KEY_DEPLOY_2_Z != Z_PROBE_ALLEN_KEY_DEPLOY_1_Z)
          destination[Z_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_2_Z;
        prepare_move_raw();

        #ifdef Z_PROBE_ALLEN_KEY_DEPLOY_3_X
          if (Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE != Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE)
            feedrate = Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE;

          // Move to trigger deployment
          if (Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE != Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE)
            feedrate = Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE;
          if (Z_PROBE_ALLEN_KEY_DEPLOY_3_X != Z_PROBE_ALLEN_KEY_DEPLOY_2_X)
            destination[X_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_3_X;
          if (Z_PROBE_ALLEN_KEY_DEPLOY_3_Y != Z_PROBE_ALLEN_KEY_DEPLOY_2_Y)
            destination[Y_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_3_Y;
          if (Z_PROBE_ALLEN_KEY_DEPLOY_3_Z != Z_PROBE_ALLEN_KEY_DEPLOY_2_Z)
            destination[Z_AXIS] = Z_PROBE_ALLEN_KEY_DEPLOY_3_Z;
          prepare_move_raw();
        #endif
      }

#else			// Manual AllenKey Deployment
{
SERIAL_PROTOCOLPGM("Put down Z-Probe.\n");
LCD_ALERTMESSAGEPGM("Put down Z-Probe.");
while(!lcd_clicked()){
	manage_heater();
	manage_inactivity();
	lcd_update();
}

lcd_reset_alert_level();
LCD_ALERTMESSAGEPGM("Continuing...");
while(lcd_clicked()) {
	manage_heater();
	manage_inactivity();
	lcd_update();
}

lcd_reset_alert_level();
delay(500);
LCD_ALERTMESSAGEPGM("    ");
}

#endif  	// Manual AllenKey Deployment

// Partially Home X,Y for safety	--- Killed by Roxy.  This goofs up a bunch of stuff.
//      destination[X_AXIS] = destination[X_AXIS]*0.75;
//      destination[Y_AXIS] = destination[Y_AXIS]*0.75;
//      prepare_move_raw(); // this will also set_current_to_destination
//
    st_synchronize();

    #if ENABLED(Z_MIN_PROBE_ENDSTOP)
      z_probe_endstop = (READ(Z_MIN_PROBE_PIN) != Z_MIN_PROBE_ENDSTOP_INVERTING);
      if (z_probe_endstop)
    #else
      z_min_endstop = (READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING);
      if (z_min_endstop)
    #endif
      {
        if (IsRunning()) {
          SERIAL_ERROR_START;
          SERIAL_ERRORLNPGM("Z-Probe failed to engage!");
          LCD_ALERTMESSAGEPGM("Err: ZPROBE");
        }
        Stop();
      }

  #endif // Z_PROBE_ALLEN_KEY
  z_probe_is_stowed = false;
  SERIAL_ECHOLNPGM("Probe deployed\n");
}

void stow_z_probe(bool doRaise) {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (marlin_debug_flags & DEBUG_LEVELING) {
      print_xyz("stow_z_probe > current_position", current_position);
    }
  #endif

  #if HAS_SERVO_ENDSTOPS

    // Retract Z Servo endstop if enabled
    if (servo_endstop_id[Z_AXIS] >= 0) {

      #if Z_RAISE_AFTER_PROBING > 0
        if (doRaise) {
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (marlin_debug_flags & DEBUG_LEVELING) {
              SERIAL_ECHOPAIR("Raise Z (after) by ", (float)Z_RAISE_AFTER_PROBING);
              SERIAL_EOL;
              SERIAL_ECHOPAIR("> SERVO_ENDSTOPS > raise_z_after_probing()");
              SERIAL_EOL;
            }
          #endif
          raise_z_after_probing(); // this also updates current_position
          st_synchronize();
        }
      #endif

      // Change the Z servo angle
      servo[servo_endstop_id[Z_AXIS]].move(servo_endstop_angle[Z_AXIS][1]);
    }

  #elif ENABLED(Z_PROBE_ALLEN_KEY)

    #ifndef MANUAL_ALLEN_KEY_DEPLOYMENT

    // Move up for safety
    feedrate = Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE;

    #if Z_RAISE_AFTER_PROBING > 0
      destination[Z_AXIS] = current_position[Z_AXIS] + Z_RAISE_AFTER_PROBING;
      prepare_move_raw(); // this will also set_current_to_destination
    #endif

    // Move to the start position to initiate retraction
    destination[X_AXIS] = Z_PROBE_ALLEN_KEY_STOW_1_X;
    destination[Y_AXIS] = Z_PROBE_ALLEN_KEY_STOW_1_Y;
    destination[Z_AXIS] = Z_PROBE_ALLEN_KEY_STOW_1_Z;
    prepare_move_raw();

    // Move the nozzle down to push the Z probe into retracted position
    if (Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE != Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE)
      feedrate = Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE;
    if (Z_PROBE_ALLEN_KEY_STOW_2_X != Z_PROBE_ALLEN_KEY_STOW_1_X)
      destination[X_AXIS] = Z_PROBE_ALLEN_KEY_STOW_2_X;
    if (Z_PROBE_ALLEN_KEY_STOW_2_Y != Z_PROBE_ALLEN_KEY_STOW_1_Y)
      destination[Y_AXIS] = Z_PROBE_ALLEN_KEY_STOW_2_Y;
    destination[Z_AXIS] = Z_PROBE_ALLEN_KEY_STOW_2_Z;
    prepare_move_raw();

    // Move up for safety
    if (Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE != Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE)
      feedrate = Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE;
    if (Z_PROBE_ALLEN_KEY_STOW_3_X != Z_PROBE_ALLEN_KEY_STOW_2_X)
      destination[X_AXIS] = Z_PROBE_ALLEN_KEY_STOW_3_X;
    if (Z_PROBE_ALLEN_KEY_STOW_3_Y != Z_PROBE_ALLEN_KEY_STOW_2_Y)
      destination[Y_AXIS] = Z_PROBE_ALLEN_KEY_STOW_3_Y;
    destination[Z_AXIS] = Z_PROBE_ALLEN_KEY_STOW_3_Z;
    prepare_move_raw();

    // Home XY for safety
    feedrate = homing_feedrate[X_AXIS] / 2;
    destination[X_AXIS] = 0;
    destination[Y_AXIS] = 0;
    prepare_move_raw(); // this will also set_current_to_destination

    st_synchronize();

    #if ENABLED(Z_MIN_PROBE_ENDSTOP)
      bool z_probe_endstop = (READ(Z_MIN_PROBE_PIN) != Z_MIN_PROBE_ENDSTOP_INVERTING);
      if (!z_probe_endstop)
    #else
      bool z_min_endstop = (READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING);
      if (!z_min_endstop)
    #endif
      {
        if (IsRunning()) {
          SERIAL_ERROR_START;
          SERIAL_ERRORLNPGM("Z-Probe failed to retract!");
          LCD_ALERTMESSAGEPGM("Err: ZPROBE");
        }
        Stop();
      }

#else				// MANUAL_ALLEN_KEY_DEPLOYMENT

  raise_z_after_probing(); // this also updates current_position
  st_synchronize();

  SERIAL_PROTOCOLPGM("Put Z-Probe Up!\n");
  LCD_ALERTMESSAGEPGM("Put Z-Probe up.");
  while(!lcd_clicked()){
     manage_heater();
     manage_inactivity();
     lcd_update();
  }

  lcd_reset_alert_level();
  LCD_ALERTMESSAGEPGM("Continuing...");
  while(lcd_clicked()) {
     manage_heater();
     manage_inactivity();
     lcd_update();
  }

  lcd_reset_alert_level();
  delay(500);
  LCD_ALERTMESSAGEPGM("      ");

  #endif // Z_PROBE_ALLEN_KEY
#endif	// MANUAL_ALLEN_KEY_DEPLOYMENT

  z_probe_is_stowed = true;
SERIAL_ECHOLNPGM("Probe stowed\n");
}

// Probe bed height at position (x,y), returns the measured z value
float probe_pt(float x, float y, float z_before, ProbeAction probe_action, int verbose_level) {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (marlin_debug_flags & DEBUG_LEVELING) {
      SERIAL_ECHOLNPGM("probe_pt >>>");
      SERIAL_ECHOPAIR("> ProbeAction:", (unsigned long)probe_action);
      SERIAL_EOL;
      print_xyz("> current_position", current_position);
    }
  #endif

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (marlin_debug_flags & DEBUG_LEVELING) {
      SERIAL_ECHOPAIR("Z Raise to z_before ", z_before);
      SERIAL_EOL;
      SERIAL_ECHOPAIR("> do_blocking_move_to_z ", z_before);
      SERIAL_EOL;
    }
  #endif

  // Move Z up to the z_before height, then move the Z probe to the given XY
  do_blocking_move_to_z(z_before); // this also updates current_position

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (marlin_debug_flags & DEBUG_LEVELING) {
      SERIAL_ECHOPAIR("> do_blocking_move_to_xy ", x - X_PROBE_OFFSET_FROM_EXTRUDER);
      SERIAL_ECHOPAIR(", ", y - Y_PROBE_OFFSET_FROM_EXTRUDER);
      SERIAL_EOL;
    }
  #endif

  do_blocking_move_to_xy(x - X_PROBE_OFFSET_FROM_EXTRUDER, y - Y_PROBE_OFFSET_FROM_EXTRUDER); // this also updates current_position

  #if DISABLED(Z_PROBE_SLED) && DISABLED(Z_PROBE_ALLEN_KEY)
    if (probe_action & ProbeDeploy) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          SERIAL_ECHOLNPGM("> ProbeDeploy");
        }
      #endif
      deploy_z_probe();
    }
  #elif ENABLED( MANUAL_ALLEN_KEY_DEPLOYMENT )
    if (probe_action & ProbeDeploy) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          SERIAL_ECHOLNPGM("> Manual ProbeDeploy");
        }
      #endif
 	    deploy_z_probe();
    }
  #endif

  setup_for_endstop_move();

  run_z_probe();

  float measured_z = current_position[Z_AXIS];

  #if DISABLED(Z_PROBE_SLED) && DISABLED(Z_PROBE_ALLEN_KEY)
    if (probe_action & ProbeStow) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          SERIAL_ECHOLNPGM("> ProbeStow (stow_z_probe will do Z Raise)");
        }
      #endif
      stow_z_probe();
    }
  #endif

  if (verbose_level > 2) {
    SERIAL_PROTOCOLPGM("Bed X: ");
    SERIAL_PROTOCOL_F(x, 3);
    SERIAL_PROTOCOLPGM(" Y: ");
    SERIAL_PROTOCOL_F(y, 3);
    SERIAL_PROTOCOLPGM(" Z: ");
    SERIAL_PROTOCOL_F(measured_z, 3);
    SERIAL_EOL;
  }

  #if DISABLED(Z_PROBE_SLED) && DISABLED(Z_PROBE_ALLEN_KEY)
    if (probe_action & ProbeStow) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          SERIAL_ECHOLNPGM("> ProbeStow (stow_z_probe will do Z Raise)");
        }
      #endif
      stow_z_probe( true );
    }
  #elif ENABLED( MANUAL_ALLEN_KEY_DEPLOYMENT )
    if (probe_action & ProbeStow) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          SERIAL_ECHOLNPGM("> Manual ProbeStow (stow_z_probe will do Z Raise)");
        }
      #endif
      stow_z_probe( true );
    }
  #endif

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (marlin_debug_flags & DEBUG_LEVELING) {
      SERIAL_ECHOLNPGM("<<< probe_pt");
    }
  #endif

  clean_up_after_endstop_move();
  return measured_z;
}

/**
 * Some planner shorthand inline functions
 */
inline void set_homing_bump_feedrate(AxisEnum axis) {
  const int homing_bump_divisor[] = HOMING_BUMP_DIVISOR;
  int hbd = homing_bump_divisor[axis];
  if (hbd < 1) {
    hbd = 10;
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Warning: Homing Bump Divisor < 1");
  }
  feedrate = homing_feedrate[axis] / hbd;
}

void run_z_probe() {
  z_probe_is_active = true;

  #if ENABLED(DELTA)

    float start_z = current_position[Z_AXIS];
    long start_steps = st_get_position(Z_AXIS);

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        SERIAL_ECHOLNPGM("run_z_probe (DELTA) 1");
      }
    #endif

    // move down slowly until you find the bed
    feedrate = homing_feedrate[Z_AXIS] / 4;
    destination[Z_AXIS] = -10;
    prepare_move_raw(); // this will also set_current_to_destination
    st_synchronize();
    endstops_hit_on_purpose(); // clear endstop hit flags

    // we have to let the planner know where we are right now as it is not where we said to go.
    long stop_steps = st_get_position(Z_AXIS);
    float mm = start_z - float(start_steps - stop_steps) / axis_steps_per_unit[Z_AXIS];
    current_position[Z_AXIS] = mm;

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        print_xyz("run_z_probe (DELTA) 2 > current_position", current_position);
      }
    #endif

    sync_plan_position_delta();

  #else // !DELTA

    plan_bed_level_matrix.set_to_identity();
    feedrate = homing_feedrate[Z_AXIS];

    // Move down until the Z probe (or endstop?) is triggered
    float zPosition = -(Z_MAX_LENGTH + 10);
    line_to_z(zPosition);
    st_synchronize();

    // Tell the planner where we ended up - Get this from the stepper handler
    zPosition = st_get_position_mm(Z_AXIS);
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS]);

    // move up the retract distance
    zPosition += home_bump_mm(Z_AXIS);
    line_to_z(zPosition);
    st_synchronize();
    endstops_hit_on_purpose(); // clear endstop hit flags

    // move back down slowly to find bed
    set_homing_bump_feedrate(Z_AXIS);

    zPosition -= home_bump_mm(Z_AXIS) * 2;
    line_to_z(zPosition);
    st_synchronize();
    endstops_hit_on_purpose(); // clear endstop hit flags

    // Get the current stepper position after bumping an endstop
    current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
    sync_plan_position();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        print_xyz("run_z_probe > current_position", current_position);
      }
    #endif

  #endif // !DELTA
  z_probe_is_active = false;
}
#endif