/**
 * Home an individual axis
 */

#include "MarlinFirmware.h"

#include "planner.h"
//#include "configuration_store.h"
//#include "parser.h"
//#include "probe_management/probe_management.h"
//#include "kinematics/coefficients.h"
#include "motion/motion.h"
#include "movement.h"
#include "host_interface/host_io.h"
#include "debug_only_routines.h"

#if HAS_SERVOS
  #include "servo.h"
#endif
#if HAS_SERVO_ENDSTOPS
  const int servo_endstop_id[] = SERVO_ENDSTOP_IDS;
  const int servo_endstop_angle[][2] = SERVO_ENDSTOP_ANGLES;
#endif

#if ENABLED(Z_DUAL_ENDSTOPS)
  void In_Homing_Process(bool state);
  void Lock_z_motor(bool state);
  void Lock_z2_motor(bool state);
#endif

#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

#define DEFINE_PGM_READ_ANY(type, reader)       \
  static inline type pgm_read_any(const type *p)  \
  { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
  static const PROGMEM type array##_P[3] =        \
      { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
  static inline type array(int axis)          \
  { return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,   MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,   MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,  HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,     MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_bump_mm,   HOME_BUMP_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir, HOME_DIR);

void set_axis_is_at_home(AxisEnum axis) {

  #if ENABLED(DUAL_X_CARRIAGE)
    if (axis == X_AXIS) {
      if (active_extruder != 0) {
        current_position[X_AXIS] = x_home_pos(active_extruder);
                 min_pos[X_AXIS] = X2_MIN_POS;
                 max_pos[X_AXIS] = max(extruder_offset[X_AXIS][1], X2_MAX_POS);
        return;
      }
      else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE) {
        float xoff = home_offset[X_AXIS];
        current_position[X_AXIS] = base_home_pos(X_AXIS) + xoff;
                 min_pos[X_AXIS] = base_min_pos(X_AXIS) + xoff;
                 max_pos[X_AXIS] = min(base_max_pos(X_AXIS) + xoff, max(extruder_offset[X_AXIS][1], X2_MAX_POS) - duplicate_extruder_x_offset);
        return;
      }
    }
  #endif

  #if ENABLED(SCARA)

    if (axis == X_AXIS || axis == Y_AXIS) {

      float homeposition[3];
      for (int i = 0; i < 3; i++) homeposition[i] = base_home_pos(i);

      // SERIAL_ECHOPGM("homeposition[x]= "); SERIAL_ECHO(homeposition[0]);
      // SERIAL_ECHOPGM("homeposition[y]= "); SERIAL_ECHOLN(homeposition[1]);
      // Works out real Homeposition angles using inverse kinematics,
      // and calculates homing offset using forward kinematics
      calculate_delta(homeposition);

      // SERIAL_ECHOPGM("base Theta= "); SERIAL_ECHO(delta[X_AXIS]);
      // SERIAL_ECHOPGM(" base Psi+Theta="); SERIAL_ECHOLN(delta[Y_AXIS]);

      for (int i = 0; i < 2; i++) delta[i] -= home_offset[i];

      // SERIAL_ECHOPGM("addhome X="); SERIAL_ECHO(home_offset[X_AXIS]);
      // SERIAL_ECHOPGM(" addhome Y="); SERIAL_ECHO(home_offset[Y_AXIS]);
      // SERIAL_ECHOPGM(" addhome Theta="); SERIAL_ECHO(delta[X_AXIS]);
      // SERIAL_ECHOPGM(" addhome Psi+Theta="); SERIAL_ECHOLN(delta[Y_AXIS]);

      calculate_SCARA_forward_Transform(delta);

      // SERIAL_ECHOPGM("Delta X="); SERIAL_ECHO(delta[X_AXIS]);
      // SERIAL_ECHOPGM(" Delta Y="); SERIAL_ECHOLN(delta[Y_AXIS]);

      current_position[axis] = delta[axis];

      // SCARA home positions are based on configuration since the actual limits are determined by the
      // inverse kinematic transform.
      min_pos[axis] = base_min_pos(axis); // + (delta[axis] - base_home_pos(axis));
      max_pos[axis] = base_max_pos(axis); // + (delta[axis] - base_home_pos(axis));
    }
    else
  #endif
  {
    current_position[axis] = base_home_pos(axis) + home_offset[axis];
    min_pos[axis] = base_min_pos(axis) + home_offset[axis];
    max_pos[axis] = base_max_pos(axis) + home_offset[axis];

    #if ENABLED(AUTO_BED_LEVELING_FEATURE) && Z_HOME_DIR < 0
      if (axis == Z_AXIS) current_position[Z_AXIS] -= zprobe_zoffset;
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        SERIAL_ECHOPAIR("set_axis_is_at_home ", (unsigned long)axis);
        SERIAL_ECHOPAIR(" > (home_offset[axis]==", home_offset[axis]);
        print_xyz(") > current_position", current_position);
      }
    #endif
  }
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

void homeaxis(AxisEnum axis) {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (marlin_debug_flags & DEBUG_LEVELING) {
      SERIAL_ECHOPAIR(">>> homeaxis(", (unsigned long)axis);
      SERIAL_CHAR(')');
      SERIAL_EOL;
    }
  #endif
  #define HOMEAXIS_DO(LETTER) \
    ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

  if (axis == X_AXIS ? HOMEAXIS_DO(X) : axis == Y_AXIS ? HOMEAXIS_DO(Y) : axis == Z_AXIS ? HOMEAXIS_DO(Z) : 0) {

    int axis_home_dir =
      #if ENABLED(DUAL_X_CARRIAGE)
        (axis == X_AXIS) ? x_home_dir(active_extruder) :
      #endif
      home_dir(axis);

    // Set the axis position as setup for the move
    current_position[axis] = 0;
    sync_plan_position();

    #if ENABLED(Z_PROBE_SLED)
      // Get Probe
      if (axis == Z_AXIS) {
        if (axis_home_dir < 0) dock_sled(false);
      }
    #endif

    #if SERVO_LEVELING && DISABLED(Z_PROBE_SLED)

      // Deploy a Z probe if there is one, and homing towards the bed
      if (axis == Z_AXIS) {
        if (axis_home_dir < 0) deploy_z_probe();
      }

    #endif

    #if HAS_SERVO_ENDSTOPS
      // Engage Servo endstop if enabled
      if (axis != Z_AXIS && servo_endstop_id[axis] >= 0)
        servo[servo_endstop_id[axis]].move(servo_endstop_angle[axis][0]);
    #endif

    // Set a flag for Z motor locking
    #if ENABLED(Z_DUAL_ENDSTOPS)
      if (axis == Z_AXIS) In_Homing_Process(true);
    #endif

    // Move towards the endstop until an endstop is triggered
    destination[axis] = 1.5 * max_length(axis) * axis_home_dir;
    feedrate = homing_feedrate[axis];
    line_to_destination();
    st_synchronize();

    // Set the axis position as setup for the move
    current_position[axis] = 0;
    sync_plan_position();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        SERIAL_ECHOLNPGM("> enable_endstops(false)");
      }
    #endif
    enable_endstops(false); // Disable endstops while moving away

    // Move away from the endstop by the axis HOME_BUMP_MM
    destination[axis] = -home_bump_mm(axis) * axis_home_dir;
    line_to_destination();
    st_synchronize();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        SERIAL_ECHOLNPGM("> enable_endstops(true)");
      }
    #endif
    enable_endstops(true); // Enable endstops for next homing move

    // Slow down the feedrate for the next move
    set_homing_bump_feedrate(axis);

    // Move slowly towards the endstop until triggered
    destination[axis] = 2 * home_bump_mm(axis) * axis_home_dir;
    line_to_destination();
    st_synchronize();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        print_xyz("> TRIGGER ENDSTOP > current_position", current_position);
      }
    #endif

    #if ENABLED(Z_DUAL_ENDSTOPS)
      if (axis == Z_AXIS) {
        float adj = fabs(z_endstop_adj);
        bool lockZ1;
        if (axis_home_dir > 0) {
          adj = -adj;
          lockZ1 = (z_endstop_adj > 0);
        }
        else
          lockZ1 = (z_endstop_adj < 0);

        if (lockZ1) Lock_z_motor(true); else Lock_z2_motor(true);
        sync_plan_position();

        // Move to the adjusted endstop height
        feedrate = homing_feedrate[axis];
        destination[Z_AXIS] = adj;
        line_to_destination();
        st_synchronize();

        if (lockZ1) Lock_z_motor(false); else Lock_z2_motor(false);
        In_Homing_Process(false);
      } // Z_AXIS
    #endif

    #if ENABLED(DELTA)
      // retrace by the amount specified in endstop_adj
      if (endstop_adj[axis] * axis_home_dir < 0) {
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            SERIAL_ECHOLNPGM("> enable_endstops(false)");
          }
        #endif
        enable_endstops(false); // Disable endstops while moving away
        sync_plan_position();
        destination[axis] = endstop_adj[axis];
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            SERIAL_ECHOPAIR("> endstop_adj = ", endstop_adj[axis]);
            print_xyz(" > destination", destination);
          }
        #endif
        line_to_destination();
        st_synchronize();
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            SERIAL_ECHOLNPGM("> enable_endstops(true)");
          }
        #endif
        enable_endstops(true); // Enable endstops for next homing move
      }
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        else {
          if (marlin_debug_flags & DEBUG_LEVELING) {
            SERIAL_ECHOPAIR("> endstop_adj * axis_home_dir = ", endstop_adj[axis] * axis_home_dir);
            SERIAL_EOL;
          }
        }
      #endif
    #endif

    // Set the axis position to its home position (plus home offsets)
    set_axis_is_at_home(axis);
    sync_plan_position();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        print_xyz("> AFTER set_axis_is_at_home > current_position", current_position);
      }
    #endif

    destination[axis] = current_position[axis];
    feedrate = 0.0;
    endstops_hit_on_purpose(); // clear endstop hit flags
    axis_known_position[axis] = true;

    #if ENABLED(Z_PROBE_SLED)
      // bring Z probe back
      if (axis == Z_AXIS) {
        if (axis_home_dir < 0) dock_sled(true);
      }
    #endif

    #if SERVO_LEVELING && DISABLED(Z_PROBE_SLED)

      // Deploy a Z probe if there is one, and homing towards the bed
      if (axis == Z_AXIS) {
        if (axis_home_dir < 0) {
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (marlin_debug_flags & DEBUG_LEVELING) {
              SERIAL_ECHOLNPGM("> SERVO_LEVELING > stow_z_probe");
            }
          #endif
          stow_z_probe();
        }
      }
      else

    #endif

    {
      #if HAS_SERVO_ENDSTOPS
        // Retract Servo endstop if enabled
        if (servo_endstop_id[axis] >= 0) {
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (marlin_debug_flags & DEBUG_LEVELING) {
              SERIAL_ECHOLNPGM("> SERVO_ENDSTOPS > Stow with servo.move()");
            }
          #endif
          servo[servo_endstop_id[axis]].move(servo_endstop_angle[axis][1]);
        }
      #endif
    }

  }

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (marlin_debug_flags & DEBUG_LEVELING) {
      SERIAL_ECHOPAIR("<<< homeaxis(", (unsigned long)axis);
      SERIAL_CHAR(')');
      SERIAL_EOL;
    }
  #endif
}
