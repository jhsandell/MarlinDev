/**
 * G28: Home all axes according to settings
 *
 * Parameters
 *
 *  None  Home to all axes with no parameters.
 *        With QUICK_HOME enabled XY will home together, then Z.
 *
 * Cartesian parameters
 *
 *  X   Home to the X endstop
 *  Y   Home to the Y endstop
 *  Z   Home to the Z endstop
 *
 */

#include "MarlinFirmware.h"

#include "planner.h"
//#include "configuration_store.h"
#include "parser.h"
//#include "probe_management/probe_management.h"
//#include "kinematics/coefficients.h"
#include "motion/motion.h"
#include "movement.h"
#include "host_interface/host_io.h"
#include "motion/homing.h"
#include "debug_only_routines.h"

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

// XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,   MIN_POS);
// XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,   MAX_POS);
// XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,  HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,     MAX_LENGTH);
// XYZ_CONSTS_FROM_CONFIG(float, home_bump_mm,   HOME_BUMP_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir, HOME_DIR);

void gcode_G28() {

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (marlin_debug_flags & DEBUG_LEVELING) {
      SERIAL_ECHOLNPGM("gcode_G28 >>>");
    }
  #endif

  // Wait for planner moves to finish!
  st_synchronize();

  // For auto bed leveling, clear the level matrix
  #if ENABLED(AUTO_BED_LEVELING_FEATURE)
    plan_bed_level_matrix.set_to_identity();
    #if ENABLED(DELTA)
      reset_bed_level();
    #endif
  #endif

  // For manual bed leveling deactivate the matrix temporarily
  #if ENABLED(MESH_BED_LEVELING)
    uint8_t mbl_was_active = mbl.active;
    mbl.active = 0;
  #endif

  setup_for_endstop_move();

  set_destination_to_current();

  feedrate = 0.0;

  #if ENABLED(DELTA)
    // A delta can only safely home all axis at the same time
    // all axis have to home at the same time

    // Pretend the current position is 0,0,0
    for (int i = X_AXIS; i <= Z_AXIS; i++) current_position[i] = 0;
    sync_plan_position();

    // Move all carriages up together until the first endstop is hit.
    for (int i = X_AXIS; i <= Z_AXIS; i++) destination[i] = 3 * Z_MAX_LENGTH;
    feedrate = 1.732 * homing_feedrate[X_AXIS];
    line_to_destination();
    st_synchronize();
    endstops_hit_on_purpose(); // clear endstop hit flags

    // Destination reached
    for (int i = X_AXIS; i <= Z_AXIS; i++) current_position[i] = destination[i];

    // take care of back off and rehome now we are all at the top
    HOMEAXIS(X);
    HOMEAXIS(Y);
    HOMEAXIS(Z);

    sync_plan_position_delta();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        print_xyz("(DELTA) > current_position", current_position);
      }
    #endif

  #else // NOT DELTA

    static bool home_all_axis = true;
    bool  homeX = code_seen(axis_codes[X_AXIS]),
          homeY = code_seen(axis_codes[Y_AXIS]),
          homeZ = code_seen(axis_codes[Z_AXIS]);

    home_all_axis = (!homeX && !homeY && !homeZ) || (homeX && homeY && homeZ);

    if (home_all_axis || homeZ) {

      #if Z_HOME_DIR > 0  // If homing away from BED do Z first

        HOMEAXIS(Z);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            print_xyz("> HOMEAXIS(Z) > current_position", current_position);
          }
        #endif

      #elif DISABLED(Z_SAFE_HOMING) && defined(Z_RAISE_BEFORE_HOMING) && Z_RAISE_BEFORE_HOMING > 0

        // Raise Z before homing any other axes
        // (Does this need to be "negative home direction?" Why not just use Z_RAISE_BEFORE_HOMING?)
        destination[Z_AXIS] = -Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            SERIAL_ECHOPAIR("Raise Z (before homing) by ", (float)Z_RAISE_BEFORE_HOMING);
            SERIAL_EOL;
            print_xyz("> (home_all_axis || homeZ) > destination", destination);
          }
        #endif
        feedrate = max_feedrate[Z_AXIS] * 60;
        line_to_destination();
        st_synchronize();

      #endif

    } // home_all_axis || homeZ

    #if ENABLED(QUICK_HOME)

      if (home_all_axis || (homeX && homeY)) {  // First diagonal move

        current_position[X_AXIS] = current_position[Y_AXIS] = 0;

        #if ENABLED(DUAL_X_CARRIAGE)
          int x_axis_home_dir = x_home_dir(active_extruder);
          extruder_duplication_enabled = false;
        #else
          int x_axis_home_dir = home_dir(X_AXIS);
        #endif

        sync_plan_position();

        float mlx = max_length(X_AXIS), mly = max_length(Y_AXIS),
              mlratio = mlx > mly ? mly / mlx : mlx / mly;

        destination[X_AXIS] = 1.5 * mlx * x_axis_home_dir;
        destination[Y_AXIS] = 1.5 * mly * home_dir(Y_AXIS);
        feedrate = min(homing_feedrate[X_AXIS], homing_feedrate[Y_AXIS]) * sqrt(mlratio * mlratio + 1);
        line_to_destination();
        st_synchronize();

        set_axis_is_at_home(X_AXIS);
        set_axis_is_at_home(Y_AXIS);
        sync_plan_position();

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            print_xyz("> QUICK_HOME > current_position 1", current_position);
          }
        #endif

        destination[X_AXIS] = current_position[X_AXIS];
        destination[Y_AXIS] = current_position[Y_AXIS];
        line_to_destination();
        feedrate = 0.0;
        st_synchronize();
        endstops_hit_on_purpose(); // clear endstop hit flags

        current_position[X_AXIS] = destination[X_AXIS];
        current_position[Y_AXIS] = destination[Y_AXIS];
        #if DISABLED(SCARA)
          current_position[Z_AXIS] = destination[Z_AXIS];
        #endif

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            print_xyz("> QUICK_HOME > current_position 2", current_position);
          }
        #endif
      }

    #endif // QUICK_HOME

    #if ENABLED(HOME_Y_BEFORE_X)
      // Home Y
      if (home_all_axis || homeY) HOMEAXIS(Y);
    #endif

    // Home X
    if (home_all_axis || homeX) {
      #if ENABLED(DUAL_X_CARRIAGE)
        int tmp_extruder = active_extruder;
        extruder_duplication_enabled = false;
        active_extruder = !active_extruder;
        HOMEAXIS(X);
        inactive_extruder_x_pos = current_position[X_AXIS];
        active_extruder = tmp_extruder;
        HOMEAXIS(X);
        // reset state used by the different modes
        memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
        delayed_move_time = 0;
        active_extruder_parked = true;
      #else
        HOMEAXIS(X);
      #endif
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          print_xyz("> homeX", current_position);
        }
      #endif
    }

    #if DISABLED(HOME_Y_BEFORE_X)
      // Home Y
      if (home_all_axis || homeY) {
        HOMEAXIS(Y);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            print_xyz("> homeY", current_position);
          }
        #endif
      }
    #endif

    // Home Z last if homing towards the bed
    #if Z_HOME_DIR < 0

      if (home_all_axis || homeZ) {

        #if ENABLED(Z_SAFE_HOMING)

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (marlin_debug_flags & DEBUG_LEVELING) {
              SERIAL_ECHOLNPGM("> Z_SAFE_HOMING >>>");
            }
          #endif

          if (home_all_axis) {

            current_position[Z_AXIS] = 0;
            sync_plan_position();

            //
            // Set the Z probe (or just the nozzle) destination to the safe homing point
            //
            // NOTE: If current_position[X_AXIS] or current_position[Y_AXIS] were set above
            // then this may not work as expected.
            destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT - X_PROBE_OFFSET_FROM_EXTRUDER);
            destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT - Y_PROBE_OFFSET_FROM_EXTRUDER);
            destination[Z_AXIS] = -Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS);    // Set destination away from bed
            feedrate = XY_TRAVEL_SPEED;

            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (marlin_debug_flags & DEBUG_LEVELING) {
                SERIAL_ECHOPAIR("Raise Z (before homing) by ", (float)Z_RAISE_BEFORE_HOMING);
                SERIAL_EOL;
                print_xyz("> home_all_axis > current_position", current_position);
                print_xyz("> home_all_axis > destination", destination);
              }
            #endif

            // This could potentially move X, Y, Z all together
            line_to_destination();
            st_synchronize();

            // Set current X, Y is the Z_SAFE_HOMING_POINT minus PROBE_OFFSET_FROM_EXTRUDER
            current_position[X_AXIS] = destination[X_AXIS];
            current_position[Y_AXIS] = destination[Y_AXIS];

            // Home the Z axis
            HOMEAXIS(Z);
          }

          else if (homeZ) { // Don't need to Home Z twice

            // Let's see if X and Y are homed
            if (axis_known_position[X_AXIS] && axis_known_position[Y_AXIS]) {

              // Make sure the Z probe is within the physical limits
              // NOTE: This doesn't necessarily ensure the Z probe is also within the bed!
              float cpx = current_position[X_AXIS], cpy = current_position[Y_AXIS];
              if (   cpx >= X_MIN_POS - X_PROBE_OFFSET_FROM_EXTRUDER
                  && cpx <= X_MAX_POS - X_PROBE_OFFSET_FROM_EXTRUDER
                  && cpy >= Y_MIN_POS - Y_PROBE_OFFSET_FROM_EXTRUDER
                  && cpy <= Y_MAX_POS - Y_PROBE_OFFSET_FROM_EXTRUDER) {
                // Set the plan current position to X, Y, 0
                current_position[Z_AXIS] = 0;
                plan_set_position(cpx, cpy, 0, current_position[E_AXIS]); // = sync_plan_position

                // Set Z destination away from bed and raise the axis
                // NOTE: This should always just be Z_RAISE_BEFORE_HOMING unless...???
                destination[Z_AXIS] = -Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS);
                feedrate = max_feedrate[Z_AXIS] * 60;  // feedrate (mm/m) = max_feedrate (mm/s)

                #if ENABLED(DEBUG_LEVELING_FEATURE)
                  if (marlin_debug_flags & DEBUG_LEVELING) {
                    SERIAL_ECHOPAIR("Raise Z (before homing) by ", (float)Z_RAISE_BEFORE_HOMING);
                    SERIAL_EOL;
                    print_xyz("> homeZ > current_position", current_position);
                    print_xyz("> homeZ > destination", destination);
                  }
                #endif

                line_to_destination();
                st_synchronize();

                // Home the Z axis
                HOMEAXIS(Z);
              }
              else {
                LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
                SERIAL_ECHO_START;
                SERIAL_ECHOLNPGM(MSG_ZPROBE_OUT);
              }
            }
            else {
              LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
              SERIAL_ECHO_START;
              SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
            }

          } // !home_all_axes && homeZ

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (marlin_debug_flags & DEBUG_LEVELING) {
              SERIAL_ECHOLNPGM("<<< Z_SAFE_HOMING");
            }
          #endif

        #else // !Z_SAFE_HOMING

          HOMEAXIS(Z);

        #endif // !Z_SAFE_HOMING

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            print_xyz("> (home_all_axis || homeZ) > final", current_position);
          }
        #endif

      } // home_all_axis || homeZ

    #endif // Z_HOME_DIR < 0

    sync_plan_position();

  #endif // else DELTA

  #if ENABLED(SCARA)
    sync_plan_position_delta();
  #endif

  #if ENABLED(ENDSTOPS_ONLY_FOR_HOMING)
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        SERIAL_ECHOLNPGM("ENDSTOPS_ONLY_FOR_HOMING enable_endstops(false)");
      }
    #endif
    enable_endstops(false);
  #endif

  // For manual leveling move back to 0,0
  #if ENABLED(MESH_BED_LEVELING)
    if (mbl_was_active) {
      current_position[X_AXIS] = mbl.get_x(0);
      current_position[Y_AXIS] = mbl.get_y(0);
      set_destination_to_current();
      feedrate = homing_feedrate[X_AXIS];
      line_to_destination();
      st_synchronize();
      current_position[Z_AXIS] = MESH_HOME_SEARCH_Z;
      sync_plan_position();
      mbl.active = 1;
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          print_xyz("mbl_was_active > current_position", current_position);
        }
      #endif
    }
  #endif

  clean_up_after_endstop_move();

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (marlin_debug_flags & DEBUG_LEVELING) {
      SERIAL_ECHOLNPGM("<<< gcode_G28");
    }
  #endif

}
