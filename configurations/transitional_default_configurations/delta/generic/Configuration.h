#include "configurations/design/default/Configuration.h"

// This determines the communication speed of the printer
// :[2400,9600,19200,38400,57600,115200,250000]
#ifndef BAUDRATE 
  #define BAUDRATE 115200
#endif

// Optional custom name for your RepStrap or other custom machine
// Displayed in the LCD "Ready" message
#ifndef CUSTOM_MACHINE_NAME
  #define CUSTOM_MACHINE_NAME "Deltabot"
#endif

//===========================================================================
//============================== Delta Settings =============================
//===========================================================================
// Enable DELTA kinematics and most of the default configuration for Deltas
#define DELTA
#ifndef KINEMATICS_INCLUDE
  #define KINEMATICS_INCLUDE GENERATE_KINEMATICS_INCLUDE(delta)
#endif

// Make delta curves from many straight lines (linear interpolation).
// This is a trade-off between visible corners (not enough segments)
// and processor overload (too many expensive sqrt calls).
#ifndef DELTA_SEGMENTS_PER_SECOND
  #define DELTA_SEGMENTS_PER_SECOND 200
#endif

// NOTE NB all values for DELTA_* values MUST be floating point, so always have a decimal point in them

// Center-to-center distance of the holes in the diagonal push rods.
#ifndef DELTA_DIAGONAL_ROD
  #define DELTA_DIAGONAL_ROD 250.0 // mm
#endif

// Horizontal offset from middle of printer to smooth rod center.
#ifndef DELTA_SMOOTH_ROD_OFFSET
  #define DELTA_SMOOTH_ROD_OFFSET 175.0 // mm
#endif

// Horizontal offset of the universal joints on the end effector.
#ifndef DELTA_EFFECTOR_OFFSET
  #define DELTA_EFFECTOR_OFFSET 33.0 // mm
#endif

// Horizontal offset of the universal joints on the carriages.
#ifndef DELTA_CARRIAGE_OFFSET
  #define DELTA_CARRIAGE_OFFSET 18.0 // mm
#endif

// Horizontal distance bridged by diagonal push rods when effector is centered.
#ifndef DELTA_RADIUS
  #define DELTA_RADIUS (DELTA_SMOOTH_ROD_OFFSET-DELTA_EFFECTOR_OFFSET-DELTA_CARRIAGE_OFFSET)
#endif

// Print surface diameter/2 minus unreachable space (avoid collisions with vertical towers).
#ifndef DELTA_PRINTABLE_RADIUS
  #define DELTA_PRINTABLE_RADIUS 140
#endif

#ifndef INVERT_Y_DIR
  #define INVERT_Y_DIR false
#endif

// Sets direction of endstops when homing; 1=MAX, -1=MIN
// :[-1,1]
#define X_HOME_DIR 1  // deltas always home to max
#define Y_HOME_DIR 1
#define Z_HOME_DIR 1

// Travel limits after homing (units are in mm)
#define X_MIN_POS (-DELTA_PRINTABLE_RADIUS)
#define Y_MIN_POS (-DELTA_PRINTABLE_RADIUS)
#define X_MAX_POS (DELTA_PRINTABLE_RADIUS)
#define Y_MAX_POS (DELTA_PRINTABLE_RADIUS)
#define Z_MAX_POS (MANUAL_Z_HOME_POS)

//#define AUTO_BED_LEVELING_FEATURE // Delete the comment to enable (remove // at the start of the line)
//#define DEBUG_LEVELING_FEATURE

#if ENABLED(AUTO_BED_LEVELING_FEATURE)

  // Enable this to sample the bed in a grid (least squares solution).
  // Note: this feature generates 10KB extra code size.
  #define AUTO_BED_LEVELING_GRID

  // Non-linear bed leveling will be used.
  // Compensate by interpolating between the nearest four Z probe values for each point.
  // Useful for deltas where the print surface may appear like a bowl or dome shape.
  // Works best with ACCURATE_BED_LEVELING_POINTS 5 or higher.
  #ifndef AUTO_BED_LEVELING_GRID_POINTS
    #define AUTO_BED_LEVELING_GRID_POINTS 9
  #endif

  // Offsets to the Z probe relative to the nozzle tip.
  // X and Y offsets must be integers.
  #ifndef X_PROBE_OFFSET_FROM_EXTRUDER
    #define X_PROBE_OFFSET_FROM_EXTRUDER 0     // Z probe to nozzle X offset: -left  +right
  #endif
  #ifndef Y_PROBE_OFFSET_FROM_EXTRUDER
    #define Y_PROBE_OFFSET_FROM_EXTRUDER -10   // Z probe to nozzle Y offset: -front +behind
  #endif
  #ifndef Z_PROBE_OFFSET_FROM_EXTRUDER
    #define Z_PROBE_OFFSET_FROM_EXTRUDER -3.5  // Z probe to nozzle Z offset: -below (always!)
  #endif

  #ifndef XY_TRAVEL_SPEED
    #define XY_TRAVEL_SPEED 4000         // X and Y axis travel speed between probes, in mm/min.
  #endif

  #ifndef Z_RAISE_AFTER_PROBING
    #define Z_RAISE_AFTER_PROBING 50    // How much the Z axis will be raised after the last probing point.
  #endif

  // Allen key retractable z-probe as seen on many Kossel delta printers - http://reprap.org/wiki/Kossel#Automatic_bed_leveling_probe
  // Deploys by touching z-axis belt. Retracts by pushing the probe down. Uses Z_MIN_PIN.
  //#define Z_PROBE_ALLEN_KEY

  #if ENABLED(Z_PROBE_ALLEN_KEY)
    // 2 or 3 sets of coordinates for deploying and retracting the spring loaded touch probe on G29,
    // if servo actuated touch probe is not defined. Uncomment as appropriate for your printer/probe.

    #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_1_X
      #define Z_PROBE_ALLEN_KEY_DEPLOY_1_X 30.0
    #endif
    #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_1_Y
      #define Z_PROBE_ALLEN_KEY_DEPLOY_1_Y DELTA_PRINTABLE_RADIUS
    #endif
    #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_1_Z
      #define Z_PROBE_ALLEN_KEY_DEPLOY_1_Z 100.0
    #endif
    #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE
      #define Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE HOMING_FEEDRATE_XYZ
    #endif
    #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_2_X
      #define Z_PROBE_ALLEN_KEY_DEPLOY_2_X 0.0
    #endif
    #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_2_Y
      #define Z_PROBE_ALLEN_KEY_DEPLOY_2_Y Z_PROBE_ALLEN_KEY_DEPLOY_1_Y
    #endif
    #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_2_Z
      #define Z_PROBE_ALLEN_KEY_DEPLOY_2_Z Z_PROBE_ALLEN_KEY_DEPLOY_1_Z
    #endif
    #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE
      #define Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE (HOMING_FEEDRATE_XYZ/10)
    #endif

    #ifndef Z_PROBE_ALLEN_KEY_STOW_1_X
      // Move the probe into position
      #define Z_PROBE_ALLEN_KEY_STOW_1_X -64.0
    #endif
    #ifndef Z_PROBE_ALLEN_KEY_STOW_1_Y
      #define Z_PROBE_ALLEN_KEY_STOW_1_Y 56.0
    #endif
    #ifndef Z_PROBE_ALLEN_KEY_STOW_1_Z
      #define Z_PROBE_ALLEN_KEY_STOW_1_Z 23.0
    #endif
    #ifndef Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE
      #define Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE HOMING_FEEDRATE_XYZ
    #endif
    #ifndef Z_PROBE_ALLEN_KEY_STOW_2_X
      // Push it down
      #define Z_PROBE_ALLEN_KEY_STOW_2_X Z_PROBE_ALLEN_KEY_STOW_1_X
    #endif
    #ifndef Z_PROBE_ALLEN_KEY_STOW_2_Y
      #define Z_PROBE_ALLEN_KEY_STOW_2_Y Z_PROBE_ALLEN_KEY_STOW_1_Y
    #endif
    #ifndef Z_PROBE_ALLEN_KEY_STOW_2_Z
      #define Z_PROBE_ALLEN_KEY_STOW_2_Z 3.0
    #endif
    #ifndef Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE
      #define Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE (HOMING_FEEDRATE_XYZ/10)
    #endif
    #ifndef Z_PROBE_ALLEN_KEY_STOW_3_X
      // Move it up to clear
      #define Z_PROBE_ALLEN_KEY_STOW_3_X Z_PROBE_ALLEN_KEY_STOW_2_X
    #endif
    #ifndef Z_PROBE_ALLEN_KEY_STOW_3_Y
      #define Z_PROBE_ALLEN_KEY_STOW_3_Y Z_PROBE_ALLEN_KEY_STOW_2_Y
    #endif
    #ifndef Z_PROBE_ALLEN_KEY_STOW_3_Z
      #define Z_PROBE_ALLEN_KEY_STOW_3_Z 50.0
    #endif
    #ifndef Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE
      #define Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE HOMING_FEEDRATE_XYZ
    #endif
  #endif

#define Z_MIN_PROBE_ENDSTOP
#endif
#define DISABLE_ZMIN_ENDSTOP


// @section homing

// The position of the homing switches
#define MANUAL_HOME_POSITIONS
// For deltabots this means top and center of the Cartesian print volume.
#ifndef MANUAL_Z_HOME_POS
  #define MANUAL_Z_HOME_POS 250
#endif

// delta homing speeds must be the same on xyz
#ifndef HOMING_FEEDRATE_XYZ
  #define HOMING_FEEDRATE_XYZ (200*60)
#endif
#define HOMING_FEEDRATE_E 0
#ifndef HOMING_FEEDRATE
  #define HOMING_FEEDRATE { HOMING_FEEDRATE_XYZ, HOMING_FEEDRATE_XYZ, HOMING_FEEDRATE_XYZ, HOMING_FEEDRATE_E }
#endif

// default settings
// delta speeds must be the same on xyz
#ifndef DEFAULT_AXIS_STEPS_PER_UNIT
  #define DEFAULT_AXIS_STEPS_PER_UNIT   {80, 80, 80, 760*1.1}  // default steps per unit for Kossel (GT2, 20 tooth)
#endif
#ifndef DEFAULT_MAX_FEEDRATE
  #define DEFAULT_MAX_FEEDRATE          {500, 500, 500, 25}    // (mm/sec)
#endif
#ifndef DEFAULT_MAX_ACCELERATION
  #define DEFAULT_MAX_ACCELERATION      {9000,9000,9000,10000}    // X, Y, Z, E maximum start speed for accelerated moves.
#endif

#define DEFAULT_ZJERK                 DEFAULT_XYJERK

#ifndef PLA_PREHEAT_FAN_SPEED
  #define PLA_PREHEAT_FAN_SPEED 255
#endif

#ifndef ABS_PREHEAT_HPB_TEMP
  #define ABS_PREHEAT_HPB_TEMP 100
#endif
#ifndef ABS_PREHEAT_FAN_SPEED
  #define ABS_PREHEAT_FAN_SPEED 255
#endif

// Delta calibration menu
// uncomment to add three points calibration menu option.
// See http://minow.blogspot.com/index.html#4918805519571907051
// If needed, adjust the X, Y, Z calibration coordinates
// in ultralcd.cpp@lcd_delta_calibrate_menu()
//#define DELTA_CALIBRATION_MENU

#define X_HOME_BUMP_MM 5
#define Y_HOME_BUMP_MM X_HOME_BUMP_MM
#define Z_HOME_BUMP_MM X_HOME_BUMP_MM
#ifndef HOMING_BUMP_DIVISOR
  #define HOMING_BUMP_DIVISOR {10, 10, 10}  // Re-Bump Speed Divisor (Divides the Homing Feedrate)
#endif

#if ENABLED(ULTIPANEL)
  #define MANUAL_FEEDRATE_XYZ 50*60
  #define MANUAL_FEEDRATE { MANUAL_FEEDRATE_XYZ, MANUAL_FEEDRATE_XYZ, MANUAL_FEEDRATE_XYZ, 60 } // Feedrates for manual moves along X, Y, Z, E from panel
#endif

// Deltas don't need slowdown
#define SLOWDOWN 0

#include "configurations/transitional_default_configurations/default/Configuration.h"
