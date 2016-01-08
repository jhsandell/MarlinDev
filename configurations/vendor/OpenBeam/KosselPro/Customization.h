// Example configuration file for OpenBeam Kossel Pro
// tested on 2015-05-19 by @Wackerbarth
// using Arduino 1.6.5 (Mac)

// FEATURES
#define AUTO_BED_LEVELING_FEATURE

// Based on the 1.1 MOTHERBOARD descriptors
#define MOTHERBOARD BOARD_BRAINWAVE_PRO

// Optional custom name displayed in the LCD "Ready" message
#define CUSTOM_MACHINE_NAME "Kossel Pro"

// 5 is 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (4.7k pullup)
#define TEMP_SENSOR_0 5
#define TEMP_SENSOR_BED 5

//===========================================================================
//============================= PID > Temperature Control ===============
//===========================================================================
// Limits current to nozzle while PID is active; 255=full current
#define PID_MAX  125
#ifndef PID_FUNCTIONAL_RANGE
  #define PID_FUNCTIONAL_RANGE 50
#endif
// Kossel Pro
#define  DEFAULT_Kp 19.30
#define  DEFAULT_Ki 3.51
#define  DEFAULT_Kd 26.56

//===========================================================================
//============================= PID > Bed Temperature Control ===============
//===========================================================================
#define PIDTEMPBED

//Kossel Pro heated bed plate with borosilicate glass
//from pidautotune (M303 E-1 S60 C8)
#define  DEFAULT_bedKp 370.25
#define  DEFAULT_bedKi 62.77
#define  DEFAULT_bedKd 545.98

//===========================================================================
//============================= Mechanical ==================================
//===========================================================================
// Make delta curves from many straight lines (linear interpolation).
// This is a trade-off between visible corners (not enough segments)
// and processor overload (too many expensive sqrt calls).
#define DELTA_SEGMENTS_PER_SECOND 160

// Center-to-center distance of the holes in the diagonal push rods.
#define DELTA_DIAGONAL_ROD 301.0 // mm

// Horizontal offset from middle of printer to smooth rod center.
#define DELTA_SMOOTH_ROD_OFFSET 212.357 // mm

// Horizontal offset of the universal joints on the end effector.
#define DELTA_EFFECTOR_OFFSET 30.0 // mm

// Horizontal offset of the universal joints on the carriages.
#define DELTA_CARRIAGE_OFFSET 30.0 // mm

// Print surface diameter/2 minus unreachable space (avoid collisions with vertical towers).
#define DELTA_PRINTABLE_RADIUS 127

// Invert the stepper direction. Change (or reverse the motor connector) if an axis goes the wrong way.
#define INVERT_X_DIR true
#define INVERT_Y_DIR true
#define INVERT_Z_DIR true

// For direct drive extruder
#define INVERT_E0_DIR true

#define USE_PROBE

//===========================================================================
//========================== Bed Tilt Compensation ==========================
//===========================================================================

#ifndef DELTA_PROBEABLE_RADIUS
  #define DELTA_PROBEABLE_RADIUS (DELTA_PRINTABLE_RADIUS-25)
#endif
#ifndef AUTO_BED_LEVELING_GRID_POINTS
  #define AUTO_BED_LEVELING_GRID_POINTS 7
#endif

// Offsets to the Z probe relative to the nozzle tip.
// X and Y offsets must be integers.
#define X_PROBE_OFFSET_FROM_EXTRUDER -23 // KosselPro actual: -22.919
#define Y_PROBE_OFFSET_FROM_EXTRUDER -6  // KosselPro actual: -6.304
// Kossel Pro note: The correct value is likely -17.45 but I'd rather err on the side of
// not giving someone a head crash. Use something like G29 Z-0.2 to adjust as needed.
#ifndef Z_PROBE_OFFSET_FROM_EXTRUDER
  #define Z_PROBE_OFFSET_FROM_EXTRUDER -17.25  // Increase this if the first layer is too thin (remember: it's a negative number so increase means closer to zero).
#endif

#ifndef Z_RAISE_BEFORE_HOMING
  #define Z_RAISE_BEFORE_HOMING 4       // (in mm) Raise Z axis before homing (G28) for Z probe clearance.
#endif

#define XY_TRAVEL_SPEED 8000            // X and Y axis travel speed between probes, in mm/min.

#ifndef Z_RAISE_BEFORE_PROBING
  #define Z_RAISE_BEFORE_PROBING 40   // How much the Z axis will be raised before traveling to the first probing point.
#endif

#ifndef Z_RAISE_BETWEEN_PROBINGS
  #define Z_RAISE_BETWEEN_PROBINGS 15   // How much the Z axis will be raised when traveling from between next probing points.
#endif
#ifndef Z_RAISE_AFTER_PROBING
  #define Z_RAISE_AFTER_PROBING 15    // How much the Z axis will be raised after the last probing point.
#endif

// Kossel Pro Allen key retractable z-probe
#define Z_PROBE_ALLEN_KEY_DEPLOY_1_X -105.00 // Move left but not quite so far that we'll bump the belt
#define Z_PROBE_ALLEN_KEY_DEPLOY_1_Y 0.00
#define Z_PROBE_ALLEN_KEY_DEPLOY_1_Z 100.0
#define Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE HOMING_FEEDRATE_XYZ
#define Z_PROBE_ALLEN_KEY_DEPLOY_2_X -110.00 // Move outward to position deploy pin to the left of the arm
#define Z_PROBE_ALLEN_KEY_DEPLOY_2_Y -125.00
#define Z_PROBE_ALLEN_KEY_DEPLOY_2_Z Z_PROBE_ALLEN_KEY_DEPLOY_1_Z
#define Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE HOMING_FEEDRATE_XYZ
#define Z_PROBE_ALLEN_KEY_DEPLOY_3_X 45.00 // Move right to trigger deploy pin
#define Z_PROBE_ALLEN_KEY_DEPLOY_3_Y Z_PROBE_ALLEN_KEY_DEPLOY_2_Y
#define Z_PROBE_ALLEN_KEY_DEPLOY_3_Z Z_PROBE_ALLEN_KEY_DEPLOY_2_Z
#define Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE (HOMING_FEEDRATE_XYZ/2)

 // Line up with bed retaining clip
#ifndef Z_PROBE_ALLEN_KEY_STOW_1_X
  #define Z_PROBE_ALLEN_KEY_STOW_1_X 34.00
#endif
#ifndef Z_PROBE_ALLEN_KEY_STOW_1_Y
  #define Z_PROBE_ALLEN_KEY_STOW_1_Y -123.00
#endif
#define Z_PROBE_ALLEN_KEY_STOW_1_Z 75.0
#define Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE HOMING_FEEDRATE_XYZ
#define Z_PROBE_ALLEN_KEY_STOW_2_X Z_PROBE_ALLEN_KEY_STOW_1_X // move down to retract probe
#define Z_PROBE_ALLEN_KEY_STOW_2_Y Z_PROBE_ALLEN_KEY_STOW_1_Y
#ifndef Z_PROBE_ALLEN_KEY_STOW_2_Z
  #define Z_PROBE_ALLEN_KEY_STOW_2_Z 0.0
#endif
#define Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE (HOMING_FEEDRATE_XYZ/2)
#define Z_PROBE_ALLEN_KEY_STOW_3_X 0.0  // return to 0,0,100
#define Z_PROBE_ALLEN_KEY_STOW_3_Y 0.0
#define Z_PROBE_ALLEN_KEY_STOW_3_Z 100.0
#define Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE HOMING_FEEDRATE_XYZ

#ifndef MANUAL_Z_HOME_POS
  #define MANUAL_Z_HOME_POS 277 // For delta: Distance between nozzle and print surface after homing.
#endif

#define XYZ_FULL_STEPS_PER_ROTATION 200
#define XYZ_MICROSTEPS 32
#define XYZ_BELT_PITCH 2
#define XYZ_PULLEY_TEETH 20
#define XYZ_STEPS (XYZ_FULL_STEPS_PER_ROTATION * XYZ_MICROSTEPS / double(XYZ_BELT_PITCH) / double(XYZ_PULLEY_TEETH))

#define DEFAULT_AXIS_STEPS_PER_UNIT   {XYZ_STEPS, XYZ_STEPS, XYZ_STEPS, 184.8}
#define DEFAULT_MAX_FEEDRATE          {200, 200, 200, 200}    // (mm/sec)
#define DEFAULT_MAX_ACCELERATION      {9000,9000,9000,9000}    // X, Y, Z, E maximum start speed for accelerated moves.

#define Z_PROBE_OFFSET_RANGE_MIN -15
#define Z_PROBE_OFFSET_RANGE_MAX -5

#define SDSUPPORT

#define EEPROM_SETTINGS

#include INCLUDE_BY_CLASS_AND_ID(design,kossel)
