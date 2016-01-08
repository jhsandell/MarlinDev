// Example configuration file for OpenBeam Kossel Pro
// tested on 2015-05-19 by @Wackerbarth
// using Arduino 1.6.5 (Mac)

// Based on the 1.1 MOTHERBOARD descriptors
#define MOTHERBOARD BOARD_RAMPS_13_EFB

// Optional custom name displayed in the LCD "Ready" message
#define CUSTOM_MACHINE_NAME "Mini Kossel"

// 7 is 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)
// 11 is 100k beta 3950 1% thermistor (4.7k pullup)
#define TEMP_SENSOR_0 7
#define TEMP_SENSOR_BED 11

#define BANG_MAX 255 // limits current to nozzle while in bang-bang mode; 255=full current
#define PID_MAX BANG_MAX // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
#define PID_FUNCTIONAL_RANGE 10 // If the temperature difference between the target temperature and the actual temperature

// Ultimaker
#define  DEFAULT_Kp 22.2
#define  DEFAULT_Ki 1.08
#define  DEFAULT_Kd 114

#define DELTA_SEGMENTS_PER_SECOND 200

// Center-to-center distance of the holes in the diagonal push rods.
#define DELTA_DIAGONAL_ROD 215.0 // mm

// Horizontal offset from middle of printer to smooth rod center.
#define DELTA_SMOOTH_ROD_OFFSET 145.0 // mm

// Horizontal offset of the universal joints on the end effector.
#define DELTA_EFFECTOR_OFFSET 19.9 // mm

// Horizontal offset of the universal joints on the carriages.
#define DELTA_CARRIAGE_OFFSET 19.5 // mm

// Print surface diameter/2 minus unreachable space (avoid collisions with vertical towers).
#define DELTA_PRINTABLE_RADIUS 90

#define INVERT_X_DIR false // DELTA does not invert
#define INVERT_Y_DIR false
#define INVERT_Z_DIR false

// For geared extruder set to false.
#define INVERT_E0_DIR false

//===========================================================================
//============================ Bed Leveling =================================
//===========================================================================

#define DELTA_PROBEABLE_RADIUS (DELTA_PRINTABLE_RADIUS - 10)
#define AUTO_BED_LEVELING_GRID_POINTS 9

// Offsets to the Z probe relative to the nozzle tip.
// X and Y offsets must be integers.
#define X_PROBE_OFFSET_FROM_EXTRUDER 0     // Z probe to nozzle X offset: -left  +right
#define Y_PROBE_OFFSET_FROM_EXTRUDER -10   // Z probe to nozzle Y offset: -front +behind
#define Z_PROBE_OFFSET_FROM_EXTRUDER -3.5  // Z probe to nozzle Z offset: -below (always!)

#define Z_RAISE_BEFORE_HOMING 15      // (in mm) Raise Z axis before homing (G28) for Z probe clearance.

#define XY_TRAVEL_SPEED 4000         // X and Y axis travel speed between probes, in mm/min.

#define Z_RAISE_BEFORE_PROBING 15   // How much the Z axis will be raised before traveling to the first probing point.
#define Z_RAISE_BETWEEN_PROBINGS 5  // How much the Z axis will be raised when traveling from between next probing points
#define Z_RAISE_AFTER_PROBING 50    // How much the Z axis will be raised after the last probing point.

// Kossel Pro Allen key retractable z-probe
#define Z_PROBE_ALLEN_KEY_DEPLOY_1_X 30.0
#define Z_PROBE_ALLEN_KEY_DEPLOY_1_Y DELTA_PRINTABLE_RADIUS
#define Z_PROBE_ALLEN_KEY_DEPLOY_1_Z 100.0
#define Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE XY_TRAVEL_SPEED
#define Z_PROBE_ALLEN_KEY_DEPLOY_2_X 0.0
#define Z_PROBE_ALLEN_KEY_DEPLOY_2_Y DELTA_PRINTABLE_RADIUS
#define Z_PROBE_ALLEN_KEY_DEPLOY_2_Z 100.0
#define Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE (XY_TRAVEL_SPEED/10)

#define Z_PROBE_ALLEN_KEY_STOW_DEPTH 20
// Move the probe into position
#define Z_PROBE_ALLEN_KEY_STOW_1_X -64.0
#define Z_PROBE_ALLEN_KEY_STOW_1_Y 56.0
#define Z_PROBE_ALLEN_KEY_STOW_1_Z 23.0
#define Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE XY_TRAVEL_SPEED
// Move the nozzle down further to push the probe into retracted position.
#define Z_PROBE_ALLEN_KEY_STOW_2_X  Z_PROBE_ALLEN_KEY_STOW_1_X
#define Z_PROBE_ALLEN_KEY_STOW_2_Y  Z_PROBE_ALLEN_KEY_STOW_1_Y
#define Z_PROBE_ALLEN_KEY_STOW_2_Z  (Z_PROBE_ALLEN_KEY_STOW_1_Z-Z_PROBE_ALLEN_KEY_STOW_DEPTH)
#define Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE (XY_TRAVEL_SPEED/10)
// Raise things back up slightly so we don't bump into anything
#define Z_PROBE_ALLEN_KEY_STOW_3_X  Z_PROBE_ALLEN_KEY_STOW_2_X
#define Z_PROBE_ALLEN_KEY_STOW_3_Y  Z_PROBE_ALLEN_KEY_STOW_2_Y
#define Z_PROBE_ALLEN_KEY_STOW_3_Z  (Z_PROBE_ALLEN_KEY_STOW_1_Z+Z_PROBE_ALLEN_KEY_STOW_DEPTH)
#define Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE (XY_TRAVEL_SPEED/2)

#define MANUAL_Z_HOME_POS 250 // For delta: Distance between nozzle and print surface after homing.

#define DEFAULT_AXIS_STEPS_PER_UNIT   {80, 80, 80, 760*1.1}  // default steps per unit for Kossel (GT2, 20 tooth)
#define DEFAULT_MAX_FEEDRATE          {500, 500, 500, 25}    // (mm/sec)
#define DEFAULT_MAX_ACCELERATION      {9000,9000,9000,10000}    // X, Y, Z, E maximum start speed for accelerated moves.
#define Z_PROBE_OFFSET_RANGE_MIN -20
#define Z_PROBE_OFFSET_RANGE_MAX 20

// PanelOne from T3P3 (via RAMPS 1.4 AUX2/AUX3)
// http://reprap.org/wiki/PanelOne
#define PANEL_ONE

#include INCLUDE_BY_CLASS_AND_ID(design,kossel)
