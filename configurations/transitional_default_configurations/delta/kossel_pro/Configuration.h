// Example configuration file for OpenBeam Kossel Pro
// tested on 2015-05-19 by @Wackerbarth
// using Arduino 1.6.6 (Mac)

// FEATURES
#define AUTO_BED_LEVELING_FEATURE

//===========================================================================
#include "boards.h"

#ifndef MOTHERBOARD
  #define MOTHERBOARD BOARD_BRAINWAVE_PRO
#endif

// Optional custom name for your machine
// Displayed in the LCD "Ready" message
#define CUSTOM_MACHINE_NAME "Kossel Pro"

//===========================================================================
//============================= Thermal Settings ============================
//===========================================================================
// 0 is unused
// 5 is 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (4.7k pullup)
#define TEMP_SENSOR_0 5
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_BED 5

//===========================================================================
//============================= PID Settings ================================
//===========================================================================
#define PID_MAX  125            // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
#ifndef PID_FUNCTIONAL_RANGE
  #define PID_FUNCTIONAL_RANGE 50 // If the temperature difference between the target temperature and the actual temperature
                                // is more then PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.
#endif

// Kossel Pro
#ifdef DEFAULT_Kp
  #define DEFAULT_Kp 19.30
  #define DEFAULT_Ki 3.51
  #define DEFAULT_Kd 26.56
#endif

//===========================================================================
//============================= PID > Bed Temperature Control ===============
//===========================================================================
#define PIDTEMPBED

#define PID_BED_INTEGRAL_DRIVE_MAX MAX_BED_POWER //limit for the integral term

//Kossel Pro heated bed plate with borosilicate glass
//from pidautotune (M303 E-1 S60 C8)
#ifndef DEFAULT_bedKp
  #define DEFAULT_bedKp 370.25
  #define DEFAULT_bedKi 62.77
  #define DEFAULT_bedKd 545.98
#endif

//===========================================================================
//============================== Delta Settings =============================
//===========================================================================
#define DELTA_SEGMENTS_PER_SECOND 160

// Center-to-center distance of the holes in the diagonal push rods.
#ifndef DELTA_DIAGONAL_ROD
  #define DELTA_DIAGONAL_ROD 301.0 // mm
#endif

// Horizontal offset from middle of printer to smooth rod center.
#ifndef DELTA_SMOOTH_ROD_OFFSET
  #define DELTA_SMOOTH_ROD_OFFSET 212.357 // mm
#endif

// Horizontal offset of the universal joints on the end effector.
#ifndef DELTA_EFFECTOR_OFFSET
  #define DELTA_EFFECTOR_OFFSET 30.0 // mm
#endif

// Horizontal offset of the universal joints on the carriages.
#ifndef DELTA_CARRIAGE_OFFSET
  #define DELTA_CARRIAGE_OFFSET 30.0 // mm
#endif

// Print surface diameter/2 minus unreachable space (avoid collisions with vertical towers).
#ifndef DELTA_PRINTABLE_RADIUS
  #define DELTA_PRINTABLE_RADIUS 127
#endif

// @section machine

// Invert the stepper direction. Change (or reverse the motor connector) if an axis goes the wrong way.
#define INVERT_X_DIR true
#define INVERT_Y_DIR true
#define INVERT_Z_DIR true

// @section extruder

// For direct drive extruder v9 set to true, for geared extruder set to false.
#define INVERT_E0_DIR true

//===========================================================================
//========================== Bed Tilt Compensation ==========================
//===========================================================================

// @section bedtilt

//#define Z_MIN_PROBE_REPEATABILITY_TEST  // If not commented out, Z-Probe Repeatability test will be included if Auto Bed Leveling is Enabled.

// set the rectangle in which to probe
#ifndef DELTA_PROBEABLE_RADIUS
  #define DELTA_PROBEABLE_RADIUS (DELTA_PRINTABLE_RADIUS-25)
#endif

// Non-linear bed leveling will be used.
// Compensate by interpolating between the nearest four Z probe values for each point.
// Useful for deltas where the print surface may appear like a bowl or dome shape.
// Works best with ACCURATE_BED_LEVELING_POINTS 5 or higher.
#define AUTO_BED_LEVELING_GRID_POINTS 7

// Offsets to the Z probe relative to the nozzle tip.
// X and Y offsets must be integers.
#define X_PROBE_OFFSET_FROM_EXTRUDER -23 // KosselPro actual: -22.919
#define Y_PROBE_OFFSET_FROM_EXTRUDER -6  // KosselPro actual: -6.304
// Kossel Pro note: The correct value is likely -17.45 but I'd rather err on the side of
// not giving someone a head crash. Use something like G29 Z-0.2 to adjust as needed.
#ifndef Z_PROBE_OFFSET_FROM_EXTRUDER
  #define Z_PROBE_OFFSET_FROM_EXTRUDER -17.25  // Increase this if the first layer is too thin (remember: it's a negative number so increase means closer to zero).
#endif


#define XY_TRAVEL_SPEED 8000         // X and Y axis travel speed between probes, in mm/min.

#define Z_RAISE_BEFORE_PROBING 100    // How much the Z axis will be raised before traveling to the first probing point.
#ifndef Z_RAISE_BETWEEN_PROBINGS
  #define Z_RAISE_BETWEEN_PROBINGS 5  // How much the Z axis will be raised when traveling from between next probing points.
#endif

#ifndef Z_RAISE_AFTER_PROBING
  #define Z_RAISE_AFTER_PROBING 40    // How much the Z axis will be raised after the last probing point.
#endif

// Allen key retractable z-probe as seen on many Kossel delta printers - http://reprap.org/wiki/Kossel#Automatic_bed_leveling_probe
// Deploys by touching z-axis belt. Retracts by pushing the probe down. Uses Z_MIN_PIN.
#define Z_PROBE_ALLEN_KEY

// Kossel Pro
#define Z_PROBE_ALLEN_KEY_DEPLOY_1_X -105.00 // Move left but not quite so far that we'll bump the belt
#define Z_PROBE_ALLEN_KEY_DEPLOY_1_Y 0.00
#define Z_PROBE_ALLEN_KEY_DEPLOY_1_Z 100.0
//define Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE HOMING_FEEDRATE_XYZ
#define Z_PROBE_ALLEN_KEY_DEPLOY_2_X -105.00 // Move outward to position deploy pin to the left of the arm
#define Z_PROBE_ALLEN_KEY_DEPLOY_2_Y -125.00
//#define Z_PROBE_ALLEN_KEY_DEPLOY_2_Z Z_PROBE_ALLEN_KEY_DEPLOY_1_Z
#define Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE HOMING_FEEDRATE_XYZ
#define Z_PROBE_ALLEN_KEY_DEPLOY_3_X 45.00 // Move right to trigger deploy pin
#define Z_PROBE_ALLEN_KEY_DEPLOY_3_Y Z_PROBE_ALLEN_KEY_DEPLOY_2_Y
#define Z_PROBE_ALLEN_KEY_DEPLOY_3_Z Z_PROBE_ALLEN_KEY_DEPLOY_2_Z
#define Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE (HOMING_FEEDRATE_XYZ/2)

#ifndef Z_PROBE_ALLEN_KEY_STOW_1_X
  #define Z_PROBE_ALLEN_KEY_STOW_1_X 36.10 // Line up with bed retaining clip
#endif
#ifndef Z_PROBE_ALLEN_KEY_STOW_1_Y
  #define Z_PROBE_ALLEN_KEY_STOW_1_Y -124.30
#endif
#define Z_PROBE_ALLEN_KEY_STOW_1_Z 75.0
//#define Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE HOMING_FEEDRATE_XYZ
//#define Z_PROBE_ALLEN_KEY_STOW_2_X Z_PROBE_ALLEN_KEY_STOW_1_X // move down to retract probe
//#define Z_PROBE_ALLEN_KEY_STOW_2_Y Z_PROBE_ALLEN_KEY_STOW_1_Y
#ifndef Z_PROBE_ALLEN_KEY_STOW_2_Z
  #define Z_PROBE_ALLEN_KEY_STOW_2_Z 4.0
#endif
#define Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE (HOMING_FEEDRATE_XYZ/2)
#define Z_PROBE_ALLEN_KEY_STOW_3_X 0.0  // return to 0,0,100
#define Z_PROBE_ALLEN_KEY_STOW_3_Y 0.0
#define Z_PROBE_ALLEN_KEY_STOW_3_Z 100.0
//#define Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE HOMING_FEEDRATE_XYZ

// @section homing

// The position of the homing switches
#define MANUAL_HOME_POSITIONS  // If defined, MANUAL_*_HOME_POS below will be used
#define BED_CENTER_AT_0_0  // If defined, the center of the bed is at (X=0, Y=0)
#ifndef MANUAL_Z_HOME_POS
  #define MANUAL_Z_HOME_POS 277 // For delta: Distance between nozzle and print surface after homing.
#endif

// @section movement

#define XYZ_FULL_STEPS_PER_ROTATION 200
#define XYZ_MICROSTEPS 32
#define XYZ_BELT_PITCH 2
#define XYZ_PULLEY_TEETH 20
#define XYZ_STEPS (XYZ_FULL_STEPS_PER_ROTATION * XYZ_MICROSTEPS / double(XYZ_BELT_PITCH) / double(XYZ_PULLEY_TEETH))

// delta speeds must be the same on xyz
#define DEFAULT_AXIS_STEPS_PER_UNIT   {XYZ_STEPS, XYZ_STEPS, XYZ_STEPS, 184.8}
#define DEFAULT_MAX_FEEDRATE          {200, 200, 200, 200}    // (mm/sec)
#define DEFAULT_MAX_ACCELERATION      {9000,9000,9000,9000}    // X, Y, Z, E maximum start speed for accelerated moves.

// Custom M code points
#define Z_PROBE_OFFSET_RANGE_MIN -15
#define Z_PROBE_OFFSET_RANGE_MAX -5

#define SDSUPPORT // Enable SD Card Support in Hardware Console

#include "configurations/transitional_default_configurations/delta/generic/Configuration.h"
