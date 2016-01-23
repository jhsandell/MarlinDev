// Sources : MarlinDev/configurations/transitional_default_configurations/delta/generic
//           Ma Configuration.h de Marlin 1.0.0 RC3

#include "configurations/design/default/Configuration.h"

// This determines the communication speed of the printer
// :[2400,9600,19200,38400,57600,115200,250000]
#define BAUDRATE 250000


// Optional custom name for your RepStrap or other custom machine
// Displayed in the LCD "Ready" message
#define CUSTOM_MACHINE_NAME "DeltaFold"

#define MOTHERBOARD BOARD_RAMPS_13_EFB

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
  #define DELTA_SEGMENTS_PER_SECOND 75
#endif

// NOTE NB all values for DELTA_* values MUST be floating point, so always have a decimal point in them

// Center-to-center distance of the holes in the diagonal push rods.
#ifndef DELTA_DIAGONAL_ROD
  #define DELTA_DIAGONAL_ROD 259.7 // mm
#endif

  // Horizontal offset from middle of printer to smooth rod center --> Augmenter la valeur descend la tete au point 0,0,0 pour corriger un deplacement convex de la tete
#ifndef DELTA_SMOOTH_ROD_OFFSET
  #define DELTA_SMOOTH_ROD_OFFSET 167.0 // mm
#endif

// Horizontal offset of the universal joints on the end effector.
#ifndef DELTA_EFFECTOR_OFFSET
  #define DELTA_EFFECTOR_OFFSET 32.81 // mm
#endif

// Horizontal offset of the universal joints on the carriages.
#ifndef DELTA_CARRIAGE_OFFSET
  #define DELTA_CARRIAGE_OFFSET 17.86 // mm
#endif

// Horizontal distance bridged by diagonal push rods when effector is centered.
#ifndef DELTA_RADIUS
  #define DELTA_RADIUS (DELTA_SMOOTH_ROD_OFFSET-DELTA_EFFECTOR_OFFSET-DELTA_CARRIAGE_OFFSET)
#endif

// Print surface diameter/2 minus unreachable space (avoid collisions with vertical towers).
#ifndef DELTA_PRINTABLE_RADIUS
  #define DELTA_PRINTABLE_RADIUS 110.0
#endif


// Mechanical endstop with COM to ground and NC to Signal uses "false" here (most common setup).
#define X_MIN_ENDSTOP_INVERTED
#define Y_MIN_ENDSTOP_INVERTED
#define Z_MIN_ENDSTOP_INVERTED
#define X_MAX_ENDSTOP_INVERTED
#define Y_MAX_ENDSTOP_INVERTED
#define Z_MAX_ENDSTOP_INVERTED
//#define Z_MIN_PROBE_ENDSTOP_INVERTED
//#define DISABLE_MAX_ENDSTOPS
#define DISABLE_MIN_ENDSTOPS



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


//===========================================================================
//============================ Bed Auto Leveling ============================
//===========================================================================

#define AUTO_BED_LEVELING_FEATURE // Delete the comment to enable (remove // at the start of the line)
#define DEBUG_LEVELING_FEATURE
//#define Z_MIN_PROBE_REPEATABILITY_TEST  // If not commented out, Z-Probe Repeatability test will be included if Auto Bed Leveling is Enabled.

#if ENABLED(AUTO_BED_LEVELING_FEATURE)

  // Enable this to sample the bed in a grid (least squares solution).
  // Note: this feature generates 10KB extra code size.


  #define AUTO_BED_LEVELING_GRID  // Deltas only support grid mode.

  #define DELTA_PROBEABLE_RADIUS (DELTA_PRINTABLE_RADIUS-30)
  #define LEFT_PROBE_BED_POSITION -DELTA_PROBEABLE_RADIUS
  #define RIGHT_PROBE_BED_POSITION DELTA_PROBEABLE_RADIUS
  #define FRONT_PROBE_BED_POSITION -DELTA_PROBEABLE_RADIUS
  #define BACK_PROBE_BED_POSITION DELTA_PROBEABLE_RADIUS

  #define MIN_PROBE_EDGE 10 // The Z probe square sides can be no smaller than this.

  // Non-linear bed leveling will be used.
  // Compensate by interpolating between the nearest four Z probe values for each point.
  // Useful for deltas where the print surface may appear like a bowl or dome shape.
  // Works best with ACCURATE_BED_LEVELING_POINTS 5 or higher.
  #ifndef AUTO_BED_LEVELING_GRID_POINTS
    #define AUTO_BED_LEVELING_GRID_POINTS 12
  #endif

  // Offsets to the Z probe relative to the nozzle tip.
  // X and Y offsets must be integers.
  #define X_PROBE_OFFSET_FROM_EXTRUDER 0     // Z probe to nozzle X offset: -left  +right
  #define Y_PROBE_OFFSET_FROM_EXTRUDER 0   // Z probe to nozzle Y offset: -front +behind
  #define Z_PROBE_OFFSET_FROM_EXTRUDER 0.4  // Z probe to nozzle Z offset: -below (always!)
  #define Z_RAISE_BEFORE_HOMING 2       // (in mm) Raise Z axis before homing (G28) for Z probe clearance.
                                        // Be sure you have this distance over your Z_MAX_POS in case.

  #define XY_TRAVEL_SPEED 4000         // X and Y axis travel speed between probes, in mm/min.
  #define Z_RAISE_BEFORE_PROBING 10   // How much the Z axis will be raised before traveling to the first probing point.
  #define Z_RAISE_BETWEEN_PROBINGS 5  // How much the Z axis will be raised when traveling from between next probing points
  #define Z_RAISE_AFTER_PROBING 10    // How much the Z axis will be raised after the last probing point.

  #define Z_MIN_PROBE_ENDSTOP

#endif // AUTO_BED_LEVELING_FEATURE

#define DISABLE_ZMIN_ENDSTOP


// @section homing

// The position of the homing switches
#define MANUAL_HOME_POSITIONS
#define BED_CENTER_AT_0_0  // If defined, the center of the bed is at (X=0, Y=0)

// For deltabots this means top and center of the Cartesian print volume.
#ifndef MANUAL_Z_HOME_POS
  #define MANUAL_X_HOME_POS 0
  #define MANUAL_Y_HOME_POS 0
  #define MANUAL_Z_HOME_POS 319
#endif

// delta homing speeds must be the same on xyz

#define HOMING_FEEDRATE_XYZ (40*60)
#define HOMING_FEEDRATE_E 0
#define HOMING_FEEDRATE { HOMING_FEEDRATE_XYZ, HOMING_FEEDRATE_XYZ, HOMING_FEEDRATE_XYZ, HOMING_FEEDRATE_E }

// default settings
// delta speeds must be the same on xyz
// Hubert - Donnees de calcul
#define FULL_STEPS_PER_ROTATION 200 // Nombre de pas pour que le moteur effectue une rotation
#define MICROSTEPS 16               // Nombre de d'impulsions par pas (configure sur la carte)
#define BELT_PITCH 2                // Longueur du pas de la courroie en mm (courroie GT2=2mm)
#define PULLEY_TEETH 20             // Nombre de dents de la poulie

#define NB_MICROSTEPS (FULL_STEPS_PER_ROTATION * MICROSTEPS / BELT_PITCH / PULLEY_TEETH) // nbr micro-pas pour daplacer les chariots de 1mm

#define NB_MICROSTEPS_EXTRUDER (FULL_STEPS_PER_ROTATION * MICROSTEPS / 6.5 / 3.14159) // (Steps * MicroSpets / extruder gear diameter / Pi)


#define DEFAULT_AXIS_STEPS_PER_UNIT   {NB_MICROSTEPS,NB_MICROSTEPS,NB_MICROSTEPS,NB_MICROSTEPS_EXTRUDER}
#define DEFAULT_MAX_FEEDRATE          {9000, 9000, 9000, 250}    // (mm/sec) {500, 500, 500, 25}
#define DEFAULT_MAX_ACCELERATION      {9000,9000,9000,10000}    // X, Y, Z, E maximum start speed for accelerated moves.

#define DEFAULT_ACCELERATION          3000    // X, Y, Z and E acceleration in mm/s^2 for printing moves
#define DEFAULT_RETRACT_ACCELERATION  3000    // E acceleration in mm/s^2 for retracts
#define DEFAULT_TRAVEL_ACCELERATION   3000    // X, Y, Z acceleration in mm/s^2 for travel (non printing) moves

// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
#define DEFAULT_XYJERK                20.0    // (mm/sec)
#define DEFAULT_ZJERK                 DEFAULT_XYJERK  // Must be same as XY for delta
#define DEFAULT_EJERK                 5.0     // (mm/sec)

#define PLA_PREHEAT_HOTEND_TEMP 180
#define PLA_PREHEAT_HPB_TEMP 70
#define PLA_PREHEAT_FAN_SPEED 128   // Insert Value between 0 and 255

#define ABS_PREHEAT_HOTEND_TEMP 240
#define ABS_PREHEAT_HPB_TEMP 100
#define ABS_PREHEAT_FAN_SPEED 128   // Insert Value between 0 and 255


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


//===========================================================================
//============================= Thermal Settings ============================
//===========================================================================
//
#define TEMP_SENSOR_0 11 // 11 is 100k beta 3950 1% thermistor (4.7k pullup)
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_3 0
#define TEMP_SENSOR_BED 0

// This makes temp sensor 1 a redundant sensor for sensor 0. If the temperatures difference between these sensors is to high the print will be aborted.
//#define TEMP_SENSOR_1_AS_REDUNDANT
#define MAX_REDUNDANT_TEMP_SENSOR_DIFF 10

// Actual temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 10  // (seconds)
#define TEMP_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HEATER_0_MINTEMP 5
#define HEATER_1_MINTEMP 5
#define HEATER_2_MINTEMP 5
#define HEATER_3_MINTEMP 5
#define BED_MINTEMP 5

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HEATER_0_MAXTEMP 245
#define HEATER_1_MAXTEMP 245
#define HEATER_2_MAXTEMP 245
#define HEATER_3_MAXTEMP 245
#define BED_MAXTEMP 100

//===========================================================================
//============================= PID Settings ================================
//===========================================================================
// PID Tuning Guide here: http://reprap.org/wiki/PID_Tuning

// Comment the following line to disable PID and enable bang-bang.
#define PIDTEMP
#define BANG_MAX 255 // limits current to nozzle while in bang-bang mode; 255=full current
#define PID_MAX BANG_MAX // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
#if ENABLED(PIDTEMP)
  //#define PID_DEBUG // Sends debug data to the serial port.
  //#define PID_OPENLOOP 1 // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
  //#define SLOW_PWM_HEATERS // PWM with very low frequency (roughly 0.125Hz=8s) and minimum state time of approximately 1s useful for heaters driven by a relay
  //#define PID_PARAMS_PER_EXTRUDER // Uses separate PID parameters for each extruder (useful for mismatched extruders)
                                    // Set/get with gcode: M301 E[extruder number, 0-2]
  #define PID_FUNCTIONAL_RANGE 10 // If the temperature difference between the target temperature and the actual temperature
                                  // is more then PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.
  #define PID_INTEGRAL_DRIVE_MAX PID_MAX  //limit for the integral term
  #define K1 0.95 //smoothing factor within the PID

//HotEnd Merlin isolee
  #define  DEFAULT_Kp 22.48
  #define  DEFAULT_Ki 1.55
  #define  DEFAULT_Kd 81.76

#endif // PIDTEMP

// For direct drive extruder v9 set to true, for geared extruder set to false.
#define INVERT_E0_DIR true
#define INVERT_E1_DIR true



//==============================LCD and SD support=============================
//#define LANGUAGE_INCLUDE GENERATE_LANGUAGE_INCLUDE(fr)
#define DISPLAY_CHARSET_HD44780_JAPAN        // this is the most common hardware
#define SDSUPPORT // Enable SD Card Support in Hardware Console
#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER
#define DELTA_CALIBRATION_MENU

//#define Z_PROBE  18
//#define Z_MIN  18



// Deltas don't need slowdown
//#define SLOWDOWN 0

#include "configurations/transitional_default_configurations/default/Configuration.h"
