#include "configurations/design/default/Configuration.h"

#define SERIAL_PORT 0

#ifndef BAUDRATE
  #define BAUDRATE 115200
#endif

// Define this to set a unique identifier for this printer, (Used by some programs to differentiate between machines)
// You can use an online service to generate a random UUID. (eg http://www.uuidgenerator.net/version4)
//#define MACHINE_UUID "00000000-0000-0000-0000-000000000000"

// This defines the number of extruders
#define EXTRUDERS 1

#define POWER_SUPPLY 1

// Define this to have the electronics keep the power supply off on startup. If you don't know what this is leave it.
//#define PS_DEFAULT_OFF

// @section temperature

#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_3 0

// This makes temp sensor 1 a redundant sensor for sensor 0. If the temperatures difference between these sensors is to high the print will be aborted.
//#define TEMP_SENSOR_1_AS_REDUNDANT

// Actual temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 10  // (seconds)
#define TEMP_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HEATER_0_MINTEMP 5
#define HEATER_1_MINTEMP HEATER_0_MINTEMP
#define HEATER_2_MINTEMP HEATER_0_MINTEMP
#define HEATER_3_MINTEMP HEATER_0_MINTEMP
#define BED_MINTEMP HEATER_0_MINTEMP

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HEATER_0_MAXTEMP 275
#define HEATER_1_MAXTEMP HEATER_0_MAXTEMP
#define HEATER_2_MAXTEMP HEATER_0_MAXTEMP
#define HEATER_3_MAXTEMP HEATER_0_MAXTEMP
#define BED_MAXTEMP 150

// If you want the M105 heater power reported in watts, define the BED_WATTS, and (shared for all extruders) EXTRUDER_WATTS
//#define EXTRUDER_WATTS (12.0*12.0/6.7) //  P=I^2/R
//#define BED_WATTS (12.0*12.0/1.1)      // P=I^2/R

//===========================================================================
//============================= PID Settings ================================
//===========================================================================
// PID Tuning Guide here: http://reprap.org/wiki/PID_Tuning

// Comment the following line to disable PID and enable bang-bang.
#define PIDTEMP
#define BANG_MAX 255 // limits current to nozzle while in bang-bang mode; 255=full current
#if ENABLED(PIDTEMP)
  //#define PID_DEBUG // Sends debug data to the serial port.
  //#define PID_OPENLOOP 1 // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
  //#define SLOW_PWM_HEATERS // PWM with very low frequency (roughly 0.125Hz=8s) and minimum state time of approximately 1s useful for heaters driven by a relay
  //#define PID_PARAMS_PER_EXTRUDER // Uses separate PID parameters for each extruder (useful for mismatched extruders)
  // Set/get with gcode: M301 E[extruder number, 0-2]
  // is more then PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.
  #define PID_INTEGRAL_DRIVE_MAX PID_MAX  //limit for the integral term
  #define K1 0.95 //smoothing factor within the PID


#endif // PIDTEMP

//===========================================================================
//============================= PID > Bed Temperature Control ===============
//===========================================================================
// Select PID or bang-bang with PIDTEMPBED. If bang-bang, BED_LIMIT_SWITCHING will enable hysteresis
//
// Uncomment this to enable PID on the bed. It uses the same frequency PWM as the extruder.
// If your PID_dT is the default, and correct for your hardware/configuration, that means 7.689Hz,
// which is fine for driving a square wave into a resistive load and does not significantly impact you FET heating.
// This also works fine on a Fotek SSR-10DA Solid State Relay into a 250W heater.
// If your configuration is significantly different than this and you don't understand the issues involved, you probably
// shouldn't use bed PID until someone else verifies your hardware works.
// If this is enabled, find your own PID constants below.

//#define BED_LIMIT_SWITCHING

// This sets the max power delivered to the bed.
// all forms of bed control obey this (PID, bang-bang, bang-bang with hysteresis)
// setting this to anything other than 255 enables a form of PWM to the bed,
// so you shouldn't use it unless you are OK with PWM on your bed.  (see the comment on enabling PIDTEMPBED)
#define MAX_BED_POWER 255 // limits duty cycle to bed; 255=full current

//#define PID_BED_DEBUG // Sends debug data to the serial port.

#if ENABLED(PIDTEMPBED)

  #define PID_BED_INTEGRAL_DRIVE_MAX MAX_BED_POWER //limit for the integral term


  // FIND YOUR OWN: "M303 E-1 C8 S90" to run autotune on the bed at 90 degreesC for 8 cycles.
#endif // PIDTEMPBED

// @section extruder

//this prevents dangerous Extruder moves, i.e. if the temperature is under the limit
//can be software-disabled for whatever purposes by
#define PREVENT_DANGEROUS_EXTRUDE
//if PREVENT_DANGEROUS_EXTRUDE is on, you can still disable (uncomment) very long bits of extrusion separately.
#define PREVENT_LENGTHY_EXTRUDE

#define EXTRUDE_MINTEMP 170
#define EXTRUDE_MAXLENGTH (X_MAX_LENGTH+Y_MAX_LENGTH) //prevent extrusion of very large distances.

//===========================================================================
//======================== Thermal Runaway Protection =======================
//===========================================================================

/**
 * Thermal Runaway Protection protects your printer from damage and fire if a
 * thermistor falls out or temperature sensors fail in any way.
 *
 * The issue: If a thermistor falls out or a temperature sensor fails,
 * Marlin can no longer sense the actual temperature. Since a disconnected
 * thermistor reads as a low temperature, the firmware will keep the heater on.
 *
 * The solution: Once the temperature reaches the target, start observing.
 * If the temperature stays too far below the target (hysteresis) for too long,
 * the firmware will halt as a safety precaution.
 */

// Enable thermal protection for all extruders
#define THERMAL_PROTECTION_HOTENDS
// Enable thermal protection for the heated bed
#define THERMAL_PROTECTION_BED

//===========================================================================
//============================== Delta Settings =============================
//===========================================================================
// Enable DELTA kinematics and most of the default configuration for Deltas
#define DELTA
#ifndef KINEMATICS_INCLUDE
  #define KINEMATICS_INCLUDE GENERATE_KINEMATICS_INCLUDE(delta)
#endif

// Horizontal distance bridged by diagonal push rods when effector is centered.
#define DELTA_RADIUS (DELTA_SMOOTH_ROD_OFFSET-DELTA_EFFECTOR_OFFSET-DELTA_CARRIAGE_OFFSET)

// Enable this option for Toshiba steppers
//#define CONFIG_STEPPERS_TOSHIBA

// @section homing

#define ENDSTOPPULLUPS

// If you want to enable the Z probe pin, but disable its use, uncomment the line below.
// This only affects a Z probe endstop if you have separate Z min endstop as well and have
// activated Z_MIN_PROBE_ENDSTOP below. If you are using the Z Min endstop on your Z probe,
// this has no effect.
//#define DISABLE_Z_MIN_PROBE_ENDSTOP

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
// :{0:'Low',1:'High'}
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders

// Disables axis when it's not being used.
// WARNING: When motors turn off there is a chance of losing position accuracy!
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false

// @section extruder

#define DISABLE_E false // For all extruders
#define DISABLE_INACTIVE_EXTRUDER true //disable only inactive extruders and keep active extruder enabled

// @section machine

// Invert the stepper direction. Change (or reverse the motor connector) if an axis goes the wrong way.

// @section extruder

// For direct drive extruder v9 set to true, for geared extruder set to false.
#define INVERT_E1_DIR false
#define INVERT_E2_DIR false
#define INVERT_E3_DIR false

// @section homing

// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
// :[-1,1]
#define X_HOME_DIR 1  // deltas always home to max
#define Y_HOME_DIR 1
#define Z_HOME_DIR 1

#define min_software_endstops true // If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops true // If true, axis won't move to coordinates greater than the defined lengths below.

// @section machine

// Travel limits after homing (units are in mm)
#define X_MIN_POS -DELTA_PRINTABLE_RADIUS
#define Y_MIN_POS -DELTA_PRINTABLE_RADIUS
#define Z_MIN_POS 0
#define X_MAX_POS DELTA_PRINTABLE_RADIUS
#define Y_MAX_POS DELTA_PRINTABLE_RADIUS
#define Z_MAX_POS MANUAL_Z_HOME_POS

//===========================================================================
//========================= Filament Runout Sensor ==========================
//===========================================================================
//#define FILAMENT_RUNOUT_SENSOR // Uncomment for defining a filament runout sensor such as a mechanical or opto endstop to check the existence of filament
// In RAMPS uses servo pin 2. Can be changed in pins file. For other boards pin definition should be made.
// It is assumed that when logic high = filament available
//                    when logic  low = filament ran out
#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  const bool FIL_RUNOUT_INVERTING = true;  // Should be uncommented and true or false should assigned
  #define ENDSTOPPULLUP_FIL_RUNOUT
  #define FILAMENT_RUNOUT_SCRIPT "M600"
#endif

//===========================================================================
//=========================== Manual Bed Leveling ===========================
//===========================================================================

//#define MANUAL_BED_LEVELING  // Add display menu option for bed leveling.
//#define MESH_BED_LEVELING    // Enable mesh bed leveling.

#if ENABLED(MANUAL_BED_LEVELING)
  #define MBL_Z_STEP 0.025  // Step size while manually probing Z axis.
#endif  // MANUAL_BED_LEVELING

#if ENABLED(MESH_BED_LEVELING)
  #define MESH_MIN_X 10
  #define MESH_MAX_X (X_MAX_POS - MESH_MIN_X)
  #define MESH_MIN_Y 10
  #define MESH_MAX_Y (Y_MAX_POS - MESH_MIN_Y)
  #define MESH_NUM_X_POINTS 3  // Don't use more than 7 points per axis, implementation limited.
  #define MESH_NUM_Y_POINTS 3
  #define MESH_HOME_SEARCH_Z 4  // Z after Home, bed somewhere below but above 0.0.
#endif  // MESH_BED_LEVELING

//===========================================================================
//============================ Bed Auto Leveling ============================
//===========================================================================

// @section bedlevel

#if ENABLED(AUTO_BED_LEVELING_FEATURE)

  // There are 2 different ways to specify probing locations:
  //
  // - "grid" mode
  //   Probe several points in a rectangular grid.
  //   You specify the rectangle and the density of sample points.
  //   This mode is preferred because there are more measurements.
  //
  // - "3-point" mode
  //   Probe 3 arbitrary points on the bed (that aren't colinear)
  //   You specify the XY coordinates of all 3 points.

  // Enable this to sample the bed in a grid (least squares solution).
  // Note: this feature generates 10KB extra code size.
  #define AUTO_BED_LEVELING_GRID

  #if ENABLED(AUTO_BED_LEVELING_GRID)

    // set the rectangle in which to probe
    #define LEFT_PROBE_BED_POSITION -DELTA_PROBEABLE_RADIUS-(X_PROBE_OFFSET_FROM_EXTRUDER)
    #define RIGHT_PROBE_BED_POSITION DELTA_PROBEABLE_RADIUS
    #define FRONT_PROBE_BED_POSITION -DELTA_PROBEABLE_RADIUS-(Y_PROBE_OFFSET_FROM_EXTRUDER)
    #define BACK_PROBE_BED_POSITION DELTA_PROBEABLE_RADIUS

    #define MIN_PROBE_EDGE 10 // The Z probe square sides can be no smaller than this.

    // Non-linear bed leveling will be used.
    // Compensate by interpolating between the nearest four Z probe values for each point.
    // Useful for deltas where the print surface may appear like a bowl or dome shape.
    // Works best with ACCURATE_BED_LEVELING_POINTS 5 or higher.

  #else  // !AUTO_BED_LEVELING_GRID

    // Arbitrary points to probe.
    // A simple cross-product is used to estimate the plane of the bed.
    #define ABL_PROBE_PT_1_X 15
    #define ABL_PROBE_PT_1_Y 180
    #define ABL_PROBE_PT_2_X 15
    #define ABL_PROBE_PT_2_Y 20
    #define ABL_PROBE_PT_3_X 170
    #define ABL_PROBE_PT_3_Y 20

  #endif // AUTO_BED_LEVELING_GRID

  // Allen key retractable z-probe as seen on many Kossel delta printers - http://reprap.org/wiki/Kossel#Automatic_bed_leveling_probe
  // Deploys by touching z-axis belt. Retracts by pushing the probe down. Uses Z_MIN_PIN.
  #define Z_PROBE_ALLEN_KEY

  // If you have enabled the bed auto leveling and are using the same Z probe for Z homing,
  // it is highly recommended you let this Z_SAFE_HOMING enabled!!!

  #define Z_SAFE_HOMING
  // When defined, it will:
  // - Allow Z homing only after X and Y homing AND stepper drivers still enabled.
  // - If stepper drivers timeout, it will need X and Y homing again before Z homing.
  // - Position the Z probe in a defined XY point before Z Homing when homing all axis (G28).
  // - Block Z homing only when the Z probe is outside bed area.

  #if ENABLED(Z_SAFE_HOMING)
    #define Z_SAFE_HOMING_X_POINT ((X_MIN_POS + X_MAX_POS) / 2)    // X point for Z homing when homing all axis (G28).
    #define Z_SAFE_HOMING_Y_POINT ((Y_MIN_POS + Y_MAX_POS) / 2)    // Y point for Z homing when homing all axis (G28).
  #endif

  #define Z_MIN_PROBE_ENDSTOP
#endif // AUTO_BED_LEVELING_FEATURE
#define DISABLE_ZMIN_ENDSTOP


// @section homing

// The position of the homing switches
#define MANUAL_HOME_POSITIONS
#define BED_CENTER_AT_0_0

#define MANUAL_X_HOME_POS 0
#define MANUAL_Y_HOME_POS 0

// delta homing speeds must be the same on xyz
#define HOMING_FEEDRATE_XYZ (200*60)
#define HOMING_FEEDRATE_E 0
#define HOMING_FEEDRATE { HOMING_FEEDRATE_XYZ, HOMING_FEEDRATE_XYZ, HOMING_FEEDRATE_XYZ, HOMING_FEEDRATE_E }

// default settings
// delta speeds must be the same on xyz

#define DEFAULT_ACCELERATION          3000    // X, Y, Z and E acceleration in mm/s^2 for printing moves
#define DEFAULT_RETRACT_ACCELERATION  3000    // E acceleration in mm/s^2 for retracts
#define DEFAULT_TRAVEL_ACCELERATION   3000    // X, Y, Z acceleration in mm/s^2 for travel (non printing) moves

// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
#define DEFAULT_XYJERK                20.0    // (mm/sec)
#define DEFAULT_ZJERK                 20.0    // (mm/sec) Must be same as XY for delta
#define DEFAULT_EJERK                 5.0     // (mm/sec)


// Custom M code points
#define CUSTOM_M_CODES
#if ENABLED(CUSTOM_M_CODES)
  #if ENABLED(AUTO_BED_LEVELING_FEATURE)
    #define CUSTOM_M_CODE_SET_Z_PROBE_OFFSET 851
  #endif
#endif

// @section extras

// EEPROM
// The microcontroller can store settings in the EEPROM, e.g. max velocity...
// M500 - stores parameters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
//define this to enable EEPROM support
//#define EEPROM_SETTINGS

#if ENABLED(EEPROM_SETTINGS)
  // To disable EEPROM Serial responses and decrease program space by ~1700 byte: comment this out:
  // Please keep turned on if you can.
  #define EEPROM_CHITCHAT
#endif

//
// M100 Free Memory Watcher
//
//#define M100_FREE_MEMORY_WATCHER // uncomment to add the M100 Free Memory Watcher for debug purpose

// @section temperature

// Preheat Constants
#define PLA_PREHEAT_HOTEND_TEMP 180
#define PLA_PREHEAT_HPB_TEMP 70
#define PLA_PREHEAT_FAN_SPEED 255   // Insert Value between 0 and 255

#define ABS_PREHEAT_HOTEND_TEMP 240
#define ABS_PREHEAT_HPB_TEMP 100
#define ABS_PREHEAT_FAN_SPEED 255   // Insert Value between 0 and 255

//==============================LCD and SD support=============================
#define DISPLAY_CHARSET_HD44780_JAPAN

#define SOFT_PWM_SCALE 0

#define FILAMENT_SENSOR_EXTRUDER_NUM 0   //The number of the extruder that has the filament sensor (0,1,2)
#define MEASUREMENT_DELAY_CM        14   //measurement delay in cm.  This is the distance from filament sensor to middle of barrel

#define DEFAULT_NOMINAL_FILAMENT_DIA 3.00  //Enter the diameter (in mm) of the filament generally used (3.0 mm or 1.75 mm) - this is then used in the slicer software.  Used for sensor reading validation
#define MEASURED_UPPER_LIMIT         3.30  //upper limit factor used for sensor reading validation in mm
#define MEASURED_LOWER_LIMIT         1.90  //lower limit factor for sensor reading validation in mm
#define MAX_MEASUREMENT_DELAY       20     //delay buffer size in bytes (1 byte = 1cm)- limits maximum measurement delay allowable (must be larger than MEASUREMENT_DELAY_CM  and lower number saves RAM)

//defines used in the code
#define DEFAULT_MEASURED_FILAMENT_DIA  DEFAULT_NOMINAL_FILAMENT_DIA  //set measured to nominal initially

//When using an LCD, uncomment the line below to display the Filament sensor data on the last line instead of status.  Status will appear for 5 sec.
//#define FILAMENT_LCD_DISPLAY


#include "Conditionals.h"

//homing hits the endstop, then retracts by this distance, before it tries to slowly bump again:
#define Z_HOME_BUMP_MM 5 // deltas need the same for all three axis
#define HOMING_BUMP_DIVISOR {10, 10, 10}  // Re-Bump Speed Divisor (Divides the Homing Feedrate)

#if ENABLED(ULTIPANEL)
  #define MANUAL_FEEDRATE_XYZ 50*60
  #define MANUAL_FEEDRATE { MANUAL_FEEDRATE_XYZ, MANUAL_FEEDRATE_XYZ, MANUAL_FEEDRATE_XYZ, 60 } // Feedrates for manual moves along X, Y, Z, E from panel
#endif

// If defined the movements slow down when the look ahead buffer is only half full
#define SLOWDOWN  0  // Delta Printers should not use slowdown

#include "configurations/transitional_default_configurations/default/Configuration.h"
#include "thermistortables.h"
