#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "configurations/design/default/Configuration.h"

// @section machine

// SERIAL_PORT selects which serial port should be used for communication with the host.
// This allows the connection of wireless adapters (for instance) to non-default port pins.
// Serial port 0 is still used by the Arduino bootloader regardless of this setting.
// :[0,1,2,3,4,5,6,7]
#define SERIAL_PORT 0

// This determines the communication speed of the printer
// Excessive character rates are unnecessary and cause lost characters
#ifndef BAUDRATE
  #define BAUDRATE 115200
#endif

// Enable the Bluetooth serial interface on AT90USB devices
//#define BLUETOOTH

// The following define selects which electronics board you have.
// Please choose the name from boards.h that matches your setup
#ifndef MOTHERBOARD
  #define MOTHERBOARD BOARD_RAMPS_13_EFB
#endif

// Define this to set a unique identifier for this printer, (Used by some programs to differentiate between machines)
// You can use an online service to generate a random UUID. (eg http://www.uuidgenerator.net/version4)
//#define MACHINE_UUID "00000000-0000-0000-0000-000000000000"

// This defines the number of extruders
// :[1,2,3,4]
#ifndef EXTRUDERS
  #define EXTRUDERS 1
#endif

// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).
// The offset has to be X=0, Y=0 for the extruder 0 hotend (default extruder).
// For the other hotends it is their distance from the extruder 0 hotend.
//#define EXTRUDER_OFFSET_X {0.0, 20.00} // (in mm) for each extruder, offset of the hotend on the X axis
//#define EXTRUDER_OFFSET_Y {0.0, 5.00}  // (in mm) for each extruder, offset of the hotend on the Y axis

//// The following define selects which power supply you have. Please choose the one that matches your setup
// 1 = ATX
// 2 = X-Box 360 203Watts (the blue wire connected to PS_ON and the red wire to VCC)
// :{1:'ATX',2:'X-Box 360'}

#define POWER_SUPPLY 1

// Define this to have the electronics keep the power supply off on startup. If you don't know what this is leave it.
//#define PS_DEFAULT_OFF

// @section temperature

//===========================================================================
//============================= Thermal Settings ============================
//===========================================================================
//
//--NORMAL IS 4.7kohm PULLUP!-- 1kohm pullup can be used on hotend sensor, using correct resistor and table
//
//// Temperature sensor settings:
// -2 is thermocouple with MAX6675 (only for sensor 0)
// -1 is thermocouple with AD595
// 0 is not used
// 1 is 100k thermistor - best choice for EPCOS 100k (4.7k pullup)
// 2 is 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)
// 3 is Mendel-parts thermistor (4.7k pullup)
// 4 is 10k thermistor !! do not use it for a hotend. It gives bad resolution at high temp. !!
// 5 is 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (4.7k pullup)
// 6 is 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup)
// 7 is 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)
// 71 is 100k Honeywell thermistor 135-104LAF-J01 (4.7k pullup)
// 8 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
// 9 is 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
// 10 is 100k RS thermistor 198-961 (4.7k pullup)
// 11 is 100k beta 3950 1% thermistor (4.7k pullup)
// 12 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup) (calibrated for Makibox hot bed)
// 13 is 100k Hisens 3950  1% up to 300°C for hotend "Simple ONE " & "Hotend "All In ONE"
// 20 is the PT100 circuit found in the Ultimainboard V2.x
// 60 is 100k Maker's Tool Works Kapton Bed Thermistor beta=3950
//
//    1k ohm pullup tables - This is not normal, you would have to have changed out your 4.7k for 1k
//                          (but gives greater accuracy and more stable PID)
// 51 is 100k thermistor - EPCOS (1k pullup)
// 52 is 200k thermistor - ATC Semitec 204GT-2 (1k pullup)
// 55 is 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (1k pullup)
//
// 1047 is Pt1000 with 4k7 pullup
// 1010 is Pt1000 with 1k pullup (non standard)
// 147 is Pt100 with 4k7 pullup
// 110 is Pt100 with 1k pullup (non standard)
// 998 and 999 are Dummy Tables. They will ALWAYS read 25°C or the temperature defined below.
//     Use it for Testing or Development purposes. NEVER for production machine.
//#define DUMMY_THERMISTOR_998_VALUE 25
//#define DUMMY_THERMISTOR_999_VALUE 100
// :{ '0': "Not used", '4': "10k !! do not use for a hotend. Bad resolution at high temp. !!", '1': "100k / 4.7k - EPCOS", '51': "100k / 1k - EPCOS", '6': "100k / 4.7k EPCOS - Not as accurate as Table 1", '5': "100K / 4.7k - ATC Semitec 104GT-2 (Used in ParCan & J-Head)", '7': "100k / 4.7k Honeywell 135-104LAG-J01", '71': "100k / 4.7k Honeywell 135-104LAF-J01", '8': "100k / 4.7k 0603 SMD Vishay NTCS0603E3104FXT", '9': "100k / 4.7k GE Sensing AL03006-58.2K-97-G1", '10': "100k / 4.7k RS 198-961", '11': "100k / 4.7k beta 3950 1%", '12': "100k / 4.7k 0603 SMD Vishay NTCS0603E3104FXT (calibrated for Makibox hot bed)", '13': "100k Hisens 3950  1% up to 300°C for hotend 'Simple ONE ' & hotend 'All In ONE'", '60': "100k Maker's Tool Works Kapton Bed Thermistor beta=3950", '55': "100k / 1k - ATC Semitec 104GT-2 (Used in ParCan & J-Head)", '2': "200k / 4.7k - ATC Semitec 204GT-2", '52': "200k / 1k - ATC Semitec 204GT-2", '-2': "Thermocouple + MAX6675 (only for sensor 0)", '-1': "Thermocouple + AD595", '3': "Mendel-parts / 4.7k", '1047': "Pt1000 / 4.7k", '1010': "Pt1000 / 1k (non standard)", '20': "PT100 (Ultimainboard V2.x)", '147': "Pt100 / 4.7k", '110': "Pt100 / 1k (non-standard)", '998': "Dummy 1", '999': "Dummy 2" }
#ifndef TEMP_SENSOR_0
  #define TEMP_SENSOR_0 -1
#endif
#ifndef TEMP_SENSOR_1
  #define TEMP_SENSOR_1 -1
#endif
#ifndef TEMP_SENSOR_2
  #define TEMP_SENSOR_2 0
#endif
#ifndef TEMP_SENSOR_3
  #define TEMP_SENSOR_3 0
#endif
#ifndef TEMP_SENSOR_BED
  #define TEMP_SENSOR_BED 0
#endif

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
#ifndef HEATER_0_MINTEMP
  #define HEATER_0_MINTEMP 5
#endif
#ifndef HEATER_1_MINTEMP
  #define HEATER_1_MINTEMP HEATER_0_MINTEMP
#endif
#ifndef HEATER_2_MINTEMP
  #define HEATER_2_MINTEMP HEATER_0_MINTEMP
#endif
#ifndef HEATER_3_MINTEMP
  #define HEATER_3_MINTEMP HEATER_0_MINTEMP
#endif
#ifndef BED_MINTEMP
  #define BED_MINTEMP HEATER_0_MINTEMP
#endif

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#ifndef HEATER_0_MAXTEMP
  #define HEATER_0_MAXTEMP 275
#endif
#ifndef HEATER_1_MAXTEMP
  #define HEATER_1_MAXTEMP HEATER_0_MAXTEMP
#endif
#ifndef HEATER_2_MAXTEMP
  #define HEATER_2_MAXTEMP HEATER_0_MAXTEMP
#endif
#ifndef HEATER_3_MAXTEMP
  #define HEATER_3_MAXTEMP HEATER_0_MAXTEMP
#endif
#ifndef BED_MAXTEMP
  #define BED_MAXTEMP 150
#endif

// If you want the M105 heater power reported in watts, define the BED_WATTS, and (shared for all extruders) EXTRUDER_WATTS
//#define EXTRUDER_WATTS (12.0*12.0/6.7) //  P=I^2/R
//#define BED_WATTS (12.0*12.0/1.1)      // P=I^2/R

//===========================================================================
//============================= PID Settings ================================
//===========================================================================
// PID Tuning Guide here: http://reprap.org/wiki/PID_Tuning

// Comment the following line to disable PID and enable bang-bang.
#define PIDTEMP
#ifndef BANG_MAX
  #define BANG_MAX 255 // limits current to nozzle while in bang-bang mode; 255=full current
#endif
#ifndef PID_MAX
  #define PID_MAX BANG_MAX // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
#endif
#if ENABLED(PIDTEMP)
  //#define PID_DEBUG // Sends debug data to the serial port.
  //#define PID_OPENLOOP 1 // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
  //#define SLOW_PWM_HEATERS // PWM with very low frequency (roughly 0.125Hz=8s) and minimum state time of approximately 1s useful for heaters driven by a relay
  //#define PID_PARAMS_PER_EXTRUDER // Uses separate PID parameters for each extruder (useful for mismatched extruders)
                                    // Set/get with gcode: M301 E[extruder number, 0-2]
  #ifndef PID_FUNCTIONAL_RANGE
    #define PID_FUNCTIONAL_RANGE 10 // If the temperature difference between the target temperature and the actual temperature
                                    // is more then PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.
  #endif
  #define PID_INTEGRAL_DRIVE_MAX PID_MAX  //limit for the integral term
  #define K1 0.95 //smoothing factor within the PID

  // If you are using a pre-configured hotend then you can use one of the value sets by uncommenting it
  // Ultimaker
  #ifndef DEFAULT_Kp
    #define  DEFAULT_Kp 22.2
    #define  DEFAULT_Ki 1.08
    #define  DEFAULT_Kd 114
  #endif
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
//#define PIDTEMPBED

//#define BED_LIMIT_SWITCHING

// This sets the max power delivered to the bed.
// all forms of bed control obey this (PID, bang-bang, bang-bang with hysteresis)
// setting this to anything other than 255 enables a form of PWM to the bed,
// so you shouldn't use it unless you are OK with PWM on your bed.  (see the comment on enabling PIDTEMPBED)
#define MAX_BED_POWER 255 // limits duty cycle to bed; 255=full current

//#define PID_BED_DEBUG // Sends debug data to the serial port.

#if ENABLED(PIDTEMPBED)

  #define PID_BED_INTEGRAL_DRIVE_MAX MAX_BED_POWER //limit for the integral term

  //120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
  //from FOPDT model - kp=.39 Tp=405 Tdead=66, Tc set to 79.2, aggressive factor of .15 (vs .1, 1, 10)
  #ifndef DEFAULT_bedKp
    #define  DEFAULT_bedKp 10.00
    #define  DEFAULT_bedKi .023
    #define  DEFAULT_bedKd 305.4
  #endif

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
//============================= Mechanical Settings =========================
//===========================================================================

// @section homing

// Coarse Endstop Settings
// Comment this out (using // at the start of the line) to disable the endstop pullup resistors
#define ENDSTOPPULLUPS

#if DISABLED(ENDSTOPPULLUPS)
  // fine endstop settings: Individual pullups. will be ignored if ENDSTOPPULLUPS is defined
  //#define ENDSTOPPULLUP_XMAX
  //#define ENDSTOPPULLUP_YMAX
  //#define ENDSTOPPULLUP_ZMAX
  //#define ENDSTOPPULLUP_XMIN
  //#define ENDSTOPPULLUP_YMIN
  //#define ENDSTOPPULLUP_ZMIN
  //#define ENDSTOPPULLUP_ZMIN_PROBE
#endif

// Mechanical endstop with COM to ground and NC to Signal uses "false" here (most common setup).
#ifdef X_MIN_ENDSTOP_INVERTED
  const bool X_MIN_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
#else
  const bool X_MIN_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.
#endif
#ifdef Y_MIN_ENDSTOP_INVERTED
  const bool Y_MIN_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
#else
  const bool Y_MIN_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.
#endif
#ifdef Z_MIN_ENDSTOP_INVERTED
  const bool Z_MIN_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
#else
  const bool Z_MIN_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.
#endif
#ifdef X_MAX_ENDSTOP_INVERTED
  const bool X_MAX_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
#else
  const bool X_MAX_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.
#endif
#ifdef Y_MAX_ENDSTOP_INVERTED
  const bool Y_MAX_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
#else
  const bool Y_MAX_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.
#endif
#ifdef Z_MAX_ENDSTOP_INVERTED
  const bool Z_MAX_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
#else
  const bool Z_MAX_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.
#endif
#ifdef Z_MIN_PROBE_ENDSTOP_INVERTED
  const bool Z_MIN_PROBE_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.
#else
  const bool Z_MIN_PROBE_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.
#endif

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
#ifndef INVERT_X_DIR
  #define INVERT_X_DIR false
#endif
#ifndef INVERT_Y_DIR
  #define INVERT_Y_DIR true
#endif
#ifndef INVERT_Z_DIR
  #define INVERT_Z_DIR false
#endif

// @section extruder

// For direct drive extruder v9 set to true, for geared extruder set to false.
#ifndef INVERT_E0_DIR
  #define INVERT_E0_DIR false
#endif
#ifndef INVERT_E1_DIR
  #define INVERT_E1_DIR false
#endif
#ifndef INVERT_E2_DIR
  #define INVERT_E2_DIR false
#endif
#ifndef INVERT_E3_DIR
  #define INVERT_E3_DIR false
#endif

// @section homing

// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
// :[-1,1]
#ifndef X_HOME_DIR
  #define X_HOME_DIR -1  // home to min
#endif
#ifndef Y_HOME_DIR
  #define Y_HOME_DIR -1
#endif
#ifndef Z_HOME_DIR
  #define Z_HOME_DIR -1
#endif

#define min_software_endstops true // If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops true  // If true, axis won't move to coordinates greater than the defined lengths below.

// @section machine

// Travel limits after homing (units are in mm)
#ifndef X_MIN_POS
  #define X_MIN_POS 0
#endif
#ifndef Y_MIN_POS
  #define Y_MIN_POS 0
#endif
#ifndef Z_MIN_POS
  #define Z_MIN_POS 0
#endif
#ifndef X_MAX_POS
  #define X_MAX_POS 200
#endif
#ifndef Y_MAX_POS
  #define Y_MAX_POS 200
#endif
#ifndef Z_MAX_POS
  #define Z_MAX_POS 200
#endif

//===========================================================================
//========================= Filament Runout Sensor ==========================
//===========================================================================
//#define FILAMENT_RUNOUT_SENSOR // Uncomment for defining a filament runout sensor such as a mechanical or opto endstop to check the existence of filament
                                 // In RAMPS uses servo pin 2. Can be changed in pins file. For other boards pin definition should be made.
                                 // It is assumed that when logic high = filament available
                                 //                    when logic  low = filament ran out
#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  const bool FIL_RUNOUT_INVERTING = true;  // Should be uncommented and true or false should assigned
  #define ENDSTOPPULLUP_FIL_RUNOUT // Uncomment to use internal pullup for filament runout pins if the sensor is defined.
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

//#define AUTO_BED_LEVELING_FEATURE // Delete the comment to enable (remove // at the start of the line)
//#define DEBUG_LEVELING_FEATURE

// By default, perform the Z-Probe Repeatability test if Auto Bed Leveling is Enabled.
#if ENABLED(AUTO_BED_LEVELING_FEATURE)
  #ifndef Z_MIN_PROBE_REPEATABILITY_TEST
    #define Z_MIN_PROBE_REPEATABILITY_TEST
  #endif
#endif
#if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
  #define USE_PROBE
#endif

#if ENABLED(AUTO_BED_LEVELING_FEATURE)
  #define USE_PROBE
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
  #ifndef AUTO_BED_LEVELING_GRID
    #define AUTO_BED_LEVELING_GRID
  #endif

  #if ENABLED(AUTO_BED_LEVELING_GRID)

    // set the rectangle in which to probe
    #ifndef LEFT_PROBE_BED_POSITION
      #define LEFT_PROBE_BED_POSITION -DELTA_PROBEABLE_RADIUS
    #endif
    #ifndef RIGHT_PROBE_BED_POSITION
      #define RIGHT_PROBE_BED_POSITION DELTA_PROBEABLE_RADIUS
    #endif
    #ifndef FRONT_PROBE_BED_POSITION
      #define FRONT_PROBE_BED_POSITION -DELTA_PROBEABLE_RADIUS
    #endif
    #ifndef BACK_PROBE_BED_POSITION
      #define BACK_PROBE_BED_POSITION DELTA_PROBEABLE_RADIUS
    #endif

    #ifndef MIN_PROBE_EDGE
      #define MIN_PROBE_EDGE 10 // The Z probe minimum square sides can be no smaller than this.
    #endif

    // Set the number of grid points per dimension.
    // You probably don't need more than 3 (squared=9).
    // Non-linear bed leveling will be used.
    // Compensate by interpolating between the nearest four Z probe values for each point.
    #ifndef AUTO_BED_LEVELING_GRID_POINTS
      #define AUTO_BED_LEVELING_GRID_POINTS 2
    #endif
    #ifndef MESH_NUM_X_POINTS
     #define MESH_NUM_X_POINTS AUTO_BED_LEVELING_GRID_POINTS
    #endif
    #ifndef MESH_NUM_Y_POINTS
     #define MESH_NUM_Y_POINTS AUTO_BED_LEVELING_GRID_POINTS
    #endif

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

  // Offsets to the Z probe relative to the nozzle tip.
  // X and Y offsets must be integers.
  #ifndef X_PROBE_OFFSET_FROM_EXTRUDER
    #define X_PROBE_OFFSET_FROM_EXTRUDER -25     // Z probe to nozzle X offset: -left  +right
  #endif
  #ifndef Y_PROBE_OFFSET_FROM_EXTRUDER
    #define Y_PROBE_OFFSET_FROM_EXTRUDER -29     // Z probe to nozzle Y offset: -front +behind
  #endif
  #ifndef Z_PROBE_OFFSET_FROM_EXTRUDER
    #define Z_PROBE_OFFSET_FROM_EXTRUDER -12.35  // Z probe to nozzle Z offset: -below (always!)
  #endif

  #ifndef Z_RAISE_BEFORE_HOMING
    #define Z_RAISE_BEFORE_HOMING 4              // (in mm) Raise Z axis before homing (G28) for Z probe clearance.
                                                 // Be sure you have this distance over your Z_MAX_POS in case.
  #endif

//#define Z_PROBE_END_SCRIPT "G1 Z10 F12000\nG1 X15 Y330\nG1 Z0.5\nG1 Z10" // These commands will be executed in the end of G29 routine.
                                                                            // Useful to retract a deployable Z probe.
  #ifndef XY_TRAVEL_SPEED
    #define XY_TRAVEL_SPEED 8000         // X and Y axis travel speed between probes, in mm/min.
  #endif

  //#define Z_PROBE_SLED // Turn on if you have a Z probe mounted on a sled like those designed by Charles Bell.
  //#define SLED_DOCKING_OFFSET 5 // The extra distance the X axis must travel to pickup the sled. 0 should be fine but you can push it further if you'd like.
  #ifndef Z_RAISE_BEFORE_PROBING
    #define Z_RAISE_BEFORE_PROBING 15   // How much the Z axis will be raised before traveling to the first probing point.
  #endif
  #ifndef Z_RAISE_BETWEEN_PROBINGS
    #define Z_RAISE_BETWEEN_PROBINGS 5  // How much the Z axis will be raised when traveling from between next probing points
  #endif
  #ifndef Z_RAISE_AFTER_PROBING
    #define Z_RAISE_AFTER_PROBING 15    // How much the Z axis will be raised after the last probing point.
  #endif


  // If you have enabled the bed auto leveling and are using the same Z probe for Z homing,
  // it is highly recommended you let this Z_SAFE_HOMING enabled!!!

  #if !DISABLED(Z_SAFE_HOMING)
    #define Z_SAFE_HOMING
  #endif

  #if ENABLED(Z_SAFE_HOMING)

    #define Z_SAFE_HOMING_X_POINT ((X_MIN_POS + X_MAX_POS) / 2)    // X point for Z homing when homing all axis (G28).
    #define Z_SAFE_HOMING_Y_POINT ((Y_MIN_POS + Y_MAX_POS) / 2)    // Y point for Z homing when homing all axis (G28).

  #endif

#endif // AUTO_BED_LEVELING_FEATURE

#ifndef MESH_NUM_X_POINTS
  #define MESH_NUM_X_POINTS 1
#endif
#ifndef MESH_NUM_Y_POINTS
  #define MESH_NUM_Y_POINTS 1
#endif

// Manual homing switch locations:
// For deltabots this means top and center of the Cartesian print volume.
#if ENABLED(MANUAL_HOME_POSITIONS)
  #define MANUAL_X_HOME_POS 0
  #define MANUAL_Y_HOME_POS 0
  #ifndef MANUAL_Z_HOME_POS
    #define MANUAL_Z_HOME_POS 0
  #endif
#endif


// delta homing speeds must be the same on xyz
#ifndef HOMING_FEEDRATE
  #define HOMING_FEEDRATE { 50*60, 50*60, 4*60, 0 }
#endif

// default settings
// delta speeds must be the same on xyz
#ifndef DEFAULT_AXIS_STEPS_PER_UNIT
  #define DEFAULT_AXIS_STEPS_PER_UNIT   {80, 80, 4000, 500}  // default steps per unit
#endif
#ifndef DEFAULT_MAX_FEEDRATE
  #define DEFAULT_MAX_FEEDRATE          {300, 300, 5, 25}    // (mm/sec)
#endif
#ifndef DEFAULT_MAX_ACCELERATION
  #define DEFAULT_MAX_ACCELERATION      {3000,3000,100,10000}    // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.
#endif

#ifndef DEFAULT_ACCELERATION
  #define DEFAULT_ACCELERATION          3000    // X, Y, Z and E acceleration in mm/s^2 for printing moves
#endif
#ifndef DEFAULT_RETRACT_ACCELERATION
  #define DEFAULT_RETRACT_ACCELERATION  3000    // E acceleration in mm/s^2 for retracts
#endif
#ifndef DEFAULT_TRAVEL_ACCELERATION
  #define DEFAULT_TRAVEL_ACCELERATION   3000    // X, Y, Z acceleration in mm/s^2 for travel (non printing) moves
#endif

// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
#ifndef DEFAULT_XYJERK
  #define DEFAULT_XYJERK              20.0    // (mm/sec)
#endif
#ifndef DEFAULT_ZJERK
  #define DEFAULT_ZJERK               0.4     // (mm/sec)
#endif
#ifndef DEFAULT_EJERK
  #define DEFAULT_EJERK               5.0     // (mm/sec)
#endif

// Custom M code points
#define CUSTOM_M_CODES
#if ENABLED(CUSTOM_M_CODES)
  #if ENABLED(AUTO_BED_LEVELING_FEATURE)
    #define CUSTOM_M_CODE_SET_Z_PROBE_OFFSET 851
    #ifndef Z_PROBE_OFFSET_RANGE_MIN
      #define Z_PROBE_OFFSET_RANGE_MIN -20
    #endif
    #ifndef Z_PROBE_OFFSET_RANGE_MAX
      #define Z_PROBE_OFFSET_RANGE_MAX 20
    #endif
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
  #define EEPROM_CHITCHAT
#endif

// Preheat Constants
#define PLA_PREHEAT_HOTEND_TEMP 180
#define PLA_PREHEAT_HPB_TEMP 70
#ifndef PLA_PREHEAT_FAN_SPEED
  #define PLA_PREHEAT_FAN_SPEED 0   // Insert Value between 0 and 255
#endif

#define ABS_PREHEAT_HOTEND_TEMP 240
#ifndef ABS_PREHEAT_HPB_TEMP
  #define ABS_PREHEAT_HPB_TEMP 110
#endif
#ifndef ABS_PREHEAT_FAN_SPEED
  #define ABS_PREHEAT_FAN_SPEED 0   // Insert Value between 0 and 255
#endif

// See also language.h
#ifndef LANGUAGE_INCLUDE
  #define LANGUAGE_INCLUDE GENERATE_LANGUAGE_INCLUDE(en)
#endif
#ifndef DISPLAY_CHARSET_HD44780_JAPAN
  #define DISPLAY_CHARSET_HD44780_JAPAN
#endif
  
#define SOFT_PWM_SCALE 0

#if ENABLED(DEACTIVATE_SERVOS_AFTER_MOVE)
  // Delay (in microseconds) before turning the servo off. This depends on the servo speed.
  // 300ms is a good value but you can try less delay.
  // If the servo can't reach the requested position, increase it.
  #define SERVO_DEACTIVATION_DELAY 300
#endif

//#define FILAMENT_SENSOR

#define FILAMENT_SENSOR_EXTRUDER_NUM 0   //The number of the extruder that has the filament sensor (0,1,2)
#define MEASUREMENT_DELAY_CM        14   //measurement delay in cm.  This is the distance from filament sensor to middle of barrel

#define DEFAULT_NOMINAL_FILAMENT_DIA 3.00  //Enter the diameter (in mm) of the filament generally used (3.0 mm or 1.75 mm) - this is then used in the slicer software.  Used for sensor reading validation
#define MEASURED_UPPER_LIMIT         3.30  //upper limit factor used for sensor reading validation in mm
#define MEASURED_LOWER_LIMIT         1.90  //lower limit factor for sensor reading validation in mm
#define MAX_MEASUREMENT_DELAY       20     //delay buffer size in bytes (1 byte = 1cm)- limits maximum measurement delay allowable (must be larger than MEASUREMENT_DELAY_CM  and lower number saves RAM)

//defines used in the code
#define DEFAULT_MEASURED_FILAMENT_DIA  DEFAULT_NOMINAL_FILAMENT_DIA  //set measured to nominal initially

#include "configurations/transitional_default_configurations/default/Configuration_adv.h"
#include "thermistortables.h"

#endif //CONFIGURATION_H
