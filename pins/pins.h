/**
 * pins.h
 */

#ifndef PINS_H
#define PINS_H

#if MB(GEN7_CUSTOM)
  #include "pins/pins_GEN7_CUSTOM.h"
#elif MB(GEN7_12)
  #include "pins/pins_GEN7_12.h"
#elif MB(GEN7_13)
  #include "pins/pins_GEN7_13.h"
#elif MB(GEN7_14)
  #include "pins/pins_GEN7_14.h"
#elif MB(CHEAPTRONIC)
  #include "pins/pins_CHEAPTRONIC.h"
#elif MB(SETHI)
  #include "pins/pins_SETHI.h"
#elif MB(RAMPS_OLD)
  #include "pins/pins_RAMPS_OLD.h"
#elif MB(RAMPS_13_EFB)
  #include "pins/pins_RAMPS_13_EFB.h"
#elif MB(RAMPS_13_EEB) || MB(RAMPS_13_EFF) || MB(RAMPS_13_EEF) || MB(RAMPS_13_SF)
  #include "pins/pins_RAMPS_13.h"
#elif MB(RAMPS_14_EFB)
  #define IS_RAMPS_14
  #include "pins/pins_RAMPS_13_EFB.h"
#elif MB(RAMPS_14_EEB) || MB(RAMPS_14_EFF) || MB(RAMPS_14_EEF) || MB(RAMPS_14_SF)
  #define IS_RAMPS_14
  #include "pins/pins_RAMPS_13.h"
#elif MB(GEN6)
  #include "pins/pins_GEN6.h"
#elif MB(GEN6_DELUXE)
  #include "pins/pins_GEN6_DELUXE.h"
#elif MB(SANGUINOLOLU_11)
  #include "pins/pins_SANGUINOLOLU_11.h"
#elif MB(SANGUINOLOLU_12)
  #include "pins/pins_SANGUINOLOLU_12.h"
#elif MB(MELZI)
  #include "pins/pins_MELZI.h"
#elif MB(STB_11)
  #include "pins/pins_STB_11.h"
#elif MB(AZTEEG_X1)
  #include "pins/pins_AZTEEG_X1.h"
#elif MB(MELZI_MAKR3D)
  #include "pins/pins_MELZI_MAKR3D.h"
#elif MB(AZTEEG_X3)
  #include "pins/pins_AZTEEG_X3.h"
#elif MB(AZTEEG_X3_PRO)
  #include "pins/pins_AZTEEG_X3_PRO.h"
#elif MB(ULTIMAKER)
  #include "pins/pins_ULTIMAKER.h"
#elif MB(ULTIMAKER_OLD)
  #include "pins/pins_ULTIMAKER_OLD.h"
#elif MB(ULTIMAIN_2)
  #include "pins/pins_ULTIMAIN_2.h"
#elif MB(3DRAG)
  #include "pins/pins_3DRAG.h"
#elif MB(K8200)
  #include "pins/pins_K8200.h"
#elif MB(TEENSYLU)
  #include "pins/pins_TEENSYLU.h"
#elif MB(RUMBA)
  #include "pins/pins_RUMBA.h"
#elif MB(PRINTRBOARD)
  #include "pins/pins_PRINTRBOARD.h"
#elif MB(BRAINWAVE)
  #include "pins/pins_BRAINWAVE.h"
#elif MB(BRAINWAVE_PRO)
  #include "pins/pins_BRAINWAVE_PRO.h"
#elif MB(SAV_MKI)
  #include "pins/pins_SAV_MKI.h"
#elif MB(TEENSY2)
  #include "pins/pins_TEENSY2.h"
#elif MB(GEN3_PLUS)
  #include "pins/pins_GEN3_PLUS.h"
#elif MB(GEN3_MONOLITHIC)
  #include "pins/pins_GEN3_MONOLITHIC.h"
#elif MB(MEGATRONICS)
  #include "pins/pins_MEGATRONICS.h"
#elif MB(MINITRONICS)
  #include "pins/pins_MINITRONICS.h"
#elif MB(MEGATRONICS_2)
  #include "pins/pins_MEGATRONICS_2.h"
#elif MB(MEGATRONICS_3)
  #include "pins/pins_MEGATRONICS_3.h"
#elif MB(OMCA_A)
  #include "pins/pins_OMCA_A.h"
#elif MB(OMCA)
  #include "pins/pins_OMCA.h"
#elif MB(RAMBO)
  #include "pins/pins_RAMBO.h"
#elif MB(MINIRAMBO)
  #include "pins/pins_MINIRAMBO.h"
#elif MB(ELEFU_3)
  #include "pins/pins_ELEFU_3.h"
#elif MB(5DPRINT)
  #include "pins/pins_5DPRINT.h"
#elif MB(LEAPFROG)
  #include "pins/pins_LEAPFROG.h"
#elif MB(BAM_DICE)
  #include "pins/pins_RAMPS_13.h"
#elif MB(BAM_DICE_DUE)
  #include "pins/pins_BAM_DICE_DUE.h"
#elif MB(FELIX2)
  #include "pins/pins_FELIX2.h"
#elif MB(MKS_BASE)
  #include "pins/pins_MKS_BASE.h"
#elif MB(RIGIDBOARD)
  #include "pins/pins_RIGIDBOARD.h"
#elif MB(MEGACONTROLLER)
  #include "pins/pins_MEGACONTROLLER.h"
#elif MB(99)
  #include "pins/pins_99.h"
#elif MB(A4JP)
  #include "pins/pins_A4JP.h"
#else
  #error Unknown MOTHERBOARD value set in Configuration.h
#endif

// Preset optional pins
#ifndef X_MS1_PIN
  #define X_MS1_PIN -1
#endif
#ifndef X_MS2_PIN
  #define X_MS2_PIN -1
#endif
#ifndef Y_MS1_PIN
  #define Y_MS1_PIN -1
#endif
#ifndef Y_MS2_PIN
  #define Y_MS2_PIN -1
#endif
#ifndef Z_MS1_PIN
  #define Z_MS1_PIN -1
#endif
#ifndef Z_MS2_PIN
  #define Z_MS2_PIN -1
#endif
#ifndef E0_MS1_PIN
  #define E0_MS1_PIN -1
#endif
#ifndef E0_MS2_PIN
  #define E0_MS2_PIN -1
#endif
#ifndef E1_MS1_PIN
  #define E1_MS1_PIN -1
#endif
#ifndef E1_MS2_PIN
  #define E1_MS2_PIN -1
#endif
#ifndef DIGIPOTSS_PIN
  #define DIGIPOTSS_PIN -1
#endif
#ifndef HEATER_3_PIN
  #define HEATER_3_PIN -1
#endif
#ifndef TEMP_3_PIN
  #define TEMP_3_PIN -1
#endif

// List of pins which to ignore when asked to change by gcode, 0 and 1 are RX and TX, do not mess with those!
#define _E0_PINS E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, HEATER_0_PIN, analogInputToDigitalPin(TEMP_0_PIN),
#if EXTRUDERS > 1
  #define _E1_PINS E1_STEP_PIN, E1_DIR_PIN, E1_ENABLE_PIN, HEATER_1_PIN, analogInputToDigitalPin(TEMP_1_PIN),
#elif ENABLED(Y_DUAL_STEPPER_DRIVERS) || ENABLED(Z_DUAL_STEPPER_DRIVERS)
  #define _E1_PINS E1_STEP_PIN, E1_DIR_PIN, E1_ENABLE_PIN,
#else
  #define _E1_PINS
#endif
#if EXTRUDERS > 2
  #define _E2_PINS E2_STEP_PIN, E2_DIR_PIN, E2_ENABLE_PIN, HEATER_2_PIN, analogInputToDigitalPin(TEMP_2_PIN),
#else
  #define _E2_PINS
#endif
#if EXTRUDERS > 3
  #define _E3_PINS E3_STEP_PIN, E3_DIR_PIN, E3_ENABLE_PIN, HEATER_3_PIN, analogInputToDigitalPin(TEMP_3_PIN),
#else
  #define _E3_PINS
#endif

#ifdef X_STOP_PIN
  #if X_HOME_DIR < 0
    #define X_MIN_PIN X_STOP_PIN
    #define X_MAX_PIN -1
  #else
    #define X_MIN_PIN -1
    #define X_MAX_PIN X_STOP_PIN
  #endif
#endif

#ifdef Y_STOP_PIN
  #if Y_HOME_DIR < 0
    #define Y_MIN_PIN Y_STOP_PIN
    #define Y_MAX_PIN -1
  #else
    #define Y_MIN_PIN -1
    #define Y_MAX_PIN Y_STOP_PIN
  #endif
#endif

#ifdef Z_STOP_PIN
  #if Z_HOME_DIR < 0
    #define Z_MIN_PIN Z_STOP_PIN
    #define Z_MAX_PIN -1
  #else
    #define Z_MIN_PIN -1
    #define Z_MAX_PIN Z_STOP_PIN
  #endif
#endif

#if ENABLED(DISABLE_MAX_ENDSTOPS)
  #undef X_MAX_PIN
  #undef Y_MAX_PIN
  #undef Z_MAX_PIN
  #define X_MAX_PIN          -1
  #define Y_MAX_PIN          -1
  #define Z_MAX_PIN          -1
#endif

#if ENABLED(DISABLE_MIN_ENDSTOPS)
  #undef X_MIN_PIN
  #undef Y_MIN_PIN
  #undef Z_MIN_PIN
  #define X_MIN_PIN          -1
  #define Y_MIN_PIN          -1
  #define Z_MIN_PIN          -1
#endif

#if ENABLED(DISABLE_Z_MIN_PROBE_ENDSTOP) || DISABLED(Z_MIN_PROBE_ENDSTOP) // Allow code to compile regardless of Z_MIN_PROBE_ENDSTOP setting.
  #undef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN    -1
#endif

#if ENABLED(DISABLE_XMAX_ENDSTOP)
  #undef X_MAX_PIN
  #define X_MAX_PIN          -1
#endif

#if ENABLED(DISABLE_XMIN_ENDSTOP)
  #undef X_MIN_PIN
  #define X_MIN_PIN          -1
#endif

#if ENABLED(DISABLE_YMAX_ENDSTOP)
  #define Y_MAX_PIN          -1
#endif

#if ENABLED(DISABLE_YMIN_ENDSTOP)
  #undef Y_MIN_PIN
  #define Y_MIN_PIN          -1
#endif

#if ENABLED(DISABLE_ZMAX_ENDSTOP)
  #undef Z_MAX_PIN
  #define Z_MAX_PIN          -1
#endif

#if ENABLED(DISABLE_ZMIN_ENDSTOP)
  #undef Z_MIN_PIN
  #define Z_MIN_PIN          -1
#endif

//
// Dual Y and Dual Z support
// These options are mutually-exclusive
//

#define __EPIN(p,q) E##p##_##q##_PIN
#define _EPIN(p,q) __EPIN(p,q)

// The Y2 axis, if any, should be the next open extruder port
#ifndef Y2_STEP_PIN
  #define Y2_STEP_PIN   _EPIN(EXTRUDERS, STEP)
  #define Y2_DIR_PIN    _EPIN(EXTRUDERS, DIR)
  #define Y2_ENABLE_PIN _EPIN(EXTRUDERS, ENABLE)
#endif

// The Z2 axis, if any, should be the next open extruder port
#ifndef Z2_STEP_PIN
  #define Z2_STEP_PIN   _EPIN(EXTRUDERS, STEP)
  #define Z2_DIR_PIN    _EPIN(EXTRUDERS, DIR)
  #define Z2_ENABLE_PIN _EPIN(EXTRUDERS, ENABLE)
#endif

#define SENSITIVE_PINS { 0, 1, \
    X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, \
    Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, \
    Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, Z_MIN_PROBE_PIN, \
    PS_ON_PIN, HEATER_BED_PIN, FAN_PIN, \
    _E0_PINS _E1_PINS _E2_PINS _E3_PINS \
    analogInputToDigitalPin(TEMP_BED_PIN) \
  }

#define HAS_DIGIPOTSS (DIGIPOTSS_PIN >= 0)

#endif //__PINS_H

