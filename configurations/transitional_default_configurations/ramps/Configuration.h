#include "configurations/design/default/Configuration.h"

#ifndef MOTHERBOARD
  #define MOTHERBOARD BOARD_RAMPS_13_EFB
#endif

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

#ifndef TEMP_SENSOR_0
  #define TEMP_SENSOR_0 1
#endif
#ifndef TEMP_SENSOR_1
  #define TEMP_SENSOR_1 0
#endif

#if ENABLED(AUTO_BED_LEVELING_FEATURE)

  #ifndef AUTO_BED_LEVELING_GRID
    #define AUTO_BED_LEVELING_GRID
  #endif

  #if ENABLED(AUTO_BED_LEVELING_GRID)
    // set the rectangle in which to probe
    #ifndef LEFT_PROBE_BED_POSITION
      #define LEFT_PROBE_BED_POSITION 15
    #endif
    #ifndef RIGHT_PROBE_BED_POSITION
      #define RIGHT_PROBE_BED_POSITION 170
    #endif
    #ifndef FRONT_PROBE_BED_POSITION
      #define FRONT_PROBE_BED_POSITION 20
    #endif
    #ifndef BACK_PROBE_BED_POSITION
      #define BACK_PROBE_BED_POSITION 170
    #endif
  #endif

#endif
  
#if ENABLED(PIDTEMP)
  // this adds an experimental additional term to the heating power, proportional to the extrusion speed.
  // if Kc is chosen well, the additional required power due to increased melting should be compensated.
  #ifndef PID_ADD_EXTRUSION_RATE
    #define PID_ADD_EXTRUSION_RATE
  #endif
  #if ENABLED(PID_ADD_EXTRUSION_RATE)
    #define DEFAULT_Kc (100) //heating power=Kc*(e_speed)
    #define LPQ_MAX_LEN 50
  #endif
#endif

#include "configurations/transitional_default_configurations/default/Configuration.h"
