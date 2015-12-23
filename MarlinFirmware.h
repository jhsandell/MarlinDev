#ifndef MARLINFIRMWARE_H
#define MARLINFIRMWARE_H

#define AS_STRING_(S)                  #S
#define AS_QUOTED_STRING(n)            AS_STRING_(n)

#define GENERATE_PROCESSOR_INCLUDE(M)  AS_STRING_(missing_##M.h)
#define GENERATE_DRIVER_INCLUDE(M)     AS_STRING_(missing_##M.h)
#define GENERATE_WIRING_INCLUDE(M)     AS_STRING_(missing_##M.h)
#define GENERATE_KINEMATICS_INCLUDE(M) AS_STRING_(missing_##M.h)
#define GENERATE_LANGUAGE_INCLUDE(M)   AS_STRING_(language_##M.h)

#include "boards.h"
#include "macros.h"

//#include "Version.h"
#include "IncludeUserCustomization.h"

// Sanity Check
#ifndef CONFIGURATION_H
  #error Failed to properly include configuration parameters
#endif

#include "pins/pins.h"
#endif //MARLINFIRMWARE_H
