#ifndef MARLINFIRMWARE_H
#define MARLINFIRMWARE_H

#define AS_QUOTED_STRING(S)           #S
#define INCLUDE_BY_CLASS_AND_ID(C,I)  AS_QUOTED_STRING(configurations/C/I/Customization.h)
#define INCLUDE_BY_PATH(P)            AS_QUOTED_STRING(P)

#include "boards.h"
#include "macros.h"

//#include "Version.h"
#include "IncludeUserCustomization.h"

// Sanity Check
#ifndef CONFIGURATION_H
  #error Failed to properly include configuration parameters
#endif

#endif //MARLINFIRMWARE_H
