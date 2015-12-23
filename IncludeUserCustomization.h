/* This module draws in all of the configuration options that the user has activated
 * It is only used once (in MarlinFirmware.h).
 * However, it is maintained as a separate file to encapsulate the logic involved in a clear manner
 */

// The following file is used for the convenience of less sophistocated users
// to allow the customization to appear in the Arduino IDE as a tab in the sketch window

#include "FirmwareCustomization.h"

#define INCLUDE_BY_CLASS_AND_ID(C,I)   AS_STRING_(configurations/C/I/Customization.h)
#define INCLUDE_BY_PATH(P)             AS_STRING_(P)

// CONFIGURATION_H (for backward compatibility) is used as a flag to indicate that the customization entries have been provided

#ifndef CONFIGURATION_H
  #ifdef CONFIGURATION_FILE
    #include INCLUDE_BY_PATH(CONFIGURATION_FILE)
  #endif
#endif

#ifndef CONFIGURATION_H
  #ifdef SPECIFIC_PRINTER_ID
    #include INCLUDE_BY_CLASS_AND_ID(custom,SPECIFIC_PRINTER_ID)
  #endif
#endif

#ifndef CONFIGURATION_H
  #ifdef VENDOR
    #define CUSTOM_CLASS vendor
    #ifdef MODEL
      #define CUSTOM_ID VENDOR/MODEL
    #else
      #define CUSTOM_ID VENDOR
    #endif
  #else
    #define CUSTOM_CLASS vendor
    #define CUSTOM_ID generic/RepRap
  #endif

  #include INCLUDE_BY_CLASS_AND_ID(CUSTOM_CLASS,CUSTOM_ID)

#endif
