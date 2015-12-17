/**
 * Test configuration values for the use of obsolete items
 */

#include "MarlinFirmware.h"

#if  defined(WATCH_TEMP_PERIOD) && (WATCH_TEMP_PERIOD > 500)
  #error WATCH_TEMP_PERIOD now uses seconds instead of milliseconds.
#endif

#if DISABLED(THERMAL_PROTECTION_HOTENDS) && (defined(WATCH_TEMP_PERIOD) || defined(THERMAL_PROTECTION_PERIOD))
  #error Thermal Runaway Protection for hotends is now enabled with THERMAL_PROTECTION_HOTENDS.
#endif

#if DISABLED(THERMAL_PROTECTION_BED) && defined(THERMAL_PROTECTION_BED_PERIOD)
  #error Thermal Runaway Protection for the bed is now enabled with THERMAL_PROTECTION_BED.
#endif

#if ENABLED(COREXZ) && ENABLED(Z_LATE_ENABLE)
  #error "Z_LATE_ENABLE can't be used with COREXZ."
#endif

#if defined(X_HOME_RETRACT_MM)
  #error [XYZ]_HOME_RETRACT_MM settings have been renamed [XYZ]_HOME_BUMP_MM.
#endif

#if defined(PROBE_SERVO_DEACTIVATION_DELAY)
  #error PROBE_SERVO_DEACTIVATION_DELAY has been replaced with DEACTIVATE_SERVOS_AFTER_MOVE and SERVO_DEACTIVATION_DELAY.
#endif

#if defined(BEEPER)
  #error BEEPER is now BEEPER_PIN. Please update your pins definitions.
#endif

#if defined(SDCARDDETECT)
  #error SDCARDDETECT is now SD_DETECT_PIN. Please update your pins definitions.
#endif

#if defined(SDCARDDETECTINVERTED)
  #error SDCARDDETECTINVERTED is now SD_DETECT_INVERTED. Please update your configuration.
#endif

#if defined(BTENABLED)
  #error BTENABLED is now BLUETOOTH. Please update your configuration.
#endif

#if defined(CUSTOM_MENDEL_NAME)
  #error CUSTOM_MENDEL_NAME is now CUSTOM_MACHINE_NAME. Please update your configuration.
#endif

#if defined(HAS_AUTOMATIC_VERSIONING)
  #error HAS_AUTOMATIC_VERSIONING deprecated - upgrade your Marlin platform in the IDE
#endif

#if defined(ENABLE_AUTO_BED_LEVELING)
  #error ENABLE_AUTO_BED_LEVELING deprecated - use AUTO_BED_LEVELING_FEATURE instead
#endif

#if defined(DELTA_PROBABLE_RADIUS)
  #error DELTA_PROBABLE_RADIUS deprecated - use DELTA_PROBEABLE_RADIUS instead#endif
#endif
