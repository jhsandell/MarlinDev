#define CUSTOMIZATION_H

#define UUID "e9cbe275-aaee-4383-9352-01c1bac200ac"

#define  STRING_CONFIG_H_AUTHOR "(R Wackerbarth)" // Who made the changes.

// Temperature Control
#define  PID_FUNCTIONAL_RANGE 70 // Use PID control when Nozzle Temperature is near target temperature

#define  X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define  Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
#define  Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)
#define  DELTA_PROBEABLE_RADIUS (DELTA_PRINTABLE_RADIUS+X_PROBE_OFFSET_FROM_EXTRUDER)

// probe at the points of a lattice grid
#define  AUTO_BED_LEVELING_GRID_X ((RIGHT_PROBE_BED_POSITION - LEFT_PROBE_BED_POSITION) / (AUTO_BED_LEVELING_GRID_POINTS - 1))
#define  AUTO_BED_LEVELING_GRID_Y ((BACK_PROBE_BED_POSITION - FRONT_PROBE_BED_POSITION) / (AUTO_BED_LEVELING_GRID_POINTS - 1))
#define  Z_PROBE_OFFSET_FROM_EXTRUDER -17.10  // Increase this if the first layer is too thin (remember: it's a negative number so increase means closer to zero).
#define  Z_RAISE_BETWEEN_PROBINGS 10  // How much the Z axis will be raised when traveling from between next probing points.
#define  Z_RAISE_AFTER_PROBING 40    // How much the Z axis will be raised after the last probing point.
#define  MANUAL_Z_HOME_POS 281 // For delta: Distance between nozzle and print surface after homing.

#define DELTA_DIAGONAL_ROD 300.0 // mm

#include "configurations/transitional_default_configurations/delta/kossel_pro/Configuration.h"
