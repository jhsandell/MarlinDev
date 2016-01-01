/**
 * G92: Set current position to given X Y Z E
 */

#include "MarlinFirmware.h"

#include "parser.h"
#include "kinematics/coefficients.h"
#include "movement.h"
#include "motion/motion.h"

void gcode_G92() {
  if (!code_seen(axis_codes[E_AXIS]))
    st_synchronize();

  bool didXYZ = false;
  for (int i = 0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i])) {
      float v = current_position[i] = code_value();
      if (i == E_AXIS)
        plan_set_e_position(v);
      else
        didXYZ = true;
    }
  }
  if (didXYZ) {
    #if ENABLED(DELTA) || ENABLED(SCARA)
      sync_plan_position_delta();
    #else
      sync_plan_position();
    #endif
  }
}
