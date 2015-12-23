// Unit_conversion.cpp

#include "MarlinFirmware.h"
#include "unit_conversion.h"

float axis_steps_per_unit[NUM_AXIS];

float as_mm(long steps, uint8_t axis) {
   return (float(steps) / axis_steps_per_unit[axis]);
}

long as_steps(float distance, uint8_t axis) {
  return (long)(distance * axis_steps_per_unit[axis]);
}