// unit_conversion.h

float as_mm(long steps, uint8_t axis);
long as_steps(float distance, uint8_t axis);
extern float axis_steps_per_unit[NUM_AXIS];
