// parser.h

const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};

bool code_has_value();

float code_value();

long code_value_long();

int16_t code_value_short();

bool code_seen(char code);

void out_of_range_error(const char* p_edge);
