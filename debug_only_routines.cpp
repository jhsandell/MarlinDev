#include "MarlinFirmware.h"
#include "debug_only_routines.h"

#if ENABLED(DEBUG_LEVELING_FEATURE)
  void print_xyz(const char* prefix, const float x, const float y, const float z) {
    SERIAL_ECHO(prefix);
    SERIAL_ECHOPAIR(": (", x);
    SERIAL_ECHOPAIR(", ", y);
    SERIAL_ECHOPAIR(", ", z);
    SERIAL_ECHOLNPGM(")");
  }
  void print_xyz(const char* prefix, const float xyz[]) {
    print_xyz(prefix, xyz[X_AXIS], xyz[Y_AXIS], xyz[Z_AXIS]);
  }
#endif
