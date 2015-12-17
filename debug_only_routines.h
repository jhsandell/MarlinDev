#include "MarlinFirmware.h"
#include "host_interface/host_io.h"

extern uint8_t marlin_debug_flags;

#if ENABLED(DEBUG_LEVELING_FEATURE)
  void print_xyz(const char* prefix, const float x, const float y, const float z);
  void print_xyz(const char* prefix, const float xyz[]);
#else
  FORCE_INLINE void print_xyz(const char* prefix, const float x, const float y, const float z) 
    { UNUSED(prefix); UNUSED(x); UNUSED(y); UNUSED(z); }
  FORCE_INLINE void print_xyz(const char* prefix, const float xyz[])
    { UNUSED(prefix); UNUSED(xyz); }
#endif
