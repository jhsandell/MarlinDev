#include "MarlinFirmware.h"

#if ENABLED(SDSUPPORT)
  int freeMemory();
#else
extern "C" {
  int freeMemory();
}
#endif //!SDSUPPORT
