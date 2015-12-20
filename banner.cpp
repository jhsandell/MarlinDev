#include "MarlinFirmware.h"
#include "banner.h"
#include "host_interface/host_io.h"
#include "language.h"
#include "Version.h"
#include "planner_queue.h"
#include "free_memory.h"

void show_banner_details() {
  SERIAL_ECHOPGM(MSG_MARLIN);
  SERIAL_ECHOLNPGM(" " SHORT_BUILD_VERSION);
#if defined(STRING_DISTRIBUTION_DATE) && defined(STRING_CONFIG_H_AUTHOR)
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
  SERIAL_ECHOPGM(STRING_DISTRIBUTION_DATE);
  SERIAL_ECHOPGM(MSG_AUTHOR);
  SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
  SERIAL_ECHOPGM("Compiled: ");
  SERIAL_ECHOLNPGM(__DATE__);
#endif
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_FREE_MEMORY);
  SERIAL_ECHO(freeMemory());
  SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
  SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
}
