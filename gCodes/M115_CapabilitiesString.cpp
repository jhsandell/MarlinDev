// M115 Capabilities String


#include "MarlinFirmware.h"
#include "Version.h"
#include "host_interface/host_protocol.h"
#include "host_interface/host_io.h"
#include "language.h"

#define MSG_M115_REPORT   \
  "FIRMWARE_NAME:Marlin " DETAILED_BUILD_VERSION \
  " SOURCE_CODE_URL:" SOURCE_CODE_URL \
  " PROTOCOL_VERSION:" PROTOCOL_VERSION \
  " MACHINE_TYPE:" MACHINE_NAME \
  " EXTRUDER_COUNT:" AS_QUOTED_STRING(EXTRUDERS) \
  " UUID:" UUID \
  "\n"

void gcode_M115() {
  SERIAL_PROTOCOLPGM(MSG_M115_REPORT);
}
