// M45 -- Check Pins

// --  *** Program hangs if M45 is repeated without disconnect/reconnect

#include "MarlinFirmware.h"
#include "host_interface/host_io.h"
#include "fastio.h"

const int sensitive_pins[] = SENSITIVE_PINS; ///< Sensitive pin list for M45

void gcode_M45() {
  millis_t stop_ms = millis() + (5 * 1000UL);
  while ((unsigned int)(millis()) < stop_ms) {
    millis_t last_ms = millis();
    while (((unsigned int)(millis() - last_ms)) < (500UL)) idle();
    for (uint8_t pin_number = 50; pin_number < NUM_DIGITAL_PINS; pin_number++) {
      uint8_t v;
      // Arduino pin numbering
      pinMode(pin_number, INPUT);
      v = digitalRead(pin_number);
      if (v)  SERIAL_PROTOCOL('1');
      else  SERIAL_PROTOCOL('0');
      if (pin_number % 10 == 9)
        SERIAL_PROTOCOL(' ');
    }
    SERIAL_EOL;
    last_ms = millis();
  }
  SERIAL_PROTOCOLPGM("Button Test Ended\n");
}
