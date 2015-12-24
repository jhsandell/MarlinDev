#ifndef PANEL_H
#define PANEL_H

#include "MarlinFirmware.h"

bool lcd_clicked();

#if ENABLED(ULTIPANEL)
  void lcd_buttons_update();
  extern volatile uint8_t buttons;  //the last checked buttons in a bit array.
  #if ENABLED(REPRAPWORLD_KEYPAD)
    extern volatile uint8_t buttons_reprapworld_keypad; // to store the keypad shift register values
  #endif
#else
  FORCE_INLINE void lcd_buttons_update() {}
#endif

#endif
