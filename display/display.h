#ifndef DISPLAY_H
#define DISPLAY_H

#include "MarlinFirmware.h"

#if ENABLED(HAS_DISPLAY)
  void lcd_setstatuspgm(const char* message, const uint8_t level=0);
  void lcd_setalertstatuspgm(const char* message);
  #define LCD_MESSAGEPGM(x) lcd_setstatuspgm(PSTR(x))
  #define LCD_ALERTMESSAGEPGM(x) lcd_setalertstatuspgm(PSTR(x))

  int lcd_strlen(char* s);
  int lcd_strlen_P(const char* s);
  void lcd_init();
  bool lcd_hasstatus();
  void lcd_setstatus(const char* message, const bool persist=false);
  void lcd_update();
  void lcd_reset_alert_level();
  bool lcd_detected(void);

#else //no LCD
  FORCE_INLINE void lcd_update() {}
  FORCE_INLINE void lcd_init() {}
  FORCE_INLINE bool lcd_hasstatus() { return false; }
  FORCE_INLINE void lcd_setstatus(const char* message, const bool persist=false) {UNUSED(message); UNUSED(persist);}
  FORCE_INLINE void lcd_setstatuspgm(const char* message, const uint8_t level=0) {UNUSED(message); UNUSED(level);}
  FORCE_INLINE void lcd_reset_alert_level() {}
  FORCE_INLINE bool lcd_detected(void) { return true; }

  #define LCD_MESSAGEPGM(x) do{}while(0)
  #define LCD_ALERTMESSAGEPGM(x) do{}while(0)

#endif

#endif
