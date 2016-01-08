// @section info

#ifndef STRING_CONFIG_H_AUTHOR
  #define STRING_CONFIG_H_AUTHOR "(none, default config)" // Who made the changes.
#endif
//#define NO_BOOTSCREEN // To suppress the bootscreen even if a display is present
#if DISABLED(NO_BOOTSCREEN)
  #define SHOW_BOOTSCREEN
#endif

#ifndef LANGUAGE_INCLUDE
  #define LANGUAGE_INCLUDE GENERATE_LANGUAGE_INCLUDE(en)
#endif
