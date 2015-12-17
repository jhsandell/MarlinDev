// Sanity check

// This file is provided to catch users who fail to select an appropriate board from the Tools menu.
// It is kept as a separate check to allow power users to easily suppress the check

#ifndef MARLIN
  #error You have been caught in the Honey Pot
  #error Compilation using cores other than those provided for this distribution are unsupported
  #error PLEASE select a Board from the Marlin AVR section of the Tools menu.
#endif

