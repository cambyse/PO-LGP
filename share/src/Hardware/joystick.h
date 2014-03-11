#ifndef MT_joystick_h
#define MT_joystick_h

#include <Core/array.h>
#include <Core/module.h>

struct JoystickInterface:Module {
  struct jsJoystick *joy;
  ACCESS(arr, joystickState);
  JoystickInterface();
  void open();
  void step();
  void close();
};

#ifdef  MT_IMPLEMENTATION
#  include "joystick.cpp"
#endif

#endif
