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

inline bool stopButtons(const arr& gamepadState){
  if(!gamepadState.N) return false;
  uint mode = uint(gamepadState(0));
  if(mode&0x10 || mode&0x20 || mode&0x40 || mode&0x80) return true;
  return false;
}

#ifdef  MT_IMPLEMENTATION
#  include "joystick.cpp"
#endif

#endif
