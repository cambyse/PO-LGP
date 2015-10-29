#ifndef MLR_gamepad_h
#define MLR_gamepad_h

#include <Core/array.h>
#include <Core/module.h>

struct GamepadInterface:Module {
  struct jsJoystick *joystick;
  ACCESSnew(arr, gamepadState)
  ACCESSnew(bool, quitSignal)
  GamepadInterface();
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

#ifdef  MLR_IMPLEMENTATION
#  include "gamepad.cpp"
#endif

#endif
