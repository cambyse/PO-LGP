#ifndef MLR_gamepad_h
#define MLR_gamepad_h

#include <Core/array.h>
#include <Core/module.h>

struct GamepadInterface:Module {
  struct jsJoystick *joystick;
  ACCESS(arr, gamepadState)
  ACCESS(bool, quitSignal)
  GamepadInterface();
  void open();
  void step();
  void close();
};

/// The buttons on gamepad have the following values.
/// Note that you can use the binary operators & and | to check for multiple
/// button presses.
enum BUTTON {
  BTN_NONE = 0,
  BTN_A = 1,
  BTN_B = 2,
  BTN_X = 4,
  BTN_Y = 8,
  BTN_LB = 16,
  BTN_RB = 32,
  BTN_LT = 64,
  BTN_RT = 128,
  BTN_BACK = 256,
  BTN_START = 512,
  BTN_LSTICK = 1024,
  BTN_RSTICK = 2048,
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
