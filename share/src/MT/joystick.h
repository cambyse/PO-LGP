#ifndef MT_joystick_h
#define MT_joystick_h

#include "array.h"
#include "process.h"

class jsJoystick;

struct JoystickInterface:public Process,public Variable{
  jsJoystick *joy;
  intA state;
  uint n;
  JoystickInterface();
  void open();
  void step();
  void close();
};

#ifdef  MT_IMPLEMENTATION
#  include "joystick.cpp"
#endif
                          
#endif
