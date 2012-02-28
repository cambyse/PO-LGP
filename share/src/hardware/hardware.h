#ifndef MT_hardware_h
#define MT_hardware_h

#include <biros/biros.h>
#include <motion/motion.h>


//===========================================================================
//
// Variables
//

struct SkinPressure:public Variable {
  arr y_real; //6D sensor reading
  
  SkinPressure():Variable("SkinPressure") {
    y_real.resize(6);
    y_real.setZero();
  }
};


struct JoystickState: public Variable {
  FIELD( intA, state );
  FIELD( uint, n );
  
  JoystickState():Variable("JoystickState") {}
  
};


//===========================================================================
//
// Processes
//

struct SchunkArm:public Process {
  struct sSchunkArm *s;

  HardwareReference *hardwareReference;

  SchunkArm();
  ~SchunkArm();
  void open();
  void step();
  void close();
};


struct SchunkHand:public Process {
  struct sSchunkHand *s;

  HardwareReference *hardwareReference;

  SchunkHand();
  ~SchunkHand();
  void open();
  void step();
  void close();
};


struct SchunkSkin:public Process {
  struct sSchunkSkin *s;

  SkinPressure *skinPressure;

  SchunkSkin();
  ~SchunkSkin();
  void open();
  void step();
  void close();
};

struct Joystick:public Process {
  struct sJoystick *s;

  JoystickState *joystickState;

  Joystick();
  ~Joystick();
  void open();
  void step();
  void close();
};

#ifdef  MT_IMPLEMENTATION
#  include "schunk.cpp"
#endif

#endif
