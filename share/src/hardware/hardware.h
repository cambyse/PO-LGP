#ifndef MT_hardware_h
#define MT_hardware_h

#include <biros/biros.h>

//===========================================================================
//
// Variables
//

struct HardwareReference;

struct SkinPressure:public Variable {
  FIELD(arr, y_real);
  
  SkinPressure():Variable("SkinPressure") {
    reg_y_real();
    y_real.resize(6);
    y_real.setZero();
  }
};

struct JoystickState: public Variable {
  FIELD(intA, state);
  FIELD(uint, n);
  FIELD(bool, exitSignal);
  
  JoystickState():Variable("JoystickState"), n(0), exitSignal(false) {
    reg_state();
    reg_n();
    reg_exitSignal();
    state.resize(1);
    state(0)=0;
  }
  
};


//===========================================================================
//
// Processes
//

//TODO:
// Process* newBumblebee(Image& left, Image& right);
// Process* newJoystick(JoystickState &joy);
// Process* newSchunkArm(HardwareReference &ref);
// Process* newSchunkHand(HardwareReference &ref);
// Process* newSchunkSkin(SkinPressure &skin);

PROCESS(Camera)

PROCESS(Laser)

PROCESS(Kinect)

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
