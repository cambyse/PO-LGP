#ifndef MLR_hardware_h
#define MLR_hardware_h

#include <System/biros.h>

//===========================================================================
//
// Variables
//

struct HardwareReference;

struct SkinPressure:public VariableData {
  arr y_real;
  
  SkinPressure():VariableData("SkinPressure") {
    reg_y_real();
    y_real.resize(6);
    y_real.setZero();
  }
};

struct GamepadState: public VariableData {
  intA state;
  uint n;
  bool exitSignal;
  
  GamepadState():VariableData("GamepadState"), n(0), exitSignal(false) {
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
Process* newGamepad(GamepadState &gamepad);
// Process* newSchunkArm(HardwareReference &ref);
// Process* newSchunkHand(HardwareReference &ref);
// Process* newSchunkSkin(SkinPressure &skin);

PROCESS(Camera)

PROCESS(Laser)

PROCESS(Kinect)



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


#ifdef  MLR_IMPLEMENTATION
#  include "schunk.cpp"
#endif

#endif
