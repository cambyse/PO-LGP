#ifndef MT_hardware_h
#define MT_hardware_h

#include <biros/biros.h>


struct SkinPressure:public Variable {
  arr y_real; //6D sensor reading
  
  SkinPressure():Variable("SkinPressure") {
    y_real.resize(6);
    y_real.setZero();
  }
};


struct JoystickState: public Variable {
  intA state;
  uint n;
  
  JoystickState():Variable("JoystickState") {}
  
};

#endif