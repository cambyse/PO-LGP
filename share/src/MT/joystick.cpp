#include "joystick.h"

#ifdef MT_PLIB

#include <plib/js.h>
#undef min
#undef max

JoystickInterface::JoystickInterface():Process("Joystick"), Variable("Joystick"){
  n=0;
};

void JoystickInterface::open(){
  jsInit();
  joy = new jsJoystick();
  /*std::cout
  <<"name     = "   <<joy->getName()
  <<"\n#axis    = " <<joy->getNumAxes()
  // <<"\n#buttons = " <<joy->getNumButtons()
  <<"\nerror?   = " <<joy->notWorking()
  <<std::endl;
  */
  n=joy->getNumAxes();
  if(joy->notWorking()) n=0;
  state.resize(10);
  state.setZero();
  step();
}

void JoystickInterface::close(){
  delete joy;
  n=0;
  state.clear();
}

void JoystickInterface::step(){
  if(!n) return;
  static int B, _B;
  static floatA A, _A;
  uint i;
  A.resize(n);
  state.resize(n+1);
  joy->rawRead(&B, A.p);
  state(0)=B;
  for(i=0; i<n-2; i++) state(i+1)=((int)A(i))>>8;
  for(; i<n  ; i++) state(i+1)=A(i);
  //cout <<"\rjoy state=" <<state <<std::flush;
  //bool change;
  //if(B!=_B || A!=_A) change=true; else change=false;
  _B=B;
  _A=A;
  //return change;
}

#else //dummy implementations
void JoystickInterface::open(){ state.resize(10); state.setZero(); MT_MSG("WARNING: dummy joystick implementation"); }
void JoystickInterface::step(){ }
void JoystickInterface::close(){ }
#endif
