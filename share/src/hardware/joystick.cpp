#include "hardware.h"

struct Joystick:Process {
  class jsJoystick *joy;
  JoystickState *joystickState;
  intA state;
  uint n;
  Joystick(JoystickState &joy);
  void open();
  void step();
  void close();
};

Process* newJoystick(JoystickState &joy){
  return new Joystick(joy);
}

Joystick::Joystick(JoystickState &joy):
  Process("Joystick"), joystickState(&joy), n(0) {
}


//===========================================================================
//
// COMPILER DEPENDEND: plib
//

#ifdef MT_PLIB

#include <plib/js.h>
#undef min
#undef max

void Joystick::open() {
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
  joystickState->set_n(n, this);
  if (joy->notWorking()) n=0;
  state.resize(n+1);
  state.setZero();
  step();
}

void Joystick::close() {
  delete joy;
  n=0;
  state.clear();
}

void Joystick::step() {
  if (!n) return;
  static int B; //, _B;
  static floatA A, _A;
  uint i;
  A.resize(n);
  state.resize(n+1);
  joy->rawRead(&B, A.p);
  state(0)=B;
  for (i=0; i<n-2; i++) state(i+1)=((int)A(i))>>8;
  for (; i<n  ; i++) state(i+1)=A(i);
  //cout <<"\rjoy state=" <<state <<std::flush;
  //bool change;
  //if(B!=_B || A!=_A) change=true; else change=false;
  //_B=B;
  _A=A;
  //return change;

  joystickState->writeAccess(this);
  joystickState->state = state;
  joystickState->exitSignal=(state(0)==16 || state(0)==32);
  joystickState->deAccess(this);

}

#else //dummy implementations
void Joystick::open() { state.resize(10); state.setZero(); MT_MSG("WARNING: dummy joystick implementation"); }
void Joystick::step() { }
void Joystick::close() { }
#endif
