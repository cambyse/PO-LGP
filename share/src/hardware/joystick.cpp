#include "hardware.h"

struct sJoystick {
  class jsJoystick *joy;
  intA state;
  uint n;
  void open();
  void step();
  void close();
};

Joystick::Joystick():Process("Joystick") {
  s = new sJoystick;
  joystickState = biros().getVariable<JoystickState>("JoystickState", this);
  s->n=0;
}

Joystick::~Joystick() {
  delete s;
}

void Joystick::open() {
  s->open();
  joystickState->set_n(s->n, this);
  joystickState->set_state(s->state, this);
}

void Joystick::step() {
  s->step();
  joystickState->writeAccess(this);
  joystickState->state = s->state;
  joystickState->exitSignal=(s->state(0)==16 || s->state(0)==32);
  joystickState->deAccess(this);
}

void Joystick::close() {
  s->close();
}


//===========================================================================
//
// COMPILER DEPENDEND: plib
//

#ifdef MT_PLIB

#include <plib/js.h>
#undef min
#undef max

void sJoystick::open() {
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
  if (joy->notWorking()) n=0;
  state.resize(n+1);
  state.setZero();
  step();
}

void sJoystick::close() {
  delete joy;
  n=0;
  state.clear();
}

void sJoystick::step() {
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
}

#else //dummy implementations
void sJoystick::open() { state.resize(10); state.setZero(); MT_MSG("WARNING: dummy joystick implementation"); }
void sJoystick::step() { }
void sJoystick::close() { }
#endif
