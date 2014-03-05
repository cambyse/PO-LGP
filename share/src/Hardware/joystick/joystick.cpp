#include "joystick.h"

#ifdef MT_PLIB

#include <plib/js.h>
#undef min
#undef max

JoystickInterface::JoystickInterface():Module("JoystickInterface"){}

void JoystickInterface::open(){
  jsInit();
  for(uint i=0;;i++){
    joy = new jsJoystick(i);
    if(joy->notWorking()) break;
    uint n=joy->getNumAxes();

    std::cout
        <<"name     = "   <<joy->getName()
       <<"\n#axis    = " <<joy->getNumAxes()
         // <<"\n#buttons = " <<joy->getNumButtons()
      <<"\nerror?   = " <<joy->notWorking()
     <<std::endl;

    if(n>=4) break;
    //iterate and try a new one
    delete joy;
  }
  step();
}

void JoystickInterface::close(){
  delete joy;
  joystickState.set()->clear();
}

void JoystickInterface::step(){
  if(joy->notWorking()) return;
  uint n=joy->getNumAxes();
  floatA A(n);
  int B;
  joy->rawRead(&B, A.p);
  joystickState.writeAccess();
  joystickState().resize(n+1);
  joystickState()(0)=B;
  for(uint i=0; i<n; i++) joystickState()(i+1)=(double)A(i)/(1<<15);
//  MT::arrayElemsep=" \t ";
//  cout <<"\r joystickState=" <<joystickState() <<std::flush;
  joystickState.deAccess();
}

#else //dummy implementations
JoystickInterface::JoystickInterface():Module("JoystickInterfaceINACTIVE"){}
void JoystickInterface::open(){ joystickState.resize(10); joystickState.setZero(); MT_MSG("WARNING: dummy joystick implementation"); }
void JoystickInterface::step(){ }
void JoystickInterface::close(){ }
#endif
