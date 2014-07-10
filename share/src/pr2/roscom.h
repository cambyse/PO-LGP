#pragma once

#include <Core/module.h>
#include <Core/array.h>

//===========================================================================
//
// utils
//

bool rosOk();

//===========================================================================
//
// variable declarations
//

//-- a basic message type for communication with the PR2 controller
struct CtrlMsg{
  arr q, qdot, fL, fR;
  double Kp_gainFactor, Kd_gainFactor, fL_gainFactor, fR_gainFactor;
  CtrlMsg():Kp_gainFactor(1.), Kd_gainFactor(1.), fL_gainFactor(0.), fR_gainFactor(0.){}
  CtrlMsg(const arr& _q, const arr& _qdot, const arr& _fL, const arr& _fR):q(_q), qdot(_qdot), fL(_fL), fR(_fR){}
};
inline void operator<<(ostream& os, const CtrlMsg& m){ os<<"BLA"; }
inline void operator>>(istream& os, CtrlMsg& m){  }

//===========================================================================
//
// modules
//

/* Modules:
- RosSpinner
- RosCtrlMsg
- RosForceMsg
- RosKinectMsg
- RosCameraMsg

Only the CtrlMsg listens to ctrl_ref and then steps and publishes. All
others only have a feedback and write into Variables.

The Spinner does only ros::spinOnce() in each step and checks
initialization on startup.

TODO: allow modules to set default loopWithBeat, listenFirst, etc
options. (In their constructor?)

*/


struct RosCom:Module{ //RosCtrlMsg
  struct sRosCom *s;
  ACCESS(CtrlMsg, ctrl_ref);
  ACCESS(CtrlMsg, ctrl_obs);

  RosCom();

  void publishJointReference(); //-> step()
  void open();
  void step();
  void close();
};

