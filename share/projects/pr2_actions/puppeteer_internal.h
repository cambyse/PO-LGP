#include <Motion/gamepad2tasks.h>
#include <Motion/feedbackControl.h>
#include <Hardware/joystick/joystick.h>
#include <System/engine.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>
#include <System/ros/roscom.h>

#include "puppeteer.h"

struct PuppeteerSystem:System{
  ACCESS(CtrlMsg, ctrl_ref);
  ACCESS(CtrlMsg, ctrl_obs);
  ACCESS(arr, joystickState);
  RosCom *ros;
  PuppeteerSystem():ros(NULL){
    addModule<JoystickInterface>(NULL, Module_Thread::loopWithBeat, .01);
    if(MT::getParameter<bool>("useRos",false))
      ros = addModule<RosCom>(NULL, Module_Thread::loopWithBeat, .001);
    connect();
  }
};


struct sPuppeteer{
  ors::KinematicWorld world;
  FeedbackMotionControl MP;
//  Gamepad2Tasks j2t;
  PuppeteerSystem S;
  arr q, qdot, zero_qdot;
  CtrlMsg refs;
  sPuppeteer():world("model.kvg"), MP(world,false)/*, j2t(MP)*/{}
  void open();
  void step(uint t);
};

