#pragma once

#include <Motion/gamepad2tasks.h>
#include <Motion/feedbackControl.h>
#include <System/engine.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>
#include <pr2/roscom.h>

#include "actionMachine.h"


struct sActionMachine{
  ors::KinematicWorld world;
  FeedbackMotionControl MP;
//  Gamepad2Tasks j2t;
  arr q, qdot, zero_qdot;
  CtrlMsg refs;
  sActionMachine():world("model.kvg"), MP(world,false)/*, j2t(MP)*/{}
//  void open();
//  void step(uint t);
};

