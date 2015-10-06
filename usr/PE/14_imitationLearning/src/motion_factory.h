#ifndef MOTION_FACTORY_H
#define MOTION_FACTORY_H

#include <Core/array.h>
#include <Motion/motion.h>
#include "ikmo.h"


struct MotionFactory {
  uint numParam;
  double costScale;
  bool vis;
  bool optConstraintsParam;
  uint nS;

  MotionFactory() {
  }

  void execMotion(IKMO &ikmo, Scene &s, arr param, bool vis=false, uint verbose=0);
  void createScenes(uint sID, mlr::Array<Scene> &trainScenes, mlr::Array<Scene> &testScenes, mlr::Array<CostWeight> &weights);
  void createScene0(Scene &s, mlr::Array<CostWeight> &weights, uint i);
  void createScenePR2(Scene &s, mlr::Array<CostWeight> &weights, uint i);
  void createScene1(Scene &s, mlr::Array<CostWeight> &weights, uint i);
  void createScene2(Scene &s, mlr::Array<CostWeight> &weights, uint i);
  void createScene3(Scene &s, mlr::Array<CostWeight> &weights, uint i);
  void createScene4(Scene &s, mlr::Array<CostWeight> &weights, uint i);
  void createScene5(Scene &s, mlr::Array<CostWeight> &weights, uint i);

};


#endif // MOTION_FACTORY_H
