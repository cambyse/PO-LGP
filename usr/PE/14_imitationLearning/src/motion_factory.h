#ifndef MOTION_FACTORY_H
#define MOTION_FACTORY_H

#include <Core/array.h>
#include <Motion/motion.h>
//#include "ikmo_bretl.h"
//#include "ikmo_cma.h"
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
  void execMotion2(Scene &s, bool vis=false, uint verbose=0);
  void createScenes(uint sID, MT::Array<Scene> &trainScenes, MT::Array<Scene> &testScenes, MT::Array<CostWeight> &weights);
  void createSceneTest(Scene &s, MT::Array<CostWeight> &weights, uint i);
  void createSceneTestRBF(Scene &s, MT::Array<CostWeight> &weights, uint i);
  void createSceneTestGaussian(Scene &s, MT::Array<CostWeight> &weights, uint i);
  void createScenePR2(Scene &s, MT::Array<CostWeight> &weights, uint i);
  void createSceneBoxSliding(Scene &s, MT::Array<CostWeight> &weights, uint i);
  void createScene1(Scene &s, MT::Array<CostWeight> &weights, uint i);
  void createScene2(Scene &s, MT::Array<CostWeight> &weights, uint i);
  void createScene3(Scene &s, MT::Array<CostWeight> &weights, uint i);
  void createScene4(Scene &s, MT::Array<CostWeight> &weights, uint i);
  void createScene5(Scene &s, MT::Array<CostWeight> &weights, uint i);
};


#endif // MOTION_FACTORY_H
