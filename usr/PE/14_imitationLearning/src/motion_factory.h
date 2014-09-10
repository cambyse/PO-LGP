#ifndef MOTION_FACTORY_H
#define MOTION_FACTORY_H

#include <Core/array.h>
#include <Motion/motion.h>
#include "ioc.h"
//#include "scene.h"


struct MotionFactory {
  uint numParam;
  double costScale;

  MotionFactory() {
  }

  void execMotion(Scene &s, arr cost_param, bool vis=false, uint verbose=0);
  void createScenes(uint sID, MT::Array<Scene> &trainScenes, MT::Array<Scene> &testScenes,uint numScenes, bool vis=false);
  void createScene0(Scene &s, uint i, bool vis=false);
  void createScene1(Scene &s, uint i, bool vis=false);
  void createScene2(Scene &s, uint i, bool vis=false);
  void createScene3(Scene &s, uint i, bool vis=false);
  void createScene4(Scene &s, uint i, bool vis=false);
};

#endif // MOTION_FACTORY_H
