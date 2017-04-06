#ifndef MOTION_FACTORY_H
#define MOTION_FACTORY_H

#include <Core/array.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <iomanip>
#include <iostream>

#include "traj_factory.h"
#include "scene.h"

struct MotionFactory {
  void execMotion(Scene &s, arr &x=NoArr, arr &lambda=NoArr, arr &_x0=NoArr, uint vis=0, uint verbose=0,mlr::String name=mlr::String());
  void loadDemonstration(arr &x,arr &lambda, KOMO &MP);
  void readDemoFromFile(const char* name,arr &x,arr &lambda);
  void writeDemoToFile(const char* name,arr &x,arr &lambda);

  // scenarios
  void loadScenarioSimple(Scenario &scenario,uint nScenes=1, bool useConstraints=false);
  void loadScenarioComplex(Scenario &scenario);
  void loadScenarioButton(Scenario &scenario,mlr::KinematicWorld &world);
  void loadScenarioVelocity(Scenario &scenario);


  void loadScenarioTestRbf(Scenario &scenario);
  void loadScenarioTestFeatSelect(Scenario &scenario);
  void loadScenarioTestDemonstrations(Scenario &scenario);
  void loadScenarioBoxSliding(Scenario &scenario);
  void loadScenarioParamEval(Scenario &scenario, uint type);
};

#endif // MOTION_FACTORY_H
