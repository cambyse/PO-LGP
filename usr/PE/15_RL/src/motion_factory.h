#ifndef MOTION_FACTORY_H
#define MOTION_FACTORY_H

#include <Core/array.h>
#include <Motion/motion.h>
#include "../src/scene.h"

struct MotionFactory {
  void execMotion(Scene &s, arr &x=NoArr, arr &lambda=NoArr, arr &_x0=NoArr, uint vis=0, uint verbose=0,MT::String name=MT::String());
  void loadScenarioTest(Scenario &scenario, bool useConstraints=false);
  void loadScenarioTestRbf(Scenario &scenario);
  void loadScenarioTestFeatSelect(Scenario &scenario);
  void loadScenarioTestDemonstrations(Scenario &scenario);
  void loadScenarioBoxSliding(Scenario &scenario);
  void loadScenarioParamEval(Scenario &scenario, uint type);

  void loadDemonstration(arr &x,arr &lambda, MotionProblem &MP);
  void readDemoFromFile(const char* name,arr &x,arr &lambda);
  void writeDemoToFile(const char* name,arr &x,arr &lambda);
};

#endif // MOTION_FACTORY_H


