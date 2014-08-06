#include <Core/util_t.h>
#include <Gui/opengl.h>

#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/taskMap_transition.h>
#include <Optim/optimization.h>
#include <Ors/ors_swift.h>
#include <Motion/taskMap_proxy.h>

#include "ioc.h"
#include "motion_factory.h"


void testSliding() {

  /// 1. Create training and test scenarios
  MotionFactory* mf = new MotionFactory();
  MT::Array<Scene > trainScenes;
  MT::Array<Scene > testScenes;
  mf->createScenes(1,trainScenes,testScenes,1,false);

  /// 2. Optimize the parameters
  arr w = ones(mf->numParam,1)*1e2;w.flatten(); arr dual;
  mf->execMotion(trainScenes(0),trainScenes(0).paramRef,false);
  //  w =  trainScenes(0).paramRef;
  IOC ioc(trainScenes,mf->numParam);
  checkAllGradients(ioc,w,1e-3);

  optConstrained(w,dual,ioc,OPT(verbose=1,stopIters=500,stopEvals=500,allowOverstep=true,constrainedMethod=squaredPenalty));
  optConstrained(w,dual,ioc,OPT(verbose=1, stopTolerance=1e-12,stopIters=500,stopEvals=500,allowOverstep=true,constrainedMethod=squaredPenalty));
  ioc.costReport();

  /// 3. Evaluate code on test scenarios
  w = fabs(w);
  for (;;) {
    for (uint i = 0; i<trainScenes.d0; i++) {
      mf->execMotion(trainScenes(i),w,true);
    }
  }
  return;
}

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);
  testSliding();
  return 0;
}
