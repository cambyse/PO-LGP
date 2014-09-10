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

#include "../src/ioc.h"
#include "../src/motion_factory.h"


void run() {

  bool visDemo = MT::getParameter<uint>("visDemo");
  bool visTest = MT::getParameter<uint>("visTest");
  bool numDem = MT::getParameter<uint>("numDem");
  uint verbose = MT::getParameter<uint>("verbose");

  /// 1. Create training and test scenarios
  MotionFactory* mf = new MotionFactory();
  MT::Array<Scene > trainScenes;
  MT::Array<Scene > testScenes;
  mf->costScale=1e4;
  mf->createScenes(MT::getParameter<uint>("scene"),trainScenes,testScenes,numDem,visDemo);

  /// 2. Define parameter and start point

  arr w = ones(mf->numParam,1)*1e2;w.flatten();
//  arr w = fabs(randn(mf->numParam,1))*1e0; w.flatten();
//  arr w =  trainScenes(0).paramRef;
//  w = w/sqrt(sumOfSqr(w));
  arr dual;
  mf->execMotion(trainScenes(0),trainScenes(0).paramRef,visTest);

  /// 2. Optimize the parameters
  IOC ioc(trainScenes,mf->numParam);
  checkAllGradients(ioc,w,1e-3);
  optConstrained(w,dual,ioc,OPT(verbose=verbose,stopTolerance=1e-7));
  optConstrained(w,dual,ioc,OPT(verbose=verbose,stopTolerance=1e-9));

  if (!MT::getParameter<bool>("learnTransitionCost")) w(0) = sumOfAbs(w.subRange(1,w.d0-1))*0.1;
  ioc.costReport(w);

  /// 3. Evaluate code on test scenarios
  w = fabs(w);
  for (;;) {
    for (uint i = 0; i<testScenes.d0; i++) {
      mf->execMotion(testScenes(i),w,true);
      mf->execMotion(trainScenes(i),w,true);
    }
  }
  return;
}

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);
  run();
  return 0;
}
