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

  /// 1. Create training and test scenarios
  MotionFactory* mf = new MotionFactory();
  mf->costScale = 1e4;
  MT::Array<Scene > trainScenes;
  MT::Array<Scene > testScenes;
  mf->createScenes(MT::getParameter<uint>("scene"),trainScenes,testScenes,numDem,visDemo);

  /// 2. Optimize the parameters
  arr w = ones(mf->numParam,1);w.flatten();
//  arr w = fabs(randn(mf->numParam,1)); w.flatten();
//  arr w =  trainScenes(0).paramRef;
  if (!MT::getParameter<bool>("learnTransitionCost")) w(0) = 0;

  arr w0 = w;
  arr dual;

  if (MT::getParameter<bool>("evalOnly")) {
    w = { 0, 0.0925703, 0.0491705, 1, 0.00788893, 0.000571313, 8.33862e-05, 0.000467952, 0.000102605};
  }else{
    IOC ioc(trainScenes,mf->numParam);
    checkAllGradients(ioc,w,1e-3);
    optConstrained(w,dual,ioc,OPT(verbose=1,stopTolerance=1e-7,allowOverstep=true,constrainedMethod=augmentedLag));
    optConstrained(w,dual,ioc,OPT(verbose=1,stopTolerance=1e-9));
    ioc.costReport(w);
  }

  // remove all weights below threshold
  double threshold = 1e-12;
  for (uint i=0;i<w.d0;i++) {
    if (w(i)<threshold) {
      w(i) = 0.;
    }
  }

  /// 3. Evaluate code on test scenarios
  w = fabs(w);
  if (!MT::getParameter<bool>("learnTransitionCost")) {
    w(0) = sumOfAbs(w.subRange(1,w.d0-1))*0.01;
  }

  /// 4. Save Parameter into file
  arr options = {trainScenes(0).MP->tau,double(trainScenes(0).MP->T),double(trainScenes(0).contactTime),mf->costScale};
  FILE("../10_IOC_PR2_EXEC/bin/data/param") << w;
  FILE("../10_IOC_PR2_EXEC/bin/data/options") << options;
  FILE("../10_IOC_PR2_EXEC/bin/data/q0") << trainScenes(0).MP->x0;

  cout << "w: " <<w << endl;
  cout << "w0: " <<w0 << endl;

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
