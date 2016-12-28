#include <Core/util.tpp>
#include <Gui/opengl.h>

#include <Motion/motion.h>
//#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>



#include <Optim/optimization.h>
#include <Ors/ors_swift.h>


#include "../src/ioc.h"
#include "../src/motion_factory.h"


void run() {

  bool visDemo = mlr::getParameter<uint>("visDemo");
  bool visTest = mlr::getParameter<uint>("visTest");
  bool numDem = mlr::getParameter<uint>("numDem");
  uint verbose = mlr::getParameter<uint>("verbose");

  /// 1. Create training and test scenarios
  MotionFactory* mf = new MotionFactory();
  mlr::Array<Scene > trainScenes;
  mlr::Array<Scene > testScenes;
  mf->costScale=1e4;
  mf->createScenes(mlr::getParameter<uint>("scene"),trainScenes,testScenes,numDem,visDemo);

  /// 2. Define parameter and start point

//  arr w = ones(mf->numParam,1)*1e2;w.reshapeFlat();
  arr w = fabs(randn(mf->numParam,1))*1e0; w.reshapeFlat();
//  arr w =  trainScenes(0).paramRef;
//  w = w/sqrt(sumOfSqr(w));
  arr dual;
  mf->execMotion(trainScenes(0),trainScenes(0).paramRef,visTest);

  /// 2. Optimize the parameters
  IOC ioc(trainScenes,mf->numParam);
  checkAllGradients(ioc,w,1e-3);
  optConstrained(w,dual,ioc,OPT(verbose=verbose,stopTolerance=1e-7));
  optConstrained(w,dual,ioc,OPT(verbose=verbose,stopTolerance=1e-9));

  if (!mlr::getParameter<bool>("learnTransitionCost")) w(0) = sumOfAbs(w.subRef(1,w.d0-1))*0.1;
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
  mlr::initCmdLine(argc,argv);
  run();
  return 0;
}
