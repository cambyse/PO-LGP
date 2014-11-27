#include <Core/util_t.h>
#include <Gui/opengl.h>

#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>
//#include <Motion/taskMap_proxy.h>
//#include <Motion/taskMap_constrained.h>
//#include <Motion/taskMap_transition.h>
#include <Optim/optimization.h>
#include <Ors/ors_swift.h>
#include "../src/motion_factory.h"
#include "../src/ikmo.h"


void run() {

  bool visTest = MT::getParameter<uint>("visTest");
  uint verbose = MT::getParameter<uint>("verbose");

  /// create some motion scenes
  MotionFactory* mf = new MotionFactory();
  MT::Array<Scene > trainScenes;
  MT::Array<Scene > testScenes;
  MT::Array<CostWeight> weights;
  mf->costScale=1e2;
  mf->createScenes(MT::getParameter<uint>("scene"),trainScenes,testScenes,weights);

  /// create ikmo problem
//  arr param = trainScenes(0).paramRef;
//  arr param = ARR(1.,1e2);
  arr param = fabs(randn(trainScenes(0).paramRef.d0,1)); param.flatten();
//  param = 0.*param + 1e0;
  cout << param << endl;

  IKMO ikmo(trainScenes,weights,param.d0);
  checkAllGradients(ikmo,param,1e-2);
  optConstrained(param,NoArr,ikmo,OPT(verbose=verbose,stopTolerance=1e-20));
  ikmo.costReport(param);

  cout << param << endl;
  mf->execMotion(ikmo,trainScenes(0),param,visTest);

  return;

  //-- TO DO --/
  //-- Lineare Constraints mit in formulierung
  //-- Parameter ohne transition costs lernen
  //-- scaling
  //-- constraints fuer kontakte verwenden
  //-- blockwise weights



/*
  /// 1. Create training and test scenarios
  MotionFactory* mf = new MotionFactory();
  MT::Array<Scene > trainScenes;
  MT::Array<Scene > testScenes;
  mf->costScale=1e2;
  mf->createScenes(MT::getParameter<uint>("scene"),trainScenes,testScenes,numDem,visDemo);

  /// 2. Define parameter and start point
  cout << "Number of parameters: " << mf->numParam << endl;
  arr w = ones(mf->numParam,1);w.flatten();
  w(1)=15.;
//  arr w = fabs(randn(mf->numParam,1))*1e-1; w.flatten();
//  arr w =  trainScenes(0).paramRef;
//  w = w/sqrt(sumOfSqr(w));
  arr dual;
  mf->execMotion(trainScenes(0),trainScenes(0).paramRef,visTest);

  /// 2. Optimize the parameters
  IOC ioc(trainScenes,mf->numParam);
  checkAllGradients(ioc,w,1e-3);

  return;
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
  */
  return;
}

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);
  run();
  return 0;
}
