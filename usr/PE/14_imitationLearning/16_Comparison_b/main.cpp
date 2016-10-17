#include <Core/util_t.h>
#include <Gui/opengl.h>

#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Ors/ors_swift.h>
#include "../src/motion_factory.h"
#include "../src/ikmo.h"


void run() {
  rnd.seed(2);
  bool visTest = mlr::getParameter<uint>("visTest");
  uint verbose = mlr::getParameter<uint>("verbose");

  /// create some motion scenes
  MotionFactory* mf = new MotionFactory();
  mlr::Array<Scene > trainScenes;
  mlr::Array<Scene > testScenes;
  mlr::Array<CostWeight> weights;
  mf->costScale=1e2;
  mf->createScenes(mlr::getParameter<uint>("scene"),trainScenes,testScenes,weights);
//  return;

  /// create ikmo problem
//  arr param = trainScenes(0).paramRef;
  arr param = ARR(1e-2,1e4,1e4,23.);

//  arr param = fabs(randn(trainScenes(0).paramRef.d0,1)); param.flatten();
//  arr param = ones(trainScenes(0).paramRef.d0,1); param.flatten();
//  param(0)=1e1;
  param.flatten();
  cout << param.subRange(0,2) << endl;
//  param.subRange(0,2) = param.subRange(0,2)/sum(param.subRange(0,2))*mf->costScale;
  mf->costScale = sum(param.subRange(0,2));
  arr param0=param;
  cout << "parameter initialization: " << param << endl;

  cout << "ikmo start" << endl;
  IKMO ikmo(trainScenes,weights,param.d0,mf->costScale);
  cout << "ikmo initializied" << endl;
  checkAllGradients(ikmo,param,1e-2);

  optConstrained(param,NoArr,ikmo,OPT(verbose=verbose,stopTolerance=1e-5,stepInc=2,aulaMuInc=1,maxStep=-1., constrainedMethod=augmentedLag, stopIters=1000,dampingInc=1.));

  ikmo.costReport(param,param0);

  mf->execMotion(ikmo,trainScenes(0),param,visTest);

  /*
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
  mlr::initCmdLine(argc,argv);
  run();
  return 0;
}
