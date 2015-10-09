#include <Core/util.tpp>
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

  bool visTest = mlr::getParameter<uint>("visTest");
  uint verbose = mlr::getParameter<uint>("verbose");

  /// create some motion scenes
  MotionFactory* mf = new MotionFactory();
  mlr::Array<Scene > trainScenes;
  mlr::Array<Scene > testScenes;
  mlr::Array<CostWeight> weights;
  mf->costScale=1e2;
  mf->createScenes(mlr::getParameter<uint>("scene"),trainScenes,testScenes,weights);

  /// create ikmo problem
//  arr param = trainScenes(0).paramRef;
//  arr param = fabs(randn(trainScenes(0).paramRef.d0,1)); param.flatten();
  arr param = ones(trainScenes(0).paramRef.d0,1); param.flatten();
  param = param/length(param)*mf->costScale;
  param(0)=1.;
//  param(0)=0.;
//  param = 0.*param + 1e0;
  arr param0=param;
  cout << "Parameter initialization: " << param << endl;

  cout << "ikmo start" << endl;
  IKMO ikmo(trainScenes,weights,param.d0);
  cout << "ikmo initializied" << endl;
  checkAllGradients(ikmo,param,1e-2);
//  return;
  optConstrained(param,NoArr,ikmo,OPT(verbose=verbose,stopTolerance=1e-2));
  optConstrained(param,NoArr,ikmo,OPT(verbose=verbose,stopTolerance=1e-9));
  ikmo.costReport(param,param0);

  mf->execMotion(ikmo,trainScenes(0),param,visTest);

  //-- TO DO --/
  //-- Lineare Constraints mit in formulierung
  //-- Parameter ohne transition costs lernen
  //-- scaling
  //-- blockwise weights

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
