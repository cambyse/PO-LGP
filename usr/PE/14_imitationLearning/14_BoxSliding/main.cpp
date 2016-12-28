#include <Core/util_t.h>
#include <Gui/opengl.h>

#include <Motion/motion.h>
//#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Ors/ors_swift.h>
#include "../src/motion_factory.h"
#include "../src/ikmo.h"


void run() {

  /// load parameter
  bool visTest = mlr::getParameter<uint>("visTest");
  uint verbose = mlr::getParameter<uint>("verbose");
  uint optRandSeed = mlr::getParameter<uint>("optRandSeed");
  uint optParam0Variant = mlr::getParameter<uint>("optParam0Variant");
  double optCostScale = mlr::getParameter<double>("optCostScale");
  double optTermCond = mlr::getParameter<double>("optTermCond");

  cout << "optRandSeed: " << optRandSeed << endl;
  rnd.seed(optRandSeed);

  /// create some motion scenes
  MotionFactory* mf = new MotionFactory();
  mlr::Array<Scene > trainScenes;
  mlr::Array<Scene > testScenes;
  mlr::Array<CostWeight> weights;
  mf->costScale=1e1;
  mf->createScenes(mlr::getParameter<uint>("scene"),trainScenes,testScenes,weights);

  /// create ikmo problem
  arr param;
  switch (optParam0Variant){
    case 0:
      param = trainScenes(0).paramRef;//+20.*fabs(randn(trainScenes(0).paramRef.d0,1));
      break;
    case 1:
      param = fabs(randn(trainScenes(0).paramRef.d0,1));
      break;
    case 2:
      param = ones(trainScenes(0).paramRef.d0,1);
      break;
  }

  param.flatten();
  param = param/sum(param)*mf->costScale;


  arr param0=param;
  cout << "Parameter initialization: " << param << endl;
  mlr::timerStart(true);
  cout << "ikmo start" << endl;
  IKMO ikmo(trainScenes,weights,param.d0,mf->costScale);
  cout << "ikmo initializied" << endl;
  checkAllGradients(ikmo,param,1e-2);


  optConstrained(param,NoArr,ikmo,OPT(verbose=verbose,stopTolerance=1e-4,stepInc=2,aulaMuInc=1,maxStep=-1., constrainedMethod=anyTimeAula, stopIters=1000,dampingInc=1.));
//  optConstrained(param,NoArr,ikmo,OPT(verbose=verbose,stopTolerance=1e-9,aulaMuInc=1,stopEvals=1000,stopIters=1000,dampingInc=1., constrainedMethod=augmentedLag,minStep=-1,maxStep=-1));
  cout << "TIME: " << mlr::timerRead() << endl;

  cout << param << endl;
  ikmo.costReport(param,param0);

  arr x;
  mf->execMotion(ikmo,trainScenes(0),param,visTest,0,x);

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
