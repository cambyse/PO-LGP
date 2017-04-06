#include <Core/util.tpp>
#include <Gui/opengl.h>

#include <KOMO/komo.h>
//#include <Motion/motionHeuristics.h>
#include <Kin/taskMaps.h>
#include <Optim/optimization.h>
#include <Kin/kin_swift.h>
#include "../src/motion_factory.h"
#include "../src/ikmo.h"

void run() {

  /// load parameter
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
  mf->costScale=optCostScale;
  mf->createScenes(mlr::getParameter<uint>("scene"),trainScenes,testScenes,weights);

  /// create ikmo problem
  arr param;
  switch (optParam0Variant){
    case 0:
      param = trainScenes(0).paramRef+20.*fabs(randn(trainScenes(0).paramRef.d0,1));
      break;
    case 1:
      param = fabs(randn(trainScenes(0).paramRef.d0,1));
      break;
    case 2:
      param = ones(trainScenes(0).paramRef.d0,1);
      break;
  }
  param.flatten();
  param(0) = 0.1;
  param.subRange(1,param.d0-1) = param.subRange(1,param.d0-1)/length(param.subRange(1,param.d0-1))*mf->costScale;

  arr param0=param;
  cout << "Parameter initialization: " << param << endl;
  cout << "ikmo start" << endl;
  IKMO ikmo(trainScenes,weights,param.d0,mf->costScale);
  cout << "ikmo initializied" << endl;
//  checkAllGradients(ikmo,param,1e-2);

  optConstrained(param,NoArr,ikmo,OPT(verbose=1, stopIters=1000, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-4));
  cout << param << endl;
  optConstrained(param,NoArr,ikmo,OPT(verbose=1, stopIters=1000, maxStep=1., stepInc=2., aulaMuInc=4,stopTolerance = optTermCond));

  ikmo.costReport(param,param0);

  mf->execMotion(ikmo,trainScenes(0),param,visTest);
}

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  run();
  return 0;
}
