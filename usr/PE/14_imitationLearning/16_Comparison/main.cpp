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
  bool visTest = MT::getParameter<uint>("visTest");
  uint verbose = MT::getParameter<uint>("verbose");

  /// create some motion scenes
  MotionFactory* mf = new MotionFactory();
  MT::Array<Scene > trainScenes;
  MT::Array<Scene > testScenes;
  MT::Array<CostWeight> weights;
  mf->costScale=1e2;
  mf->createScenes(MT::getParameter<uint>("scene"),trainScenes,testScenes,weights);
//  return;

  /// create ikmo problem
//  arr param = trainScenes(0).paramRef;
//  arr param = fabs(randn(trainScenes(0).paramRef.d0,1)); param.flatten();
  arr param = ones(trainScenes(0).paramRef.d0,1); param.flatten();
  param(0)=1e2;
  param = param/sum(param)*mf->costScale;


  arr param0=param;
  cout << "parameter initialization: " << param << endl;

  cout << "ikmo start" << endl;
  IKMO ikmo(trainScenes,weights,param.d0,mf->costScale);
  cout << "ikmo initializied" << endl;
  checkAllGradients(ikmo,param,1e-2);

  optConstrained(param,NoArr,ikmo,OPT(verbose=verbose,stopTolerance=1e-9));
  write(LIST<arr>(param),"w.out");

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
  MT::initCmdLine(argc,argv);
  run();
  return 0;
}
