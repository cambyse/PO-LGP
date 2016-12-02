#include <Core/util_t.h>
#include <Gui/opengl.h>

#include <Motion/motion.h>
//#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Ors/ors_swift.h>
#include "../src/motion_factory.h"
#include "../src/ikmo_bretl.h"


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

//  uint numLambda = sum(trainScenes.last().lambdaRef/(trainScenes.last().lambdaRef+1e-12));
//  cout << trainScenes.last().lambdaRef/(trainScenes.last().lambdaRef) << endl;

  uint count=0;
  for (uint n=0;n<trainScenes.last().lambdaRef.d0;n++){
    if (trainScenes.last().lambdaRef(n)!=0){
      count++;
    }
  }
  cout << "numLambda: " << count << endl;

  /// create ikmo problem
//  arr param = trainScenes(0).paramRef;
//  arr param = randn(trainScenes(0).paramRef.d0,1); param.flatten();
  arr param = ones(trainScenes(0).paramRef.d0,1); param.flatten();
  param.append(ones(count));
//  param =1e-7;
//  param(0)=-1;
//  param(0)=1e1;
//  param = param/sum(param)*mf->costScale;

  arr param0=param;
  cout << "parameter initialization: " << param << endl;

  cout << "ikmo start" << endl;
  IKMO ikmo(trainScenes,weights,param.d0,mf->costScale);
  cout << "ikmo initializied" << endl;
  checkAllGradients(ikmo,param,1e-2);

  optConstrained(param,NoArr,ikmo,OPT(verbose=verbose,stopTolerance=1e-5));

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
