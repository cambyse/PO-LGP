#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>

#include "../src/motion_factory.h"
#include "../src/cost_weight.h"
#include "../src/inverse_motion.h"

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  MotionFactory* mf = new MotionFactory();
  Scenario scenario;
  scenario.costScale = 1e3;
//  mf->loadScenarioTest(scenario);
  mf->loadScenarioTest(scenario,true);

  InverseMotionProblem IMP(scenario);
  arr param0 = IMP.initParam(InverseMotionProblem::ONES);
  arr param = param0;

  checkJacobianCP(IMP,param0,1e-2);
  optConstrained(param,NoArr,IMP,OPT(verbose=1,stopTolerance=1e-5,stepInc=2,aulaMuInc=10,maxStep=-1., constrainedMethod=augmentedLag, stopIters=1000,dampingInc=1.));
  IMP.costReport(param,param0);
  scenario.setParam(param);

  mf->execMotion(scenario.scenes(0),NoArr,NoArr,NoArr,mlr::getParameter<uint>("IMP/visTest"),0);
  for (uint i =0;i<scenario.scenes.d0;i++) {
    mf->execMotion(scenario.scenes(i),NoArr,NoArr,NoArr,true,0);
  }
  return 0;
}
