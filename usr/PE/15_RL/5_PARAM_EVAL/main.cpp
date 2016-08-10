#include <Core/util_t.h>
#include <Gui/opengl.h>
#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Ors/ors_swift.h>
#include "../src/motion_factory.h"
#include "../src/inverse_motion.h"


void learnParam() {
  MotionFactory* mf = new MotionFactory();
  Scenario scenario;
  scenario.costScale = 1e3;
  mf->loadScenarioParamEval(scenario,mlr::getParameter<uint>("IMP/evalType"));

  InverseMotionProblem IMP(scenario);

//  arr param0 = IMP.initParam(InverseMotionProblem::VEC,IMP.scenario.paramGT);
//  arr param0 = IMP.initParam(InverseMotionProblem::ONES);
  arr param0 = IMP.scenario.paramGT;
  arr param = param0;
  cout << "param0: " << param0 << endl;
  checkAllGradients(IMP,param0,1e-3);


  OptOptions o;
  o.stopTolerance = mlr::getParameter<double>("IMP/stopTolerance"); o.constrainedMethod=augmentedLag; o.verbose=1; o.aulaMuInc=2;
  optConstrained(param,NoArr,IMP,o);
  IMP.costReport(param,param0);

  param >> FILE("param.out");

  param = param/sqrt(sum(param%param)/exp(6));
  scenario.setParam(param);

  mf->execMotion(scenario.scenes(0),NoArr,NoArr,NoArr,mlr::getParameter<uint>("IMP/visTest"),0);
}

void runParam() {
  MotionFactory* mf = new MotionFactory();
  Scenario scenario;
  scenario.costScale = 1e3;
  mf->loadScenarioParamEval(scenario,1);

  arr param;
  param << FILE("param.out");
  scenario.setParam(param);

  param = param/sqrt(sum(param%param)/exp(9));
  mf->execMotion(scenario.scenes(0),NoArr,NoArr,NoArr,2,1);
}



int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  learnParam();
//  runParam();
  return 0;
}
