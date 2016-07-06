#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>

#include "../../src/motion_factory.h"
#include "../../src/cost_weight.h"
#include "../../src/inverse_motion.h"

void testBasics() {
  MotionFactory* mf = new MotionFactory();
  Scenario scenario;
  scenario.costScale = 1e2;
  mf->loadScenarioTest(scenario,false);
  InverseMotionProblem IMP(scenario);

  arr param0 = IMP.initParam(InverseMotionProblem::RAND);
  arr param = param0;

  checkJacobianCP(IMP,param0,1e-2);
  optConstrainedMix(param,NoArr,IMP,OPT(verbose=0,stopTolerance=1e-5,stepInc=2,aulaMuInc=1,maxStep=-1., constrainedMethod=augmentedLag, stopIters=1000,dampingInc=1.));
  IMP.costReport(param,param0);

  scenario.setParam(param);
//  mf->execMotion(scenario.scenes(0),NoArr,NoArr,NoArr,mlr::getParameter<uint>("IMP/visTest"),0);
  for (uint i =0;i<scenario.scenes.d0;i++){
    mf->execMotion(scenario.scenes(i),NoArr,NoArr,NoArr,1,0);
  }
}

void testRbf() {
  MotionFactory* mf = new MotionFactory();
  Scenario scenario;
  scenario.costScale = 1e2;
  mf->loadScenarioTestRbf(scenario);
  InverseMotionProblem IMP(scenario);

  arr param0 = IMP.initParam(InverseMotionProblem::RAND);
  arr param = param0;

  checkJacobianCP(IMP,param0,1e-2);

  optConstrainedMix(param,NoArr,IMP,OPT(verbose=0,stopTolerance=1e-5,stepInc=2,aulaMuInc=1,maxStep=-1., constrainedMethod=augmentedLag, stopIters=1000,dampingInc=1.));
  IMP.costReport(param,param0);

  scenario.setParam(param);
  for (uint i =0;i<scenario.scenes.d0;i++){
    mf->execMotion(scenario.scenes(i),NoArr,NoArr,NoArr,1,0);
  }
}

void testFeatureSelection() {
  MotionFactory* mf = new MotionFactory();
  Scenario scenario;
  scenario.costScale = 1e2;
  mf->loadScenarioTestFeatSelect(scenario);
  InverseMotionProblem IMP(scenario);

  arr param0 = IMP.initParam(InverseMotionProblem::RAND);
  arr param = param0;

  checkJacobianCP(IMP,param0,1e-2);

  optConstrainedMix(param,NoArr,IMP,OPT(verbose=0,stopTolerance=1e-7,stepInc=2,aulaMuInc=1,maxStep=-1., constrainedMethod=augmentedLag, stopIters=1000,dampingInc=1.));
  IMP.costReport(param,param0);

  scenario.setParam(param);
  mf->execMotion(scenario.scenes(0),NoArr,NoArr,NoArr,mlr::getParameter<uint>("IMP/visTest"),0);
}

void testDemonstrations() {
  MotionFactory* mf = new MotionFactory();
  Scenario scenario;
  scenario.costScale = 1e2;
  mf->loadScenarioTestDemonstrations(scenario);
  InverseMotionProblem IMP(scenario);

  arr param0 = IMP.initParam(InverseMotionProblem::RAND);
  arr param = param0;

  checkJacobianCP(IMP,param0,1e-2);

  OptOptions o;
  o.stopTolerance = 1e-6; o.constrainedMethod=anyTimeAula; o.verbose=1; o.aulaMuInc=1.01;
  optConstrainedMix(param,NoArr,IMP,o);
  IMP.costReport(param,param0);

  scenario.setParam(param);
  mf->execMotion(scenario.scenes(0),NoArr,NoArr,NoArr,mlr::getParameter<uint>("IMP/visTest"),0);
}

int main(int argc,char **argv){
  rnd.seed(3);
  mlr::initCmdLine(argc,argv);
  TEST(Basics);
//  TEST(Rbf);
//  TEST(FeatureSelection);
//  TEST(Demonstrations);
  return 0;
}
