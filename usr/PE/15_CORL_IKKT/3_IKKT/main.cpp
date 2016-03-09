#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <pr2/trajectoryInterface.h>

#include "../../src/motion_factory.h"
#include "../../src/cost_weight.h"
#include "../../src/inverse_motion.h"

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  mlr::String folder = mlr::getParameter<mlr::String>("folder");
  ors::KinematicWorld world(STRING(folder<<"modelaug.kvg"));
  ors::KinematicWorld world_pr2("../../../../share/projects/pr2_gamepadControl/model.kvg");

  TrajectoryInterface *ti = new TrajectoryInterface(world,world_pr2);

  MotionFactory* mf = new MotionFactory();
  Scenario scenario;
  scenario.costScale = 1e2;
  mf->loadScenarioButton(scenario,world);

  InverseMotionProblem IMP(scenario);
  arr param0 = IMP.initParam(InverseMotionProblem::VEC,scenario.paramGT);
  arr param = param0;

//  checkJacobianCP(IMP,param0,1e-2);
  optConstrained(param,NoArr,IMP,OPT(verbose=1,stopTolerance=1e-7,stepInc=2,aulaMuInc=2,maxStep=-1., constrainedMethod=augmentedLag, stopIters=1000,dampingInc=1.));
  IMP.costReport(param,param0);
  scenario.setParam(param);

  /// connect with trajectory interface
  arr x0 = scenario.scenes.last().x0;// + randn(scenario.scenes.last().x0.d0);
  arr x;
  mf->execMotion(scenario.scenes(0),x,NoArr,x0,true,0);

  ti->gotoPositionPlan(x0);
  ti->executeTrajectoryPlan(x,10.,true,true);

  for (;;) {
    for (uint i =0;i<scenario.scenes.d0;i++) {
      arr x0 = scenario.scenes.last().x0;// + randn(scenario.scenes.last().x0.d0);
      mf->execMotion(scenario.scenes(i),NoArr,NoArr,x0,true,0);
    }
  }

  return 0;
}
