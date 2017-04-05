#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Optim/optimization.h>
#include <pr2/trajectoryInterface.h>

#include "../../src/motion_factory.h"
#include "../../src/cost_weight.h"
#include "../../src/inverse_motion.h"

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  // init parameter
  mlr::String method = mlr::String("IKKT");
  mlr::String taskName = mlr::getParameter<mlr::String>("taskName");
  mlr::String folder = mlr::getParameter<mlr::String>("folder");
  bool learnIKKT = mlr::getParameter<bool>("IMP/learnIKKT",false);
  // init kinematic worlds & trajectoriy interface
  mlr::KinematicWorld world(STRING(folder<<"modelaug.kvg"));
  mlr::KinematicWorld world_pr2("../../../../share/projects/pr2_gamepadControl/model.kvg");
  TrajectoryInterface *ti = new TrajectoryInterface(world,world_pr2);
  double duration = mlr::getParameter<double>("duration");

  // init task
  MotionFactory* mf = new MotionFactory();
  Scenario scenario;
  if (taskName == "door") {

  } else if (taskName == "grasp") {

  } else if (taskName == "button") {
    scenario.costScale = 1e2;
    mf->loadScenarioButton(scenario,world);
  }

  // learn IKKT cost weights
  arr param;
  if (learnIKKT) {
    InverseMotionProblem IMP(scenario);
    arr param0 = IMP.initParam(InverseMotionProblem::VEC,scenario.paramGT);
    param = param0;
    checkJacobianCP(IMP,param0,1e-2);
    optConstrained(param,NoArr,IMP,OPT(verbose=1,stopTolerance=1e-7,stepInc=2,aulaMuInc=2,maxStep=-1., constrainedMethod=augmentedLag, stopIters=1000,dampingInc=1.));
    write(LIST<arr>(param),STRING(folder<<"Ikkt_param.dat"));
    IMP.costReport(param,param0);
  } else {
    param << FILE(STRING(folder<<"Ikkt_param.dat")); param.flatten();
  }


  scenario.setParam(param);

  // execute trajectories on the robot from different initial positions
  arr x0 = scenario.scenes.last().x0;
  uint count = 0;
  arr X;
  mf->execMotion(scenario.scenes(0),X,NoArr,x0,true,0);

  ti->gotoPositionPlan(x0);
  ti->executeTrajectoryPlan(X,duration,true,true);

  ti->logging(folder,method,count);
  count++;

  arr x0pr2;

  for (count = 7;count<10;count++) {
    x0pr2 << FILE(STRING(folder<<"q"<<count<<".dat")); x0pr2.flatten();
    transferQbetweenTwoWorlds(x0,x0pr2,world,world_pr2);
    mf->execMotion(scenario.scenes(0),X,NoArr,x0,true,0);
    ti->gotoPositionPlan(x0);
    ti->executeTrajectoryPlan(X,duration,true,true);
    ti->logging(folder,method,count);
  }

  return 0;
}
