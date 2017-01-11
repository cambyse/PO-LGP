#include <Algo/spline.h>
#include <Core/array.h>
#include <Gui/plot.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Motion/phase_optimization.h>
#include <Kin/kin.h>
#include <pr2/roscom.h>
#include <RosCom/trajectoryInterface.h>

#include "src/conbopt.h"
#include "../../src/task_manager.h"
#include "../../src/plotUtil.h"

enum MODE {DEMO_REC=0,CORL_OPT=1};

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  /// ///////////////////////////////////////////////////////////////////////////////////////
  /// Initialization ///////////////////////////////////////////////////////////////////////
  /// /////////////////////////////////////////////////////////////////////////////////////
  // init parameters
  MODE mode = MODE(mlr::getParameter<uint>("mode"));
  uint count = mlr::getParameter<uint>("count",1);
  bool useRos = mlr::getParameter<bool>("useRos");
  mlr::String folder = mlr::getParameter<mlr::String>("folder");
  mlr::String taskName = mlr::getParameter<mlr::String>("taskName");
  double duration = mlr::getParameter<double>("duration");
  bool useReverseMotion = mlr::getParameter<bool>("CORL/useReverseMotion",false);
  bool pauseMotion = mlr::getParameter<bool>("CORL/pauseMotion",false);
  bool visualize = mlr::getParameter<bool>("CORL/visualize");
  bool useMarker = mlr::getParameter<bool>("useMarker",false);
  uint markerID; if (useMarker) markerID = mlr::getParameter<uint>("markerID");

  // init worlds + trajectory interface
  mlr::KinematicWorld world_pr2("../../../../share/projects/pr2_gamepadControl/model.kvg");
  mlr::KinematicWorld world;
  switch (mode) {
    case DEMO_REC:
      world = mlr::KinematicWorld(STRING(folder<<"model.kvg"));
      break;
    case CORL_OPT:
      world = mlr::KinematicWorld(STRING(folder<<"modelaug.kvg"));
      break;
  }
  TrajectoryInterface *ti;
  ti = new TrajectoryInterface(world,world_pr2);
  ti->world_pr2->gl().update(); ti->world_pr2->gl().resize(800,800); world.gl().update(); world.gl().resize(800,800);
  
  // init task
  TaskManager *task;
  if (taskName == "door") {
    task = new DoorTask(world);
  } else if (taskName == "grasp") {
    task = new GraspTask(world);
  } else if (taskName == "button") {
    task = new ButtonTask(world);
  }

  // init algorithm
  arr X, theta;
  arr Xdemo, XdemoPR2, FLdemo, Mdemo;
  mlr::String method;
  ConBOpt *cbo;
  switch (mode) {
    case DEMO_REC:
    {
      method = STRING("DEMO");
      ti->recordDemonstration(XdemoPR2,duration,0.05,5.);
      if (visualize) { world_pr2.watch(true,"Press Enter to visualize motion"); displayTrajectory(XdemoPR2,-1,world_pr2,"demonstration "); world_pr2.watch(true,"Press Enter to execute motion"); }
      ti->gotoPosition(XdemoPR2[0]);
      ti->executeTrajectory(XdemoPR2,duration,true);
      ti->logging(folder,method,0);
      if (useReverseMotion) {ti->executeTrajectory(XdemoPR2,duration);}
      return 0;
    }
      break;
    case CORL_OPT:
    {
      method = STRING("CORL");
      // load demonstration
      XdemoPR2 << FILE(STRING(folder<<"0_DEMO_X.dat"));
      transferQbetweenTwoWorlds(Xdemo,XdemoPR2,world,world_pr2);
      FLdemo << FILE(STRING(folder<<"0_DEMO_FL.dat"));
      if (useMarker) Mdemo << FILE(STRING(folder<<"0_DEMO_M"<<markerID<<".dat"));
      // compute contact phase
      task->computeConstraintTime(FLdemo,Xdemo);
      // setup visualization
      task->updateVisualization(world,Xdemo);
      if (visualize) displayTrajectory(Xdemo,-1,world,"demonstration "); //world.watch(true,"Press Enter to start learning");
      // init corl algorithm
      arr paramLimit;
      task->getParamLimit(paramLimit);
      cbo = new ConBOpt(task->nParam,paramLimit,folder,method);
    }
      break;
  }

  // reload last dataset
  if (count>1) {
    /// load from file
    arr XPR2;
    XPR2 << FILE(STRING(folder<<"/"<<count<<"_"<<method<<"_Xref"<<".dat"));
    transferQbetweenTwoWorlds(X,XPR2,world,world_pr2);
    cbo->load(count);
    count++;
  }

  // go to initial position
  ti->gotoPosition(XdemoPR2[0]);

  /// ///////////////////////////////////////////////////////////////////////////////////////
  /// Learning loop ////////////////////////////////////////////////////////////////////////
  /// /////////////////////////////////////////////////////////////////////////////////////
  for(;;) {
    cout << "#############################################\niteration: " << count << endl;
    // search for next candidate
    cbo->evaluate(theta);
    // project theta into x
    switch (mode) {
      case DEMO_REC:
        break;
      case CORL_OPT:
        task->transformTrajectory(X,theta,Xdemo);
        break;
    }
    // execute trajectory on robot
    task->updateVisualization(world,X,Xdemo);
    ti->executeTrajectoryPlan(X,duration,true,visualize);
    // evaluate trajectory
    bool success = (useRos?task->success(ti->logM,Mdemo):true);
    double reward = (useRos?task->reward(ti->logFL):rnd.gauss());
    double cost = task->cost(X);
    cout << "cost: " << cost << ", reward: " << reward << ", success: " << success << endl;
    cbo->addDatapoint(theta,ARR(reward-cost),ARR(success));
    // bookkeeping
    ti->logging(folder,method,count);
    cbo->save(count);
    // reverse trajectory
    if (success) {
      if (useReverseMotion) ti->executeTrajectoryPlan(X,duration,false,false,true); // TODO: implement alternative motions
    } else if (pauseMotion){
      ti->pauseMotion();
    }
    // goto initial position
    ti->gotoPositionPlan(X[0],mlr::getParameter<double>("CORL/gotoInitDuration",5.));
    count++;
  }
  return 0;
}
