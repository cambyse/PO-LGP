#include <Algo/spline.h>
#include <Core/array.h>
#include <Gui/plot.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Motion/phase_optimization.h>
#include <Ors/ors.h>
#include <pr2/roscom.h>
#include <pr2/trajectoryInterface.h>

#include "src/conbopt.h"
#include "../../src/task_manager.h"
#include "../../src/plotUtil.h"

enum MODE {DEMO_REC=0,PARAM_OPT=1,DOF_OPT=2};

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  /// ///////////////////////////////////////////////////////////////////////////////////////
  /// Initialization ///////////////////////////////////////////////////////////////////////
  /// /////////////////////////////////////////////////////////////////////////////////////
  // init parameters
  MODE mode = MODE(mlr::getParameter<uint>("mode"));
  bool useRos = mlr::getParameter<bool>("useRos");
  bool useReverseMotion = mlr::getParameter<bool>("useReverseMotion",false);
  bool useMarker = mlr::getParameter<bool>("useMarker",false);
  bool pauseMotion = mlr::getParameter<bool>("pauseMotion",false);
  bool visualize = mlr::getParameter<bool>("visualize");
  double duration = mlr::getParameter<double>("duration");
  mlr::String taskName = mlr::getParameter<mlr::String>("taskName");
  mlr::String folder = mlr::getParameter<mlr::String>("folder");
  uint count = mlr::getParameter<uint>("count",1);
  uint markerID; if (useMarker) markerID = mlr::getParameter<uint>("markerID");

  // init worlds + trajectory interface
  ors::KinematicWorld world_pr2("../../../../share/projects/pr2_gamepadControl/model.kvg");
  ors::KinematicWorld world;
  switch (mode) {
    case DEMO_REC:
      world = ors::KinematicWorld(STRING(folder<<"model.kvg"));
      break;
    case PARAM_OPT:
      world = ors::KinematicWorld(STRING(folder<<"model.kvg"));
      break;
    case DOF_OPT:
      world = ors::KinematicWorld(STRING(folder<<"modelaug.kvg"));
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
      method = STRING("Demo");
      ti->recordDemonstration(XdemoPR2,duration,0.05,5.);
      if (visualize) { world_pr2.watch(true,"Press Enter to visualize motion"); displayTrajectory(XdemoPR2,-1,world_pr2,"demonstration "); world_pr2.watch(true,"Press Enter to execute motion"); }
      ti->gotoPosition(XdemoPR2[0]);
      ti->executeTrajectory(XdemoPR2,duration,true);
      ti->logging(folder,method,0);
      if (useReverseMotion) {ti->executeTrajectory(XdemoPR2,duration);}
      return 0;
    }
      break;
    case PARAM_OPT:
    {
      method = STRING("Param");
      // load demonstration
      XdemoPR2 << FILE(STRING(folder<<"0_Demo_X.dat"));
      transferQbetweenTwoWorlds(Xdemo,XdemoPR2,world,world_pr2);
      FLdemo << FILE(STRING(folder<<"0_Demo_FL.dat"));
      if (useMarker) Mdemo << FILE(STRING(folder<<"0_Demo_M"<<markerID<<".dat"));
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
    case DOF_OPT:
    {
      method = STRING("Dof");
      // load demonstration
      XdemoPR2 << FILE(STRING(folder<<"0_Dof_X.dat"));
      Xdemo << FILE(STRING(folder<<"0_Dof_X_Aug.dat")); // contains augmented joint values
      FLdemo << FILE(STRING(folder<<"0_Demo_FL.dat"));
      if (useMarker) Mdemo << FILE(STRING(folder<<"0_Demo_M"<<markerID<<".dat"));
      // compute contact phase
      task->computeConstraintTime(FLdemo,Xdemo);
      // setup visualization
      task->updateVisualization(world,Xdemo);
      if (visualize) displayTrajectory(Xdemo,-1,world,"demonstration "); //world.watch(true,"Press Enter to start learning");
      // init corl algorithm
      arr dofLimit;
      task->getDofLimit(dofLimit);
      cbo = new ConBOpt(task->nDof,dofLimit,folder,method);
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
    cout << "\niteration: " << count << endl;
    // search for next candidate
    cbo->evaluate(theta);
    // project theta into x
    switch (mode) {
      case DEMO_REC:
        break;
      case PARAM_OPT:
        task->transformTrajectory(X,theta,Xdemo);
        break;
      case DOF_OPT:
        task->transformTrajectoryDof(X,theta,Xdemo);
        break;
    }
    // execute trajectory on robot
    task->updateVisualization(world,X,Xdemo);
    ti->executeTrajectoryPlan(X,duration,true,visualize);
    // evaluate trajectory
    bool success = (useRos?task->success(ti->logM,Mdemo):true);
    double reward = (useRos?task->reward(ti->logFL):rnd.gauss());
    double cost = task->cost(X);
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
    ti->gotoPositionPlan(X[0]);
    count++;
  }
  return 0;
}
