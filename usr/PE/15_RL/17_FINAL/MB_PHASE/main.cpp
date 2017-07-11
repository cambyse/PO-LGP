#include <Algo/spline.h>
#include <Core/array.h>
#include <Plot/plot.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Optim/optimization.h>
#include <Kin/kin.h>
#include <pr2/roscom.h>
#include <Motion/phase_optimization.h>
#include <pr2/trajectoryInterface.h>

#include "../../src/plotUtil.h"
#include "../../src/traj_factory.h"
#include "../src/task_manager.h"

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  cout << mlr::getParameter<double>("duration") << endl;
  bool useRos = mlr::getParameter<bool>("useRos");
  bool visualize = mlr::getParameter<bool>("visualize");
  double duration = mlr::getParameter<double>("duration");
  mlr::String folder = mlr::getParameter<mlr::String>("folder");
  mlr::String taskName = mlr::getParameter<mlr::String>("taskName");

  mlr::KinematicWorld world(STRING("../model.kvg"));
  TaskManager *task;
  if (taskName == "door") {
    task = new DoorTask(world);
  } else if (taskName == "grasp") {
    task = new GraspTask(world);
  }

  TrajectoryInterface *mi;
  if (useRos) mi = new TrajectoryInterface(world);

  arr Xdemo,Mdemo,FLdemo;
  Xdemo << FILE(STRING(folder<<"/mbX.dat"));
  Mdemo << FILE(STRING(folder<<"/Mdemo.dat"));
  FLdemo << FILE(STRING(folder<<"/mbFLact.dat"));
  arr x0 = Xdemo[0];

  task->computeConstraintTime(FLdemo,Xdemo);

  world.gl().resize(800,800);
  task->updateVisualization(world,Xdemo);
  if (visualize) displayTrajectory(Xdemo,-1,world,"demonstration");

  arr X = Xdemo;
  uint count = 0;
  for (;;) {
    cout << "iteration: " << count << endl;
    /// apply phase optimization
    uint k = 2;
    PhaseOptimization P(X,k,1);
    arr sOpt = P.getInitialization();
    optConstrainedMix(sOpt, NoArr, Convert(P),OPT(verbose=1,stopTolerance=1e-4));
    arr Xres;
    P.getSolution(Xres,sOpt);
    arr Xn = Xres;

    if (visualize) {
//      world.watch(true,"press enter to visualize trajectory");
      displayTrajectory(Xn,Xn.d0,world,"");
//      world.watch(true,"press enter to execute candidate");
    }

    /// goto iniitial position
    if (useRos) mi->gotoPosition(x0);

    /// execute trajectory on robot
    if (useRos) mi->executeTrajectory(Xn,duration,true);

    /// evaluate cost function
    bool result;
    result = task->success(mi->logMact, Mdemo);
    cout << "result: " << result << endl;

    /// logging
    if (result) {
      X = Xn;
      if (useRos) mi->logging(STRING(folder<<"/phase"),count);
      count++;

      arr Xreverse = X; Xreverse.reverseRows();
      if (useRos) mi->executeTrajectory(Xreverse,duration);
    }else{break;}
  }

  return 0;
}
