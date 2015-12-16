#include <Algo/spline.h>
#include <Core/array.h>
#include <Gui/plot.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/pr2_heuristics.h>
#include <Optim/optimization.h>
#include <Ors/ors.h>
#include <pr2/roscom.h>
#include <pr2/trajectoryInterface.h>

#include "src/mf_strategy.h"
#include "../../src/plotUtil.h"
#include "../../src/task_manager.h"
#include "../../src/plotUtil.h"

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  /// load parameters
  bool useRos = mlr::getParameter<bool>("useRos");
  bool visualize = mlr::getParameter<bool>("visualize",false);
  bool useMarker = mlr::getParameter<bool>("useMarker",false);
  bool execReverseMotion = mlr::getParameter<bool>("execReverseMotion",false);
  double duration = mlr::getParameter<double>("duration");
  mlr::String folder = mlr::getParameter<mlr::String>("folder");
  mlr::String taskName = mlr::getParameter<mlr::String>("taskName");

  /// init task
  ors::KinematicWorld world(STRING(folder<<"model.kvg"));
  TaskManager *task;
  if (taskName == "door") {
    task = new DoorTask(world);
  } else if (taskName == "grasp") {
    task = new GraspTask(world);
  } else if (taskName == "button") {
    task = new ButtonTask(world);
  }

  /// init trajectory interface
  TrajectoryInterface *mi;
  if (useRos) mi = new TrajectoryInterface(world);
  world.gl().update(); world.gl().resize(800,800);

  /// load base trajectory
  arr Xbase,FLbase,Mbase;
  Xbase << FILE(STRING(folder<<"/PO_Xref"<<mlr::getParameter<uint>("count")<<".dat"));
  FLbase << FILE(STRING(folder<<"/FLbase.dat"));
  if (useMarker) Mbase << FILE(STRING(folder<<"/mbM"<<mlr::getParameter<uint>("count")<<".dat"));

  folder = STRING(folder<<"/PE_");
  taskName = STRING(taskName<<"PE");

  task->computeConstraintTime(FLbase,Xbase);
  task->updateVisualization(world,Xbase);
  if (visualize) displayTrajectory(Xbase,-1,world,"demonstration");

  /// initialize model free strategy
  arr paramLimit;
  task->getParamLimit(paramLimit);
  MF_strategy *mfs = new MF_strategy(paramLimit.d0, paramLimit, folder, taskName);
  arr x0;
  arr Xreverse, x, Xn;
  double y, ys;

  uint count = 0;
  if (mlr::getParameter<bool>("MF_restartLearning")) {
    /// restart learning
    x = zeros(paramLimit.d0); x.flatten();
    task->transformTrajectory(Xn,x,Xbase);
    x0 = Xn[0];
    if (useRos) {
      mi->gotoPosition(x0); mi->executeTrajectory(Xn,duration,true);
      y = task->reward(mi->logFL);
      ys = 1.;
      mi->logging(folder,count);
      if (execReverseMotion) {Xreverse = Xn; Xreverse.reverseRows(); mi->executeTrajectory(Xreverse,duration);}
    } else {
      /// dummy values for simulation
      y = task->reward(Xn);
      ys = 1.;
      write(LIST<arr>(Xn),STRING(folder<<"X"<<count<<".dat"));
      write(LIST<arr>(FLbase),STRING(folder<<"FL"<<count<<".dat"));
      if (useMarker) write(LIST<arr>(Mbase),STRING(folder<<"M"<<count<<".dat"));
    }

    mfs->addDatapoint(x,ARR(y),ARR(ys));
    cout <<"x = "<< x <<"   | y = "<<y<<"   | ys = "<<ys << endl;
    count++;
  } else {
    /// reload last dataset
    mfs->load(STRING(folder));
    count = mfs->X.d0;
  }

  /// learning loop
  for(;;) {
    cout << "\n Iteration: " << count << endl;
    /// choose next datapoint
    mfs->evaluate(x);

    /// transform trajectory
    x.flatten();
    bool result = task->transformTrajectory(Xn,x,Xbase);

    if (visualize) {
      task->updateVisualization(world,Xn,Xbase);
      world.watch(true,"press enter to visualize trajectory");
      displayTrajectory(Xn,Xn.d0,world,"");
      world.watch(true,"press enter to execute candidate");
    }
    x0 = Xn[0];

    if (useRos&&result) {
      /// goto iniitial position
      mi->gotoPosition(x0);
      /// execute trajectory on robot
      mi->executeTrajectory(Xn,duration,true);
      /// evaluate cost functions
      if (result) result = task->success(mi->logM, Mbase);
      y = task->reward(mi->logFL);
      ys = (result) ?(1.):(-1.);
    } else {
      /// dummy values for debugging
      y = task->reward(Xn);
      ys = (result) ?(1.):(-1.);
      write(LIST<arr>(Xn),STRING(folder<<"X"<<count<<".dat"));
      write(LIST<arr>(Xn),STRING(folder<<"Xref"<<count<<".dat"));
      write(LIST<arr>(FLbase),STRING(folder<<"FL"<<count<<".dat"));
      if (useMarker) write(LIST<arr>(Mbase),STRING(folder<<"M"<<count<<".dat"));
    }

    mfs->addDatapoint(x,ARR(y),ARR(ys));
    cout <<"x = "<< x <<" y = "<<y<<" ys = "<<ys << endl;

    /// logging
    mfs->save(folder);
    if (useRos && result) { mi->logging(folder,count); }

    if (result) {
      if (useRos && execReverseMotion) {Xreverse = Xn; Xreverse.reverseRows(); mi->executeTrajectory(Xreverse,duration);}
    } else {
      if (useRos) {mi->pauseMotion();}
    }
    count++;
  }

  mi->~TrajectoryInterface();
  return 0;
}
