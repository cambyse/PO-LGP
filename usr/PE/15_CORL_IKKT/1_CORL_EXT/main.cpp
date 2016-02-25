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

#include "../1_CORL_PE/src/mf_strategy.h"
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
//  ors::KinematicWorld world(STRING(folder<<"model.kvg"));
  ors::KinematicWorld worldaug(STRING(folder<<"modelaug.kvg"));
  ors::KinematicWorld world_pr2("../../../../share/projects/pr2_gamepadControl/model.kvg");

  for (uint i=0;i<world_pr2.joints.N;i++) {
    cout << world_pr2.joints(i)->qIndex << " "<< world_pr2.joints(i)->name << endl;
  }

  return 0;


  TaskManager *task;
  if (taskName == "door") {
    task = new DoorTask(worldaug);
  } else if (taskName == "grasp") {
    task = new GraspTask(worldaug);
  } else if (taskName == "button") {
    task = new ButtonTask(worldaug);
  }

  /// init trajectory interface
  TrajectoryInterface *ti;
  ti = new TrajectoryInterface(worldaug,world_pr2);
  worldaug.gl().update(); worldaug.gl().resize(800,800);
  ti->world_pr2->gl().update(); ti->world_pr2->gl().resize(800,800);

  /// load base trajectory
  arr Xbase,FLbase,Mbase;
  Xbase << FILE(STRING(folder<<"Xaug.dat"));
  FLbase << FILE(STRING(folder<<"FLbase.dat"));
  if (useMarker) Mbase << FILE(STRING(folder<<"Maug.dat"));

  folder = STRING(folder<<"/DE_");
  taskName = STRING(taskName<<"DE");

  task->computeConstraintTime(FLbase,Xbase);
  task->updateVisualization(worldaug,Xbase);
  if (visualize) displayTrajectory(Xbase,-1,worldaug,"demonstration");

  /// initialize model free strategy
  arr dofLimit;
  task->getDofLimit(dofLimit);
  MF_strategy *mfs = new MF_strategy(dofLimit.d0, dofLimit, folder,taskName);
  arr x0;
  arr Xreverse, x, Xn, Xn_pr2;
  double y, ys;

  uint count = 0;
  if (mlr::getParameter<bool>("restartLearning")) {
    /// restart learning
    x = zeros(dofLimit.d0); x.flatten();
    task->transformTrajectoryDof(Xn,x,Xbase);
    x0 = Xn[0];

    if (useRos) {
      ti->gotoPositionPlan(x0); ti->executeTrajectory(Xn,duration,true);
      y = task->reward(ti->logFL);
      ys = 1.;
      ti->logging(folder,count);
      write(LIST<arr>(Xn),STRING(folder<<"Xref"<<count<<"aug.dat"));
      if (execReverseMotion) {Xreverse = Xn; Xreverse.reverseRows(); ti->executeTrajectoryPlan(Xreverse,duration);}
    } else {
      /// dummy values for simulation
      y = task->reward(Xn);
      ys = 1.;
      transferQbetweenTwoWorlds(Xn_pr2,Xn,world_pr2,worldaug);
      write(LIST<arr>(Xn_pr2),STRING(folder<<"Xref"<<count<<".dat"));
      write(LIST<arr>(Xn_pr2),STRING(folder<<"X"<<count<<".dat"));
      write(LIST<arr>(Xn),STRING(folder<<"Xref"<<count<<"aug.dat"));
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
    Xn.clear();
    bool result = task->transformTrajectoryDof(Xn,x,Xbase);

    if (visualize) {
      task->updateVisualization(worldaug,Xn);
      worldaug.watch(true,"press enter to visualize trajectory");
      displayTrajectory(Xn,Xn.d0,worldaug,"");
      worldaug.watch(true,"press enter to execute candidate");
    }
    x0 = Xn[0];

    if (!result) {cout << "result=false" << endl;return 0; }

    if (useRos) {
      /// goto iniitial position
      ti->gotoPositionPlan(x0);
      /// execute trajectory on robot
      ti->executeTrajectoryPlan(Xn,duration,true);
      /// evaluate cost functions
      if (result) result = task->success(ti->logM, Mbase);
      y = task->reward(ti->logFL);
      ys = (result) ?(1.):(-1.);
    } else {
      /// dummy values for simulation
      y = task->reward(Xn);
      ys = (result) ?(1.):(-1.);
      transferQbetweenTwoWorlds(Xn_pr2,Xn,*ti->world_pr2,*ti->world_plan);
      write(LIST<arr>(Xn_pr2),STRING(folder<<"Xref"<<count<<".dat"));
      write(LIST<arr>(Xn_pr2),STRING(folder<<"X"<<count<<".dat"));
      write(LIST<arr>(FLbase),STRING(folder<<"FL"<<count<<".dat"));
      if (useMarker) write(LIST<arr>(Mbase),STRING(folder<<"M"<<count<<".dat"));
    }

    mfs->addDatapoint(x,ARR(y),ARR(ys));
    cout <<"x = "<< x <<" y = "<<y<<" ys = "<<ys << endl;

    /// logging
    mfs->save(folder);
    if (useRos && result) { ti->logging(folder,count); }

    if (result) {
      if (useRos && execReverseMotion) {Xreverse = Xn; Xreverse.reverseRows(); ti->executeTrajectoryPlan(Xreverse,duration);}
    } else {
      if (useRos) {ti->pauseMotion();}
    }
    count++;
  }

  ti->~TrajectoryInterface();
  return 0;
}
