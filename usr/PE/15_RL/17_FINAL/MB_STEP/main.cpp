#include <Algo/spline.h>
#include <Core/array.h>
#include <Gui/plot.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Kin/kin.h>
#include <pr2/roscom.h>
#include <pr2/trajectoryInterface.h>
#include "../../src/traj_factory.h"
#include "../src/mb_strategy.h"
#include "../src/task_manager.h"
#include "../../src/plotUtil.h"

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
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
  world.gl().resize(800,800);
  arr Xreverse;
  uint count;

  /// ----- load demonstration ---------------------------
  arr Xdemo,Fdemo,Mdemo;
  if (mlr::getParameter<bool>("loadDemoFromFile")) {
    Xdemo << FILE(STRING(folder<<"/Xdemo.dat"));
    Fdemo << FILE(STRING(folder<<"/Fdemo.dat"));
    Mdemo << FILE(STRING(folder<<"/Mdemo.dat"));
    count = 0;


//    Xdemo << FILE(STRING(folder<<"/mfX4.dat"));
//    Fdemo << FILE(STRING(folder<<"/mfFLact4.dat"));
//    Mdemo << FILE(STRING(folder<<"/mfMact4.dat"));
//    Xdemo << FILE(STRING(folder<<"/phaseX.dat"));
//    Fdemo << FILE(STRING(folder<<"/phaseFLact.dat"));
//    Mdemo << FILE(STRING(folder<<"/phaseMact.dat"));
//    count = 27;
  } else {
    mi->recordDemonstration(Xdemo,duration);

    if (visualize) { world.watch(true,"Press Enter to play motion"); displayTrajectory(Xdemo,-1,world,"demonstration "); world.watch(true,"Press Enter to continue"); }
    mi->gotoPosition(Xdemo[0]);
    mi->executeTrajectory(Xdemo,duration,true);

    Xdemo = mi->logXact;  Fdemo = mi->logFLact; Mdemo = mi->logMact;
    write(LIST<arr>(Xdemo),STRING(folder<<"/Xdemo.dat")); write(LIST<arr>(Fdemo),STRING(folder<<"/Fdemo.dat")); write(LIST<arr>(Mdemo),STRING(folder<<"/Mdemo.dat"));

    count = 0;
    if (useRos) {Xreverse = Xdemo; Xreverse.reverseRows(); mi->executeTrajectory(Xreverse,duration);}
  }

  /// compute contact phase
  task->computeConstraintTime(Fdemo,Xdemo);

  arr x0 = Xdemo[0];
  arr X = Xdemo;
  arr Xn = Xdemo;

  /// setup visualization
  world.gl().resize(800,800);
  task->updateVisualization(world,Xdemo);

  if (visualize) displayTrajectory(Xdemo,-1,world,"demonstration ");
  world.watch(true);

  MB_strategy mbs(Xdemo,world,duration,*task);

  for(;;){
    cout << "iteration: " << count << endl;
    /// goto iniitial position
    if (useRos) mi->gotoPosition(x0);

    /// search for next candidate
    mbs.evaluate(Xn);

    if (visualize) {
      task->updateVisualization(world,Xn);
//      world.watch(true,"press enter to visualize trajectory");
      displayTrajectory(Xn,Xn.d0,world,"");
//      world.watch(true,"press enter to execute trajectory");
    }

    /// execute trajectory on robot
    if (useRos) mi->executeTrajectory(Xn,duration,true);

    /// evaluate cost function
    bool result;
    if (useRos) {
      result = task->success(mi->logMact, Mdemo);
      cout << "result: " << result << endl;

      /// logging
      if (result) {
        X = Xn;
        mi->logging(STRING(folder<<"/mb"),count);
        if (useRos) {Xreverse = X; Xreverse.reverseRows(); mi->executeTrajectory(Xreverse,duration);}
      } else { mi->pauseMotion(); }
    }else{
      write(LIST<arr>(X),STRING(folder<<"/mbX.dat"));
      write(LIST<arr>(X),STRING(folder<<"/mbX"<<count<<".dat"));
      result = true;
    }
    count++;
  }
  return 0;
}
