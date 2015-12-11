#include <Algo/spline.h>
#include <Core/array.h>
#include <Gui/plot.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Ors/ors.h>
#include <pr2/roscom.h>
#include <pr2/trajectoryInterface.h>

#include <Motion/phase_optimization.h>
#include "src/mb_strategy.h"
#include "../../src/task_manager.h"
#include "../../src/plotUtil.h"

enum MODE {MB_STEP=0,PHASE_STEP=1};

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  /// load parameters
  MODE mode = MODE(mlr::getParameter<uint>("mode"));
  bool useRos = mlr::getParameter<bool>("useRos");
  bool visualize = mlr::getParameter<bool>("visualize");
  double duration = mlr::getParameter<double>("duration");
  mlr::String taskName = mlr::getParameter<mlr::String>("taskName");
  mlr::String folder = mlr::getParameter<mlr::String>("folder");
  uint count = mlr::getParameter<uint>("count",0);


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

  folder = STRING(folder<<"/mb");

  /// init trajectory interface
  TrajectoryInterface *mi;
  if (useRos) mi = new TrajectoryInterface(world);
  world.gl().update(); world.gl().resize(800,800);
  arr Xreverse, Xbase, FLbase, Mbase;

  if (mlr::getParameter<bool>("recordDemonstration")) {
    /// record demonstration
    mi->recordDemonstration(Xbase,duration);

    if (visualize) { world.watch(true,"Press Enter to play motion"); displayTrajectory(Xbase,-1,world,"demonstration "); world.watch(true,"Press Enter to continue"); }
    mi->gotoPosition(Xbase[0]);
    mi->executeTrajectory(Xbase,duration,true);

    Xbase = mi->logXact;  FLbase = mi->logFL; Mbase = mi->logM;
    write(LIST<arr>(Xbase),STRING(folder<<"/Xbase.dat")); write(LIST<arr>(FLbase),STRING(folder<<"/Fbase.dat")); write(LIST<arr>(Mbase),STRING(folder<<"/Mbase.dat"));
    mi->logging(folder,count);
    count++;
    if (useRos) {Xreverse = Xbase; Xreverse.reverseRows(); mi->executeTrajectory(Xreverse,duration);}
  } else {
    /// load from file
    Xbase << FILE(STRING(folder<<"X"<<count<<".dat"));
    FLbase << FILE(STRING(folder<<"FL"<<count<<".dat"));
    Mbase << FILE(STRING(folder<<"M"<<count<<".dat"));
    count++;
  }

  /// compute contact phase
  task->computeConstraintTime(FLbase,Xbase);
  displayTrajectory(Xbase,-1,world,"");
  arr x0 = Xbase[0];
  arr X = Xbase;
  arr Xn = Xbase;

  /// setup visualization
  task->updateVisualization(world,Xbase);
  if (visualize) displayTrajectory(Xbase,-1,world,"demonstration ");
  world.watch(true);


  MB_strategy mbs(Xbase,world,duration,*task);

  for(;;) {
    cout << "iteration: " << count << endl;
    /// goto iniitial position
    if (useRos) mi->gotoPosition(x0);

    /// search for next candidate
    switch (mode) {
      case MB_STEP:
        mbs.evaluate(Xn);
        break;
      case PHASE_STEP:
        PhaseOptimization P(X,2);
        arr sOpt = P.getInitialization();
        optConstrainedMix(sOpt, NoArr, Convert(P),OPT(verbose=1,stopTolerance=mlr::getParameter<double>("phase_stopTolerance")));
        P.getSolution(Xn,sOpt);
        break;
    }

    /// visualize trajectory candidate
    if (visualize) {
      task->updateVisualization(world,Xn);
      world.watch(true,"press enter to visualize trajectory");
      displayTrajectory(Xn,Xn.d0,world,"");
      world.watch(true,"press enter to execute trajectory");
    }
    /// execute trajectory on robot
    if (useRos) mi->executeTrajectory(Xn,duration,true);

    /// evaluate cost function
    bool result;
    if (useRos) {
      result = task->success(mi->logM, Mbase);
      cout << "result: " << result << endl;

      /// logging
      if (result) {
        X = Xn;
        mi->logging(folder,count);
        if (useRos) {Xreverse = X; Xreverse.reverseRows(); mi->executeTrajectory(Xreverse,duration);}
      } else { mi->pauseMotion(); }
    }else{
      write(LIST<arr>(X),STRING(folder<<"X.dat"));
      write(LIST<arr>(Xn),STRING(folder<<"X"<<count<<".dat"));
      write(LIST<arr>(FLbase),STRING(folder<<"FL"<<count<<".dat"));
      write(LIST<arr>(Mbase),STRING(folder<<"M"<<count<<".dat"));

      result = true;
    }
    count++;
  }
  return 0;
}
