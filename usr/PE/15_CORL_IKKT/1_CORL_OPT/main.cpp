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

enum MODE {TRAJ_OPT=0,PHASE_OPT=1};

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  /// load parameters
  MODE mode = MODE(mlr::getParameter<uint>("mode"));
  bool useRos = mlr::getParameter<bool>("useRos");
  bool visualize = mlr::getParameter<bool>("visualize");
  bool execReverseMotion = mlr::getParameter<bool>("execReverseMotion",false);
  bool useMarker = mlr::getParameter<bool>("useMarker",false);
  double duration = mlr::getParameter<double>("duration");
  mlr::String taskName = mlr::getParameter<mlr::String>("taskName");
  mlr::String folder = mlr::getParameter<mlr::String>("folder");
  uint count = mlr::getParameter<uint>("count",0);

  /// init task
  ors::KinematicWorld world(STRING(folder<<"model.kvg"));
  ors::KinematicWorld world_pr2("../../../../share/projects/pr2_gamepadControl/model.kvg");

  TaskManager *task;
  if (taskName == "door") {
    task = new DoorTask(world);
  } else if (taskName == "grasp") {
    task = new GraspTask(world);
  } else if (taskName == "button") {
    task = new ButtonTask(world);
  }

  mlr::String folderMode;
  if (mode == TRAJ_OPT) {
    folderMode = STRING(folder<<"/TO_");
  } else if (mode == PHASE_OPT) {
    folderMode = STRING(folder<<"/PO_");
  }

  /// init trajectory interface
  TrajectoryInterface *ti;
  ti = new TrajectoryInterface(world,world_pr2);
  ti->world_pr2->gl().update(); ti->world_pr2->gl().resize(800,800);
  world.gl().update(); world.gl().resize(800,800);
  arr Xreverse, Xbase, Xbase_pr2, FLbase, Mbase;

  if (mlr::getParameter<bool>("recordDemonstration")) {
    /// record demonstration
    ti->recordDemonstration(Xbase_pr2,duration);

    if (visualize) { world.watch(true,"Press Enter to visualize motion"); displayTrajectory(Xbase,-1,world,"demonstration "); world.watch(true,"Press Enter to execute motion"); }
    ti->gotoPosition(Xbase_pr2[0]);
    ti->executeTrajectory(Xbase,duration,true);

    Xbase_pr2 = ti->logX;  FLbase = ti->logFL; Mbase = ti->logM;
    write(LIST<arr>(Xbase),STRING(folder<<"Xbase.dat")); write(LIST<arr>(FLbase),STRING(folder<<"FLbase.dat"));
    if (useMarker) write(LIST<arr>(Mbase),STRING(folder<<"Mbase.dat"));
    ti->logging(folderMode,count);
    count++;
    if (useRos && execReverseMotion) {Xreverse = Xbase_pr2; Xreverse.reverseRows(); ti->executeTrajectory(Xreverse,duration);}
  } else {
    /// load from file
    Xbase_pr2 << FILE(STRING(folder<<mlr::getParameter<mlr::String>("baseFile")<<".dat"));
    FLbase << FILE(STRING(folder<<"FLbase.dat"));
    if (useMarker) Mbase << FILE(STRING(folder<<"Mbase.dat"));
    count++;
  }

  /// goto iniitial position
  transferQbetweenTwoWorlds(Xbase,Xbase_pr2,*ti->world_plan,*ti->world_pr2);
  if (useRos) ti->gotoPosition(Xbase_pr2[0],10.);


  /// compute contact phase
  task->computeConstraintTime(FLbase,Xbase);
  displayTrajectory(Xbase,-1,world,"");
  arr x0 = Xbase[0];
  arr X = Xbase;
  arr Xn = Xbase;

  /// setup visualization
  task->updateVisualization(world,Xbase);
  if (visualize) displayTrajectory(Xbase,-1,world,"demonstration "); world.watch(true,"Press Enter to start learning");
  MB_strategy mbs(Xbase,world,duration,*task);

  for(;;) {
    cout << "\niteration: " << count << endl;
    /// goto iniitial position
    if (useRos) ti->gotoPositionPlan(x0);

    /// search for next candidate
    switch (mode) {
      case TRAJ_OPT:
        mbs.evaluate(Xn);
        break;
      case PHASE_OPT:
        PhaseOptimization P(X,2);
        arr sOpt = P.getInitialization();
        optConstrained(sOpt, NoArr, Convert(P),OPT(verbose=1,stopTolerance=mlr::getParameter<double>("phase_stopTolerance")));
        P.getSolution(Xn,sOpt);
        break;
    }

    /// visualize trajectory candidate
    if (visualize) {
      task->updateVisualization(world,Xn,Xbase);
      world.watch(true,"press enter to visualize trajectory");
      displayTrajectory(Xn,Xn.d0,world,"");
      world.watch(true,"press enter to execute trajectory");
    }
    /// execute trajectory on robot
    if (useRos) ti->executeTrajectoryPlan(Xn,duration,true);

    /// evaluate cost function
    bool result;
    if (useRos) {
      result = task->success(ti->logM, Mbase);
      cout << "result: " << result << endl;

      /// logging
      if (result) {
        X = Xn;
        ti->logging(folderMode,count);
        if (useRos && execReverseMotion) {Xreverse = X; Xreverse.reverseRows(); ti->executeTrajectoryPlan(Xreverse,duration);}
      } else { ti->pauseMotion(); }
    }else{
      /// for simulation only
      transferQbetweenTwoWorlds(Xbase_pr2,Xn,*ti->world_pr2,*ti->world_plan);
      write(LIST<arr>(Xbase_pr2),STRING(folderMode<<"Xref"<<count<<".dat"));
      write(LIST<arr>(Xbase_pr2),STRING(folderMode<<"X"<<count<<".dat"));
      write(LIST<arr>(FLbase),STRING(folderMode<<"FL"<<count<<".dat"));
      if (useMarker) write(LIST<arr>(Mbase),STRING(folderMode<<"M"<<count<<".dat"));

      result = true;
    }
    count++;
  }
  return 0;
}
