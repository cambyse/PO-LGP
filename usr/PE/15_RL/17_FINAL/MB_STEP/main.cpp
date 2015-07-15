#include <Algo/spline.h>
#include <Core/array.h>
#include <Gui/plot.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Ors/ors.h>
#include <pr2/roscom.h>
#include <System/engine.h>
#include "../../src/plotUtil.h"
#include "../../src/phase_optimization.h"
#include "../../src/traj_factory.h"
#include "../src/mb_strategy.h"
#include "../src/motion_interface.h"
#include "../src/task_manager.h"

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);
  bool useRos = MT::getParameter<bool>("useRos");
  bool visualize = MT::getParameter<bool>("visualize");
  double duration = MT::getParameter<double>("duration");
  MT::String folder = MT::getParameter<MT::String>("folder");

  ors::KinematicWorld world(STRING("../model.kvg"));
  DoorTask *task = new DoorTask(world);
  Motion_Interface *mi;
  if (useRos) mi = new Motion_Interface(world);

  arr Xreverse;
  uint count;

  /// ----- load demonstration ---------------------------
  arr Xdemo,Fdemo,Mdemo;
  if (MT::getParameter<bool>("loadDemoFromFile")) {
    Xdemo << FILE(STRING(folder<<"/Xmb.dat"));
    Fdemo << FILE(STRING(folder<<"/Fdemo.dat"));
    Mdemo << FILE(STRING(folder<<"/Mdemo.dat"));

//    Xdemo << FILE(STRING(folder<<"/mbX.dat"));
//    Fdemo << FILE(STRING(folder<<"/mbFLact.dat"));
//    Mdemo << FILE(STRING(folder<<"/Mdemo.dat"));
//    count = 15;

  } else {
    mi->recordDemonstration(Xdemo,duration);
    if (visualize) displayTrajectory(Xdemo,-1,world,"demonstration ");
    world.watch(true);
    mi->gotoPosition(Xdemo[0]);
    mi->executeTrajectory(Xdemo,duration);

    Xdemo = mi->Xact;
    Fdemo = mi->FLact;
    Mdemo = mi->Mact;
    write(LIST<arr>(Xdemo),STRING(folder<<"/Xdemo.dat"));
    write(LIST<arr>(Fdemo),STRING(folder<<"/Fdemo.dat"));
    write(LIST<arr>(Mdemo),STRING(folder<<"/Mdemo.dat"));

    count = 0;

    if (useRos) {Xreverse = Xdemo; Xreverse.reverseRows(); mi->executeTrajectory(Xreverse,duration);}
  }


  /// compute contact phase
  task->computeConstraintTime(Fdemo);

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
      world.watch(true,"press enter to visualize trajectory");
      displayTrajectory(Xn,Xn.d0,world,"");
      world.watch(true,"press enter to execute trajectory");
    }

    /// execute trajectory on robot
    if (useRos) mi->executeTrajectory(Xn,duration);

    /// evaluate cost function
    bool result;
    if (useRos) {
      result = task->success(mi->Mact, Mdemo);
      cout << "result: " << result << endl;

      /// logging
      if (result) {
        X = Xn;
        mi->logging(STRING(folder<<"/mb"),count);
        if (useRos) {Xreverse = X; Xreverse.reverseRows(); mi->executeTrajectory(Xreverse,duration);}
      } else { break; }

    }else{
      write(LIST<arr>(X),STRING(folder<<"/mbX.dat"));
      write(LIST<arr>(X),STRING(folder<<"/mbX"<<count<<".dat"));
      result = true;
    }

    count++;
  }
  return 0;
}
