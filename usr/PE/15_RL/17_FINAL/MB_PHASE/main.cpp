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

  arr Xdemo,Mdemo,FLdemo;
  Xdemo << FILE(STRING(folder<<"/mbX.dat"));
  Mdemo << FILE(STRING(folder<<"/Mdemo.dat"));
  FLdemo << FILE(STRING(folder<<"/mbFLact.dat"));
  arr x0 = Xdemo[0];

  task->computeConstraintTime(FLdemo);

  world.gl().resize(800,800);
  task->updateVisualization(world,Xdemo);
  if (visualize) displayTrajectory(Xdemo,-1,world,"demonstration");

  arr X = Xdemo;
  uint count = 0;
  for (;;) {
    /// apply phase optimization
    uint k = 2;
    PhaseOptimization P(X,k,1);
    arr sOpt = P.getInitialization();
    optConstrained(sOpt, NoArr, Convert(P),OPT(verbose=1,stopTolerance=1e-4));
    arr Xres;
    P.getSolution(Xres,sOpt);
    arr Xn = Xres;

    if (visualize) {
      world.watch(true,"press enter to visualize trajectory");
      displayTrajectory(Xn,Xn.d0,world,"");
      world.watch(true,"press enter to execute candidate");
    }

    /// goto iniitial position
    if (useRos) mi->gotoPosition(x0);

    /// execute trajectory on robot
    if (useRos) mi->executeTrajectory(Xn,duration);

    /// evaluate cost function
    bool result;
//    cout << "Enter result:  success [1] or failure [0]: "<<endl;
    //    std::cin >> result;
    result = task->success(mi->Mact, Mdemo);
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
