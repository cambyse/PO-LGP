#include <Algo/spline.h>
#include <Core/array.h>
#include <Gui/plot.h>
#include <Gui/opengl.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Ors/ors.h>
#include <pr2/roscom.h>
#include <System/engine.h>
#include <Motion/pr2_heuristics.h>
#include "../../src/plotUtil.h"
#include "../../src/phase_optimization.h"
#include "../../src/traj_factory.h"
#include "../src/mf_strategy.h"
#include "../src/motion_interface.h"
#include "../src/task_manager.h"



int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  bool useRos = MT::getParameter<bool>("useRos");
  bool visualize = MT::getParameter<bool>("visualize");
  double duration = MT::getParameter<double>("duration");
  MT::String folder = MT::getParameter<MT::String>("folder");

  ors::KinematicWorld world(STRING("../model.kvg"));
  Motion_Interface *mi;
  DoorTask *task = new DoorTask(world);
  if (useRos) mi = new Motion_Interface(world);

  arr Xdemo,FLdemo,Mdemo;
  Xdemo << FILE(STRING(folder<<"/phaseX.dat"));
  FLdemo << FILE(STRING(folder<<"/phaseFLact.dat"));
  Mdemo << FILE(STRING(folder<<"/Mdemo.dat"));

  task->computeConstraintTime(FLdemo);

  world.gl().resize(800,800);
  task->updateVisualization(world,Xdemo);
  if (visualize) displayTrajectory(Xdemo,-1,world,"demonstration");

  /// initialize model free strategy
  arr paramLim;
  paramLim.append(~ARR(-0.04,0.04)); // hand opening
  paramLim.append(~ARR(-0.1,0.1)); // hand position
  MF_strategy *mfs = new MF_strategy(2,paramLim);
  arr x0 = Xdemo[0];

  arr Xreverse, x, Xn;
  double y, ys;

  uint count;
  if (MT::getParameter<bool>("MF_restartLearning")) {
    x = ARR(0.0,0.); x.flatten();

    task->transformTrajectory(Xn,x,Xdemo);
    if (useRos) {
      mi->gotoPosition(x0); mi->executeTrajectory(Xn,duration);
      Xreverse = Xn; Xreverse.reverseRows(); mi->executeTrajectory(Xreverse,duration);
      y = task->reward(mi->FLact);
      ys = 1.;
    } else {y = task->reward(Xn); ys = 1.;}

    mfs->addDatapoint(x,ARR(y),ARR(ys));
    cout << x <<" "<<y<<" "<<ys << endl;

    count = 0;
  } else {
    mfs->load(STRING(folder<<"/"));
    count = mfs->X.d0-1;
  }


  for(;;) {
    /// choose next datapoint
    mfs->evaluate(x);

    /// transform trajectory
    x.flatten();
    bool result = task->transformTrajectory(Xn,x,Xdemo);

    if (visualize) {
      task->updateVisualization(world,Xn);
//      world.watch(true,"press enter to visualize trajectory");
      displayTrajectory(Xn,Xn.d0,world,"");
//      world.watch(true,"press enter to execute candidate");
    }
    x0 = Xn[0];


    if (useRos&&result) {
      /// goto iniitial position
      mi->gotoPosition(x0);
      /// execute trajectory on robot
      mi->executeTrajectory(Xn,duration);
      /// evaluate cost functions
      if (result) result = task->success(mi->Mact, Mdemo);
      y = task->reward(mi->FLact);
      ys = (result) ?(1.):(-1.);

    } else {
      /// dummy values for debugging
      y = task->reward(Xn);
      ys = (result) ?(1.):(-1.);
    }

    mfs->addDatapoint(x,ARR(y),ARR(ys));
    cout <<"x = "<< x <<" y = "<<y<<" ys = "<<ys << endl;


    /// logging
    mfs->save(STRING(folder<<"/"));
    if (useRos) {
      mi->logging(STRING(folder<<"/mf"),count);
    }

    if (result) {
      if (useRos) {Xreverse = Xn; Xreverse.reverseRows(); mi->executeTrajectory(Xreverse,duration);}
    } else {
      if (useRos) {mi->sendZeroGains(20.);}
    }

    count++;
  }

  mi->~Motion_Interface();
  return 0;
}
