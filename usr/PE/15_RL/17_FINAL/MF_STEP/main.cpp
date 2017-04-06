#include <Algo/spline.h>
#include <Core/array.h>
#include <Plot/plot.h>
#include <Gui/opengl.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Optim/optimization.h>
#include <Kin/kin.h>
#include <pr2/roscom.h>


#include <pr2/trajectoryInterface.h>

#include "../../src/plotUtil.h"
#include "../../src/traj_factory.h"
#include "../src/mf_strategy.h"
#include "../src/task_manager.h"

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  bool useRos = mlr::getParameter<bool>("useRos");
  bool visualize = mlr::getParameter<bool>("visualize");
  double duration = mlr::getParameter<double>("duration");
  mlr::String folder = mlr::getParameter<mlr::String>("folder");
  mlr::String taskName = mlr::getParameter<mlr::String>("taskName");

  mlr::KinematicWorld world(STRING("../model.kvg"));
  TrajectoryInterface *mi;
  DoorTask *task = new DoorTask(world);
  if (useRos) mi = new TrajectoryInterface(world);

  arr Xdemo,FLdemo,Mdemo;
  Xdemo << FILE(STRING(folder<<"/phaseX.dat"));
  FLdemo << FILE(STRING(folder<<"/phaseFLact.dat"));
  Mdemo << FILE(STRING(folder<<"/Mdemo.dat"));

  task->computeConstraintTime(FLdemo,Xdemo);

  world.gl().resize(800,800);
  task->updateVisualization(world,Xdemo);
  if (visualize) displayTrajectory(Xdemo,-1,world,"demonstration");

  /// initialize model free strategy
  arr paramLim;
  paramLim.append(~ARR(-0.06,0.06)); // hand opening
  paramLim.append(~ARR(-0.12,0.12)); // hand position
  MF_strategy *mfs = new MF_strategy(2,paramLim,taskName);
  arr x0 = Xdemo[0];

  arr Xreverse, x, Xn;
  double y, ys;

  uint count;
  if (mlr::getParameter<bool>("MF_restartLearning")) {
    x = ARR(0.0,0.); x.flatten();

    task->transformTrajectory(Xn,x,Xdemo);
    if (useRos) {
      mi->gotoPosition(x0); mi->executeTrajectory(Xn,duration,true);
      y = task->reward(mi->logFLact);
      ys = 1.;
      Xreverse = Xn; Xreverse.reverseRows(); mi->executeTrajectory(Xreverse,duration);
    } else {y = task->reward(Xn); ys = 1.;}

    mfs->addDatapoint(x,ARR(y),ARR(ys));
    cout <<"x = "<< x <<"   | y = "<<y<<"   | ys = "<<ys << endl;

    count = 0;
  } else {
    mfs->load(STRING(folder<<"/"));
    count = mfs->X.d0-1;
  }


  for(;;) {
    cout << "\n Iteration: " << count << endl;
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
      mi->executeTrajectory(Xn,duration,true);
      /// evaluate cost functions
      if (result) result = task->success(mi->logMact, Mdemo);
      y = task->reward(mi->logFLact);
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
    if (useRos && result) { mi->logging(STRING(folder<<"/mf"),count); }

    if (result) {
      if (useRos) {Xreverse = Xn; Xreverse.reverseRows(); mi->executeTrajectory(Xreverse,duration);}
    } else {
      if (useRos) {mi->pauseMotion();}
    }

    count++;
  }

  mi->~TrajectoryInterface();
  return 0;
}
