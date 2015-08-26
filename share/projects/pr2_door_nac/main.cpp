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
#include "src/plotUtil.h"
#include "src/traj_factory.h"
#include "src/task_door.h"


int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  /// load parameters
  bool useRos = MT::getParameter<bool>("useRos");
  bool visualize = MT::getParameter<bool>("visualize");
  double duration = MT::getParameter<double>("duration");
  MT::String folder = MT::getParameter<MT::String>("folder");


  ors::KinematicWorld world(STRING("model.kvg"));
  DoorTask *task = new DoorTask(world);


  /// load demonstration from file
  arr Xdemo,FLdemo,Mdemo;
  Xdemo << FILE(STRING(folder<<"/Xdemo.dat"));
  FLdemo << FILE(STRING(folder<<"/FLdemo.dat"));
  Mdemo << FILE(STRING(folder<<"/Mdemo.dat"));

  /// compute parts of motion where robot is in contact with handle
  task->computeConstraintTime(FLdemo,Xdemo);

  /// visualize demonstration
  world.gl().resize(800,800);
  task->updateVisualization(world,Xdemo);
  if (visualize) displayTrajectory(Xdemo,-1,world,"demonstration");

  /// define parameter limits
  arr paramLim;
  paramLim.append(~ARR(-0.06,0.06)); // hand opening
  paramLim.append(~ARR(-0.12,0.12)); // hand position

  /// set initial parameter
  arr param = ARR(0.,0.); param.flatten();

  uint count = 0;
  arr x0 = Xdemo[0];
  arr Xn;
  double y;
  /// learning loop
  for(;;) {

    /// update parameter
    param = param + randn(2)*0.01;

    /// transform trajectory w.r.t. param
    /// result tells if the transformation was successful or if some limit is violated
    bool result = task->transformTrajectory(Xn,param,Xdemo);

    if (visualize) {
      task->updateVisualization(world,Xn);
      world.watch(true,"press enter to visualize trajectory");
      displayTrajectory(Xn,Xn.d0,world,"");
      world.watch(true,"press enter to execute candidate");
    }

    /// compute reward function
    /// currently consist of one term that measures squared acceleration
    /// and another term that measures how close the current trajectory is to the demonstration

    if (result) {
      arr Xdd;
      getAcc(Xdd,Xn,1.);
      y = exp(-0.5*sumOfAbs(Xdd)) + exp(-0.5*(sumOfAbs(Xn-Xdemo)/Xn.d0));
    }else{
      y = 0.;
    }

    cout << "\nIteration: " << count << endl;
    cout << "param: " << param << endl;
    cout << "reward: " << y << endl;

    count++;
  }

  return 0;
}
