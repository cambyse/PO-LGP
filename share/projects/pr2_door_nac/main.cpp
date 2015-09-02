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
#include "src/functional.h"



int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  /// load parameters
  bool useRos = MT::getParameter<bool>("useRos");
  bool visualize = MT::getParameter<bool>("visualize");
  double duration = MT::getParameter<double>("duration");
  MT::String folder = MT::getParameter<MT::String>("folder");


  ors::KinematicWorld world(STRING("model.kvg"));
  DoorTask *task = new DoorTask(world);
  arr Xdemo,FLdemo,Mdemo;
  Xdemo << FILE(STRING(folder<<"/Xdemo.dat"));
  FLdemo << FILE(STRING(folder<<"/FLdemo.dat"));
  Mdemo << FILE(STRING(folder<<"/Mdemo.dat"));



  /// define parameter limits
  arr paramLim;
  paramLim.append(~ARR(-0.06,0.06)); // hand opening
  paramLim.append(~ARR(-0.12,0.12)); // hand position

  /// set initial parameter
  arr param = ARR(-0.06,-0.12);
  param.flatten();

  uint count = 0;
  //arr x0 = Xdemo[0];
  arr Xn;
  double y;
  /// learning loop
  double best_reward = 0;
  arr best_param;




  int numEpisode =2; //number of episodes to evaluate Gradient (at each iteration);
  int H = 1; //horizon
  int numCentres = 1;
  //int numRuns = 10; // runs for averaging performance
  int numIterations=30; //number of gradient updates
  uint kernel_type = 1;// RBF Kernel
  mdp::RKHSPol rkhs1(world,Xdemo,FLdemo,Mdemo,paramLim,numCentres,H,numEpisode,kernel_type,numIterations);
  MT::rnd.clockSeed();
  arr rewards;
  rkhs1.Algorithm = 0;//NAC
  rkhs1.dim_A = 2;

  arr start(1);//multi-armed bandits setting
  start(0) = 0.0;
  rkhs1.setStart(start); //this always be called lastly

  rewards = rkhs1.run();


  for(;;) {
      param = param + ARR(0.03,0.05) ;//= param + rand(2)*0.01;

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

      cout<<"The reward is: "<<y<<endl;

  }

  return 0;
}
