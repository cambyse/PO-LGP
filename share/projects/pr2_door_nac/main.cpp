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
#include "src/motion_interface.h"



int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  /// load parameters
  bool loadFunctionalFile = mlr::getParameter<bool>("loadFunctionalFile");
  bool useRos = mlr::getParameter<bool>("useRos");
  bool visualize = mlr::getParameter<bool>("visualize");
  double duration = mlr::getParameter<double>("duration");
  mlr::String folder = mlr::getParameter<mlr::String>("folder");


  ors::KinematicWorld world(STRING("model.kvg"));
  DoorTask *task = new DoorTask(world);
  Motion_Interface *mi;
  if (useRos) mi = new Motion_Interface(world);

  arr Xreverse;
  arr Xdemo,Fdemo,Mdemo;

  /// load demonstration from file or record from demonstration
  if (mlr::getParameter<bool>("loadDemoFromFile")) {
    Xdemo << FILE(STRING(folder<<"/Xdemo.dat"));
    Fdemo << FILE(STRING(folder<<"/Fdemo.dat"));
    Mdemo << FILE(STRING(folder<<"/Mdemo.dat"));
  } else {
    mi->recordDemonstration(Xdemo,duration);
    if (visualize) displayTrajectory(Xdemo,-1,world,"demonstration ");
    world.watch(true);
    mi->gotoPosition(Xdemo[0]);
    mi->executeTrajectory(Xdemo,duration,true);

    Xdemo = mi->Xact;
    Fdemo = mi->FLact;
    Mdemo = mi->Mact;
    write(LIST<arr>(Xdemo),STRING(folder<<"/Xdemo.dat"));
    write(LIST<arr>(Fdemo),STRING(folder<<"/Fdemo.dat"));
    write(LIST<arr>(Mdemo),STRING(folder<<"/Mdemo.dat"));

    if (useRos) {Xreverse = Xdemo; Xreverse.reverseRows(); mi->executeTrajectory(Xreverse,duration);}
  }

  /// compute contact phase
  task->computeConstraintTime(Fdemo,Xdemo);

  /// define parameter limits
  arr paramLim;
  paramLim.append(~ARR(-0.06,0.06)); // hand opening
  paramLim.append(~ARR(-0.12,0.12)); // hand position

  /// set initial parameter
  arr param = ARR(-0.06,-0.12);
  param.flatten();

  uint count = 0;
  arr Xn;
  double y;
  /// learning loop
  double best_reward = 0;
  arr best_param;





  int numEpisode = 3; //number of episodes to evaluate Gradient (at each iteration);
  int H = 1; //horizon
  int numCentres = 1;
  //int numRuns = 10; // runs for averaging performance
  int numIterations = 30; //number of gradient updates
  uint kernel_type = 0;// 0 is RBF Kernel
  mdp::RKHSPol rkhs1(world,useRos,Xdemo,Fdemo,Mdemo,paramLim,numCentres,H,numEpisode,kernel_type,numIterations);
  mlr::rnd.clockSeed();
  arr rewards;
  rkhs1.dim_A = 2;


  rkhs1.Algorithm = atoi(argv[1]);//NAC


  arr start(1);//multi-armed bandits setting
  start(0) = 0.0;
  rkhs1.setStart(start); //this always be called lastly


  if(loadFunctionalFile) rkhs1.loadOldFuncPolicy();
  rewards = rkhs1.run();

  if(rkhs1.Algorithm==0)
      write(LIST<arr>(rewards),STRING("results_PG.dat"));
  else
      write(LIST<arr>(rewards),STRING("results_NPG.dat"));

/*/

  arr forces;
  for(;;) {
      param = param + ARR(0.03,0.05) ;//= param + rand(2)*0.01;

      bool result = task->transformTrajectory(Xn,param,Xdemo);

      if (visualize) {
        task->updateVisualization(world,Xn);
        world.watch(true,"press enter to visualize trajectory");
        displayTrajectory(Xn,Xn.d0,world,"");
        world.watch(true,"press enter to execute candidate");
      }

      /// execute trajectory Xn
      if (result && useRos) {
        mi->gotoPosition(Xn[0]);
        mi->executeTrajectory(Xn,duration,true);
        forces = mi->FLact;
        Xreverse = Xn; Xreverse.reverseRows(); mi->executeTrajectory(Xreverse,duration);
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
/*/
  return 0;
}
