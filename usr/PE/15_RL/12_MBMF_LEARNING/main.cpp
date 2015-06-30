#include <Algo/spline.h>
#include <Core/array.h>
#include <Gui/plot.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Ors/ors.h>
#include <pr2/roscom.h>
#include <System/engine.h>
#include "../src/plotUtil.h"
#include "../src/phase_optimization.h"
#include "../src/traj_factory.h"
#include "mb_strategy.h"
#include "mf_strategy.h"
#include "motion_interface.h"
#include "mbmfl.h"
#include "task_manager.h"

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);
  bool useRos = MT::getParameter<bool>("useRos");
  bool visualize = MT::getParameter<bool>("visualize");
  double duration = MT::getParameter<double>("duration");

  /// -----------------------------------------------------------
  /// initialization --------------------------------------------
  /// -----------------------------------------------------------
  MT::String sceneName = MT::getParameter<MT::String>("sceneName");
  cout <<"Task: "<< sceneName << endl;
  ors::KinematicWorld world(STRING(sceneName<<".ors"));
  cout << world.getJointByName("l_gripper_joint")->qIndex << endl;
  cout << world.getJointByName("l_gripper_r_finger_tip_joint")->qIndex << endl;
  cout << world.getJointByName("l_wrist_roll_joint")->qIndex << endl;
  cout << world.getJointByName("l_forearm_roll_joint")->qIndex << endl;
  cout << world.getJointByName("l_elbow_flex_joint")->qIndex << endl;
  cout << world.getJointByName("l_upper_arm_roll_joint")->qIndex << endl;
  TaskManager *tm;
  if (sceneName=="donut") {
    tm = new DonutTask();
  } else if (sceneName=="door") {
    tm = new DoorTask();
  }
  Motion_Interface *mi;
  if (useRos) mi = new Motion_Interface(world);

  /// -----------------------------------------------------------
  /// load demonstration ----------------------------------------
  /// -----------------------------------------------------------
  arr Xdemo;
  if (MT::getParameter<bool>("loadDemoFromFile"))
  { //-- load demo from file
    Xdemo << FILE(STRING("data/"<<sceneName<<"Xdemo"));
  }
  else
  { //-- record demo from robot (& save to file)
    if (useRos) {
      mi->recordDemonstration(Xdemo,duration);
    } else {
      tm->createSynthethicDemonstration(Xdemo,world);
    }
    write(LIST<arr>(Xdemo),STRING("data/"<<sceneName<<"Xdemo"));
  }
  arr X = Xdemo;
  arr x0 = Xdemo[0];
  tm->initTask(world,Xdemo);

  /// -----------------------------------------------------------
  /// visualize demonstration -----------------------------------
  /// -----------------------------------------------------------
  world.gl().resize(800,800);
  TrajFactory tf;
  arr Pdemo1,P1,Pdemo2,P2;
  tf.compFeatTraj(Xdemo,Pdemo1,world,new DefaultTaskMap(posTMT,world,"endeffC1"));
  tf.compFeatTraj(Xdemo,P1,world,new DefaultTaskMap(posTMT,world,"endeffC1"));
  tf.compFeatTraj(Xdemo,Pdemo2,world,new DefaultTaskMap(posTMT,world,"endeffC2"));
  tf.compFeatTraj(Xdemo,P2,world,new DefaultTaskMap(posTMT,world,"endeffC2"));

  world.gl().add(drawRedLine,&(Pdemo1));
  world.gl().add(drawYellowLine,&(P1));
  world.gl().add(drawGreenLine,&(Pdemo2));
  world.gl().add(drawBlueLine,&(P2));

  if (visualize) {
    displayTrajectory(Xdemo,Xdemo.d0,world,"demonstration");
    world.watch(true);
  }

  /// -----------------------------------------------------------
  /// Initialize Learning Structures ----------------------------
  /// -----------------------------------------------------------
  MBMFL *mbmfl = new MBMFL(sceneName);

  if (MT::getParameter<bool>("loadMBMFL")) {mbmfl->loadMBMFL();}
  enum LEARNING_MODE {MODEL_BASED_STEP=0,MODEL_BASED_PHASE=1,MODEL_FREE=2};
  LEARNING_MODE learning_mode = /*MODEL_FREE;//*/MODEL_BASED_STEP;

  MB_strategy mbs(Xdemo,world,*tm);
  MF_strategy mfs(tm->paramDim,*tm);
  arr X_old = X;
  arr param = ARR(0.0);
  double paramR;


  /// -----------------------------------------------------------
  /// LEARNING LOOP ---------------------------------------------
  /// -----------------------------------------------------------
  for(;;){
    /// initialize robot and scene
    if (useRos) mi->gotoPosition(x0);

    /// select next trajectory X
    switch (learning_mode){
      case (MODEL_BASED_STEP):{ //-- model based step improvement strategy
        cout << "Model based step improvement" << endl;
        mbs.evaluate(X);
        break;}
      case (MODEL_BASED_PHASE):{ //-- model based phase improvement strategy
        cout << "Model based phase improvement" << endl;
        uint k = 2;
        PhaseOptimization P(X,k,1);
        arr sOpt = P.getInitialization();
        optConstrained(sOpt, NoArr, Convert(P),OPT(verbose=1,stopTolerance=1e-4));
        arr Xres;
        P.getSolution(Xres,sOpt);
        X = Xres;
        break;}
      case (MODEL_FREE):{  //-- model free improvement strategy
        cout << "Model free improvement" << endl;
        mfs.evaluate(param,paramR,mbmfl->data_param,mbmfl->data_result,mbmfl->data_cost);
        cout << "parameter " << param << endl;
        tm->applyModelFreeExploration(X,X_old,param);
        break;}
    }

    if (visualize) {
      tf.compFeatTraj(X,P1,world,new DefaultTaskMap(posTMT,world,"endeffC1"));
      tf.compFeatTraj(X,P2,world,new DefaultTaskMap(posTMT,world,"endeffC2"));
      displayTrajectory(X,X.d0,world,"next candidate");
      world.watch(true);
    }

    /// execute trajectory on robot
    if (useRos) mi->executeTrajectory(X,duration);

    /// evaluate cost function
    bool result;
    cout << "Enter result:  success [1] or failure [0]: "<<endl;
    std::cin >> result;
//    result = false;

    switch (learning_mode){
      case (MODEL_BASED_STEP):{ //-- model based step improvement strategy
        if (result) {
          X_old = X;
        } else{
          cout << "Switch to model based phase strategy" << endl;
          learning_mode = MODEL_BASED_PHASE;
        }
        break;}
      case (MODEL_BASED_PHASE): { //-- model based phase improvement strategy
        cout << "Switch to model free strategy" << endl;
        if (result) {
          X_old = X;
          tm->initTask(world,X_old);// reinit contact times
        }
        learning_mode = MODEL_FREE;
        mfs.recomputeCosts(*tm,X_old);
        arr Xtmp;
        tm->applyModelFreeExploration(Xtmp,X_old,param);
        paramR = tm->rewardFunction(Xtmp);
        mbmfl->addDatapoint(X_old,param,true,paramR);
        break; }
      case (MODEL_FREE): {  //-- model free improvement strategy
        if (result){
          mbmfl->addDatapoint(X,param,true,paramR);
        }else{
          mbmfl->addDatapoint(X,param,false,paramR);
        }
        break;}
    }
  }
  return 0;
}
