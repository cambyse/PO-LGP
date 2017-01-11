#include <RosCom/roscom.h>
#include <RosCom/spinner.h>
#include <Actions/gamepadControl.h>
#include <Control/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Kin/kinViewer.h>

#include <sensor_msgs/JointState.h>
#include <RosCom/baxter.h>


#include <Motion/taskMap_default.h>
#include <Control/taskController.h>

#include <Roopi/roopi.h>


// =================================================================================================

void setMoveUpTask(TaskControllerModule& tcm){
  TaskMap_Default *map = new TaskMap_Default(posTMT, tcm.modelWorld.get(), "endeffL");
  arr y;
  map->phi(y, NoArr, tcm.modelWorld.get());
  y(2) += .1;
  CtrlTask *task = new CtrlTask("endeff", map);
  task->prec = 1000.;
  task->setTarget(y);
  task->setGains(diag(ARR(10.,10.,10.)), diag(ARR(5.,5.,5.)));

  tcm.ctrlTasks.set() = { task };
}

// =================================================================================================

void setForceLimitTask(TaskControllerModule& tcm){
  TaskMap_Default *map = new TaskMap_Default(posTMT, tcm.modelWorld.get(), "endeffL");
  arr y;
  map->phi(y, NoArr, tcm.modelWorld.get());
  y(2) += .1;
  CtrlTask *task = new CtrlTask("endeff", map);
  task->prec = 1000.;
  task->setTarget(y);
  task->setGains(diag(ARR(10.,10.,10.)), diag(ARR(5.,5.,5.)));

  tcm.ctrlTasks.set() = { task };
}

// =================================================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("compliantControl");

#if 1
  Roopi R;
#else
  Access_typed<CtrlMsg> ctrl_ref(NULL, "ctrl_ref");
  Access_typed<CtrlMsg> ctrl_obs(NULL, "ctrl_obs");
  Access_typed<arr>     pr2_odom(NULL, "pr2_odom");

  Access_typed<arr> q_ref(NULL, "q_ref");
  Access_typed<sensor_msgs::JointState> jointState(NULL, "jointState");

  TaskControllerModule tcm;

  OrsViewer view;
  OrsPoseViewer controlview({"ctrl_q_real", "ctrl_q_ref"}, tcm.realWorld);

  RosCom_Spinner spinner; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages

  if(mlr::getParameter<bool>("useRos")){
    mlr::String robot = mlr::getParameter<mlr::String>("robot", "pr2");
    if(robot=="pr2"){
      new SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> ("/marc_rt_controller/jointState", ctrl_obs);
      new PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>          ("/marc_rt_controller/jointReference", ctrl_ref);
      new SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>       ("/robot_pose_ekf/odom_combined", pr2_odom);
    }
    if(robot=="baxter"){
      new Subscriber<sensor_msgs::JointState> ("/robot/joint_states", jointState);
      new SendPositionCommandsToBaxter(tcm.modelWorld.get());
    }
  }

  threadOpenModules(true);
#endif

  mlr::wait(1.);
  cout <<"NOW!" <<endl;

//  setMoveUpTask(tcm);

  TaskMap_Default posMap(posTMT, R.tcm()->modelWorld.get(), "endeffL");
   CtrlTask* posLaw = new CtrlTask("endeffMove", &posMap);
   posLaw->maxAcc = 0.1;
   posLaw->maxVel = 0.1;
   arr Kp = diag(ARR(10.0,10.0,10.0));
   arr Kd = diag(ARR(5.0,5.0,5.0));
   posLaw->setGains(Kp,Kd);
   posLaw->setTarget(ARR(0.5,0.5,0.7));
   posLaw->prec = 1000.0;
//   robot->addTask(posLaw);

   TaskMap_Default orientationMap(vecTMT, R.tcm()->modelWorld.get(), "endeffL", mlr::Vector(1.0,0.0,0.0));
   CtrlTask* orientationLaw = new CtrlTask("endeffMove", &orientationMap);
   orientationLaw->maxAcc = 0.1;
   orientationLaw->maxVel = 0.5;
   Kp = diag(ARR(10.0,10.0,10.0));
   Kd = diag(ARR(5.0,5.0,5.0));
   orientationLaw->setGains(Kp,Kd);
   orientationLaw->setTarget(ARR(0.0,0.0,-1.0));
   orientationLaw->prec = 1000.0;
//   robot->addTask(orientationLaw);

   R.tcm()->ctrlTasks.set() = { posLaw, orientationLaw };

   while(true) {
     if(orientationLaw->isConverged(.05)) break;
     cout <<"ori err=" <<orientationLaw->error() <<endl;
     mlr::wait(0.1);
     if(moduleShutdown().getValue()>0) break;
   }
   cout << "converged" << endl;

   Kp = diag(ARR(10.0, 10.0, 0.0));
   Kd = diag(ARR(5.0, 5.0, 0.0));
   posLaw->setGains(Kp,Kd);
   //posLaw->prec = eye(3)*10.0;
   //posLaw->prec(2,2) = 0.0;

   TaskMap_Default forceMap(pos1DTMT, R.tcm()->modelWorld.get(), "endeffL", mlr::Vector(.0,0.0,-1.0));
   CtrlTask* forceLaw = new CtrlTask("endeffForce", &forceMap);
   forceLaw->setGains(ARR(0.0), ARR(20.0));
   forceLaw->setTarget(ARR(0.), ARR(0.1));
//   forceLaw->f_alpha = 0.001;
//   forceLaw->f_gamma = 0.9995;
//   forceLaw->f_ref = ARR(-2.0);
   forceLaw->prec = 1000.0;

   R.tcm()->ctrlTasks.set() = { posLaw, orientationLaw, forceLaw };

//   mlr::wait(4.0);
//   arr yRef = posLaw->y_ref;
//   for(uint i = 0; i < 30; i++) {
//     yRef(1) -= 0.01;
//     posLaw->setTarget(yRef);
//     mlr::wait(0.2);
//   }


//  for(;;){
////    cout <<R.tcm()->ctrl_obs.get()->fL <<endl;

//    if(moduleShutdown().getValue()>0) break;
//  }

  moduleShutdown().waitForValueGreaterThan(0);

#if 0
  threadCloseModules();
  cout <<"bye bye" <<endl;
#endif
  return 0;
}

