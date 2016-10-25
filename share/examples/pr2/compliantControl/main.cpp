#include <Core/module.h>
#include <Roopi/roopi.h>
#include <Control/TaskControllerModule.h>

#include <Roopi/act_followPath.h>

arr generateEightTrajectory(arr startPos) {
  arr traj0 = startPos;
  arr traj;

  double off = 0.1;
  double radius = 0.15;

  uint n = 100;

  arr t = linspace(0,.75,n);
  arr x_traj = sin((t+.125)*2.*M_PI)*radius;
  arr y_traj = cos((t+.125)*2.*M_PI)*radius;

  // circles
  arr c1 = catCol(traj0(0)+0.*x_traj,traj0(1)+y_traj-radius-off,traj0(2)+x_traj);
  arr c2 = catCol(traj0(0)+0.*x_traj,traj0(1)-y_traj+radius+off,traj0(2)+x_traj);

  // lines
  double m = (c1(0,2) - c2(c2.d0-1,2))/(c1(0,1) - c2(c2.d0-1,1));
  arr center = (c2[0] - c1[c1.d0-1])*.5+c1[c1.d0-1];
  arr xlin = linspace(c1(0,1),c2(0,1),n/3) -center(1);
  arr ylin = xlin*m;

  arr l1 = catCol(traj0(0)+0.*xlin,center(1)+xlin,center(2)-ylin);
  xlin.reverseRows();
  arr l2 = catCol(traj0(0)+0.*xlin,center(1)+xlin,center(2)-ylin);

  // circle 2
  traj.append(c1);
  traj.append(l1);
  traj.append(c2);
  traj.append(l2);

  traj.shift(round(l1.d0*0.5)*3);

  //traj.append(traj);
  return traj;
}


// =================================================================================================


void tests() {
  Roopi R;

  arr preTrajJointState = FILE("preTrajState");

  R.gotToJointConfiguration(preTrajJointState, 10.0);
  R.holdPosition();

  CtrlTask* c = R.createCtrlTask("eight", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffL"));
  c->setGains(20.0,5.0);
  c->setC(ARR(1000.0));
  arr traj = generateEightTrajectory(R.getTaskValue(c));
  R.followTaskTrajectory(c, 15.0, traj);
  //R.holdPosition(); //Hold position TODO: unsmooth

   mlr::wait(1.0);

    /*
  // move arms in a good position with motion planner
  arr endeffRAway = ARR(0.4,-0.8,.7);
  //R.goToPosition(endeffRAway, "endeffR", 5.0,true);

  arr trajStart = ARR(0.5,0.0,.7);
  //R.goToPosition(trajStart, "endeffL", 2.0);

  mlr::wait(1.0);

  //execute a eight trajectory in task space
  CtrlTask* c = R.createCtrlTask("eight", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffL"));
  c->setGains(30.0,5.0);
  c->setC(ARR(1000.0));
  arr traj = generateEightTrajectory(R.getTaskValue(c));
  R.followTaskTrajectory(c, 15.0, traj);

  // move endeff in task space with maxVel
  arr yEndeffL = R.getTaskValue(c);
  yEndeffL(0) += 0.3;
  R.modifyCtrlTaskGains(c, ARR(10.0), ARR(5.0), 0.05);
  R.modifyCtrlTaskReference(c, yEndeffL);

  mlr::wait(5.0);

  FollowPath fp(R, "circle", traj, new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffR"), 15.0);
fp.threadLoop();
  //moduleShutdown().waitForValueGreaterThan(0);
  */
}

void testKugel() {
  ors::KinematicWorld world(mlr::mlrPath("data/pr2_model/pr2_model.ors").p);
  world.watch(true);
}

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  tests();
  //testKugel();
  return 0;
}














#if 0
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

void bla() {
  //rosCheckInit("compliantControl");

#if 1
  Roopi R;

  //R.getPlanWorld().watch();

  //R.goToPosition(ARR(0.2,0.5,1.0), "endeffL", 10.0);

  CtrlTask* c = new CtrlTask("bla", new TaskMap_Default(posTMT, R.tcm()->modelWorld.get()(), "endeffL"));

  //arr traj = ~ARR(0.3,0.3,1.0);
  //traj.append(~ARR(0.5,0.3,0.3));
  //traj.append(~ARR(1.5,0.2,1.0));
  //arr y;
  //c->map.phi(y, NoArr, R.tcm()->modelWorld.get()());
  arr traj = generateEightTrajectory(R.getTaskValue(c));

  c->setGains(10.0,5.0);
  c->prec = 1000.0;

  R.followTaskTrajectory(c, 30.0, traj);

  R.syncPlanWorld();
  R.planWorld.watch();

mlr::wait(100.0);


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

   TaskMap_Default orientationMap(vecTMT, R.tcm()->modelWorld.get(), "endeffL", ors::Vector(1.0,0.0,0.0));
   CtrlTask* orientationLaw = new CtrlTask("endeffMove", &orientationMap);
   orientationLaw->maxAcc = 0.1;
   orientationLaw->maxVel = 0.5;
   Kp = diag(ARR(10.0,10.0,10.0));
   Kd = diag(ARR(5.0,5.0,5.0));
   orientationLaw->setGains(Kp,Kd);
   orientationLaw->setTarget(ARR(0.0,0.0,-1.0));
   orientationLaw->prec = 1000.0;
//   robot->addTask(orientationLaw);

   //R.tcm()->ctrlTasks.set() = { posLaw, orientationLaw };

   R.addCtrlTasks({posLaw, orientationLaw});

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

   TaskMap_Default forceMap(pos1DTMT, R.tcm()->modelWorld.get(), "endeffL", ors::Vector(.0,0.0,-1.0));
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
}

#endif
