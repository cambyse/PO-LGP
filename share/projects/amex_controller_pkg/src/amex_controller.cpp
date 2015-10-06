#include "amex_controller.h"

AmexController::AmexController(ros::NodeHandle &_nh, ors::KinematicWorld &_world, arr &_refPlan, arr &_q0, double _TRef, bool _useGoalPub) {
  nh = _nh;
  world = _world;
  useGoalPub = _useGoalPub;
  refPlan = _refPlan;
  dtAmex = MT::getParameter<double>("dtAmex");
  folder = MT::getParameter<String>("evalName");

  x0 = _refPlan[0];
  q0 = _q0; arr qd0 = 0.*q0;
  q = q0; qd =qd0;
  TRef = _TRef;
  NUM_JOINTS = q0.d0;

  goalMO = new MObject(&world, MT::String("goal"), MObject::GOAL , 0.000, {1.,0.,0.});
  amex = new AdaptiveMotionExecution(world,refPlan,dtAmex,TRef,x0,q0,*goalMO,true);

  world.setJointState(q0,qd0);
}

void AmexController::startController(){
  ros::Rate loop_rate(1./dtAmex);
  double dtReal = dtAmex;

  cout << amex->goal << endl;
  /// Run Controller until goal is reached
  while (ros::ok() && (amex->s.last() < 0.97))
  {
    double last_time = ros::Time::now().toSec();
    runAmex(dtReal);
    loop_rate.sleep();
    dtReal = ros::Time::now().toSec() - last_time ;
    cout << "Time: " << dtReal << endl;
    cout << "Phase: " << amex->s.last() << endl;
  }

  /// Set Task PD Gains
//  setNaturalGainsSrv.request.decayTime = 0.5;
//  setNaturalGainsSrv.request.dampingRatio = 0.9;
//  setNaturalGainsClient.call(setNaturalGainsSrv);

  /// Set Velocity to zero after execution and position to target position
//  for(uint i=0;i<3;i++) setPosTargetSrv.request.pos[i] = amex->goal(i);
  for(uint i=0;i<3;i++) setPosTargetSrv.request.vel[i] = 0.;
  for(uint i=0;i<3;i++) setVecTargetSrv.request.vel[i] = 0.;
  setPosTargetClient.call(setPosTargetSrv);
  setVecTargetClient.call(setVecTargetSrv);

  ROS_INFO("-------------------------");
  ROS_INFO("AMEX CONTROLLER: COMPLETE");
  ROS_INFO("-------------------------");
}

void AmexController::runAmex(double dtReal) {
  /// get current goal
  if (useGoalPub) {
    getGoalClient.call(getGoalSrv);
    goalMO->setPosition(refFrame + ARR(getGoalSrv.response.x,getGoalSrv.response.y,getGoalSrv.response.z));
  }

  /// get current state of real time controllers
  getJointStateClient.call(getJointStateSrv);
  for(uint i=0;i<NUM_JOINTS;i++) q(i) = getJointStateSrv.response.q[i];
  for(uint i=0;i<NUM_JOINTS;i++) qd(i) = getJointStateSrv.response.qd[i];

  if (dtReal > 1.) {
    dtReal = dtAmex;
  }

  /// Get current task state
  arr stateVec, yNext, ydNext;
  world.setJointState(q,qd);
  world.kinematicsPos(state,NoArr,world.getBodyByName("endeffR"));
  world.kinematicsVec(stateVec,NoArr,world.getBodyByName("endeffR"));
  state.append(stateVec);


  /// Adapt motion and compute next state
  amex->iterate(state,dtReal);
  amex->getNextState(yNext,ydNext);

  /// Send next target to Realtime Controller
  for(uint i=0;i<3;i++) setPosTargetSrv.request.pos[i] = yNext(i);
  for(uint i=0;i<3;i++) setVecTargetSrv.request.pos[i] = yNext(i+3);
  for(uint i=0;i<3;i++) setPosTargetSrv.request.vel[i] = ydNext(i);
  for(uint i=0;i<3;i++) setVecTargetSrv.request.vel[i] = ydNext(i+3);
  setPosTargetClient.call(setPosTargetSrv);
  setVecTargetClient.call(setVecTargetSrv);

  cout << "\n \n" << endl;
}

void AmexController::initController(){
  /// Set Joint PD Gains
  ///[0.5,0.5,0.5,0.5,0.1,0.1,0.1]
  ///[400,400,250,200,100,100,100]
  ///[3.,3.,4,3.,2.,4,1.5]
  acc_gains = {0.25,0.25,0.25,0.25,0.1,0.1,0.1};
  i_gains = {200,200,150,100,100,100,100};
  i_claim = {3.,3.,3.,3.,2.,3.,1.5};

  acc_gains = acc_gains*20.;
  i_gains = i_gains*3.;
  cout << "acc_gains" << acc_gains << endl;
  cout << "i_gains" << i_gains << endl;
  cout << "i_claim" << i_claim << endl;

  for(uint i=0;i<NUM_JOINTS;i++) setJointGainsSrv.request.acc_gains[i] = acc_gains(i);
  for(uint i=0;i<NUM_JOINTS;i++) setJointGainsSrv.request.i_gains[i] = i_gains(i);
  for(uint i=0;i<NUM_JOINTS;i++) setJointGainsSrv.request.i_claim[i] = i_claim(i);
  setJointGainsClient.call(setJointGainsSrv);

  /// Set Filter Parameters
  setFilterGainsSrv.request.u_filt = 0.997;
  setFilterGainsSrv.request.qd_filt = 0.95;
  setFilterGainsClient.call(setFilterGainsSrv);

  /// Set Task PD Gains
  setNaturalGainsSrv.request.decayTime = dtAmex;
  setNaturalGainsSrv.request.dampingRatio = 0.9;
  setNaturalGainsClient.call(setNaturalGainsSrv);

  /// Check Initial State
  getJointStateClient.call(getJointStateSrv);
  for(uint i=0;i<NUM_JOINTS;i++) q(i) = getJointStateSrv.response.q[i];
  for(uint i=0;i<NUM_JOINTS;i++) qd(i) = getJointStateSrv.response.qd[i];

  world.setJointState(q,qd);

  arr stateVec;
  world.kinematicsPos(state,NoArr,world.getBodyByName("endeffR"));
  world.kinematicsVec(stateVec,NoArr,world.getBodyByName("endeffR"));
  state.append(stateVec);
  cout << "robot initial state: " << state << endl;
  cout << "plan initial state: " << x0 << endl;
}

void AmexController::initRosServices(){
  setPosTargetClient = nh.serviceClient<tree_controller_pkg::SetPosTarget>("/tree_rt_controller/set_pos_target",true);
  setVecTargetClient = nh.serviceClient<tree_controller_pkg::SetVecTarget>("/tree_rt_controller/set_vec_target",true);
  getJointStateClient = nh.serviceClient<tree_controller_pkg::GetJointState>("/tree_rt_controller/get_joint_state",true);
  setJointGainsClient = nh.serviceClient<tree_controller_pkg::SetJointGains>("/tree_rt_controller/set_joint_gains",true);
  setTaskGainsClient = nh.serviceClient<tree_controller_pkg::SetTaskGains>("/tree_rt_controller/set_task_gains",true);
  setNaturalGainsClient = nh.serviceClient<tree_controller_pkg::SetNaturalGains>("/tree_rt_controller/set_natural_gains",true);
  setFilterGainsClient = nh.serviceClient<tree_controller_pkg::SetFilterGains>("/tree_rt_controller/set_filter_gains",true);

  setPosTargetSrv.request.pos.resize(3); setPosTargetSrv.request.vel.resize(3);
  setVecTargetSrv.request.pos.resize(3); setVecTargetSrv.request.vel.resize(3);
  setJointGainsSrv.request.acc_gains.resize(NUM_JOINTS);
  setJointGainsSrv.request.i_claim.resize(NUM_JOINTS); setJointGainsSrv.request.i_gains.resize(NUM_JOINTS);

  getJointStateClient.waitForExistence();
  setPosTargetClient.waitForExistence();
  setVecTargetClient.waitForExistence();
  setJointGainsClient.waitForExistence();
  setTaskGainsClient.waitForExistence();
  setNaturalGainsClient.waitForExistence();
  setFilterGainsClient.waitForExistence();

  if (useGoalPub) {
    refFrame = conv_vec2arr(world.getBodyByName("torso_lift_link")->X.pos);
    getGoalClient = nh.serviceClient<goal_publisher::GetGoal>("/get_goal",true);
    getGoalClient.waitForExistence();
  }
}
