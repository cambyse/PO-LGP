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

  goalMO = new MObject(&world, MT::String("goal"), MObject::GOAL , 0.000, ARRAY(1.,0.,0.));
  amex = new AdaptiveMotionExecution(world,refPlan,dtAmex,TRef,x0,q0,*goalMO,true);

  world.setJointState(q0,qd0);
}

void AmexController::startController(){
  ros::Rate loop_rate(1./dtAmex);
  double dtReal = dtAmex;

  /// Run Controller until goal is reached
  while (ros::ok() && (amex->s.last() < 0.95))
  {
    double last_time = ros::Time::now().toSec();
    runAmex(dtReal);
    loop_rate.sleep();
    dtReal = ros::Time::now().toSec() - last_time ;
    cout << "Time: " << dtReal << endl;
  }
  ROS_INFO("-------------------------");
  ROS_INFO("AMEX CONTROLLER: COMPLETE");
  ROS_INFO("-------------------------");

  /// Set Velocity to zero after execution
  for(uint i=0;i<NUM_JOINTS;i++) setPosTargetSrv.request.vel[i] = 0.;
  for(uint i=0;i<NUM_JOINTS;i++) setVecTargetSrv.request.vel[i] = 0.;
  setPosTargetClient.call(setPosTargetSrv);
  setVecTargetClient.call(setVecTargetSrv);
}

void AmexController::runAmex(double dtReal) {
    /// get current goal
    if (useGoalPub) {
      getGoalClient.call(getGoalSrv);
      goalMO->setPosition(refFrame + ARRAY(getGoalSrv.response.x,getGoalSrv.response.y,getGoalSrv.response.z));
    }

    /// get current state of real time controllers
    getJointStateClient.call(getJointStateSrv);
    for(uint i=0;i<NUM_JOINTS;i++) q(i) = getJointStateSrv.response.q[i];
    for(uint i=0;i<NUM_JOINTS;i++) qd(i) = getJointStateSrv.response.qd[i];

    if (dtReal > 1.) {
      dtReal = dtAmex;
    }
    cout << "qd: " <<qd << endl;

    /// Get current task state
    arr stateVec, yNext, ydNext;
    world.setJointState(q,qd);
    world.kinematicsPos(state,NoArr,world.getBodyByName("endeffR")->index);
    world.kinematicsVec(stateVec,NoArr,world.getBodyByName("endeffR")->index);
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
  pos_gains = {150,150,30,30,10,6,6};
  vel_gains = { 60, 60, 4,10, 2,2,2};
//  vel_gains = { 0.,0.,0.,0.,0.,0.,0.};
  cout << "pos_gains" << pos_gains << endl;
  cout << "vel_gains" << vel_gains << endl;

  for(uint i=0;i<NUM_JOINTS;i++) setJointGainsSrv.request.pos_gains[i] = pos_gains(i);
  for(uint i=0;i<NUM_JOINTS;i++) setJointGainsSrv.request.vel_gains[i] = vel_gains(i);
  setJointGainsClient.call(setJointGainsSrv);

  /// Check Initial State
  getJointStateClient.call(getJointStateSrv);
  for(uint i=0;i<NUM_JOINTS;i++) q(i) = getJointStateSrv.response.q[i];
  for(uint i=0;i<NUM_JOINTS;i++) qd(i) = getJointStateSrv.response.qd[i];

  world.setJointState(q,qd);

  arr stateVec;
  world.kinematicsPos(state,NoArr,world.getBodyByName("endeffR")->index);
  world.kinematicsVec(stateVec,NoArr,world.getBodyByName("endeffR")->index);
  state.append(stateVec);
  cout << "robot initial state: " << state << endl;
  cout << "plan initial state: " << x0 << endl;
}

void AmexController::initRosServices(){
  setPosTargetClient = nh.serviceClient<tree_controller_pkg::SetPosTarget>("/tree_rt_controller/set_pos_target",true);
  setVecTargetClient = nh.serviceClient<tree_controller_pkg::SetVecTarget>("/tree_rt_controller/set_vec_target",true);
  getJointStateClient = nh.serviceClient<tree_controller_pkg::GetJointState>("/tree_rt_controller/get_joint_state",true);
  setJointGainsClient = nh.serviceClient<tree_controller_pkg::SetJointGains>("/tree_rt_controller/set_joint_gains",true);

  setPosTargetSrv.request.pos.resize(NUM_JOINTS); setPosTargetSrv.request.vel.resize(NUM_JOINTS);
  setVecTargetSrv.request.pos.resize(NUM_JOINTS); setVecTargetSrv.request.vel.resize(NUM_JOINTS);
  setJointGainsSrv.request.pos_gains.resize(NUM_JOINTS); setJointGainsSrv.request.vel_gains.resize(NUM_JOINTS);

  getJointStateClient.waitForExistence();
  setPosTargetClient.waitForExistence();
  setVecTargetClient.waitForExistence();
  setJointGainsClient.waitForExistence();

  if (useGoalPub) {
    refFrame = ARRAY(world.getBodyByName("torso_lift_link")->X.pos);
    getGoalClient = nh.serviceClient<goal_publisher::GetGoal>("/get_goal",true);
    getGoalClient.waitForExistence();
  }
}
