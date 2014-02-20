// ROS INCLUDES
#include <ros/ros.h>
#include <ros/timer.h>
#include <ros/service_client.h>
#include <my_controller_pkg/SetState.h>
#include <my_controller_pkg/SetGains.h>
#include <my_controller_pkg/GetState.h>
#include <my_controller_pkg/GetGains.h>
#include <my_controller_pkg/SetFilter.h>
#include <goal_publisher/GetGoal.h>

#include <std_srvs/Empty.h>

// ORS INCLUDES
#include <Core/array.h>
#include <Gui/opengl.h>
#include <Ors/ors.h>
#include <Motion/feedbackControl.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include "../../../../examples/Motion/pfc/pfc.h"
#include "../../../../examples/Motion/splines/spline.h"
#include "../../../../examples/Motion/pfc/mobject.h"

// C++ INCLUDES
#include <GL/glu.h>
#include <iomanip>

#define VISUALIZATION 0
#define LOGGING 1

class TrajAmex{
private:
  ros::NodeHandle nh;         //
  MObject* goalMO;
  FeedbackMotionControl *FMC;
  ros::Timer timerRunAmex;

  std::vector<ros::ServiceClient> rt_set_state_clients;
  std::vector<ros::ServiceClient> rt_set_gains_clients;
  std::vector<ros::ServiceClient> rt_get_state_clients;
  std::vector<ros::ServiceClient> rt_get_gains_clients;
  std::vector<ros::ServiceClient> rt_set_filter_clients;
  ros::ServiceClient get_goal_client;
  goal_publisher::GetGoal goalSub;
  arr p_gains, d_gains;

  arr q, qd, qdd;             // Joint position, velocity, acceleration
  arr state;                  // Task position
  arr tauRef;                 // Reference trajectory
  double TRef;                // Final time of reference trajectory
  arr q0;                     // Initial position joint space
  arr x0;                     // Initial position task space
  double dtAmex;              // Time step between sending trajectories
  double regularization ;     // Regularization parameter for PD control
  arr refFrame;               // Reference Coordinate Frame
  bool useGoalPub;            // Use Goal from /get_goal topic

  // BOOKKEEPING VARS
  String folder;

  arr q_bk;    // joint angles at each time step
  arr x_bk;    // task variables at each time step
  arr goal_bk; // goal position at each time step
  arr ct_bk;   // computational time at each time step
  arr s_bk;    // phase variable at each time step


public:
  ors::KinematicWorld world;
  Pfc* pfc;
  PDtask *taskPos, *taskVec, *taskHome, *taskCol;

  TrajAmex(ros::NodeHandle &_nh) {
    nh = _nh;

    dtAmex = MT::getParameter<double>("dtAmex");
    folder = MT::getParameter<String>("evalName");

    rt_set_state_clients.push_back(nh.serviceClient<my_controller_pkg::SetState>("/ors_rt_r_upper_arm_roll_joint/set_state",true));
    rt_set_state_clients.push_back(nh.serviceClient<my_controller_pkg::SetState>("/ors_rt_r_shoulder_pan_joint/set_state",true));
    rt_set_state_clients.push_back(nh.serviceClient<my_controller_pkg::SetState>("/ors_rt_r_shoulder_lift_joint/set_state",true));
    rt_set_state_clients.push_back(nh.serviceClient<my_controller_pkg::SetState>("/ors_rt_r_forearm_roll_joint/set_state",true));
    rt_set_state_clients.push_back(nh.serviceClient<my_controller_pkg::SetState>("/ors_rt_r_elbow_flex_joint/set_state",true));
    rt_set_state_clients.push_back(nh.serviceClient<my_controller_pkg::SetState>("/ors_rt_r_wrist_flex_joint/set_state",true));
    rt_set_state_clients.push_back(nh.serviceClient<my_controller_pkg::SetState>("/ors_rt_r_wrist_roll_joint/set_state",true));

    rt_get_state_clients.push_back(nh.serviceClient<my_controller_pkg::GetState>("/ors_rt_r_upper_arm_roll_joint/get_state",true));
    rt_get_state_clients.push_back(nh.serviceClient<my_controller_pkg::GetState>("/ors_rt_r_shoulder_pan_joint/get_state",true));
    rt_get_state_clients.push_back(nh.serviceClient<my_controller_pkg::GetState>("/ors_rt_r_shoulder_lift_joint/get_state",true));
    rt_get_state_clients.push_back(nh.serviceClient<my_controller_pkg::GetState>("/ors_rt_r_forearm_roll_joint/get_state",true));
    rt_get_state_clients.push_back(nh.serviceClient<my_controller_pkg::GetState>("/ors_rt_r_elbow_flex_joint/get_state",true));
    rt_get_state_clients.push_back(nh.serviceClient<my_controller_pkg::GetState>("/ors_rt_r_wrist_flex_joint/get_state",true));
    rt_get_state_clients.push_back(nh.serviceClient<my_controller_pkg::GetState>("/ors_rt_r_wrist_roll_joint/get_state",true));

    rt_set_gains_clients.push_back(nh.serviceClient<my_controller_pkg::SetGains>("/ors_rt_r_upper_arm_roll_joint/set_gains"));
    rt_set_gains_clients.push_back(nh.serviceClient<my_controller_pkg::SetGains>("/ors_rt_r_shoulder_pan_joint/set_gains"));
    rt_set_gains_clients.push_back(nh.serviceClient<my_controller_pkg::SetGains>("/ors_rt_r_shoulder_lift_joint/set_gains"));
    rt_set_gains_clients.push_back(nh.serviceClient<my_controller_pkg::SetGains>("/ors_rt_r_forearm_roll_joint/set_gains"));
    rt_set_gains_clients.push_back(nh.serviceClient<my_controller_pkg::SetGains>("/ors_rt_r_elbow_flex_joint/set_gains"));
    rt_set_gains_clients.push_back(nh.serviceClient<my_controller_pkg::SetGains>("/ors_rt_r_wrist_flex_joint/set_gains"));
    rt_set_gains_clients.push_back(nh.serviceClient<my_controller_pkg::SetGains>("/ors_rt_r_wrist_roll_joint/set_gains"));

    rt_set_filter_clients.push_back(nh.serviceClient<my_controller_pkg::SetFilter>("/ors_rt_r_upper_arm_roll_joint/set_filter"));
    rt_set_filter_clients.push_back(nh.serviceClient<my_controller_pkg::SetFilter>("/ors_rt_r_shoulder_pan_joint/set_filter"));
    rt_set_filter_clients.push_back(nh.serviceClient<my_controller_pkg::SetFilter>("/ors_rt_r_shoulder_lift_joint/set_filter"));
    rt_set_filter_clients.push_back(nh.serviceClient<my_controller_pkg::SetFilter>("/ors_rt_r_forearm_roll_joint/set_filter"));
    rt_set_filter_clients.push_back(nh.serviceClient<my_controller_pkg::SetFilter>("/ors_rt_r_elbow_flex_joint/set_filter"));
    rt_set_filter_clients.push_back(nh.serviceClient<my_controller_pkg::SetFilter>("/ors_rt_r_wrist_flex_joint/set_filter"));
    rt_set_filter_clients.push_back(nh.serviceClient<my_controller_pkg::SetFilter>("/ors_rt_r_wrist_roll_joint/set_filter"));


    for (uint i = 0; i<rt_set_gains_clients.size() ;i++) {
      rt_set_state_clients[i].waitForExistence();
      rt_get_state_clients[i].waitForExistence();
      rt_set_filter_clients[i].waitForExistence();
    }

    get_goal_client = nh.serviceClient<goal_publisher::GetGoal>("/get_goal",true);
    if (get_goal_client.waitForExistence(ros::Duration(3.0))){
      useGoalPub=true;
      get_goal_client.call(goalSub);
      cout << "goal from ar tracker: " << endl;
      cout << goalSub.response.x << goalSub.response.y << goalSub.response.z << endl;
    } else {
      useGoalPub=false;
      cout << "goal from target.ors file used" << endl;
    }
  }

  // initialize ors
  void initOrs(int argc, char** argv, String scene) {
    MT::initCmdLine(argc,argv);

    world.init(scene);
#if VISUALIZATION
    world.gl().resize(800, 800);
#endif
    makeConvexHulls(world.shapes);
    world.getJointState(q, qd);
  }



  // Create reference trajectory from q0 to goal
  void createReferenceTrajectory() {
    // Plan Trajectory
    MotionProblem MP(world);
    MP.loadTransitionParameters();

    arr refGoal;
    refGoal = ARRAY(MP.world.getBodyByName("goalRef")->X.pos);
    refFrame = ARRAY(world.getBodyByName("torso_lift_link")->X.pos);
  /*  if (useGoalPub) {
      refFrame = ARRAY(world.getBodyByName("torso_lift_link")->X.pos);
      // Get Reference Goal
      get_goal_client.call(goalSub);
      refGoal = refFrame + ARRAY(goalSub.response.x,goalSub.response.y,goalSub.response.z);
      world.getBodyByName("goalRef")->X.pos = refGoal;
    } else{

    }
    */
    cout << "refGoal: "<<refGoal << endl;

    //-- create an optimal trajectory to goalRef
    arr qLimits = { -224, 44,   //17 r_upper_arm_roll_joint
                    -130, 40,   //18 r_shoulder_pan_joint
                    -30, 80,    //19 r_shoulder_lift_joint
                    -360, 360,  //20 r_forearm_roll_joint
                    -133, 0,    //21 r_elbow_flex_joint
                    -130, 0,    //22 r_wrist_flex_joint
                    -360, 360}; //23 r_wrist_roll_joint
    qLimits = M_PI*qLimits/180.;
    qLimits.reshape(7,2);

    //-- create an optimal trajectory to trainTarget
    TaskCost *c;
    c = MP.addTaskMap("position", new DefaultTaskMap(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));
    MP.setInterpolatingCosts(c, MotionProblem::finalOnly,
                             refGoal, 1e1,
                             ARRAY(0.,0.,0.), 1e-4);
    MP.setInterpolatingVelCosts(c, MotionProblem::finalOnly,
                                ARRAY(0.,0.,0.), 1e1,
                                ARRAY(0.,0.,0.), 0.);

    c = MP.addTaskMap("qlimits", new DefaultTaskMap(qLimitsTMT,world, NULL, NoVector, NULL, NoVector, qLimits));
    MP.setInterpolatingCosts(c,MotionProblem::constant,ARRAY(0.),1e2);

    c = MP.addTaskMap("homing", new DefaultTaskMap(qItselfTMT,world, NULL, NoVector, NULL, NoVector));
    MP.setInterpolatingCosts(c,MotionProblem::constant,ARRAY(0.),1e-3);

    q0 = {-10.,-20.,20.,0.,-20.,-20.,0.};
    //    q0 = M_PI*q0/180.;
    //    cout <<"q0 "<< q0 << endl;
    my_controller_pkg::GetState getStateSrv;
    for(uint i=0;i<rt_set_state_clients.size();i++) {
      rt_get_state_clients[i].call(getStateSrv);
      q0(i) = getStateSrv.response.pos;
    }
    cout << q0 << endl;

    //-- create the Optimization problem (of type kOrderMarkov)
    MP.x0 = q0;

    MotionProblemFunction MPF(MP);
    uint T=MPF.get_T();
    uint k=MPF.get_k();
    uint n=MPF.dim_x();
    double dt = MP.tau;
    cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

    arr x(T+1,n); x.setZero();
    optNewton(x, Convert(MPF), OPT(verbose=0, stopIters=20, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));
    MP.costReport(false);
    displayTrajectory(x, 1, world, "planned trajectory", 0.0001);


    //-- Transform trajectory into task space
    arr kinPos, kinVec, xRefPos, xRefVec;
    // store cartesian coordinates and endeffector orientation
    for (uint t=0;t<=T;t++) {
      world.setJointState(x[t]);
      world.calcBodyFramesFromJoints();
      world.kinematicsPos(kinPos,NoArr,MP.world.getBodyByName("endeff")->index);
      world.kinematicsVec(kinVec,NoArr,MP.world.getBodyByName("endeff")->index);
      xRefPos.append(~kinPos);
      xRefVec.append(~kinVec);
    }

    tauRef = ~cat(~xRefPos,~xRefVec);

    // set initial positions
    q0 = x[0]; arr dq0 = 0.*q0;
    q = q0; qd = dq0;
    x0 = tauRef[0];
    world.setJointState(q,qd);
    world.calcBodyFramesFromJoints();
    TRef = T*dt;
  }


  void initController() {
    ROS_INFO("INIT AMEX CONTROLLER: START");
    goalMO = new MObject(&world, MT::String("goal"), MObject::GOAL , 0.000, ARRAY(1.,0.,0.));

    FMC = new FeedbackMotionControl(world, false);
    regularization = MT::getParameter<double>("regularization");

    FMC->nullSpacePD.prec=0.;
    taskPos = FMC->addPDTask("pos", dtAmex*10, 1, posTMT, "endeff");
    taskVec = FMC->addPDTask("vec", dtAmex*5, 1, vecTMT, "endeff",ARR(0.,0.,1.));
    taskHome = FMC->addPDTask("home", .02, 0.5, qItselfTMT);

    double posGainP, posGainD, posPrec, vecGainP, vecGainD, vecPrec, homeGainP, homeGainD, homePrec;

    // Gains dependent on Frequency
    posGainP = 0.01/dtAmex;
    posGainD = 1./dtAmex;
    posPrec = 1.;
    vecGainP = 0.01/dtAmex;
    vecGainD = 1./dtAmex;
    vecPrec = 1.;

    homeGainP = 0;
    homeGainD = MT::getParameter<double>("homeGainD");
    homePrec = 0.1;

    cout << "dt " << dtAmex << endl;
    cout << "regularization " << regularization << endl;
    cout << "posGainP " << posGainP << endl;
    cout << "posGainD "<< posGainD << endl;
    cout << "vecGainP "<< vecGainP << endl;
    cout << "vecGainD "<< vecGainD << endl;
    cout << "homeGainD "<< homeGainD << endl;

    taskPos->setGains(posGainP,posGainD); taskPos->prec=posPrec;
    taskVec->setGains(vecGainP,vecGainD); taskVec->prec=vecPrec;
    taskHome->setGains(homeGainP,homeGainD); taskHome->prec=homePrec;

    pfc = new Pfc(world,tauRef,dtAmex,TRef,x0,q0,*goalMO,true);
    ROS_INFO("INIT AMEX CONTROLLER: COMPLETE");


    // Set PD Parameter
    //17 r_upper_arm_roll_joint
    //18 r_shoulder_pan_joint
    //19 r_shoulder_lift_joint
    //20 r_forearm_roll_joint
    //21 r_elbow_flex_joint
    //22 r_wrist_flex_joint -2Nm
    //23 r_wrist_roll_joint

    //    arr p_gains = {30,200,100,10,30,1,3};
    //    arr d_gains = {1,20,10,0.5,3,0,0};
    //    arr p_gains = {0.,0.,0.,0.,0.5,0.01,0.03};
    //    arr d_gains = {2,20,20,1,7,2,2};
    //    arr p_gains = {0.,0.,0.,0.,10,1,3};
    p_gains = {30,150,150,10,30,6,6};
    d_gains = {4,60,60,2,10,2,2};
    cout << "p_gains" << p_gains << endl;
    cout << "d_gains" << d_gains << endl;

    my_controller_pkg::SetGains setGainsSrv;
    for(uint i=0;i<rt_set_state_clients.size();i++) {
      setGainsSrv.request.pos_gain = p_gains(i);
      setGainsSrv.request.vel_gain = d_gains(i);
      setGainsSrv.request.i_gain = p_gains(i);
      rt_set_gains_clients[i].call(setGainsSrv);
    }

    arr i_max_param = {0.1, 1, 2, 0.1, 2, 2, 0.1};
    // Set Filter Parameter
    my_controller_pkg::SetFilter setFilterSrv;
    for(uint i=0;i<rt_set_filter_clients.size();i++) {
      setFilterSrv.request.i_filt_param = 1.;
      setFilterSrv.request.vel_filt_param = 0.97;
      setFilterSrv.request.i_max_param = i_max_param(i);
      rt_set_filter_clients[i].call(setFilterSrv);
    }


    // Check Initial State
    my_controller_pkg::GetState getStateSrv;
    for(uint i=0;i<rt_set_state_clients.size();i++) {
      rt_get_state_clients[i].call(getStateSrv);
      q(i) = getStateSrv.response.pos;
      qd(i) = getStateSrv.response.vel;
    }
    arr stateVec, yNext, ydNext;
    world.setJointState(q,qd);
    world.calcBodyFramesFromJoints();
    world.kinematicsPos(state,NoArr,world.getBodyByName("endeff")->index);
    world.kinematicsVec(stateVec,NoArr,world.getBodyByName("endeff")->index);
    state.append(stateVec);
    cout << "robot initial state: " << state << endl;
    cout << "plan initial state: " << x0 << endl;

    MT::wait();
  }

  void startController() {
    ros::Rate loop_rate(1./dtAmex);
    double dt = dtAmex;

    while (ros::ok() && (pfc->s.last() < 0.95))
    {
      double last_time = ros::Time::now().toSec();
      runAmex(dt);
      loop_rate.sleep();
      dt = ros::Time::now().toSec() - last_time ;
      cout << "Time: " << dt << endl;
    }
    ROS_INFO("AMEX CONTROLLER: COMPLETE");

    // Set Velocity to zero after execution
    my_controller_pkg::SetState setStateSrv;
    for(uint i=0;i<rt_set_state_clients.size();i++) {
      setStateSrv.request.pos = q(i);
      setStateSrv.request.vel = 0.;
      rt_set_state_clients[i].call(setStateSrv);
    }

    // Set Gains to zero after execution
    my_controller_pkg::SetGains setGainsSrv;
    for(uint i=0;i<rt_set_state_clients.size();i++) {
      setGainsSrv.request.pos_gain = p_gains(i);
      setGainsSrv.request.vel_gain = d_gains(i);
      setGainsSrv.request.i_gain = 0.;
      rt_set_gains_clients[i].call(setGainsSrv);
    }


#if LOGGING
    write(LIST<arr>(q_bk),STRING(folder<<"q_bk.output"));
    write(LIST<arr>(x_bk),STRING(folder<<"x_bk.output"));
    write(LIST<arr>(goal_bk),STRING(folder<<"goal_bk.output"));
    write(LIST<arr>(ct_bk),STRING(folder<<"ct_bk.output"));
    write(LIST<arr>(s_bk),STRING(folder<<"s_bk.output"));

    write(LIST<arr>(tauRef),STRING(folder<<"xRef.output"));
    write(ARR(dtAmex),STRING(folder<<"dtAmex.output"));
#endif
  }

  // Run amex
  void runAmex(double dtReal) {
    // get current goal
    if (useGoalPub) {
      get_goal_client.call(goalSub);
      goalMO->setPosition(refFrame + ARRAY(goalSub.response.x,goalSub.response.y,goalSub.response.z));
    }

    // get current state of real time controllers
    my_controller_pkg::GetState getStateSrv;
    for(uint i=0;i<rt_set_state_clients.size();i++) {
      rt_get_state_clients[i].call(getStateSrv);
      q(i) = getStateSrv.response.pos;
      qd(i) = getStateSrv.response.vel;
    }

    if (dtReal > 1.) {
      dtReal = dtAmex;
    }
    cout << "qd: " <<qd << endl;

    // Get current task state
    arr stateVec, yNext, ydNext;
    world.setJointState(q,qd);
    world.calcBodyFramesFromJoints();
    world.kinematicsPos(state,NoArr,world.getBodyByName("endeff")->index);
    world.kinematicsVec(stateVec,NoArr,world.getBodyByName("endeff")->index);
    state.append(stateVec);

#if VISUALIZATION
    world.calcShapeFramesFromBodies();
    world.gl().update();
#endif

#if LOGGING
    q_bk.append(~q);
    x_bk.append(~state);
    goal_bk.append(~goalMO->position);
    ct_bk.append(ros::Time::now().toSec());
    s_bk.append(pfc->s.last());
#endif

    pfc->iterate(state,dtReal);
    pfc->getNextState(yNext,ydNext);

    taskPos->y_ref = yNext.subRange(0,2);
    taskPos->v_ref = ydNext.subRange(0,2);
    taskVec->y_ref = yNext.subRange(3,5);
    taskVec->v_ref = ydNext.subRange(3,5);

    qdd = FMC->operationalSpaceControl(regularization);
    double tau = dtAmex;
    q += tau*qd + 0.5*tau*tau*qdd;
    qd += tau*qdd;

    cout << "s " << pfc->s.last() << endl;
    cout << "qd " << qd << endl;

    my_controller_pkg::SetState setStateSrv;
    for(uint i=0;i<rt_set_state_clients.size();i++) {
      setStateSrv.request.pos = q(i);
      setStateSrv.request.vel = qd(i);
      rt_set_state_clients[i].call(setStateSrv);
    }
    cout << "\n \n" << endl;
  }

};




int main(int argc, char** argv)
{
  ros::init(argc, argv, "rt_amex");
  ros::NodeHandle nh;

  String scene = STRING("scene");
  TrajAmex trajAmex(nh);

  trajAmex.initOrs(argc,argv,scene);
  trajAmex.createReferenceTrajectory();

  trajAmex.initController();

  trajAmex.startController();
#if VISUALIZATION
  //  trajAmex.world.gl().add(drawPoint,&(trajAmex.taskPos->y_ref));
  //  trajAmex.world.gl().add(drawActTraj,&(trajAmex.pfc->traj));
  //  trajAmex.world.gl().add(drawPlanTraj,&(trajAmex.pfc->trajWrap->points));
#endif

  return 0;
}

