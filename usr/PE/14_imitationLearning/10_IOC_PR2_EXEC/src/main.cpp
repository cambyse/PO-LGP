#include <Ors/ors.h>
#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/taskMap_transition.h>
#include <Ors/ors_swift.h>
#include <Core/array-vector.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/JointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <goal_publisher/GetGoal.h>

typedef actionlib::SimpleActionClient<control_msgs::JointTrajectoryAction> TrajClient;

class RobotArm
{
private:
  TrajClient* traj_client_;

public:
  RobotArm()
  {
    traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action",true);

    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }
  ~RobotArm()
  {
    delete traj_client_;
  }

  double fRand(double fMin, double fMax)
  {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
  }

  void startTrajectory(control_msgs::JointTrajectoryGoal goal)
  {
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }

  control_msgs::JointTrajectoryGoal armExtensionTrajectory(arr &x,uint type,double dt)
  {
    ROS_INFO("START INIT MOTION");
    control_msgs::JointTrajectoryGoal goal;

    goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

    double qLowerLimit[7] = {-2.2854, -0.5236, -3.9, -1.7, -3.14, -2. ,-3.14};
    double qUpperLimit[7] = {0.714602, 0.6, 0.8, 0., 3.14, 0., 3.14};


    /// single final position
    if (type==1) {
      goal.trajectory.points.resize(1);
      goal.trajectory.points[0].positions.resize(7);
      goal.trajectory.points[0].velocities.resize(7);

      // randomly sample for each joint a final position
      for (uint i = 0; i< goal.trajectory.joint_names.size();i++){
        goal.trajectory.points[0].positions[i] = x(i);
        goal.trajectory.points[0].velocities[i] = 0.0;
      }
      goal.trajectory.points[0].time_from_start = ros::Duration(5.);
    }

    /// whole trajectory
    if (type==2) {
      goal.trajectory.points.resize(x.d0);


      arr xd;
      getVel(xd,x,dt);

      for (uint t =0; t<x.d0; t++) {
        goal.trajectory.points[t].positions.resize(7);
        goal.trajectory.points[t].velocities.resize(7);
        goal.trajectory.points[t].positions = VECTOR(x[t]);
        goal.trajectory.points[t].velocities = VECTOR(xd[t]);
        goal.trajectory.points[t].time_from_start = ros::Duration(dt*double(t));
      }

//      // randomly sample for each joint a goal position
//      for (uint i = 0; i< goal.trajectory.joint_names.size();i++){
//        goal.trajectory.points[0].positions[i] = x(0);
//        goal.trajectory.points[0].velocities[i] = 0.0;
//      }
    }

    // return the goal
    return goal;
  }

  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }
};

void createTrajectory(arr &x, arr &options, const arr &q0, const arr &markerPos, const ors::Quaternion &markerQuat) {
  ors::KinematicWorld* world = new ors::KinematicWorld("scene");
  arr q, qdot;
  world->getJointState(q, qdot);
  makeConvexHulls(world->shapes);
  world->swift().setCutoff(10.);
  world->swift();

  //- load motion parameters
  arr param;
  FILE("data/param") >> param; param.flatten();
  FILE("data/options") >> options; options.flatten();
  double dt = options(0);
  uint T = options(1);
  double contactTime = options(2);
  double costScale = options(3);
  cout << "Cost parameter: " << param << endl;
  cout << "Options: dt: " << dt << " T: " << T << " contactTime: " << contactTime << " costScale: " << costScale << endl;

  world->setJointState(q0);


  arr refFrame = ARRAY(world->getBodyByName("torso_lift_link")->X.pos);
  ors::Quaternion refFrameQuat = world->getBodyByName("torso_lift_link")->X.rot;
  ors::Quaternion objQuat = refFrameQuat*markerQuat;
  arr markerOffset = objQuat.getArr()*ARRAY(0.126,0.0,0.0415);

  markerOffset= markerOffset + ARRAY(0.01,0.,0.);

  arr objPos0 = refFrame + markerOffset + markerPos;
  arr objPosT = refFrame + markerOffset + markerPos + ARRAY(0.21,0.,0.);
  arr objVec0;
  arr objVecT;

  /// define drawer position
  uint i = MT::getParameter<uint>("drawerId");
  world->getBodyByName("drawer1")->X.rot =  objQuat;
  world->getBodyByName("wall1")->X.pos =  objPosT+ARRAY(-0.1,0.165+0.026+0.03,-0.);

  if (i==2){
    objPosT = objPosT - ARRAY(0.21,0.33+0.024,0.);
    objPos0 = objPos0 - ARRAY(0.21,0.33+0.024,0.);
    world->getBodyByName("wall1")->X.pos =  objPosT+ARRAY(-0.1,-0.165-0.026-0.03,-0.);
  } else if (i==3) {
    objPosT = objPosT - ARRAY(0.21,0.,0.163+0.005);
    objPos0 = objPos0 - ARRAY(0.21,0.,0.163+0.005);
    world->getBodyByName("wall1")->X.pos =  objPosT+ARRAY(-0.1,0.165+0.026+0.03,-0.);
    world->getBodyByName("wall2")->X.pos =  objPosT+ARRAY(-0.1,0.,-0.0815-0.1);
  } else if (i==4) {
    objPosT = objPosT - ARRAY(0.21,0.33+0.024,0.163+0.005);
    objPos0 = objPos0 - ARRAY(0.21,0.33+0.024,0.163+0.005);
    world->getBodyByName("wall1")->X.pos =  objPosT+ARRAY(-0.1,-0.165-0.026-0.03,-0.);
  }

//  world->getBodyByName("wall2")->X.pos =  objPosT+ARRAY(-0.1,0.,0.0815+0.07);

  world->getBodyByName("drawer1")->X.pos =  objPos0;
  world->getBodyByName("targetRef")->X.pos =  refFrame + markerPos;
  world->getBodyByName("goalDrawer1")->X.rot = objQuat;
  world->getBodyByName("goalDrawer1")->X.pos = objPosT;
  world->calc_fwdPropagateShapeFrames();
  world->watch();

  MotionProblem* MP = new MotionProblem(*world,false);
  MP->useSwift=true;
  MP->loadTransitionParameters();
  MP->makeContactsAttractive=false;
  MP->T = T;
  MP->tau = dt;
  MP->x0 = q0;

  ors::Shape *grasp = world->getShapeByName("endeffR");
  ors::Body *tar = world->getBodyByName("drawer1");

  // add graph operator
  ors::GraphOperator *op1 = new ors::GraphOperator();
  op1->symbol = ors::GraphOperator::addRigid;
  op1->timeOfApplication = contactTime;
  op1->fromId = world->getBodyByName("endeffR")->index;
  op1->toId = world->getBodyByName("drawer1")->index;
  world->operators.append(op1);

  arr xDemTaskVec;
  ors::Vector *tmp = new ors::Vector(0.,-1.,0);
  world->kinematicsVec(xDemTaskVec,NoArr,world->getBodyByName("goalDrawer1"),tmp);
  cout << "xDemTaskVec: "<< xDemTaskVec << endl;
  /// set motion parameter
  param = param/sqrt(sumOfSqr(param))*costScale;

  MP->H_rate_diag = param(0);
  uint N = 1;//world->getJointStateDimension();
  uint stay = 5;
  TaskCost *c;
  c =MP->addTask("posT", new DefaultTaskMap(posTMT, grasp->index) );
  c->setCostSpecs(MP->T,MP->T, objPosT, param(N++));
  c =MP->addTask("posC", new DefaultTaskMap(posTMT, grasp->index) );
  c->setCostSpecs(contactTime,contactTime, objPos0, param(N++));
  c =MP->addTask("vecT", new DefaultTaskMap(vecTMT, grasp->index,ors::Vector(1.,0.,0)) );
  c->setCostSpecs(MP->T,MP->T, xDemTaskVec, param(N++));
  c =MP->addTask("vecC", new DefaultTaskMap(vecTMT, grasp->index,ors::Vector(1.,0.,0)) );
  c->setCostSpecs(contactTime,contactTime, xDemTaskVec, param(N++));
  c =MP->addTask("velT", new DefaultTaskMap(posTMT, grasp->index) );
  c->map.order = 1;
  c->setCostSpecs(MP->T-stay,MP->T, zeros(3), param(N++));
  c =MP->addTask("velC", new DefaultTaskMap(posTMT, grasp->index) );
  c->map.order = 1;
  c->setCostSpecs(contactTime-stay,contactTime+stay, zeros(3), param(N++));
  TaskMap *tm_contact = new PairCollisionConstraint(*world,"wall1","drawer1",0.02);
  TaskCost *c4 = MP->addTask("contact_endeff",tm_contact);
  c4->map.constraint = false;
  c4->setCostSpecs(contactTime,MP->T, ARR(0.) ,param(N++));
  TaskMap *tm_contact2 = new PairCollisionConstraint(*world,"wall2","drawer1",0.02);
  TaskCost *c5 = MP->addTask("contact_endeff2",tm_contact2);
  c5->map.constraint = false;
  c5->setCostSpecs(contactTime,MP->T, ARR(0.) ,param(N++));
  c = MP->addTask("collisionConstraints1", new PairCollisionConstraint(*world,"wall1","drawer1",0.03));
  MP->setInterpolatingCosts(c, MotionProblem::constant,ARR(0.),1e0);
  c = MP->addTask("collisionConstraints2", new PairCollisionConstraint(*world,"wall2","drawer1",0.03));
  MP->setInterpolatingCosts(c, MotionProblem::constant,ARR(0.),1e0);


  /// optimize motion
  MotionProblemFunction MPF(*MP);
  cout <<"Problem parameters:"<<" T=" <<MPF.get_T()<<" k=" <<MPF.get_k()<<" n=" <<MPF.dim_x() << " dt=" << MP->tau <<" # joints=" << N<<endl;
  arr lambdaTrain(2*T+1); lambdaTrain.setZero();
  x = repmat(~MP->x0,T+1,1);
  optConstrained(x, lambdaTrain, Convert(MPF), OPT(verbose=0,stopTolerance=1e-3));//, allowOverstep=true));
  //    optConstrained(x, lambdaTrain, Convert(MPF), OPT(verbose=0,stopTolerance=1e-4,  allowOverstep=true));
  MP->costReport(true);
  displayTrajectory(x,MP->T,MP->world,"t");
}


int main(int argc, char** argv)
{
  /// initialize ROS
  ros::init(argc,argv,"pr2_ioc_exec");
  ros::NodeHandle nh;
  RobotArm arm;



  /// set pr2 to initial position
  arr q0;
  q0 << FILE("data/q0");
//  q0 = {0.2680288463170052, -0.1322401316072627, 0.4791822152351035, -0.1990329349906086, -0.7742841596713335, -0.09206057706848164, -1.298894153706367};
//  q0 = {0.3891, 0.2219, 0.00613, -1.845, 3.06, -1.655, -3.075};
  q0 = {-1.0670164129825366, -0.18984927424139628, -0.03315145501668293, -1.944099090007652, 4.835668485707074, -1.748271129579642, -3.2440909535190334};
  cout << "Initial Position: " << q0 << endl;
  cout << "Send initial position to PR2 [Press Key]" << endl;
  MT::wait();
  arm.startTrajectory(arm.armExtensionTrajectory(q0,1,0.));
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }


  /// get marker position
  ros::ServiceClient get_init_goal_client = nh.serviceClient<goal_publisher::GetGoal>("/get_goal");
  goal_publisher::GetGoal goalMarker;
  arr marker;
  ors::Quaternion markerQuat;
  if (get_init_goal_client.waitForExistence(ros::Duration(10.0))){
    get_init_goal_client.call(goalMarker);
    marker = ARR(goalMarker.response.x,goalMarker.response.y,goalMarker.response.z);
    markerQuat = ors::Quaternion(goalMarker.response.q_w,goalMarker.response.q_x,goalMarker.response.q_y,goalMarker.response.q_z);
  }
  cout << "Marker position: " << marker << endl;
  cout << "Marker quaternion: " << markerQuat << endl;


  /// optimize trajectory
  arr x;
  arr options;
  createTrajectory(x,options,q0,marker,markerQuat);
  double dt = options(0);
  cout << dt << endl;

  /// send trajectory to robot
  cout << "Send trajectory to PR2 [Press Key]" << endl;
  MT::wait();
  arm.startTrajectory(arm.armExtensionTrajectory(x,2,dt));
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }
}
