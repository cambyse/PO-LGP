
// ROS includes
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/JointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

// Ros loggin
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Time.h>


// ors includes
#include <Core/util.h>
#include <Ors/ors.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Gui/opengl.h>
#include <GL/glu.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include "../../examples/Motion/pfc/mobject.h"
#include "../../examples/Motion/pfc/pfc.h"

#include <vector>
#include <stdlib.h>

#define LOGGING 1

OpenGL gl("feedback_control",800,800);
typedef actionlib::SimpleActionClient<control_msgs::JointTrajectoryAction> TrajClient;

class PfControl{
private:
  ors::KinematicWorld G;
  Pfc* pfc;
  MObject* goalMO;
  arr xRef, x0, x0_opt;
  arr joints;
  double dt;
  bool useOrientation = true;
  bool useCollAvoid = false;
  double fPos_deviation,fVec_deviation,yCol_deviation,w_reg;

  control_msgs::JointTrajectoryGoal goal;
  ros::NodeHandle nh_;
  ros::Subscriber jointSub_;
  TrajClient* traj_client_;
  ros::Duration timer;
  rosbag::Bag bag;

public:
  PfControl(ros::NodeHandle &nh)
  {
    nh_ = nh;
    traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action",true);
    jointSub_ =  nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &PfControl::syncJoints, this);

    fPos_deviation = 1e-2;
    fVec_deviation = 1e-3;
    yCol_deviation = 3e-1;
    w_reg = 100.;

    x0_opt = ARRAY(0., 0. ,0. ,0. ,-0.2 ,-0.2 ,0.);

    goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(7);
    goal.trajectory.points[0].velocities.resize(7);
    goal.trajectory.points[0].accelerations.resize(7);

#ifdef LOGGING
    bag.open("real_test.bag", rosbag::bagmode::Write);
#endif
  }

  void plotResult(){
    pfc->plotState();
  }

  void init_ors(int argc, char** argv)
  {
    MT::initCmdLine(argc,argv);

    init(G, gl, "../git/mlr/share/examples/Motion/pfc/model.kvg");
    makeConvexHulls(G.shapes);
  }

  void optimize_trajectory()
  {
    MotionProblem P(&G);
    P.loadTransitionParameters();

    TaskCost *c;
    c = P.addTask("position", new DefaultTaskMap(posTMT,G,"endeff", ors::Vector(0., 0., 0.)));

    P.setInterpolatingCosts(c, MotionProblem::finalOnly,
                            ARRAY(P.world.getBodyByName("goalRef")->X.pos), 1e4,
                            ARRAY(0.,0.,0.), 1e-3);
    P.setInterpolatingVelCosts(c, MotionProblem::finalOnly,
                               ARRAY(0.,0.,0.), 1e3,
                               ARRAY(0.,0.,0.), 0.);

    if (useOrientation) {
      c = P.addTask("orientation", new DefaultTaskMap(vecTMT,G,"endeff",ors::Vector(0., 0., 0.)));
      P.setInterpolatingCosts(c, MotionProblem::finalOnly,
                              ARRAY(0.,0.,-1.), 1e4,
                              ARRAY(0.,0.,0.), 1e-3);
    }

    if (useCollAvoid) {
      c = P.addTask("collision", new DefaultTaskMap(collTMT, 0, ors::Vector(0., 0., 0.), 0, ors::Vector(0., 0., 0.), ARR(.1)));
      P.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1e0);
    }

    //-- set start position for optimizer
    P.x0 = x0_opt;

    MotionProblemFunction F(P);
    uint T=F.get_T();
    uint k=F.get_k();
    uint n=F.dim_x();
    dt=P.tau;

    cout <<"Problem parameters:"<<" T="<<T<<" k="<<k<<" n="<<n<<"dt="<<dt<<" # joints=" <<G.getJointStateDimension()<<endl;
    arr x(T+1,n);
    x.setZero();

    //-- optimize
    optNewton(x, Convert(F), OPT(verbose=0, stopIters=20, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));

    P.costReport();
    gl.watch();
    displayTrajectory(x, 1, G, gl,"planned trajectory");

    //------------------------------------------------//
    // Transform trajectory into task space
    //------------------------------------------------//
    arr kinPos, kinVec, xRefPos, xRefVec;
    // store cartesian coordinates and endeffector orientation
    for (uint t=0;t<=T;t++) {
      G.setJointState(x[t]);
      G.kinematicsPos(kinPos,P.world.getBodyByName("endeff")->index);
      G.kinematicsVec(kinVec,P.world.getBodyByName("endeff")->index);
      xRefPos.append(~kinPos);
      xRefVec.append(~kinVec);
    }

    xRef = xRefPos;
    if (useOrientation) {
      xRef = ~cat(~xRef,~xRefVec);
    }

    goalMO = new MObject(&G, MT::String("goal"), MObject::GOAL , 0.00, ARRAY(0.,1.,0.));

    x0 = xRef[0]; // TODO: READ FROM SENSORS
    cout << x0 << endl;
    pfc = new Pfc(G, xRef,T,x0, *goalMO, useOrientation, useCollAvoid,fPos_deviation,fVec_deviation,yCol_deviation,w_reg);

  }



  void execTrajectoryUpdate()
  {
    //    ROS_INFO("execTrajectoryUpdate");
    arr qd, q, qdd,qd_old;

    G.setJointState(joints);
    //    gl.update();

    G.getJointState(q,qd_old);
    pfc->computeIK(q,qd);
    qd = qd/dt;
    qdd = (qd-qd_old)/dt;
    //    qd = qd + qdd*dt;

    // send 1 step trajectory to ROS
    goal.trajectory.header.stamp = ros::Time::now();
    uint i;
    for (i=0;i<goal.trajectory.joint_names.size();i++)
    {
      goal.trajectory.points[0].positions[i] = q(i);
      goal.trajectory.points[0].velocities[i] = qd(i);
      //      goal.trajectory.points[0].accelerations[i] = qdd(i);
    }

    goal.trajectory.points[0].time_from_start = ros::Duration(dt);
    traj_client_->sendGoal(goal);

#ifdef LOGGING
    bag.write("pfc_goalTraj",ros::Time::now(),goal);
#endif
  }


  void syncJoints(const sensor_msgs::JointState::ConstPtr& msg)
  {
    //    ROS_INFO("syncJoints");
#ifdef LOGGING
    bag.write("pfc_currState",ros::Time::now(),msg);
#endif
    //18 r_shoulder_pan_joint
    //19 r_shoulder_lift_joint
    //17 r_upper_arm_roll_joint
    //20 r_forearm_roll_joint
    //21 r_elbow_flex_joint
    //22 r_wrist_flex_joint
    //23 r_wrist_roll_joint

    joints.clear();
    joints.append(msg->position[18]);
    joints.append(msg->position[19]);
    joints.append(msg->position[17]);
    joints.append(msg->position[20]);
    joints.append(msg->position[21]);
    joints.append(msg->position[22]);
    joints.append(msg->position[23]);

  }

  void run()
  {
    ROS_INFO("run");
    ros::Rate loop_rate(1/dt);
    uint i =0;
    while (ros::ok() && (pfc->s.last() < 0.9))
    {
#ifdef LOGGING
      std_msgs::UInt32 ik; ik.data = i; bag.write("pfc_iterator", ros::Time::now(), ik);
      std_msgs::Float32 fk; fk.data = pfc->s.last(); bag.write("pfc_phaseVariable", ros::Time::now(), fk);
      std_msgs::Time tk; tk.data = ros::Time::now(); bag.write("pfc_time", ros::Time::now(), tk);
#endif
      ros::spinOnce();
      execTrajectoryUpdate();
      ros::spinOnce();
      loop_rate.sleep();
    }
#ifdef LOGGING
    bag.close();
#endif
  }

  void init_robot()
  {
    // send 1 step trajectory to ROS
    control_msgs::JointTrajectoryGoal goal;
    goal.trajectory.header.stamp = ros::Time::now();

    goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(7);
    goal.trajectory.points[0].velocities.resize(7);

    uint i;
    for (i=0;i<goal.trajectory.joint_names.size();i++)
    {
      goal.trajectory.points[0].positions[i] = x0_opt(i);
      goal.trajectory.points[0].velocities[i] = 0.;
    }

    goal.trajectory.points[0].time_from_start = ros::Duration(10.0);

    traj_client_->sendGoal(goal);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc,argv,"feedback_control");
  ros::NodeHandle nh;

  PfControl pfcontrol(nh);

  /* INITIALIZE ORS
   */
  pfcontrol.init_ors(argc,argv);

  /* CREATE TRAJECTORY
   * read in from text file
   * or create with optimizer
   */
  pfcontrol.optimize_trajectory();

  /* BRING ROBOT IN THE START STATE
     */
  pfcontrol.init_robot();

  gl.update();
  gl.watch();

  /* Start Controller
     */
  pfcontrol.run();

  /* Log Trajectories
       */
  pfcontrol.plotResult();
  return 0;
}
