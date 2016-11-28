#include <Core/array.h>
#include <Geo/geo.h>
#include <Gui/opengl.h>
#include <Hardware/gamepad/gamepad.h>
#include <Motion/feedbackControl.h>
#include <Motion/pr2_heuristics.h>
#include <pr2/roscom.h>
#include <pr2/rosmacro.h>
#include <pr2/rosalvar.h>


#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <articulation_models/models/factory.h>
#include <articulation_msgs/TrackMsg.h>
#include <articulation_msgs/ParamMsg.h>
#include <articulation_msgs/ModelMsg.h>

#include <articulation_msgs/ArticulatedObjectMsg.h>
#include <articulation_msgs/ArticulatedObjectSrv.h>


using namespace articulation_models;
using namespace articulation_msgs;

void detectDOFdoor() {
  mlr::KinematicWorld G("door.ors");
  G.meldFixedJoints();
  G.removeUselessBodies();
  makeConvexHulls(G.shapes);

  G.watch(false);
  G.gl().resize(800,800);
  arr q;
  G.getJointState(q);
  MotionProblem MP(G);
  cout <<"joint dimensionality=" <<q.N <<endl;

  Task *t;
  t = MP.addTask("transitions", new TransitionTaskMap(G));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e-1);

  double contactT = MP.T/2.;
  // position task maps
  t = MP.addTask("position", new DefaultTaskMap(posTMT, G, "endeffL", NoVector, "target",NoVector));
  t->setCostSpecs(contactT-10., contactT, {0.}, 1e2);
  t = MP.addTask("door_joint", new TaskMap_qItself(G.getJointByName("frame_door")->qIndex, G.getJointStateDimension()));
  t->setCostSpecs(MP.T, MP.T, {-.7}, 1e2);

  t = MP.addTask("contact", new PointEqualityConstraint(G, "endeffL",NoVector, "target",NoVector));
  t->setCostSpecs(contactT, MP.T, {0.}, 1.);
  t = MP.addTask("door_fixation", new qItselfConstraint(G.getJointByName("frame_door")->qIndex, G.getJointStateDimension()));
  t->setCostSpecs(0.,contactT+10, {0.}, 1.);

  MotionProblemFunction MF(MP);
  arr X = MP.getInitialization();
  arr lambda;

  optConstrained(X, lambda, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2));

  cout << lambda << endl;
  MP.costReport();

  /// get contact point trajectory
  arr Y;
  for (uint i=contactT;i<X.d0;i++){
    G.setJointState(X[i]);
    arr tmp = conv_vec2arr(G.getShapeByName("endeffL")->X.pos);
    Y.append(~tmp);
  }

  /// detect DOF
  mlr::Transformation T;

  ModelMsg model_msg;
  model_msg.name = "rotational";

  /// define parameter prior
  ParamMsg sigma_param;
  sigma_param.name = "sigma_position";
  sigma_param.value = 0.001;
  sigma_param.type = ParamMsg::PRIOR;

//  sigma_param.name = "sigma_orientation";
//  sigma_param.value = 1e-1;
//  sigma_param.type = ParamMsg::PRIOR;

  sigma_param.name = "rot_mode";
  sigma_param.value = 0;
  sigma_param.type = ParamMsg::PRIOR;


  model_msg.params.push_back(sigma_param);
  model_msg.track.header.stamp = ros::Time();
  model_msg.track.header.frame_id = "/";

  MultiModelFactory factory;

  double noise = 0.;
  /// convert trajectory into geometry_msg
  for (uint i = 0; i < Y.d0; i++) {
    geometry_msgs::Pose pose;
    pose.position.x = Y(i,0) + randn(1)*noise; pose.position.y = Y(i,1) + randn(1)*noise; pose.position.z = Y(i,2) + randn(1)*noise;
    pose.orientation.x = 0; pose.orientation.y = 0; pose.orientation.z = 0; pose.orientation.w = 1;
    model_msg.track.pose.push_back(pose);
  }

  /// fit model
  GenericModelPtr model_instance = factory.restoreModel(model_msg);
  model_instance->fitModel();
  model_instance->evaluateModel();

  cout << "model class = "<< model_instance->getModelName() << endl;
  cout << "	radius = "<<model_instance->getParam("rot_radius")<< endl;
  cout << "	center.x = "<<model_instance->getParam("rot_center.x")<< endl;
  cout << "	center.y = "<<model_instance->getParam("rot_center.y")<< endl;
  cout << "	center.z = "<<model_instance->getParam("rot_center.z")<< endl;

  cout << "	rot_axis.x = "<<model_instance->getParam("rot_axis.x")<< endl;
  cout << "	rot_axis.y = "<<model_instance->getParam("rot_axis.y")<< endl;
  cout << "	rot_axis.z = "<<model_instance->getParam("rot_axis.z")<< endl;
  cout << "	rot_axis.w = "<<model_instance->getParam("rot_axis.w")<< endl;
  cout << "	sigma_position = "<<model_instance->getParam("sigma_position")<< endl;
  cout << "	sigma_orientation = "<<model_instance->getParam("sigma_orientation")<< endl;

  cout << "	log LH = " << model_instance->getParam("loglikelihood")<< endl;

  T.pos = mlr::Vector(model_instance->getParam("rot_center.x"),model_instance->getParam("rot_center.y"),model_instance->getParam("rot_center.z"));
  T.rot = mlr::Quaternion(model_instance->getParam("rot_axis.w"),model_instance->getParam("rot_axis.x"),model_instance->getParam("rot_axis.y"),model_instance->getParam("rot_axis.z"));

  G.getBodyByName("dof_axis")->X = T;
  G.calc_fwdPropagateFrames();

  G.watch(true);
  for(;;) {displayTrajectory(X, 1, G, "planned trajectory");}
}


void detectDOFrot() {
  ModelMsg model_msg;
  model_msg.name = "rotational";

  /// define parameter prior
  ParamMsg sigma_param;
  sigma_param.name = "sigma_position";
  sigma_param.value = 0.001;
  sigma_param.type = ParamMsg::PRIOR;

  sigma_param.name = "rot_mode";
  sigma_param.value = 0;
  sigma_param.type = ParamMsg::PRIOR;

  model_msg.params.push_back(sigma_param);
  model_msg.track.header.stamp = ros::Time();
  model_msg.track.header.frame_id = "/";

  MultiModelFactory factory;

  double noise = 0.;
  uint n = 100;
  /// convert trajectory into geometry_msg
  for (uint i = 0; i < n; i++) {
    geometry_msgs::Pose pose;
    pose.position.x = 1+sin(i*M_PI/double(n)) + randn(1)*noise; pose.position.y = cos(i*M_PI/double(n)) + randn(1)*noise; pose.position.z = randn(1)*noise;
    pose.orientation.x = 0; pose.orientation.y = 0; pose.orientation.z = 0; pose.orientation.w = 1;
    model_msg.track.pose.push_back(pose);
  }

  /// fit model
  GenericModelPtr model_instance = factory.restoreModel(model_msg);
  model_instance->fitModel();
  model_instance->evaluateModel();

  cout << "model class = "<< model_instance->getModelName() << endl;
  cout << "	radius = "<<model_instance->getParam("rot_radius")<< endl;
  cout << "	center.x = "<<model_instance->getParam("rot_center.x")<< endl;
  cout << "	center.y = "<<model_instance->getParam("rot_center.y")<< endl;
  cout << "	center.z = "<<model_instance->getParam("rot_center.z")<< endl;

  cout << "	rot_axis.x = "<<model_instance->getParam("rot_axis.x")<< endl;
  cout << "	rot_axis.y = "<<model_instance->getParam("rot_axis.y")<< endl;
  cout << "	rot_axis.z = "<<model_instance->getParam("rot_axis.z")<< endl;
  cout << "	rot_axis.w = "<<model_instance->getParam("rot_axis.w")<< endl;
  cout << "	sigma_position = "<<model_instance->getParam("sigma_position")<< endl;
  cout << "	sigma_orientation = "<<model_instance->getParam("sigma_orientation")<< endl;

  cout << "	log LH = " << model_instance->getParam("loglikelihood")<< endl;

}


void detectDOFtrans() {
  ModelMsg model_msg;
  model_msg.name = "prismatic";

  /// define parameter prior
  ParamMsg sigma_param;
  sigma_param.name = "sigma_position";
  sigma_param.value = 0.001;
  sigma_param.type = ParamMsg::PRIOR;

  model_msg.params.push_back(sigma_param);
  model_msg.track.header.stamp = ros::Time();
  model_msg.track.header.frame_id = "/";

  MultiModelFactory factory;

  double noise = 0.;
  uint n = 100;
  /// convert trajectory into geometry_msg
  for (uint i = 0; i < n; i++) {
    geometry_msgs::Pose pose;
    pose.position.x = i/double(n) + randn(1)*noise; pose.position.y = 2.*i/double(n) + randn(1)*noise; pose.position.z = 0.1 + randn(1)*noise;
    pose.orientation.x = 0; pose.orientation.y = 0; pose.orientation.z = 0; pose.orientation.w = 1;
    model_msg.track.pose.push_back(pose);
  }

  /// fit model
  GenericModelPtr model_instance = factory.restoreModel(model_msg);
  model_instance->fitModel();
  model_instance->evaluateModel();

  cout << "model class = "<< model_instance->getModelName() << endl;
  cout << "	rigid_position.x = "<<model_instance->getParam("rigid_position.x")<< endl;
  cout << "	rigid_position.y = "<<model_instance->getParam("rigid_position.y")<< endl;
  cout << "	rigid_position.z = "<<model_instance->getParam("rigid_position.z")<< endl;

  cout << "	rigid_orientation.x = "<<model_instance->getParam("rigid_orientation.x")<< endl;
  cout << "	rigid_orientation.y = "<<model_instance->getParam("rigid_orientation.y")<< endl;
  cout << "	rigid_orientation.z = "<<model_instance->getParam("rigid_orientation.z")<< endl;
  cout << "	rigid_orientation.w = "<<model_instance->getParam("rigid_orientation.w")<< endl;

  cout << "	rigid_height = "<<model_instance->getParam("rigid_height")<< endl;
  cout << "	rigid_width = "<<model_instance->getParam("rigid_width")<< endl;

  cout << "	prismatic_dir.x = "<<model_instance->getParam("prismatic_dir.x")<< endl;
  cout << "	prismatic_dir.y = "<<model_instance->getParam("prismatic_dir.y")<< endl;
  cout << "	prismatic_dir.z = "<<model_instance->getParam("prismatic_dir.z")<< endl;
  cout << "	sigma_position = "<<model_instance->getParam("sigma_position")<< endl;
  cout << "	sigma_orientation = "<<model_instance->getParam("sigma_orientation")<< endl;

  cout << "	log LH = " << model_instance->getParam("loglikelihood")<< endl;
}

void detectDOFstruct() {
  ArticulatedObjectMsg msg_request,msg_response;

  /// define parameter prior
  ParamMsg sigma_param;
  sigma_param.name = "sigma_position";
  sigma_param.value = 0.001;
  sigma_param.type = ParamMsg::PRIOR;
  double noise = 0.;
  uint n = 100;

  TrackMsg msg_track0, msg_track1, msg_track2;

  for (uint i = 0; i < n; i++) {
    geometry_msgs::Pose pose12;
    double s = M_PI*i/double(n);
    pose12.position.x = cos(s) + randn(1)*noise; pose12.position.y = sin(s) + randn(1)*noise; pose12.position.z = 0.0 + randn(1)*noise;
    pose12.orientation.x = 0; pose12.orientation.y = 0; pose12.orientation.z = 0; pose12.orientation.w = 1;
    msg_track1.pose.push_back(pose12);
    msg_track2.pose.push_back(pose12);
    geometry_msgs::Pose pose0;
    pose0.position.x = 0.; pose0.position.y = 0.; pose0.position.z = 0.;
    pose0.orientation.x = 0; pose0.orientation.y = 0; pose0.orientation.z = 0; pose0.orientation.w = 1;
    msg_track0.pose.push_back(pose0);
  }
  for (uint i = 0; i < n; i++) {
    geometry_msgs::Pose pose1;
    pose1.position.x = -1. + randn(1)*noise; pose1.position.y = randn(1)*noise; pose1.position.z = randn(1)*noise;
    pose1.orientation.x = 0; pose1.orientation.y = 0; pose1.orientation.z = 0; pose1.orientation.w = 1;
    msg_track1.pose.push_back(pose1);
    geometry_msgs::Pose pose2;
    double s = M_PI*i/double(n);
    pose2.position.x = -1. + randn(1)*noise; pose2.position.y = cos(s)-1. + randn(1)*noise; pose2.position.z = sin(s) + randn(1)*noise;
    pose2.orientation.x = 0; pose2.orientation.y = 0; pose2.orientation.z = 0; pose2.orientation.w = 1;
    msg_track2.pose.push_back(pose2);
    geometry_msgs::Pose pose0;
    pose0.position.x = 0.; pose0.position.y = 0.; pose0.position.z = 0.;
    pose0.orientation.x = 0; pose0.orientation.y = 0; pose0.orientation.z = 0; pose0.orientation.w = 1;
    msg_track0.pose.push_back(pose0);
  }

  msg_request.header.stamp = ros::Time();

  msg_request.parts.push_back(msg_track0);
  msg_request.parts.push_back(msg_track1);
  msg_request.parts.push_back(msg_track2);
  msg_request.params.push_back(sigma_param);

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<ArticulatedObjectSrv>("/fit_models",true);
  ArticulatedObjectSrv srv;
  srv.request.object = msg_request;
  srv.response.object = msg_request;
  ros::spinOnce();

  if (client.call(srv)) {
    ROS_INFO("Sum: ");
  } else {
    ROS_ERROR("Failed to call service add_two_ints");
    return;
  }

  msg_response = srv.response.object;

  ModelMsg model_msg;
  for (uint i =0;i<msg_response.models.size();i++) {
    model_msg = msg_response.models[i];
    cout << model_msg.name << endl;
    for (uint j=0;j<model_msg.params.size();j++) {
      cout <<  model_msg.params[j].name << ": " << model_msg.params[j].value << endl;
    }
    cout << "##########################################" << endl;
  }
}


int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  ros::init(argc, argv, "DOFE");

//  detectDOFrot();
//  detectDOFtrans();
  detectDOFdoor();
//  detectDOFstruct();

  return 0;
}
