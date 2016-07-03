#include <Core/array.h>
#include <Geo/geo.h>
#include <Gui/opengl.h>
#include <Hardware/gamepad/gamepad.h>
#include <Control/taskController.h>
#include <Motion/pr2_heuristics.h>
#include <RosCom/roscom.h>
#include <RosCom/rosmacro.h>
#include <RosCom/subscribeAlvarMarkers.h>


#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <articulation_models/models/factory.h>
#include <articulation_msgs/TrackMsg.h>
#include <articulation_msgs/ParamMsg.h>
#include <articulation_msgs/ModelMsg.h>

using namespace articulation_models;
using namespace articulation_msgs;

void detectDOF(arr &X, ors::Transformation &T) {
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
  for (uint i = 0; i < X.d0; i++) {
    geometry_msgs::Pose pose;
    pose.position.x = X(i,0) + randn(1)*noise; pose.position.y = X(i,1) + randn(1)*noise; pose.position.z = X(i,2) + randn(1)*noise;
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

  T.pos = ors::Vector(model_instance->getParam("rot_center.x"),model_instance->getParam("rot_center.y"),model_instance->getParam("rot_center.z"));
  T.rot = ors::Quaternion(model_instance->getParam("rot_axis.w"),model_instance->getParam("rot_axis.x"),model_instance->getParam("rot_axis.y"),model_instance->getParam("rot_axis.z"));
}

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  ors::KinematicWorld G("door.ors");
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
  t = MP.addTask("transitions", new TaskMap_Transition(G));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e-1);

  double contactT = MP.T/2.;
  // position task maps
  t = MP.addTask("position", new TaskMap_Default(posTMT, G, "endeffL", NoVector, "target",NoVector));
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
  ors::Transformation T;
  detectDOF(Y,T);
  G.getBodyByName("dof_axis")->X = T;
  G.calc_fwdPropagateFrames();

  G.watch(true);
  for(;;) {displayTrajectory(X, 1, G, "planned trajectory");}
  return 0;
}
