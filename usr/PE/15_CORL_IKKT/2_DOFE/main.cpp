#include <Core/array.h>
#include <Geo/geo.h>
#include <Gui/opengl.h>
#include <Ors/ors.h>
#include <Hardware/gamepad/gamepad.h>
#include <Motion/feedbackControl.h>
#include <Motion/pr2_heuristics.h>
#include <pr2/roscom.h>
#include <pr2/rosmacro.h>
#include <pr2/rosalvar.h>
#include "../../src/task_manager.h"
#include "../../src/plotUtil.h"
#include "../../src/traj_factory.h"

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


void detectDOFrot(arr &X, ors::Transformation &T) {
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

void detectDOFtrans(arr &X, ors::Vector &prismatic_dir, arr &Q) {
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

  /// set estimated joint configuration trajectory
  Q.clear();
  for (uint i=0;i<X.d0;i++) {
    V_Configuration v = model_instance->getConfiguration(i);
    Q.append(v(0));
  }

  prismatic_dir = ors::Vector(model_instance->getParam("prismatic_dir.x"),model_instance->getParam("prismatic_dir.y"),model_instance->getParam("prismatic_dir.z"));
}



int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  ros::init(argc, argv, "DOFE");

  mlr::String taskName = mlr::getParameter<mlr::String>("taskName");
  mlr::String folder = mlr::getParameter<mlr::String>("folder");
  bool useMarker = mlr::getParameter<bool>("useMarker",false);

  /// init task
  ors::KinematicWorld world(STRING(folder<<"model.kvg"));
  TaskManager *task;
  if (taskName == "door") {
    task = new DoorTask(world);
  } else if (taskName == "grasp") {
    task = new GraspTask(world);
  } else if (taskName == "button") {
    task = new ButtonTask(world);
  }

  /// load demo from file
  arr mfX,mfY,mfYS;
  mfX<<FILE(STRING(folder<<"mfX.dat"));
  mfY<<FILE(STRING(folder<<"mfY.dat")); mfY.flatten();
  mfYS<<FILE(STRING(folder<<"mfYS.dat")); mfYS.flatten();

  uint i = mfY.maxIndex();
  CHECK(mfYS(i)==1,"");

  arr Xbase,FLbase,Mbase;
  Xbase << FILE(STRING(folder<<"/mfXref"<<i<<".dat"));
  FLbase << FILE(STRING(folder<<"/FLbase.dat"));
  if (useMarker) Mbase << FILE(STRING(folder<<"/mbM"<<i<<".dat"));

  /// extract contact point trajectory
  world.gl().update(); world.gl().resize(800,800);
  task->computeConstraintTime(FLbase,Xbase);
  task->updateVisualization(world,Xbase);

  TrajFactory tf;
  arr yL;
  tf.compFeatTraj(Xbase,yL,world,new DefaultTaskMap(posTMT,world,"endeffL"));
  arr Xcp = yL.rows(task->conStart(0),task->conEnd(0));

  /// add DOF to kinematic world
  ors::Vector prismatic_dir;
  arr Q;
//  detectDOFtrans(Xcp,prismatic_dir,Q);
  prismatic_dir = ors::Vector(0.,0.,1.);
  Q = Xcp.col(2); Q.flatten();
  Q = Q - Q(0);

  ors::Quaternion rot;
  rot.setDiff(Vector_z,prismatic_dir);

  ors::Body *b1 = new ors::Body(world);
  b1->name = "b1";
  b1->type = ors::BodyType::dynamicBT;
  b1->X.pos = Xcp[0];
  b1->X.rot = rot;
  ors::Shape *b1_shape = new ors::Shape(world,*b1);
  b1_shape->type = ors::ShapeType::boxST;
  b1_shape->name = "b1_shape";
  arr size = ARRAY(0.1, 0.1, 0.001, 0.);
  memmove(b1_shape->size, size.p, 4*sizeof(double));
  arr color = ARRAY(0., 0., 1.);
  memmove(b1_shape->color, color.p, 3*sizeof(double));

  ors::Body *b2 = new ors::Body(world);
  b2->name = "b2";
  b2->type = ors::BodyType::dynamicBT;
  b2->X.pos = Xcp[0];
  b2->X.rot = rot;
  ors::Shape *b2_shape = new ors::Shape(world,*b2);
  b2_shape->type = ors::ShapeType::boxST;
  b2_shape->name = "b2_shape";
  size = ARRAY(0.1,0.1, 0.001, 0.);
  memmove(b2_shape->size, size.p, 4*sizeof(double));
  color = ARRAY(0.,1.,0.);
  memmove(b2_shape->color, color.p, 3*sizeof(double));
  world.calc_fwdPropagateFrames();

  ors::Joint *b2_b1 = new ors::Joint(world,b2,b1);
  b2_b1->name = "b2_b1";
  b2_b1->A.pos = ARR(0, 0, .0);
  b2_b1->B.pos = ARR(0, 0, .0);
  b2_b1->type = ors::JointType::JT_transZ;
  b2_b1->qIndex = world.getJointStateDimension()-1;

  world.calc_fwdPropagateFrames();

  /// augment trajectory with DOF
  arr Q2;
  Q2 = zeros(Xbase.d0);
  Q2.subRange(Q2.d0-Q.d0,Q2.d0-1) = Q;
  cout << Q2 << endl;
  Xbase = ~Xbase;
  Xbase.append(Q2);
  Xbase = ~Xbase;


  /// save kinematic world
//  world>>FILE(STRING(folder<<"modelaug.ors"));

  ors::KinematicWorld worldaug(STRING(folder<<"modelaug.kvg"));
  /// convert trajectory between two kinematic worlds
  arr X2;
  transferQbetweenTwoWorlds(X2,Xbase,worldaug,world);

  write(LIST(X2),STRING(folder<<"Xaug.dat"));
  write(LIST(FLbase),STRING(folder<<"FLaug.dat"));
  if (useMarker) write(LIST(Mbase),STRING(folder<<"Maug.dat"));

  task->~TaskManager();
  for(;;) {displayTrajectory(X2, 1, worldaug, "planned trajectory");}

  // TODO:
  // copmute external joint value more general with contact times
  return 0;
}
