#include <Motion/gamepad2tasks.h>
#include <Motion/feedbackControl.h>
#include <Hardware/gamepad/gamepad.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>
#include <pr2/roscom.h>
#include <pr2/rosmacro.h>
#include <pr2/rosalvar.h>
#include <pr2/trajectoryInterface.h>
#include <Core/util.h>

//#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <iomanip>
#include <Ors/ors_swift.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <articulation_models/models/factory.h>
#include <articulation_msgs/TrackMsg.h>
#include <articulation_msgs/ParamMsg.h>
#include <articulation_msgs/ModelMsg.h>

using namespace articulation_models;
using namespace articulation_msgs;

void makeNewWorld(ors::Transformation T);

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


void changeColor2(void*){  orsDrawAlpha = 1.; }

void turnPlate() {
  ors::KinematicWorld world("model.kvg");
  makeConvexHulls(world.shapes);

  TrajectoryInterface *ti = new TrajectoryInterface(world);
  ti->world->gl().update();
  ti->world->gl().resize(800,800); //segmentation fault again
  ti->world->gl().add(changeColor2);

  arr q = ti->world->getJointState();

  MotionProblem MP(world);

  Task *t;

  t = MP.addTask("transitions", new TransitionTaskMap(world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

  double contactT = MP.T/2;

  t = MP.addTask("box_wheel_plate_joint", new TaskMap_qItself(world.getJointByName("box_wheel_plate_joint")->qIndex, world.getJointStateDimension()));
  t->setCostSpecs(MP.T-20, MP.T, {-3.14/2}, 1e3);

  t = MP.addTask("contact", new PointEqualityConstraint(world, "endeffC1",NoVector, "knob_cp2",NoVector));
  t->setCostSpecs(contactT, MP.T, {0.}, 10.);
  t = MP.addTask("contact", new PointEqualityConstraint(world, "endeffC2",NoVector, "knob_cp1",NoVector));
  t->setCostSpecs(contactT, MP.T, {0.}, 10.);

  t = MP.addTask("fix_joint", new qItselfConstraint(world.getJointByName("box_wheel_plate_joint")->qIndex, world.getJointStateDimension()));
  t->setCostSpecs(0.,contactT, {0.}, 1.);

  t = MP.addTask("qLimits", new LimitsConstraint());
  t->setCostSpecs(0., MP.T, {0.}, 1.);

  //ShapeL except = world.getBodyByName("l_wrist_roll_link")->shapes;
  //t = MP.addTask("collision", new ProxyConstraint(allExceptListedPTMT, shapesToShapeIndices(except), 0.1));
  //t->setCostSpecs(0., MP.T, {0.}, 1.);
  t = MP.addTask("collisionConstraints", new CollisionConstraint(.1));
  t->setCostSpecs(0., MP.T, {0.}, 100.);

  //t = MP.addTask("direction", new VelAlignConstraint(world, "l_wrist_roll_link_0", ors::Vector(0.297358,-0.277855,0.682291), "l_wrist_roll_link_0", ors::Vector(0.297358,-0.277855,0.682291)));
  //t = MP.addTask("direction", new VelAlignConstraint(world, "l_wrist_roll_link_0", ors::Vector(0.297358,-0.277855,0.682291), "l_wrist_roll_link_0", ors::Vector(0.297358,-0.277855,0.682291)));
  //t = MP.addTask("direction", new DefaultTaskMap(world, "endeffL", ors::Vector(1,0,0), "ve_shape", NoVector));
  //t = MP.addTask("direction", new DefaultTaskMap(vecAlignTMT, world, "endeffL", ors::Vector(1,0,0), "ve_shape", ors::Vector(1,0,0)));
  //t->setCostSpecs(contactT-10, contactT, {0.9}, 10.);

  MotionProblemFunction MF(MP);

  arr x = MP.getInitialization();

  optConstrainedMix(x, NoArr, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2));
  MP.costReport();
  ti->executeTrajectory(x, 10);

  /// get contact point trajectory
  arr Y;
  for (uint i=contactT;i<x.d0;i++){
    world.setJointState(x[i]);
    arr tmp = conv_vec2arr(world.getShapeByName("endeffC1")->X.pos);
    Y.append(~tmp);
  }

  /// detect DOF
  ors::Transformation T;
  detectDOF(Y,T);
  ti->world->getBodyByName("dof_axis")->X = T;
  ti->world->calc_fwdPropagateFrames();
  ti->world->watch(true);
  cout << T.rot << endl;
  ti->~TrajectoryInterface();
  makeNewWorld(T);

  //world.getBodyByName("dof_axis")->X = T;
  //world.calc_fwdPropagateFrames();

  //world.watch(true);

  //q(ti->world->getJointByName("l_gripper_l_finger_joint")->qIndex)+= 0.2;

  //ti->~TrajectoryInterface();
  //ti->executeTrajectory(x, 10);
  //ti->executeTrajectory(x, 10);
  //ti->executeTrajectory(x, 10);
  //ti->executeTrajectory(x, 10);

}

void makeNewWorld(ors::Transformation T) {
  ors::KinematicWorld world("model_without.kvg");
  makeConvexHulls(world.shapes);



  ors::Body *fixedBody = new ors::Body(world);
  ors::Vector pos = T.pos;
  pos.x += 0.075;
  fixedBody->X.pos = pos;
  //fixedBody->X.rot = T.rot;
  //fixedBody->X.pos = ors::Vector(1, 0, 0.8);
  fixedBody->X.rot = ors::Quaternion(-sqrt(2)/2,0.,0.,sqrt(2)/2);
  fixedBody->type = ors::BodyType::staticBT;
  ors::Shape *fixedBodyShape = new ors::Shape(world, *fixedBody);
  fixedBodyShape->type = ors::ShapeType::boxST;
  arr size = ARRAY(0.8, 0.15, 0.5, 0.);
  memmove(fixedBodyShape->size, size.p, 4*sizeof(double));

  ors::Body *knobBody = new ors::Body(world);
  //knobBody-> = ors::Vector(0, -0.015, 0);
  //knobBody->X.rot = ors::Quaternion(-sqrt(2)/2,0.,0.,sqrt(2)/2);
  knobBody->type = ors::BodyType::staticBT;
  ors::Shape *knobBodyShape = new ors::Shape(world, *knobBody);
  knobBodyShape->rel.pos = ors::Vector(0, -0.015, 0);
  knobBodyShape->type = ors::ShapeType::boxST;
  size = ARRAY(0.04, 0.03, 0.03, 0.);
  memmove(knobBodyShape->size, size.p, 4*sizeof(double));
  arr color = ARRAY(1.,0.,0.);
  memmove(knobBodyShape->color, color.p, 3*sizeof(double));

  ors::Shape *knobCP1Shape = new ors::Shape(world, *knobBody);
  knobCP1Shape->rel.pos = ors::Vector(0.02, -0.02, 0.);
  knobCP1Shape->type = ors::ShapeType::markerST;
  knobCP1Shape->name = "knobCP1Shape";
  size = ARRAY(0.05, 0.03, 0.03, 0.);
  memmove(knobCP1Shape->size, size.p, 4*sizeof(double));
  color = ARRAY(0.,1.,0.);
  memmove(knobCP1Shape->color, color.p, 3*sizeof(double));

  ors::Shape *knobCP2Shape = new ors::Shape(world, *knobBody);
  knobCP2Shape->rel.pos = ors::Vector(-0.02, -0.02, 0.);
  knobCP2Shape->type = ors::ShapeType::markerST;
  knobCP2Shape->name = "knobCP2Shape";
  size = ARRAY(0.05, 0.03, 0.03, 0.);
  memmove(knobCP2Shape->size, size.p, 4*sizeof(double));
  color = ARRAY(0.,1.,0.);
  memmove(knobCP2Shape->color, color.p, 3*sizeof(double));

  ors::Joint *knobJoint = new ors::Joint(world,fixedBody,knobBody);
  knobJoint->name = "knobJoint";
  knobJoint->A.pos = ARR(0., -0.1, 0.);
  //frame_door->B.pos = ARR(0, -door_w/2.-0.0085, .0);
  knobJoint->type = ors::JointType::JT_hingeY;


  world.calc_fwdPropagateFrames();


  TrajectoryInterface *ti = new TrajectoryInterface(world);

  ti->world->gl().resize(800,800); //segmentation fault again
  ti->world->gl().add(changeColor2);

  arr q = ti->world->getJointState();

  MotionProblem MP(world);

  Task *t;

  t = MP.addTask("transitions", new TransitionTaskMap(world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e1);

  double contactT = MP.T/2;

  t = MP.addTask("knobJoint", new TaskMap_qItself(world.getJointByName("knobJoint")->qIndex, world.getJointStateDimension()));
  t->setCostSpecs(MP.T-5, MP.T, {-3.14/2}, 1e3);

  t = MP.addTask("contact", new PointEqualityConstraint(world, "endeffC1",NoVector, "knobCP2Shape",NoVector));
  t->setCostSpecs(contactT, MP.T, {0.}, 10.);
  t = MP.addTask("contact", new PointEqualityConstraint(world, "endeffC2",NoVector, "knobCP1Shape",NoVector));
  t->setCostSpecs(contactT, MP.T, {0.}, 10.);

  t = MP.addTask("fix_joint", new qItselfConstraint(world.getJointByName("knobJoint")->qIndex, world.getJointStateDimension()));
  t->setCostSpecs(0.,contactT, {0.}, 1.);

  t = MP.addTask("qLimits", new LimitsConstraint());
  t->setCostSpecs(0., MP.T, {0.}, 1.);

  t = MP.addTask("collisionConstraints", new CollisionConstraint(.1));
  t->setCostSpecs(0., MP.T, {0.}, 100.);

  MotionProblemFunction MF(MP);

  arr x = MP.getInitialization();

  optConstrainedMix(x, NoArr, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2));

  ti->executeTrajectory(x, 10);
  ti->world->watch(true);

/*
  ors::Body *fixedBody = new ors::Body(world);
  //fixedBody->X.pos = T.pos;
  //fixedBody->X.rot = T.rot;
  fixedBody->X.pos = ors::Vector(1, 0, 0.8);
  fixedBody->X.rot = ors::Quaternion(-sqrt(2)/2,0.,0.,sqrt(2)/2);
  fixedBody->type = ors::BodyType::staticBT;
  ors::Shape *fixedBodyShape = new ors::Shape(world, *fixedBody);
  fixedBodyShape->type = ors::ShapeType::boxST;
  arr size = ARRAY(0.8, 0.15, 0.5, 0.);
  memmove(fixedBodyShape->size, size.p, 4*sizeof(double));

  ors::Body *knobBody = new ors::Body(world);
  //knobBody-> = ors::Vector(0, -0.015, 0);
  //knobBody->X.rot = ors::Quaternion(-sqrt(2)/2,0.,0.,sqrt(2)/2);
  knobBody->type = ors::BodyType::staticBT;
  ors::Shape *knobBodyShape = new ors::Shape(world, *knobBody);
  knobBodyShape->rel.pos = ors::Vector(0, -0.015, 0);
  knobBodyShape->type = ors::ShapeType::boxST;
  size = ARRAY(0.04, 0.03, 0.03, 0.);
  memmove(knobBodyShape->size, size.p, 4*sizeof(double));
  arr color = ARRAY(1.,0.,0.);
  memmove(knobBodyShape->color, color.p, 3*sizeof(double));

  ors::Shape *knobCP1Shape = new ors::Shape(world, *knobBody);
  knobCP1Shape->rel.pos = ors::Vector(0.02, -0.02, 0.);
  knobCP1Shape->type = ors::ShapeType::markerST;
  size = ARRAY(0.05, 0.03, 0.03, 0.);
  memmove(knobCP1Shape->size, size.p, 4*sizeof(double));
  color = ARRAY(0.,1.,0.);
  memmove(knobCP1Shape->color, color.p, 3*sizeof(double));

  ors::Shape *knobCP2Shape = new ors::Shape(world, *knobBody);
  knobCP2Shape->rel.pos = ors::Vector(-0.02, -0.02, 0.);
  knobCP2Shape->type = ors::ShapeType::markerST;
  size = ARRAY(0.05, 0.03, 0.03, 0.);
  memmove(knobCP2Shape->size, size.p, 4*sizeof(double));
  color = ARRAY(0.,1.,0.);
  memmove(knobCP2Shape->color, color.p, 3*sizeof(double));


  ors::Joint *knobJoint = new ors::Joint(world,fixedBody,knobBody);
  knobJoint->name = "knobJoint";
  knobJoint->A.pos = ARR(0., -0.1, 0.);
  //frame_door->B.pos = ARR(0, -door_w/2.-0.0085, .0);
  knobJoint->type = ors::JointType::JT_hingeY;

  /*
  body wheel_plate {}
  shape(wheel_plate) {type=4 color=[0 0 1] rel=<T d(90 1 0 0)> size=[0 0 .02 .15]}
  shape knob(wheel_plate) {type=0 color=[1 0 0] rel=<T t(0 -0.015 0)>  size=[0.04 0.03 0.03 0]}
  shape knob_cp1(wheel_plate) {type=5 color=[0 1 0] rel=<T t(0.02 -0.02 0)>  size=[0.05 0.03 0.03 0]}
  shape knob_cp2(wheel_plate) {type=5 color=[0 1 0] rel=<T t(-0.02 -0.02 0)>  size=[0.05 0.03 0.03 0]}

  joint box_wheel_plate_joint(box wheel_plate) {type=1 A=<T t(0 -0.1 0)>}
*/

//                     body box {X=<T t(1 0 0.8) d(-90 0 0 90)>}
 //                    shape box_shape(box) {type=0 rel=<T t(0 0 0)> size=[0.8 0.15 0.5 0]}

  /*ors::Body *frame = new ors::Body(G);
  frame->X.pos = ors::Vector(1.2, 1.2, door_h/2.);
  frame->type = ors::BodyType::staticBT;
  ors::Shape *frame_shape = new ors::Shape(G,*frame);
  frame_shape->type = ors::ShapeType::cylinderST;
  arr size = ARRAY(0.,0.,door_h,0.0085);
  memmove(frame_shape->size, size.p, 4*sizeof(double));
  arr color = ARRAY(0.,0.,0.);
  memmove(frame_shape->color, color.p, 3*sizeof(double));

  ors::Body *door = new ors::Body(G);
  door->type = ors::BodyType::staticBT;
  ors::Shape *door_shape = new ors::Shape(G,*door);
  door_shape->type = ors::ShapeType::boxST;
  size = ARRAY(.038, door_w, door_h, 0.);
  memmove(door_shape->size, size.p, 4*sizeof(double));
  color = ARRAY(0.5,0.5,0.);
  memmove(door_shape->color, color.p, 3*sizeof(double));


  ors::Body *handle = new ors::Body(G);
  handle->type = ors::BodyType::staticBT;
  ors::Shape *handle_shape = new ors::Shape(G,*handle);
  handle_shape->type = ors::ShapeType::boxST;
  size = ARRAY(.014, handle_w, handle_h, 0.);
  memmove(handle_shape->size, size.p, 4*sizeof(double));
  color = ARRAY(0.,0.,0.);
  memmove(handle_shape->color, color.p, 3*sizeof(double));

  ors::Joint *frame_door = new ors::Joint(G,frame,door);
  frame_door->name = "frame_door";
  frame_door->A.pos = ARR(0, 0, .0);
  frame_door->B.pos = ARR(0, -door_w/2.-0.0085, .0);
  frame_door->type = ors::JointType::JT_hingeZ;

  ors::Joint *door_handle = new ors::Joint(G,door,handle);
  door_handle->name = "door_handle";
  door_handle->A.pos = ARR(-0.07, -door_w/2.+handle_y, -door_h/2.+handle_z);
  door_handle->B.pos = ARR(0, handle_w/2. -0.015, .0);
  door_handle->type = ors::JointType::JT_hingeX;
  door_handle->limits = ARR(-0.5,0.5);

  ors::Shape *cp1 = new ors::Shape(G,*handle);
  cp1->name = "cp1";
  cp1->rel.pos = ors::Vector(0,0.05, handle_h/2.);
  cp1->type = ors::ShapeType::sphereST;
  size=ARRAY(.1, .1, .1, .01);
  memmove(cp1->size, size.p, 4*sizeof(double));
  color = ARRAY(1.,0.,0.);
  memmove(cp1->color, color.p, 3*sizeof(double));

  ors::Shape *cp2 = new ors::Shape(G,*handle);
  cp2->name = "cp2";
  cp2->rel.pos = ors::Vector(0,-0.02, -handle_h/2.);
  cp2->type = ors::ShapeType::sphereST;
  size=ARRAY(.1, .1, .1, .01);
  memmove(cp2->size, size.p, 4*sizeof(double));
  color = ARRAY(0.,1.,0.);
  memmove(cp2->color, color.p, 3*sizeof(double));*/

//  world.calc_fwdPropagateFrames();

  //world.watch(true);
/*
  TrajectoryInterface *ti = new TrajectoryInterface(world);

  ti->world->gl().resize(800,800); //segmentation fault again
  ti->world->gl().add(changeColor2);

  arr q = ti->world->getJointState();

  MotionProblem MP(world);

  Task *t;

  t = MP.addTask("transitions", new TransitionTaskMap(world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e1);

  double contactT = MP.T/2;

  //t = MP.addTask("box_wheel_plate_joint", new TaskMap_qItself(world.getJointByName("knobJoint")->qIndex, world.getJointStateDimension()));
  //t->setCostSpecs(MP.T-20, MP.T, {-3.14/2}, 1e3);

  t = MP.addTask("contact", new PointEqualityConstraint(world, "endeffC1",NoVector, "ve_Shape",NoVector));
  t->setCostSpecs(contactT, MP.T, {0.}, 10.);
  //t = MP.addTask("contact", new PointEqualityConstraint(world, "endeffC2",NoVector, "knobCP1Shape",NoVector));
  //t->setCostSpecs(contactT, MP.T, {0.}, 10.);

  //t = MP.addTask("fix_joint", new qItselfConstraint(world.getJointByName("knobJoint")->qIndex, world.getJointStateDimension()));
  //t->setCostSpecs(0.,contactT, {0.}, 1.);

  t = MP.addTask("qLimits", new LimitsConstraint());
  t->setCostSpecs(0., MP.T, {0.}, 1.);

  //ShapeL except = world.getBodyByName("l_wrist_roll_link")->shapes;
  //t = MP.addTask("collision", new ProxyConstraint(allExceptListedPTMT, shapesToShapeIndices(except), 0.1));
  //t->setCostSpecs(0., MP.T, {0.}, 1.);
  //t = MP.addTask("collisionConstraints", new CollisionConstraint(.1));
  //t->setCostSpecs(0., MP.T, {0.}, 100.);

  //t = MP.addTask("direction", new VelAlignConstraint(world, "l_wrist_roll_link_0", ors::Vector(0.297358,-0.277855,0.682291), "l_wrist_roll_link_0", ors::Vector(0.297358,-0.277855,0.682291)));
  //t = MP.addTask("direction", new VelAlignConstraint(world, "l_wrist_roll_link_0", ors::Vector(0.297358,-0.277855,0.682291), "l_wrist_roll_link_0", ors::Vector(0.297358,-0.277855,0.682291)));
  //t = MP.addTask("direction", new VelAlignConstraint(world, "endeffL", ors::Vector(1,0,0), "ve_shape", NoVector));
  //t->setCostSpecs(contactT-10, contactT, {0.}, 1000.);

  MotionProblemFunction MF(MP);

  arr x = MP.getInitialization();

  optConstrainedMix(x, NoArr, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2));

  ti->executeTrajectory(x, 10);
  ti->world->watch(true);
/*
  /// get contact point trajectory
  arr Y;
  for (uint i=contactT;i<x.d0;i++){
    world.setJointState(x[i]);
    arr tmp = conv_vec2arr(world.getShapeByName("endeffC1")->X.pos);
    Y.append(~tmp);
  }

  /// detect DOF
  ors::Transformation Tr;
  detectDOF(Y,Tr);
  ti->world->getBodyByName("dof_axis")->X = Tr;
  ti->world->calc_fwdPropagateFrames();
  ti->world->watch(true);
  cout << Tr.rot << endl;
  ti->~TrajectoryInterface();
*/
}

void turnBox() {
  ors::KinematicWorld world("model.kvg");
  makeConvexHulls(world.shapes);

  TrajectoryInterface *ti = new TrajectoryInterface(world);

  ti->world->gl().resize(800,800);
  ti->world->gl().add(changeColor2);

  arr q = ti->world->getJointState();

  MotionProblem MP(world);
  cout <<"joint dimensionality=" <<q.N <<endl;
//  MP.useSwift=false;

//  arr y,lim=G.getLimits();
//  G.kinematicsLimitsCost(y, NoArr, lim);
//  cout <<catCol(q,lim) <<endl <<y <<endl;

  //-- setup the motion problem
  Task *t;
  t = MP.addTask("transitions", new TransitionTaskMap(world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e-1);

  double contactT = MP.T/2.;
  //position task maps
  //t = MP.addTask("position", new DefaultTaskMap(posTMT, world, "endeffL", NoVector, "target",NoVector));
  //t->setCostSpecs(contactT-10., contactT-10., {0.}, 1e2);

  t = MP.addTask("plate_joint", new TaskMap_qItself(world.getJointByName("box_plate")->qIndex, world.getJointStateDimension()));
  t->setCostSpecs(MP.T-5, MP.T, {-3.14/2}, 1e3);

  //t = MP.addTask("door_joint", new TaskMap_qItself(G.getJointByName("frame_door")->qIndex, G.getJointStateDimension()));
  //t->setCostSpecs(MP.T, MP.T, {-.7}, 1e2);

  // constraints
  //t = MP.addTask("contact", new PointEqualityConstraint(world, "endeffL",NoVector, "target",NoVector));
  //t->setCostSpecs(contactT, MP.T, {0.}, 1.);

  t = MP.addTask("contact", new PointEqualityConstraint(world, "endeffC1",NoVector, "cp2",NoVector));
  t->setCostSpecs(contactT, MP.T, {0.}, 1.);
  t = MP.addTask("contact", new PointEqualityConstraint(world, "endeffC2",NoVector, "cp1",NoVector));
  t->setCostSpecs(contactT, MP.T, {0.}, 1.);
  //t = MP.addTask("direction2", new VelAlignConstraint(world, "endeffL",NoVector, "target", ors::Vector(-1.,0.,0.),0.4));
  //t->setCostSpecs(contactT+0., MP.T, {0.}, 1.);

  t = MP.addTask("handle_fixation", new qItselfConstraint(world.getJointByName("box_plate")->qIndex, world.getJointStateDimension()));
  t->setCostSpecs(0.,contactT, {0.}, 1.);

//  t = MP.addTask("direction1", new VelAlignConstraint(G, "endeffL",NoVector, "handle", ors::Vector(0.,0.,-1.),0.7));
//  t->setCostSpecs(MP.T/2.+5., MP.T/2.+10., {0.}, 1.);

//  t = MP.addTask("collision", new CollisionConstraint(0.05));
  ShapeL except = world.getBodyByName("l_wrist_roll_link")->shapes;
  t = MP.addTask("collision", new ProxyConstraint(allExceptListedPTMT, shapesToShapeIndices(except), 0.05));
  t->setCostSpecs(0., MP.T, {0.}, 1.);

  t = MP.addTask("qLimits", new LimitsConstraint());
  t->setCostSpecs(0., MP.T, {0.}, 1.);


  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr x = MP.getInitialization();
  arr lambda = zeros(x.d0,2);
  cout << x.d0 << endl;
  cout << x.d1 << endl;
  cout << q.d0 << endl;
  cout << q.d1 << endl;

  optConstrainedMix(x, NoArr, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2));
//
  ti->executeTrajectory(x, 10);

}
/*
void vel() {
  ors::KinematicWorld G("table.ors");

    arr q;
    G.getJointState(q);
    q.setZero();
    G.setJointState(q+0.1);
    cout << "q: " << q << endl;
    //G.watch(true);
    MotionProblem MP(G);

    //-- setup the motion problem
    Task *t;
    t = MP.addTask("transitions", new TransitionTaskMap(G));
    t->map.order=2; //make this an acceleration task!
    t->setCostSpecs(0, MP.T, {0.}, 1e1);

  //  t = MP.addTask("final_vel", new TransitionTaskMap(G));
  //  t->map.order=1; //make this a velocity task!
  //  t->setCostSpecs(MP.T-4, MP.T, {0.}, 1e2);

    t = MP.addTask("position", new DefaultTaskMap(posTMT, G, "endeff", NoVector, NULL, G.getBodyByName("target")->X.pos));
    t->setCostSpecs(MP.T, MP.T, {0.}, 1000);

  //  t = MP.addTask("collision", new PairCollisionConstraint(G, "table", "endeff", 0.1));
  //  t->setCostSpecs(0., MP.T, {0.}, 1.);

    t = MP.addTask("contact", new VelAlignConstraint(G, "endeff",NoVector, "table", ors::Vector(0,1.,0.)));
    t->setCostSpecs(0, MP.T, {0.}, 10.);


    //-- create the Optimization problem (of type kOrderMarkov)
    MotionProblemFunction MF(MP);
    arr x = MP.getInitialization();
    arr lambda = zeros(x.d0,2);

  optConstrainedMix(x, NoArr, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2));
  //  optConstrainedMix(x, lambda, Convert(MF));
    //checkGradient(Convert(MF),x,1e-3);

  //  cout << lambda << endl;
  //  MP.costReport();
  TrajectoryInterface *ti = new TrajectoryInterface(G);

  ti->world->gl().resize(800,800);
  ti->world->gl().add(changeColor2);
  ti->executeTrajectory(x, 10);
  ti->world->watch(true);
}*/

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  turnPlate();
  //ors::Transformation T;
  //makeNewWorld(T);
  return 0;
}
