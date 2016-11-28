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
#include <pr2/trajectoryInterface.h>

#include "../../src/task_manager.h"
#include "../../src/plotUtil.h"
#include "../../src/traj_factory.h"
#include "src/articulation_interface.h"

#include <ros/ros.h>

void detectButtonDOF() {
  mlr::String taskName = mlr::getParameter<mlr::String>("taskName");
  mlr::String folder = mlr::getParameter<mlr::String>("folder");
  bool useMarker = mlr::getParameter<bool>("useMarker",false);

  /// init task
  mlr::KinematicWorld world_pr2("../../../../share/projects/pr2_gamepadControl/model.kvg");
  mlr::KinematicWorld world(STRING(folder<<"model.kvg"));

  TaskManager *task;
  if (taskName == "door") {
    task = new DoorTask(world);
  } else if (taskName == "grasp") {
    task = new GraspTask(world);
  } else if (taskName == "button") {
    task = new ButtonTask(world);
  }

  /// load demo from file
  uint i;
  i << FILE(STRING(folder<<"bestIdx_Param_CBO.dat"));

  arr X,XPR2,FLdemo;
  XPR2 << FILE(STRING(folder<<i<<"_Param_Xref.dat"));
  transferQbetweenTwoWorlds(X,XPR2,world,world_pr2);
  FLdemo << FILE(STRING(folder<<"0_Demo_FL.dat"));

  /// extract contact point trajectory
  world.gl().update(); world.gl().resize(800,800);
  task->computeConstraintTime(FLdemo,X);
  task->updateVisualization(world,X);

  TrajectoryInterface *ti;
  ti = new TrajectoryInterface(world,world_pr2);

  TrajFactory tf;
  arr yL;
  tf.compFeatTraj(X,yL,world,new DefaultTaskMap(posTMT,world,"endeffC1"));
  arr Xcp = yL.rows(task->conStart(0),task->conEnd(0));

  /// add DOF to kinematic world
  mlr::Vector prismatic_dir;
  arr Q;
//  detectDOFtrans(Xcp,prismatic_dir,Q);
  prismatic_dir = mlr::Vector(0.,0.,1.);
  Q = Xcp.col(2); Q.flatten();
  Q = Q - Q(0);

  mlr::Quaternion rot;
  rot.setDiff(Vector_z,prismatic_dir);
  /// add new joint to world
  mlr::Body *b1 = new mlr::Body(world);
  b1->name = "b1";
  b1->type = mlr::BodyType::dynamicBT;
  b1->X.pos = Xcp[0];
  b1->X.rot = rot;
  mlr::Shape *b1_shape = new mlr::Shape(world,*b1);
  b1_shape->type = mlr::ShapeType::boxST;
  b1_shape->name = "b1_shape";
  arr size = ARRAY(0.1, 0.1, 0.001, 0.);
  memmove(b1_shape->size, size.p, 4*sizeof(double));
  arr color = ARRAY(0., 0., 1.);
  memmove(b1_shape->color, color.p, 3*sizeof(double));

  mlr::Body *b2 = new mlr::Body(world);
  b2->name = "b2";
  b2->type = mlr::BodyType::dynamicBT;
  b2->X.pos = Xcp[0];
  b2->X.rot = rot;
  mlr::Shape *b2_shape = new mlr::Shape(world,*b2);
  b2_shape->type = mlr::ShapeType::boxST;
  b2_shape->name = "b2_shape";
  size = ARRAY(0.1,0.1, 0.001, 0.);
  memmove(b2_shape->size, size.p, 4*sizeof(double));
  color = ARRAY(0.,1.,0.);
  memmove(b2_shape->color, color.p, 3*sizeof(double));
  world.calc_fwdPropagateFrames();

  mlr::Shape *cp1 = new mlr::Shape(world,*b1);
  cp1->name = "cp1";
  cp1->type = mlr::ShapeType::markerST;
  cp1->rel.pos = mlr::Vector(0.,0.,0.);
  size = ARRAY(0.01,0., 0., 0.);
  memmove(cp1->size, size.p, 4*sizeof(double));
  world.calc_fwdPropagateFrames();

  mlr::Joint *b2_b1 = new mlr::Joint(world,b2,b1);
  b2_b1->name = "b2_b1";
  b2_b1->A.pos = ARR(0, 0, .0);
  b2_b1->B.pos = ARR(0, 0, .0);
  b2_b1->type = mlr::JointType::JT_transZ;
  b2_b1->qIndex = world.getJointStateDimension()-1;

  world.calc_fwdPropagateFrames();

  /// augment trajectory with DOF
  arr Q2;
  Q2 = zeros(X.d0);
  Q2.subRange(task->conStart(0),task->conEnd(0)-1) = Q;
  X = ~X;   X.append(Q2);   X = ~X;

  /// save kinematic world
  world>>FILE(STRING(folder<<"modelaug.ors"));

  mlr::KinematicWorld worldaug(STRING(folder<<"modelaug.kvg"));
  TrajectoryInterface *ti2 = new TrajectoryInterface(worldaug,world_pr2);
  /// convert trajectory between two kinematic worlds
  arr Xaug;
  transferQbetweenTwoWorlds(Xaug,X,worldaug,world);
  write(LIST(Xaug),STRING(folder<<"0_Dof_X_Aug.dat"));
  write(LIST(XPR2),STRING(folder<<"0_Dof_X.dat"));

  worldaug.watch(false);
  worldaug.gl().resize(800,800);

  for(;;) {displayTrajectory(Xaug, 1, worldaug, "planned trajectory");}
}


int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  ros::init(argc, argv, "DOFE");

  detectButtonDOF();
//  detectDoorDOF();

  return 0;

}
