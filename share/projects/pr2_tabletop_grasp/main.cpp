#include <Motion/gamepad2tasks.h>
#include <Motion/feedbackControl.h>
#include <Hardware/gamepad/gamepad.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>
#include <Motion/phase_optimization.h>
#include <Optim/opt-constrained.h>
#include <pr2/roscom.h>
#include <pr2/rosmacro.h>
#include <pr2/rosalvar.h>
#include <pr2/trajectoryInterface.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros_msg/ObjId.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <visualization_msgs/MarkerArray.h>


void changeColor2(void*){  orsDrawAlpha = 0.5; }

void graspBox(){
  ors::KinematicWorld world("model_plan.kvg");
  ors::KinematicWorld world_pr2("model.kvg");
  makeConvexHulls(world.shapes);
  TrajectoryInterface *ti = new TrajectoryInterface(world,world_pr2);

  ti->world_plan->watch(false);
  ti->world_pr2->watch(false);
  ti->world_plan->gl().resize(800,800);
  ti->world_pr2->gl().resize(800,800);
  ti->world_pr2->gl().add(changeColor2);


  ors::Shape *object = world.getShapeByName("box");
  /// move robot to initial position
  //  arr q;
  //  ti->getState(q);
  //  write(LIST<arr>(q),"q0.dat");
  //  q << FILE("q0.dat"); q.flatten();
  //  q(ti->world_pr2->getJointByName("torso_lift_joint")->qIndex) += 0.1;

  arr X;
  MotionProblem MP(world);
  Task *t;
  t = MP.addTask("transitions", new TransitionTaskMap(world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e-1);

  t = MP.addTask("pos1", new DefaultTaskMap(posTMT, world, "endeffL", NoVector, object->name,ors::Vector(0.,0.,0.1)));
  t->setCostSpecs(70, 80, {0.}, 1e2);
  t = MP.addTask("rot1", new DefaultTaskMap(vecAlignTMT, world, "endeffL", ors::Vector(1.,0.,0.), "base_link_0",ors::Vector(0.,0.,-1.)));
  t->setCostSpecs(70, MP.T, {1.}, 1e1);
  t = MP.addTask("rot2", new DefaultTaskMap(vecAlignTMT, world, "endeffL", ors::Vector(0.,0.,1.), object->name,ors::Vector(1.,0.,0.)));
  t->setCostSpecs(70, MP.T, {1.}, 1e1);
  t = MP.addTask("pos2", new DefaultTaskMap(posTMT, world, "endeffL", NoVector, object->name,ors::Vector(0.,0.,0.)));
  t->setCostSpecs(MP.T-5, MP.T, {0.}, 1e2);
  t = MP.addTask("limit", new TaskMap_qLimits());
  t->setCostSpecs(0, MP.T, {0.}, 1e2);
  ShapeL shaps = {
    world.getShapeByName("l_forearm_roll_link_0"), world.getShapeByName("table"),
    world.getShapeByName("l_elbow_flex_link_0"), world.getShapeByName("table")
  };
  t = MP.addTask("collision", new ProxyConstraint(pairsPTMT, shapesToShapeIndices(shaps), 0.1));
  t->setCostSpecs(0., MP.T, {0.}, 1.);

  MotionProblemFunction MPF(MP);
  X = MP.getInitialization();

  optConstrained(X, NoArr, Convert(MPF), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2));

  MP.costReport(true);
  for (;;)
    displayTrajectory(X, 1, *ti->world_plan, "planned trajectory");

  ti->gotoPositionPlan(X[0]);
  ti->executeTrajectoryPlan(X,10.,true,true);
  ti->~TrajectoryInterface();
}

void obj_id_callback(const visualization_msgs::MarkerArrayConstPtr &msg_ma, const obj_id_pkg::ObjIdConstPtr &msg_oid) {
  cout << "HERE" << endl;
  // TODO
  // convert all of the markerarrays into ors objects
  // issue command to grasp object with id msg_oid->obj_id
}

void TEST(PointCloud) {
  ors::KinematicWorld world("model_plan.kvg");
  world.watch(false);
  world.gl().resize(800,800);
  world.gl().add(changeColor2);

  ors::Shape *pcShape = new ors::Shape(world,NoBody);
  pcShape->type = ors::pointCloudST;
  pcShape->name = "pcShape";
  uint N=1000;
  arr scale = eye(3)*0.01;
  scale(1,1) = .05;
  scale(2,2) = .1;
  scale(1,2) = .1;
  arr points = randn(N,3)*scale+1.;
  pcShape->mesh.V = points;
  pcShape->mesh.computeNormals();
  world.calc_fwdPropagateFrames();
  world.watch(true);

  /// fit box to pointcloud
  arr center = sum(points,0)/double(points.d0);
  center.flatten();

  arr Y,v,W;
  pca(Y,v,W,points);
  arr dir = W.col(0); dir.flatten();

  ors::Body *boxBody = new ors::Body(world);
  boxBody->name = "boxBody";
  boxBody->type = ors::BodyType::dynamicBT;
  boxBody->X.pos = center;
  boxBody->X.rot.setDiff(ors::Vector(1.,0.,0.),dir);
  ors::Shape *boxShape = new ors::Shape(world,*boxBody);
  boxShape->type = ors::boxST;
  boxShape->name = "boxShape";
  arr size = ARRAY(Y.col(0).max()-Y.col(0).min(),Y.col(1).max()-Y.col(1).min(),Y.col(2).max()-Y.col(2).min(), 0.);
  memmove(boxShape->size, size.p, 4*sizeof(double));
  arr color = ARRAY(0.1,0.5,0.1);
  memmove(boxShape->color, color.p, 3*sizeof(double));
  world.calc_fwdPropagateFrames();
  world.watch(true);
}

void glDrawMesh(void *classP) {
  ((ors::Mesh*)classP)->glDraw();
}

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  testPointCloud();
//  graspBox();
  return 0;

  return 0;
}
