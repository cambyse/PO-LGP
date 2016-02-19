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
#include <ros_msg/ObjId.msg>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

void changeColor2(void*){  orsDrawAlpha = 1.; }

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

  arr X;
  MotionProblem MP(world);
  Task *t;
  t = MP.addTask("transitions", new TransitionTaskMap(world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e-1);

  t = MP.addTask("position", new DefaultTaskMap(posTMT, world, "endeffL", NoVector, "box",NoVector));
  t->setCostSpecs(50, 50, {0.}, 1e2);

  MotionProblemFunction MPF(MP);
  X = MP.getInitialization();

  optConstrained(X, NoArr, Convert(MPF), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2));

  MP.costReport();
  displayTrajectory(X, 1, world, "planned trajectory");

  ti->gotoPositionPlan(X[0]);
  ti->executeTrajectoryPlan(X,10.,true,true);
  ti->~TrajectoryInterface();
}

void obj_id_callback(const MarkerArrayPtr &msg_ma, const ObjIdPtr& msg_oid) {
  cout << "HERE" << endl;
  // TODO
  // convert all of the markerarrays into ors objects
  // issue command to grasp object with id msg_oid->obj_id
}

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
//  testTrajectoryInterface();
  graspBox();

  ros::init(argc, argv, "pr2_tabletop_grasp");

  ros::NodeHandle nh;

  message_filters::Subscriber<MarkerArray> cluster_sub(nh, "/tabletop/clusters", 1);
  message_filters::Subscriber<ros_msg::ObjId> obj_id_sub(nh, "/eyespy/obj_id", 1);
  TimeSynchronizer<MarkerArray, ros_msg::ObjId> sync(cluster_sub, obj_id_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
