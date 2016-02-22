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
#include <visualization_msgs/MarkerArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros_msg/ObjId.h>
#include <ros_msg/MarkerArray.h>


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

  ors::Shape *object = world.getShapeByName("box");

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

// void obj_id_callback(const visualization_msgs::MarkerArrayConstPtr &msg_ma, const obj_id_pkg::ObjIdConstPtr &msg_oid) {
//   cout << "HERE" << endl;
//   // TODO
//   // convert all of the markerarrays into ors objects
//   // issue command to grasp object with id msg_oid->obj_id
// }

// Mutex mutex;
// bool update_ma = true;
// visualization_msgs::MarkerArray ma;

// void cluster_callback(const visualization_msgs::MarkerArrayPtr &msg) {
//   cout << "Cluster_callback" << endl;
//   mutex.lock();
//   ma = *msg;
//   mutex.unlock();
// }

// // void oid_callback(const obj_id_pkg::ObjIdPtr &msg) {
// void oid_callback(const std_msgs::String &msg) {
//   cout << "Oid_callback" << endl;
//   mutex.lock();
//   visualization_msgs::MarkerArray marker_array = ma;
//   mutex.unlock();

//   // insert actual grasping code
// }

// void cluster_callback(const visualization_msgs::MarkerArray &msg) {
//   obj_id_pkg::MarkerArray ma();
//   ma.header = msg->markers[0].header;
//   ma.marker_array = msg;
// }

struct PR2Grasp {
  ros::NodeHandle &nh;
  ros::Subscriber cluster_sub;
  ros::Publisher cluster_pub;
  ros::Publisher obj_id_pub;

  message_filters::Subscriber<obj_id_pkg::MarkerArray> *eyespy_cluster_sub;
  message_filters::Subscriber<obj_id_pkg::ObjId> *eyespy_oid_sub;
  message_filters::TimeSynchronizer<obj_id_pkg::MarkerArray, obj_id_pkg::ObjId> *eyespy_sync;

  PR2Grasp(ros::NodeHandle &nh_);

  void eyespy_cluster_callback(const visualization_msgs::MarkerArray &msg);
  void eyespy_grasp_callback(const obj_id_pkg::MarkerArrayConstPtr &msg_ma, const obj_id_pkg::ObjIdConstPtr &msg_oid);
  void run(void);
};

PR2Grasp::PR2Grasp(ros::NodeHandle &nh_): nh(nh_) {
  cluster_sub = nh.subscribe("/tabletop/clusters", 10, &PR2Grasp::eyespy_cluster_callback, this);
  cluster_pub = nh.advertise<obj_id_pkg::MarkerArray>("/eyespy/clusters", 10);
  obj_id_pub = nh.advertise<obj_id_pkg::ObjId>("/eyespy/obj_id", 10);

  eyespy_cluster_sub = new message_filters::Subscriber<obj_id_pkg::MarkerArray>(nh, "/eyespy/clusters", 10);
  eyespy_oid_sub = new message_filters::Subscriber<obj_id_pkg::ObjId>(nh, "/eyespy/obj_id", 10);
  eyespy_sync = new message_filters::TimeSynchronizer<obj_id_pkg::MarkerArray, obj_id_pkg::ObjId>(*eyespy_cluster_sub, *eyespy_oid_sub, 10);
  // void (PR2Grasp::*callback)(const obj_id_pkg::MarkerArray &, const obj_id_pkg::ObjId &) = &this->eyespy_grasp_callback;
  // &PR2Grasp::eyespy_grasp_callback;
  // eyespy_sync->registerCallback(boost::bind(callback, this, _1, _2));
  // eyespy_sync->registerCallback(boost::bind(this->eyespy_grasp_callback, this, _1, _2));
  eyespy_sync->registerCallback(boost::bind(&PR2Grasp::eyespy_grasp_callback, this, _1, _2));
}

void PR2Grasp::eyespy_cluster_callback(const visualization_msgs::MarkerArray &msg) {
  obj_id_pkg::MarkerArray obj_id_ma;
  obj_id_ma.header = msg.markers[0].header;
  obj_id_ma.markers = msg.markers;

  cluster_pub.publish(obj_id_ma);

//   obj_id_pkg::ObjId oid;
//   oid.header = obj_id_ma.header;
//   oid.obj_id = 1;
//   obj_id_pub.publish(oid);
}

void PR2Grasp::eyespy_grasp_callback(const obj_id_pkg::MarkerArrayConstPtr &msg_ma, const obj_id_pkg::ObjIdConstPtr &msg_oid) {
  cout << "hello" << endl;
}

void PR2Grasp::run(void) {
  ros::spin();
}

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
//  testTrajectoryInterface();
  // graspBox();
  // return 0;

  ros::init(argc, argv, "pr2_tabletop_grasp");

  ros::NodeHandle nh;
  PR2Grasp pr2grasp(nh);
  pr2grasp.run();


  // ros::Subscriber ma_sub = nh.subscribe("/tabletop/clusters", 10, cluster_callback);

//   message_filters::Subscriber<obj_id_pkg::MarkerArray> cluster_sub(nh, "/eyespy/clusters", 1);
//   message_filters::Subscriber<obj_id_pkg::ObjId> oid_sub(nh, "/eyespy/obj_id", 1);
//   message_filters::TimeSynchronizer<visualization_msgs::MarkerArray, obj_id_pkg::ObjId> sync(cluster_sub, oid_sub, 10);
//   sync.registerCallback(boost::bind(&callback, _1, _2));
  
  // ros::Subscriber cluster_sub = nh.subscribe("/tabletop/clusters", 10, cluster_callback);
  // ros::Subscriber oid_sub = nh.subscribe("/eyespy/obj_id", 10, oid_callback);

  // ros::spin();

  return 0;
}
