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

void TEST(TrajectoryInterface){
  ors::KinematicWorld world("model.kvg");
  makeConvexHulls(world.shapes);

  TrajectoryInterface *ti = new TrajectoryInterface(world);

  ti->world->gl().resize(800,800);
  ti->world->gl().add(changeColor2);

  arr q;
  arr lim;
  double alpha = 0.8;
  uintA qIdxList;
  rnd.clockSeed();
  lim = ti->world->getLimits();

  /// define the joints that should be used
  qIdxList.append(ti->world->getJointByName("l_elbow_flex_joint")->qIndex);
  qIdxList.append(ti->world->getJointByName("l_wrist_roll_joint")->qIndex);
  qIdxList.append(ti->world->getJointByName("l_wrist_flex_joint")->qIndex);
  qIdxList.append(ti->world->getJointByName("l_forearm_roll_joint")->qIndex);
  qIdxList.append(ti->world->getJointByName("l_upper_arm_roll_joint")->qIndex);
  qIdxList.append(ti->world->getJointByName("l_shoulder_lift_joint")->qIndex);


  for (;;) {
    q = ti->world->getJointState();

    /// sample a random goal position
    for (uint i=0;i<qIdxList.d0;i++) {
      uint qIdx = qIdxList(i);
      q(qIdx) = lim(qIdx,0)+rand(1)*(lim(qIdx,1)-lim(qIdx,0))*alpha;
    }

    ti->gotoPosition(q,5.,true);
    ti->logging("data/",2);
  }

  ti->~TrajectoryInterface();
}

void TEST(RecordReplay) {
  ors::KinematicWorld world("model.kvg");
  makeConvexHulls(world.shapes);

  TrajectoryInterface *ti = new TrajectoryInterface(world);

  ti->world->gl().resize(800,800);
  ti->world->gl().add(changeColor2);
  ti->world->watch(true);

  arr X;
  ti->recordDemonstration(X,10.);
  cout << X << endl;
  ti->gotoPosition(X[0]);
  ti->executeTrajectory(X,10.,true);
  ti->logging("data/",1);

  /// load demo from file
  arr Y;
  Y <<FILE("data/Xdes1.dat");

  ti->gotoPosition(Y[0]);
  ti->executeTrajectory(Y,10.);
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
  testTrajectoryInterface();
//  testRecordReplay();

  ros::init(argc, argv, "pr2_tabletop_grasp");

  ros::NodeHandle nh;

  message_filters::Subscriber<MarkerArray> cluster_sub(nh, "/tabletop/clusters", 1);
  message_filters::Subscriber<ros_msg::ObjId> obj_id_sub(nh, "/eyespy/obj_id", 1);
  TimeSynchronizer<MarkerArray, ros_msg::ObjId> sync(cluster_sub, obj_id_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
