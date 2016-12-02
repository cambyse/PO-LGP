#include <Control/gamepad2tasks.h>
#include <Control/taskController.h>
#include <Hardware/gamepad/gamepad.h>
#include <Gui/opengl.h>

#include <Motion/phase_optimization.h>
#include <Optim/lagrangian.h>

#include <RosCom/roscom.h>
#include <RosCom/subscribeAlvarMarkers.h>
#include <RosCom/trajectoryInterface.h>

void TEST(TrajectoryInterface){
  mlr::KinematicWorld world_plan("model_plan.kvg");
  mlr::KinematicWorld world_robot("model.kvg");
  makeConvexHulls(world_plan.shapes);
  TrajectoryInterface *ti = new TrajectoryInterface(world_plan,world_robot);

  ti->syncState();
  ti->syncMarker();

  ti->moveRightGripper(0.01);
//  ti->syncMarker();

  arr q;
  arr lim;
  double alpha = 0.8;
  uintA qIdxList;
  rnd.clockSeed();
  lim = ti->world_plan->getLimits();

  /// define the joints that should be used
  qIdxList.append(ti->world_plan->getJointByName("l_elbow_flex_joint")->qIndex);
  qIdxList.append(ti->world_plan->getJointByName("l_wrist_roll_joint")->qIndex);
  qIdxList.append(ti->world_plan->getJointByName("l_wrist_flex_joint")->qIndex);
  qIdxList.append(ti->world_plan->getJointByName("l_forearm_roll_joint")->qIndex);
  qIdxList.append(ti->world_plan->getJointByName("l_upper_arm_roll_joint")->qIndex);
  qIdxList.append(ti->world_plan->getJointByName("l_shoulder_lift_joint")->qIndex);

  for (;;) {
    q = ti->world_plan->getJointState();

    /// sample a random goal position
    for (uint i=0;i<qIdxList.d0;i++) {
      uint qIdx = qIdxList(i);
      q(qIdx) = lim(qIdx,0)+rand(1).last()*(lim(qIdx,1)-lim(qIdx,0))*alpha;
    }

    ti->gotoPositionPlan(q,15.,true,true);
    ti->logging("data/","test",2);
  }

  ti->~TrajectoryInterface();
}

void TEST(RecordReplay) {
  mlr::KinematicWorld world_plan("model_plan.kvg");
  mlr::KinematicWorld world_robot("model.kvg");

  TrajectoryInterface *ti = new TrajectoryInterface(world_plan,world_robot);

  arr X;
  ti->recordDemonstration(X,10.);
  cout << X << endl;
  ti->gotoPosition(X[0]);
  ti->executeTrajectory(X,10.,true);
  ti->logging("data/","test",1);

  /// load demo from file
  arr Y;
  Y <<FILE("data/1_test_Xdes.dat");

  ti->gotoPosition(Y[0]);
  ti->executeTrajectory(Y,10.);
  ti->~TrajectoryInterface();
}

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

//  testTrajectoryInterface();
  testRecordReplay();
  return 0;
}
