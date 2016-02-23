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

    ti->gotoPosition(q,5.,true,true);
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

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  testTrajectoryInterface();
//  testRecordReplay();
  return 0;
}
