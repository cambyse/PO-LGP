#include <Motion/gamepad2tasks.h>
#include <Motion/feedbackControl.h>
#include <Hardware/gamepad/gamepad.h>
#include <System/engine.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>
#include <pr2/roscom.h>
#include <pr2/rosmacro.h>
#include <pr2/rosalvar.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Core/array-vector.h>
#include <pr2/trajectoryInterface.h>


void changeColor2(void*){  orsDrawAlpha = 1.; }

void TEST(TrajectoryInterface){
  ors::KinematicWorld world("model.kvg");
  makeConvexHulls(world.shapes);

  TrajectoryInterface *ti = new TrajectoryInterface(world);

  ti->world->gl().resize(800,800);
  ti->world->gl().add(changeColor2);

  arr q = ti->world->getJointState();
  q(ti->world->getJointByName("r_elbow_flex_joint")->qIndex)+= 0.2;

  ti->gotoPosition(q);
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
  arr Y;
  Y <<FILE("data/Xdes1.dat");
  ti->gotoPosition(Y[0]);
  ti->executeTrajectory(Y,10.);
  ti->~TrajectoryInterface();
}

int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);
  testTrajectoryInterface();
//  testRecordReplay();
  return 0;
}
