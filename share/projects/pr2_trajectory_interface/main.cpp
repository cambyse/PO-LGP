#include <Motion/gamepad2tasks.h>
#include <Control/taskController.h>
#include <Hardware/gamepad/gamepad.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>
#include <Motion/phase_optimization.h>
#include <Optim/opt-constrained.h>

#include <RosCom/roscom.h>
#include <RosCom/rosmacro.h>
#include <RosCom/rosalvar.h>
#include <RosCom/trajectoryInterface.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

void changeColor2(void*){  orsDrawAlpha = 1.; }
/*
void graspBox(){


  ors::Shape *object = ti->world_plan->getShapeByName("box");
  /// move robot to initial position
  //  arr q;
  //  ti->getState(q);
  //  write(LIST<arr>(q),"q0.dat");
  //  q << FILE("q0.dat"); q.flatten();
  //  q(ti->world_pr2->getJointByName("torso_lift_joint")->qIndex) += 0.1;

  arr X;
  MotionProblem MP(*ti->world_plan);
  Task *t;
  t = MP.addTask("transitions", new TransitionTaskMap(world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e-1);

  t = MP.addTask("pos1", new DefaultTaskMap(posTMT, *ti->world_plan, "endeffL", NoVector, object->name,ors::Vector(0.,0.,0.1)));
  t->setCostSpecs(70, 80, {0.}, 1e2);
  t = MP.addTask("rot1", new DefaultTaskMap(vecAlignTMT, *ti->world_plan, "endeffL", ors::Vector(1.,0.,0.), "base_link_0",ors::Vector(0.,0.,-1.)));
  t->setCostSpecs(70, MP.T, {1.}, 1e1);
  t = MP.addTask("rot2", new DefaultTaskMap(vecAlignTMT, *ti->world_plan, "endeffL", ors::Vector(0.,0.,1.), object->name,ors::Vector(1.,0.,0.)));
  t->setCostSpecs(70, MP.T, {1.}, 1e1);
  t = MP.addTask("pos2", new DefaultTaskMap(posTMT, *ti->world_plan, "endeffL", NoVector, object->name,ors::Vector(0.,0.,0.)));
  t->setCostSpecs(MP.T-5, MP.T, {0.}, 1e2);
  t = MP.addTask("limit", new LimitsConstraint());
  t->setCostSpecs(0, MP.T, ARR(0.), 1e2);
  ShapeL shaps = {
    ti->world_plan->getShapeByName("l_forearm_roll_link_0"), ti->world_plan->getShapeByName("table"),
    ti->world_plan->getShapeByName("l_elbow_flex_link_0"), ti->world_plan->getShapeByName("table")
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
}*/

void TEST(TrajectoryInterface){
  ors::KinematicWorld world("model_plan.kvg");
  ors::KinematicWorld world_pr2("model.kvg");
  makeConvexHulls(world.shapes);
  TrajectoryInterface *ti = new TrajectoryInterface(world,world_pr2);

  ti->world_plan->watch(false);
  ti->world_pr2->watch(false);
  ti->world_plan->gl().resize(800,800);
  ti->world_pr2->gl().resize(800,800);
  ti->world_pr2->gl().add(changeColor2);


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
      q(qIdx) = lim(qIdx,0)+rand(1)*(lim(qIdx,1)-lim(qIdx,0))*alpha;
    }

    ti->gotoPositionPlan(q,5.,true,true);
    ti->logging("data/",2);
  }

  ti->~TrajectoryInterface();
}
/*
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
}*/

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  testTrajectoryInterface();
//  testRecordReplay();
  return 0;
}
