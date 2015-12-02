#include <Motion/gamepad2tasks.h>
#include <Motion/feedbackControl.h>
#include <Hardware/gamepad/gamepad.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>
#include <pr2/roscom.h>
#include <pr2/rosmacro.h>
#include <pr2/rosalvar.h>
#include <pr2/trajectoryInterface.h>

//#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <iomanip>
#include <Ors/ors_swift.h>

void changeColor2(void*){  orsDrawAlpha = 1.; }

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
  mlr::initCmdLine(argc, argv);
  turnBox();
  //testTrajectoryInterface();
//  testRecordReplay();
  return 0;
}
