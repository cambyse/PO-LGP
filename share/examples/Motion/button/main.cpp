#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <iomanip>
#include <Ors/ors_swift.h>
#include <Ors/ors.h>

void TEST(Button){
  mlr::KinematicWorld world("model.kvg");

  world.meldFixedJoints();
  world.removeUselessBodies();
  makeConvexHulls(world.shapes);

  world.watch(false);  world.gl().resize(800,800);
  world.watch(true);
  arr q;
  world.getJointState(q);
  cout << q << endl;

  MotionProblem MP(world,false);

  cout << "joint dimensionality=" <<q.N <<endl;

  Task *t;
  t = MP.addTask("transitions", new TaskMap_Transition(world),sumOfSqrTT);
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e-1);

  double contactT = MP.T/2.;
  t = MP.addTask("prePos", new TaskMap_Default(posTMT, world, "endeffL", NoVector, "button",mlr::Vector(0.,0.,0.2)),sumOfSqrTT);
  t->setCostSpecs(contactT-10., contactT-10, {0.}, 1e2);
  t = MP.addTask("preVec", new TaskMap_Default(vecAlignTMT, world, "endeffL", mlr::Vector(1.,0.,0.), "button",mlr::Vector(0.,0.,-1.)),sumOfSqrTT);
  t->setCostSpecs(contactT-10., contactT-10, {1.}, 1e2);
  t = MP.addTask("button_joint", new TaskMap_qItself(world.getJointByName("stand_button")->qIndex, world.getJointStateDimension()),sumOfSqrTT);
  t->setCostSpecs(MP.T, MP.T, {-.1}, 1e2);

  t = MP.addTask("endeff_button", new PointEqualityConstraint(world,"endeffL",NoVector,"cp1",NoVector),eqTT);
  t->setCostSpecs(contactT+1,MP.T, {0.}, 1.);
  t = MP.addTask("button_fixation", new qItselfConstraint(world.getJointByName("stand_button")->qIndex, world.getJointStateDimension()),eqTT);
  t->setCostSpecs(0.,contactT, {0.}, 1.);
  t = MP.addTask("qLimits", new LimitsConstraint(),ineqTT);
  t->setCostSpecs(0., MP.T, {0.}, 1.);

  //-- create the Optimization problem (of type kOrderMarkov)
  arr X  = MP.getInitialization();
  X.reshape(MP.T,world.getJointStateDimension());
  arr lambda;

  optConstrained(X, lambda, Convert(MP), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2));
  cout << lambda << endl;
  cout << lambda.d0 << endl;
  cout << X.d0 << endl;


  mlr::KinematicWorld world_pr2("../../../projects/pr2_gamepadControl/model.kvg");
  world_pr2.watch(true);
  arr X_pr2;
  transferQbetweenTwoWorlds(X_pr2,X,world_pr2,world);
  displayTrajectory(X_pr2, 1, world_pr2, "planned trajectory");

  arr q_pr2 = world_pr2.getJointState();
  arr q_plan;
  transferQbetweenTwoWorlds(q_plan,q_pr2,world,world_pr2);
  world.setJointState(q_plan);
  world.watch(true);

  MP.costReport();
  for (;;) {
    displayTrajectory(X, 1, world, "planned trajectory");
    displayTrajectory(X, 1, world, "planned trajectory");
  }
}

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);
  testButton();
  return 0;
}
