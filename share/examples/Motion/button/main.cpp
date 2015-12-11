#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <iomanip>
#include <Ors/ors_swift.h>
#include <Ors/ors.h>

void TEST(Button){
  ors::KinematicWorld G("model.kvg");

  G.meldFixedJoints();
  G.removeUselessBodies();
  makeConvexHulls(G.shapes);

  G.watch(false);  G.gl().resize(800,800);
  G.watch(true);
  arr q;
  G.getJointState(q);
  cout << q << endl;

  MotionProblem MP(G,false);

  cout << "joint dimensionality=" <<q.N <<endl;

  Task *t;
  t = MP.addTask("transitions", new TransitionTaskMap(G));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e-1);

  double contactT = MP.T/2.;
  t = MP.addTask("prePos", new DefaultTaskMap(posTMT, G, "endeffL", NoVector, "button",ors::Vector(0.,0.,0.2)));
  t->setCostSpecs(contactT-10., contactT-10, {0.}, 1e2);
  t = MP.addTask("preVec", new DefaultTaskMap(vecAlignTMT, G, "endeffL", ors::Vector(1.,0.,0.), "button",ors::Vector(0.,0.,-1.)));
  t->setCostSpecs(contactT-10., contactT-10, {1.}, 1e2);
  t = MP.addTask("button_joint", new TaskMap_qItself(G.getJointByName("stand_button")->qIndex, G.getJointStateDimension()));
  t->setCostSpecs(MP.T, MP.T, {-.1}, 1e2);

  t = MP.addTask("endeff_button", new PointEqualityConstraint(G,"endeffL",NoVector,"cp1",NoVector));
  t->setCostSpecs(contactT+1,MP.T, {0.}, 1.);
  t = MP.addTask("button_fixation", new qItselfConstraint(G.getJointByName("stand_button")->qIndex, G.getJointStateDimension()));
  t->setCostSpecs(0.,contactT, {0.}, 1.);
  t = MP.addTask("qLimits", new LimitsConstraint());
  t->setCostSpecs(0., MP.T, {0.}, 1.);

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr X = MP.getInitialization();
  arr lambda;

  optConstrainedMix(X, lambda, Convert(MF), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2));
  cout << lambda << endl;
  cout << lambda.d0 << endl;
  cout << X.d0 << endl;

//  X.delColumns(G.getJointByName("stand_button")->qIndex);
//  write(LIST<arr>(X),STRING("X.dat"));

  MP.costReport();
  for (;;) {
    displayTrajectory(X, 1, G, "planned trajectory");
    displayTrajectory(X, 1, G, "planned trajectory");
  }
}

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);
  testButton();
  return 0;
}
