#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Perception/videoEncoder.h>
#include <iomanip>
#include <Ors/ors_swift.h>

//===========================================================================

void TEST(Door1){
  ors::KinematicWorld G("door1.ors");

  arr q;
  G.getJointState(q);
//  q.setZero();
//  G.setJointState(q+0.5);
  cout << "q: " << q << endl;
  G.watch(false);
  MotionProblem MP(G);


  //-- setup the motion problem
  Task *t;
  t = MP.addTask("transitions", new TransitionTaskMap(G));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

  t = MP.addTask("position", new DefaultTaskMap(posTMT, G, "endeff", NoVector, NULL, G.getShapeByName("target")->X.pos));
  t->setCostSpecs(MP.T/2-5, MP.T/2, {0.}, 1e3);

  t = MP.addTask("door_joint", new TaskMap_qItself(G.getJointByName("door_joint")->qIndex, G.joints.N));
  t->setCostSpecs(MP.T, MP.T, {-.4}, 1e2);

  t = MP.addTask("door_fix", new TaskMap_qItself(G.getJointByName("door_joint")->qIndex, G.joints.N));
  t->setCostSpecs(0, MP.T/2, {0.}, 1e2);

  t = MP.addTask("contact", new PointEqualityConstraint(G, "endeff",NoVector, "target",NoVector));
  t->setCostSpecs(MP.T/2., MP.T, {0.}, 1.);

//  t = MP.addTask("direction", new VelAlignConstraint(G, "endeff",NoVector, "door", ors::Vector(-1.,0.,0.),0.7));
//  t->setCostSpecs(MP.T/2., MP.T, {0.}, 1.);

  t = MP.addTask("collision", new CollisionConstraint(0.05));
  t->setCostSpecs(0., MP.T, {0.}, 1.);

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr x = MP.getInitialization();
  arr lambda = zeros(x.d0,2);

  optConstrainedMix(x, NoArr, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=.5, stepInc=2., aulaMuInc=1.5));
//  optConstrained(x, lambda, Convert(MF));
//  checkGradient(Convert(MF),x,1e-3);

//  cout << lambda << endl;
  MP.costReport();
  displayTrajectory(x, 1, G, "planned trajectory");


}


void TEST(Door2){
  ors::KinematicWorld G("door2.ors");

  arr q;
  G.getJointState(q);
//  q.setZero();
//  G.setJointState(q);
  G.getJointState(q);
  cout << "q: " << q << endl;
  G.watch(true);
  MotionProblem MP(G);


  //-- setup the motion problem
  Task *t;
  t = MP.addTask("transitions", new TransitionTaskMap(G));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

  double contactT = MP.T/2.;

  // position task maps
//  t = MP.addTask("position", new DefaultTaskMap(posTMT, G, "endeff", NoVector, NULL, G.getShapeByName("target")->X.pos));
//  t->setCostSpecs(contactT-3., contactT, {0.}, 1e2);
//  t = MP.addTask("handle_joint", new TaskMap_qItself(G.getJointByName("door_handle")->qIndex, G.joints.N));
//  t->setCostSpecs(contactT+5., contactT+10., {.3}, 1e2);
//  t = MP.addTask("door_joint", new TaskMap_qItself(G.getJointByName("frame_door")->qIndex, G.joints.N));
//  t->setCostSpecs(MP.T, MP.T, {.4}, 1e2);

//  // velocity task maps


//  // constraints
//  t = MP.addTask("contact", new PointEqualityConstraint(G, "endeff",NoVector, "target",NoVector));
//  t->setCostSpecs(contactT, MP.T, {0.}, 1.);

////  t = MP.addTask("direction1", new VelAlignConstraint(G, "endeff",NoVector, "handle", ors::Vector(0.,0.,-1.),0.7));
////  t->setCostSpecs(MP.T/2.+5., MP.T/2.+10., {0.}, 1.);

//  t = MP.addTask("direction2", new VelAlignConstraint(G, "endeff",NoVector, "door", ors::Vector(1.,0.,0.),0.7));
//  t->setCostSpecs(MP.T/2.+15., MP.T, {0.}, 1.);

//  t = MP.addTask("door_fixation", new qItselfConstraint(G.getJointByName("frame_door")->qIndex, G.joints.N));
//  t->setCostSpecs(0.,contactT, {0.}, 1.);
//  t->map.order=1;

//  t = MP.addTask("handle_fixation", new qItselfConstraint(G.getJointByName("door_handle")->qIndex, G.joints.N));
//  t->setCostSpecs(0.,contactT, {0.}, 1.);
//  t->map.order=1;

//  t = MP.addTask("collision", new CollisionConstraint(0.05));
//  t->setCostSpecs(0., MP.T, {0.}, 1.);

//  t = MP.addTask("qLimits", new LimitsConstraint());
//  t->setCostSpecs(0., MP.T, {0.}, 1.);

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr x = MP.getInitialization();
  rndGauss(x,.1,true);
  arr lambda = zeros(x.d0,2);

  optConstrained(x, lambda, Convert(MF));
  checkGradient(Convert(MF),x,1e-3);

//  cout << lambda << endl;
  MP.costReport();
  for(;;) {displayTrajectory(x, 1, G, "planned trajectory");}
}

//===========================================================================

void TEST(Door3){
  ors::KinematicWorld G("door3.ors");
  G.meldFixedJoints();
  G.removeUselessBodies();
  makeConvexHulls(G.shapes);
  FILE("z.ors") <<G;


  arr q;
  G.getJointState(q);
  G.watch(false);
  MotionProblem MP(G);
  cout <<"joint dimensionality=" <<q.N <<endl;
//  MP.useSwift=false;

//  arr y,lim=G.getLimits();
//  G.kinematicsLimitsCost(y, NoArr, lim);
//  cout <<catCol(q,lim) <<endl <<y <<endl;

  //-- setup the motion problem
  Task *t;
  t = MP.addTask("transitions", new TransitionTaskMap(G));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e-1);

  double contactT = MP.T/2.;
  // position task maps
  t = MP.addTask("position", new DefaultTaskMap(posTMT, G, "endeffL", NoVector, "target",NoVector));
  t->setCostSpecs(contactT-10., contactT, {0.}, 1e2);

  t = MP.addTask("handle_joint", new TaskMap_qItself(G.getJointByName("door_handle")->qIndex, G.getJointStateDimension()));
  t->setCostSpecs(contactT+10., contactT+10., {.3}, 1e3);

  t = MP.addTask("door_joint", new TaskMap_qItself(G.getJointByName("frame_door")->qIndex, G.getJointStateDimension()));
  t->setCostSpecs(MP.T, MP.T, {-.7}, 1e2);

  // constraints
  t = MP.addTask("contact", new PointEqualityConstraint(G, "endeffL",NoVector, "target",NoVector));
  t->setCostSpecs(contactT, MP.T, {0.}, 1.);

//  t = MP.addTask("direction2", new VelAlignConstraint(G, "endeffL",NoVector, "door", ors::Vector(-1.,0.,0.),0.4));
//  t->setCostSpecs(contactT+0., MP.T, {0.}, 1.);

  t = MP.addTask("door_fixation", new qItselfConstraint(G.getJointByName("frame_door")->qIndex, G.getJointStateDimension()));
  t->setCostSpecs(0.,contactT+10, {0.}, 1.);

  t = MP.addTask("handle_fixation", new qItselfConstraint(G.getJointByName("door_handle")->qIndex, G.getJointStateDimension()));
  t->setCostSpecs(0.,contactT, {0.}, 1.);

  //  t = MP.addTask("direction1", new VelAlignConstraint(G, "endeffL",NoVector, "handle", ors::Vector(0.,0.,-1.),0.7));
  //  t->setCostSpecs(MP.T/2.+5., MP.T/2.+10., {0.}, 1.);

//  t = MP.addTask("collision", new CollisionConstraint(0.05));
  ShapeL except = G.getBodyByName("l_wrist_roll_link")->shapes;
  t = MP.addTask("collision", new ProxyConstraint(allExceptListedPTMT, shapesToShapeIndices(except), 0.05));
  t->setCostSpecs(0., MP.T, {0.}, 1.);

  t = MP.addTask("qLimits", new LimitsConstraint());
  t->setCostSpecs(0., MP.T, {0.}, 1.);

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr x = MP.getInitialization();
  arr lambda = zeros(x.d0,2);

  optConstrainedMix(x, NoArr, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2));
//  optConstrained(x, lambda, Convert(MF),OPT(constrainedMethod = ConstrainedMethodType::augmentedLag,stopTolerance = 1e-1));
//  optConstrained(x, lambda, Convert(MF),OPT(constrainedMethod = ConstrainedMethodType::augmentedLag,stopTolerance = 1e-3));
//  checkGradient(Convert(MF),x,1e-3);

//  cout << lambda << endl;
  MP.costReport();
  for(;;) {displayTrajectory(x, 1, G, "planned trajectory");}
}


int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);
//  testDoor1();
//  testDoor2();
  testDoor3();

  return 0;
}


