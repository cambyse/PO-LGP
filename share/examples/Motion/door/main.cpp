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
  G.getJointState(q);
  cout << "q: " << q << endl;
  G.watch(true);
  MotionProblem MP(G);


  //-- setup the motion problem
  Task *t;
  t = MP.addTask("transitions", new TransitionTaskMap(G));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

  t = MP.addTask("position", new DefaultTaskMap(posTMT, G, "endeff", NoVector, NULL, G.getShapeByName("target")->X.pos));
  t->setCostSpecs(MP.T/2-3., MP.T/2., {0.}, 1e3);

  t = MP.addTask("door_joint", new TaskMap_qItself(G.getJointByName("door_joint")->index, G.joints.N));
  t->setCostSpecs(MP.T, MP.T, {-.4}, 1e2);

  t = MP.addTask("contact", new PointEqualityConstraint(G, "endeff",NoVector, "target",NoVector));
  t->setCostSpecs(MP.T/2., MP.T, {0.}, 1.);

  t = MP.addTask("direction", new VelAlignConstraint(G, "endeff",NoVector, "door", ors::Vector(-1.,0.,0.),0.7));
  t->setCostSpecs(MP.T/2., MP.T, {0.}, 1.);

  t = MP.addTask("collision", new CollisionConstraint(0.05));
  t->setCostSpecs(0., MP.T, {0.}, 1.);

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr x = MP.getInitialization();
  arr lambda = zeros(x.d0,2);

  optConstrained(x, lambda, Convert(MF));
  checkGradient(Convert(MF),x,1e-3);

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

  t = MP.addTask("position", new DefaultTaskMap(posTMT, G, "endeff", NoVector, NULL, G.getShapeByName("target")->X.pos));
  t->setCostSpecs(MP.T/2-3., MP.T/2., {0.}, 1e2);

  t = MP.addTask("door_joint", new TaskMap_qItself(G.getJointByName("frame_door")->index, G.joints.N));
  t->setCostSpecs(MP.T, MP.T, {.4}, 1e2);

  t = MP.addTask("contact", new PointEqualityConstraint(G, "endeff",NoVector, "target",NoVector));
  t->setCostSpecs(MP.T/2., MP.T, {0.}, 1.);

  t = MP.addTask("handle_joint", new TaskMap_qItself(G.getJointByName("door_handle")->index, G.joints.N));
  t->setCostSpecs(MP.T/2.+5., MP.T/2.+10., {.3}, 1e1);

  t = MP.addTask("direction1", new VelAlignConstraint(G, "endeff",NoVector, "handle", ors::Vector(0.,0.,-1.),0.7));
  t->setCostSpecs(MP.T/2.+5., MP.T/2.+10., {0.}, 1.);

  t = MP.addTask("direction2", new VelAlignConstraint(G, "endeff",NoVector, "door", ors::Vector(1.,0.,0.),0.7));
  t->setCostSpecs(MP.T/2.+11., MP.T, {0.}, 1.);

//  t = MP.addTask("collision", new CollisionConstraint(0.05));
//  t->setCostSpecs(0., MP.T, {0.}, 1.);

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr x = MP.getInitialization();
  arr lambda = zeros(x.d0,2);

  optConstrained(x, lambda, Convert(MF));
  checkGradient(Convert(MF),x,1e-3);

//  cout << lambda << endl;
  MP.costReport();
  displayTrajectory(x, 1, G, "planned trajectory");
}

//===========================================================================

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);
//  testDoor1();
  testDoor2();

  return 0;
}


