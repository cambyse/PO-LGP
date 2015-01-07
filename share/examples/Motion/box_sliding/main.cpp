#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Perception/videoEncoder.h>
#include <iomanip>
#include <Ors/ors_swift.h>


//===========================================================================

void TEST(Box1){
  ors::KinematicWorld G("box.ors");

//  G.meldFixedJoints();
//  G.removeUselessBodies();
  makeConvexHulls(G.shapes);

//  cout <<G.getBodyByName("endeffM")->X.pos << endl;
//  cout <<G.getBodyByName("endeffL")->X.pos << endl;
//  arr dist = ARRAY(G.getBodyByName("endeffM")->X.pos - G.getBodyByName("endeffL")->X.pos);
//  cout << sqrt((sum(dist%dist)))/2 << endl;
//  for (;;){
//    ors::KinematicWorld G("box.ors");
//    G.watch(true);
//  }
  arr q;
  G.getJointState(q);
//  G.setJointState(q*0.);

  G.watch(true);

  MotionProblem MP(G);
  cout <<"joint dimensionality=" <<q.N <<endl;

  //-- setup the motion problem
  Task *t;
  t = MP.addTask("transitions", new TransitionTaskMap(G));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e-1);

  double conT = MP.T/2.;
  // position task maps
  t = MP.addTask("position", new DefaultTaskMap(posTMT, G, "box", NoVector, "boxTarget",NoVector));
  t->setCostSpecs(MP.T-2,MP.T-2, {0.}, 1e3);
  t = MP.addTask("orientation", new DefaultTaskMap(vecAlignTMT, G, "box", ors::Vector(0.,1.,0), "boxTarget",ors::Vector(0.,1.,0)));
  t->setCostSpecs(MP.T-2,MP.T-2, {1.}, 5e3);

  t = MP.addTask("velT", new TaskMap_qItself());
  t->map.order=1;
  t->setCostSpecs(MP.T,MP.T, zeros(G.getJointStateDimension()), 1e1);
  t = MP.addTask("velC", new TaskMap_qItself());
  t->map.order=1;
  t->setCostSpecs(conT,conT, zeros(G.getJointStateDimension()), 1e1);

  // constraints
  t = MP.addTask("contact1", new PointEqualityConstraint(G, "endeffL",NoVector, "boxP1",NoVector));
  t->setCostSpecs(conT, MP.T, {0.}, 1.);
  t = MP.addTask("contact2", new PointEqualityConstraint(G, "endeffM",NoVector, "boxP2",NoVector));
  t->setCostSpecs(conT, MP.T, {0.}, 1.);

  t = MP.addTask("box_fixation1", new qItselfConstraint(G.getJointByName("table_box")->qIndex, G.getJointStateDimension()));
  t->setCostSpecs(0.,conT, ARR(0.), 1.);
  t = MP.addTask("box_fixation2", new qItselfConstraint(G.getJointByName("table_box")->qIndex+1, G.getJointStateDimension()));
  t->setCostSpecs(0.,conT, ARR(0.), 1.);
  t = MP.addTask("box_fixation2", new qItselfConstraint(G.getJointByName("table_box")->qIndex+2, G.getJointStateDimension()));
  t->setCostSpecs(0.,conT, ARR(0.), 1.);

  t = MP.addTask("velocity_dir", new VelAlignConstraint(G, "endeffC",NoVector, "box", ors::Vector(1,0,0),.8));
  t->setCostSpecs(conT, MP.T, {0.}, 1.);

  t = MP.addTask("collision", new ProxyConstraint(allPTMT,{} , 0.01));
  t->setCostSpecs(0., MP.T, {0.}, 1.);

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr x = MP.getInitialization();

  optConstrainedMix(x, NoArr, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-3));
//  checkGradient(Convert(MF),x,1e-3);

  MP.costReport();
  for(;;) {displayTrajectory(x, 1, G, "planned trajectory");}
}


int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);
  testBox1();

  return 0;
}


