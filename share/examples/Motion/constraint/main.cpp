#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Perception/videoEncoder.h>
#include <iomanip>
#include <Kin/kin_swift.h>


void saveTrajectory(const arr& x, mlr::KinematicWorld& G, OpenGL& gl) {
  VideoEncoder_libav_simple vid;
  for(uint t=0; t<x.d0; t++) {
    G.setJointState(x[t]);
    gl.update(STRING("step " <<std::setw(3) <<t <<'/' <<x.d0-1).p, true, false);
    flip_image(gl.captureImage);
    vid.addFrame(gl.captureImage);
    //    write_ppm(gl.captureImage, STRING("vid/t"<<t<<".ppm"));
  }
  vid.close();
}

void TEST(Stickiness){
  mlr::KinematicWorld G(mlr::getParameter<mlr::String>("orsFile"));

  bool hardConstraint=true;

  KOMO MP(G);

  //-- setup the motion problem
  Task *t;

  t = MP.addTask("transitions", new TaskMap_Transition(G));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

  t = MP.addTask("final_vel", new TaskMap_Transition(G));
  t->map.order=1; //make this an acceleration task!
  t->setCostSpecs(MP.T-4, MP.T, {0.}, 1e2);

  t = MP.addTask("position", new TaskMap_Default(posTMT, G, "endeff", NoVector, NULL, G.getBodyByName("target")->X.pos));
  t->setCostSpecs(MP.T, MP.T, {0.}, 1e3);

  if(hardConstraint){
    t = MP.addTask("collisionConstraints", new CollisionConstraint());
    t->setCostSpecs(0, MP.T, {0.}, 1.);

    Task *sticky = MP.addTask("collisionStickiness", new ConstraintStickiness(t->map));
    sticky->setCostSpecs(0, MP.T, {0.}, 1.e1);
  }else{
    t = MP.addTask("collision", new TaskMap_Proxy(allPTMT, {}, {.1}));
    t->setCostSpecs(0, MP.T, {0.}, 1.);
  }


  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr x(MF.get_T()+1,MF.dim_x());
  x.setZero();

  Convert CP(MF);
  for(uint k=0;k<1;k++){
    optConstrained(x, MP.dualMatrix, CP);
    MP.costReport();
    for(uint i=0;i<1;i++) displayTrajectory(x, 1, G, "planned trajectory");
  }
}

//===========================================================================

void TEST(EqualityConstraints){
  mlr::KinematicWorld G("chain.ors");
  KOMO MP(G);

  //-- setup the motion problem
  Task *t;
  t = MP.addTask("transitions", new TaskMap_Transition(G));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

  t = MP.addTask("final_vel", new TaskMap_Transition(G));
  t->map.order=1; //make this a velocity task!
  t->setCostSpecs(MP.T-4, MP.T, {0.}, 1e2);

#if 1
  t = MP.addTask("position", new TaskMap_Default(posTMT, G, "endeff", NoVector, NULL, G.getBodyByName("target")->X.pos));
  t->setCostSpecs(MP.T, MP.T, {0.}, 1e3);
#else
  t = MP.addTask("position", new PointEqualityConstraint(G, "endeff", NoVector, NULL, G.getBodyByName("target")->X.pos));
  t->setCostSpecs(MP.T, MP.T, {0.}, 1e1);
#endif

  t = MP.addTask("ballEqCon", new PointEqualityConstraint(G, "point", NoVector, NULL, G.getShapeByName("point")->X.pos));
  t->setCostSpecs(0, MP.T, {0.}, 1.);

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr x = MP.getInitialization();
  cout << x << endl;
  optConstrained(x, MP.dualMatrix, Convert(MF));
  MP.costReport();
  displayTrajectory(x, 1, G, "planned trajectory");
}


//===========================================================================

void TEST(ClosedKinematicChain){
  mlr::KinematicWorld G("closed_chain.ors");
  KOMO MP(G);

  arr q;
  G.getJointState(q);
  q.setZero();
  G.setJointState(q);
  cout << "q: " << q << endl;
  G.watch(true);

  //-- setup the motion problem
  Task *t;
  t = MP.addTask("transitions", new TaskMap_Transition(G));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

  t = MP.addTask("final_vel", new TaskMap_Transition(G));
  t->map.order=1; //make this a velocity task!
  t->setCostSpecs(MP.T-4, MP.T, {0.}, 1e2);

  t = MP.addTask("ballEqCon", new PointEqualityConstraint(G, "endeff1", NoVector, "endeff2", NoVector));
  t->setCostSpecs(0, MP.T, {0.}, 1.);

  t = MP.addTask("position", new TaskMap_Default(posTMT, G, "endeff1", NoVector, NULL, G.getBodyByName("target")->X.pos));
  t->setCostSpecs(MP.T, MP.T, {0.}, 1e3);


  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr x = MP.getInitialization();
  optConstrained(x, MP.dualMatrix, Convert(MF),OPT(stopTolerance=1e-5));
  MP.costReport();
  displayTrajectory(x, 1, G, "planned trajectory");
}

//===========================================================================

void TEST(ContactConstraint){
  mlr::KinematicWorld G("table.ors");
  makeConvexHulls(G.shapes);
  G.swift().setCutoff(10.);
  G.swift();
  G.swift().step(G,true);
//  G.reportProxies();
//  cout << G.proxies.d0 << endl;
//  G.watch(true);


  KOMO MP(G,false);
  MP.useSwift = true;

  arr q;
  G.getJointState(q);
  q.setZero();
  G.setJointState(q+0.1);
  cout << "q: " << q << endl;
  G.watch(true);

  //-- setup the motion problem
  Task *t;
  t = MP.addTask("transitions", new TaskMap_Transition(G));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

//  t = MP.addTask("final_vel", new TaskMap_Transition(G));
//  t->map.order=1; //make this a velocity task!
//  t->setCostSpecs(MP.T-4, MP.T, {0.}, 1e2);

//  t = MP.addTask("position", new TaskMap_Default(posTMT, G, "endeff", NoVector, NULL, G.getBodyByName("target")->X.pos));
//  t->setCostSpecs(MP.T, MP.T, {0.}, 1e1);

  t = MP.addTask("collision", new PairCollisionConstraint(G, "table", "endeff", 0.01));
  t->setCostSpecs(0., MP.T, {0.}, 1.);

  t = MP.addTask("contact", new ContactEqualityConstraint(G, "table", "endeff", 0.02));
  t->setCostSpecs(MP.T/2-10., MP.T/2+10, {0.}, 1.);


  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr x = MP.getInitialization();
  arr lambda = zeros(x.d0,2);
  optConstrained(x, lambda, Convert(MF));
  cout << lambda << endl;
  MP.costReport();
  displayTrajectory(x, 1, G, "planned trajectory");
}


//===========================================================================

void TEST(VelConstraint){
  mlr::KinematicWorld G("table.ors");

  arr q;
  G.getJointState(q);
  q.setZero();
  G.setJointState(q+0.1);
  cout << "q: " << q << endl;
  G.watch(true);
  KOMO MP(G);


  //-- setup the motion problem
  Task *t;
  t = MP.addTask("transitions", new TaskMap_Transition(G));
  t->map.order=1; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e1);

//  t = MP.addTask("final_vel", new TaskMap_Transition(G));
//  t->map.order=1; //make this a velocity task!
//  t->setCostSpecs(MP.T-4, MP.T, {0.}, 1e2);

  t = MP.addTask("position", new TaskMap_Default(posTMT, G, "endeff", NoVector, NULL, G.getBodyByName("target")->X.pos));
  t->setCostSpecs(MP.T, MP.T, {0.}, 1e3);

//  t = MP.addTask("collision", new PairCollisionConstraint(G, "table", "endeff", 0.1));
//  t->setCostSpecs(0., MP.T, {0.}, 1.);

  t = MP.addTask("contact", new VelAlignConstraint(G, "endeff",NoVector, "table", mlr::Vector(0.,0.,1.)));
  t->setCostSpecs(0., MP.T, {0.}, 1.);


  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr x = MP.getInitialization();
  arr lambda = zeros(x.d0,2);


  optConstrained(x, lambda, Convert(MF));
  checkGradient(Convert(MF),x,1e-3);

//  cout << lambda << endl;
//  MP.costReport();
  displayTrajectory(x, 1, G, "planned trajectory");
}

void TEST(qItselfConstraint){
  mlr::KinematicWorld G("table.ors");

  arr q;
  G.getJointState(q);
  q.setZero();
  G.setJointState(q+0.1);
  G.watch(true);
  KOMO MP(G);


  //-- setup the motion problem
  Task *t;
  t = MP.addTask("transitions", new TaskMap_Transition(G));
  t->map.order=1; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.}, 1e1);

  t = MP.addTask("position", new TaskMap_Default(posTMT, G, "endeff", NoVector, NULL, G.getBodyByName("target")->X.pos));
  t->setCostSpecs(MP.T, MP.T, {0.}, 1e3);

  t = MP.addTask("qItself", new qItselfConstraint(3,G.getJointStateDimension()));
  t->setCostSpecs(0., MP.T/2., {0.}, 1.);
  t->map.order=1;


  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr x = MP.getInitialization();
  arr lambda = zeros(x.d0,2);


  optConstrained(x, lambda, Convert(MF));
  checkGradient(Convert(MF),x,1e-3);

//  cout << lambda << endl;
//  MP.costReport();
  displayTrajectory(x, 1, G, "planned trajectory");
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);
//  testStickiness();
  testEqualityConstraints();
//  testClosedKinematicChain();
//  testContactConstraint();
//  testVelConstraint();
//  testqItselfConstraint();
  return 0;
}


