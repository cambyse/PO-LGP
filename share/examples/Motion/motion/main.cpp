#include <Core/util.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <Ors/ors_swift.h>

//===========================================================================

void TEST(PR2reach){
  ors::KinematicWorld G(MT::getParameter<MT::String>("orsFile"));
  makeConvexHulls(G.shapes);
  for(ors::Shape *s:G.shapes) s->cont=true;
  G.getShapeByName("target")->cont=false;
  cout <<"loaded model: n=" <<G.q.N <<endl;

  MotionProblem MP(G);
  MP.loadTransitionParameters();

  //-- setup the motion problem
  TaskCost *c;
  c = MP.addTask("endeff_pos", new DefaultTaskMap(posTMT, G, "endeff"));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(MP.world.getShapeByName("target")->X.pos), 1e3);

  c = MP.addTask("endeff_vel", new DefaultTaskMap(posTMT, G, "endeff"));
//  c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, G));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, NoArr, 1e1);
  c->map.order=1; //make this a velocity variable!

//#define CONSTRAINT
#ifndef CONSTRAINT
  c = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0}, .1));
#else
  c = MP.addTask("collisionConstraints", new CollisionConstraint(.1));
#endif
  MP.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1e-0);

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr x = replicate(MP.x0, MP.T+1);

  //-- optimize
  for(uint k=0;k<5;k++){
    MT::timerStart();
    ors::KinematicWorld::setJointStateCount=0;
#ifndef CONSTRAINT
    optNewton(x, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=.5, stepInc=2., nonStrictSteps=(!k?15:5)));
#else
    optConstrained(x, NoArr, Convert(MF), OPT(verbose=1, stopIters=100, damping=1., maxStep=1., nonStrictSteps=5));
#endif

    cout <<"** optimization time=" <<MT::timerRead()
        <<" setJointStateCount=" <<ors::KinematicWorld::setJointStateCount <<endl;
    MP.costReport();
    write(LIST<arr>(x),"z.output");
    gnuplot("load 'z.costReport.plt'", false, true);
    displayTrajectory(x, 1, G, "planned trajectory", 0.01);
  }
}

//===========================================================================

void TEST(Basics){
  ors::KinematicWorld G("test.ors");
  G.getShapeByName("target")->cont=false;

  MotionProblem MP(G);
  MP.loadTransitionParameters();

  //-- setup the motion problem
  TaskCost *c;
  c = MP.addTask("position", new DefaultTaskMap(posTMT, G, "endeff", ors::Vector(0, 0, 0)));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(MP.world.getShapeByName("target")->X.pos), 1e3);

  c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, G));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, NoArr, 1e1);
  c->map.order=1; //make this a velocity variable!

//#define CONSTRAINT
#ifndef CONSTRAINT
  c = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0}, .1));
#else
  c = MP.addTask("collisionConstraints", new CollisionConstraint(.1));
#endif
  MP.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1e-0);

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction MF(MP);
  arr x = replicate(MP.x0, MP.T+1);
  rndGauss(x,.01,true); //don't initialize at a singular config

  //gradient check: will fail in case of collisions
  for(uint k=0;k<0;k++){
    checkJacobian(Convert(MF), x, 1e-4);
    rndUniform(x,-1.,1.);
  }

  //initialize trajectory
  for(uint t=0;t<=MP.T;t++) x[t]() = MP.x0;

  //-- optimize
  for(uint k=0;k<5;k++){
    MT::timerStart();
#ifndef CONSTRAINT
    optNewton(x, Convert(MF), OPT(verbose=2, stopIters=20, damping=.1));
#else
    optConstrained(x, NoArr, Convert(MF), OPT(verbose=1, stopIters=100, damping=1., maxStep=1., nonStrictSteps=5));
#endif
    cout <<"** optimization time=" <<MT::timerRead() <<endl;
    MP.costReport();
//    checkJacobian(Convert(MF), x, 1e-5);
    write(LIST<arr>(x),"z.output");
    gnuplot("plot 'z.output' us 1,'z.output' us 2,'z.output' us 3", false, true);
    gnuplot("load 'z.costReport.plt'", false, true);
    displayTrajectory(x, 1, G, "planned trajectory", 0.01);
  }
}

//===========================================================================

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  testPR2reach();
//  testBasics();
  
  return 0;
}


