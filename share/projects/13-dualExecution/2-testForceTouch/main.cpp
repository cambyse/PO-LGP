#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_constrained.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Optim/constrained.h>


arr getSimpleTrajectory(ors::KinematicWorld& G){
  MotionProblem P(&G, NULL, false);
  P.loadTransitionParameters();
  arr x = P.getInitialization();

  //-- setup the motion problem
  TaskCost *c;
  c = P.addTaskMap("position",
                   new DefaultTaskMap(posTMT, G, "endeff", NoVector));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly,
                          ARRAY(P.ors->getShapeByName("target")->X.pos), 1e2);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e1);

  //c = P.addTaskMap("collisionConstraints", new CollisionConstraint());
  c = P.addTaskMap("planeConstraint", new PlaneConstraint(G, "endeff", ARR(0,0,-1,.7)));

  MotionProblemFunction MF(P);
  Convert ConstrainedP(MF);
  UnconstrainedProblem UnConstrainedP(ConstrainedP);
  UnConstrainedP.mu = 10.;

  for(uint k=0;k<5;k++){
    optNewton(x, UnConstrainedP, OPT(verbose=2, stopIters=100, useAdaptiveDamping=false, damping=1e-3, stopTolerance=1e-4, maxStep=.5));
//    optNewton(x, UCP, OPT(verbose=2, stopIters=100, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));
    P.costReport();
//    displayTrajectory(x, 1, G, gl,"planned trajectory");
    UnConstrainedP.augmentedLagrangian_LambdaUpdate(x, .9);
    P.dualMatrix = UnConstrainedP.lambda;
    UnConstrainedP.mu *= 2.;
  }
  P.costReport();
  return x;
}


int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  OpenGL gl;
  ors::KinematicWorld G;
  init(G, gl, MT::getParameter<MT::String>("orsFile"));

  arr x = getSimpleTrajectory(G);
  arr x2 = reverseTrajectory(x);
  x.append(x2);

  for(uint i=0;i<3;i++)
    displayTrajectory(x, 1, G, gl,"planned trajectory");

  return 0;
}


