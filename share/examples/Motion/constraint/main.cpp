#include <Core/util.h>
#include <Motion/motion.h>
#include <Motion/taskMap_constrained.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <Optim/constrained.h>

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

  OpenGL gl;
  ors::Graph G;
  init(G, gl, MT::getParameter<MT::String>("orsFile"));

  bool con=true;

  MotionProblem P(&G);
  P.loadTransitionParameters();

  //-- setup the motion problem
  TaskCost *c;
  c = P.addDefaultTaskMap_Bodies("position", posTMT, "endeff", ors::Transformation().setText("<t(0 0 .2)>"));
  P.setInterpolatingCosts(c, MotionProblem::constFinalMid,
                          ARRAY(P.ors->getBodyByName("target")->X.pos), 1e3,
                          ARRAY(0.,0.,0.), 1e-3);
  P.setInterpolatingVelCosts(c, MotionProblem::constFinalMid,
                             ARRAY(0.,-1.,0.), 1e3, //final desired velocity: v_y=-1 (hit ball from behind)
                             ARRAY(0.,0.,0.), 0.);

  if(con){
    c = P.addCustomTaskMap("collisionConstraints", new CollisionConstraint());
  }else{
    c = P.addDefaultTaskMap("collision", collTMT, 0, Transformation_Id, 0, Transformation_Id, ARR(.1));
    P.setInterpolatingCosts(c, MotionProblem::constFinalMid, ARRAY(0.), 1e-0);
  }

  
  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction F(P);
  Convert CP(F);
  UnconstrainedProblem UCP(CP);
  UCP.mu = 10.;

  arr x(F.get_T()+1,F.dim_x());
  x.setZero();

  if(con){
    for(uint k=0;k<10;k++){
      checkAll(CP, x, 1e-4);
      optNewton(x, UCP, OPT(verbose=2, stopIters=100, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));
      P.costReport();
      write(LIST<arr>(x),"z.output");
      gnuplot("plot 'z.output' us 1,'z.output' us 2,'z.output' us 3", false, true);
      displayTrajectory(x, 1, G, gl,"planned trajectory");

      //UCP.mu *= 10;
      UCP.augmentedLagrangian_LambdaUpdate(x, .9);
    }
  }else{
    optNewton(x, CP, OPT(verbose=2, stopIters=100, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));
  }

  P.costReport();
  write(LIST<arr>(x),"z.output");
  gnuplot("plot 'z.output' us 1,'z.output' us 2,'z.output' us 3", false, true);
  displayTrajectory(x, 1, G, gl,"planned trajectory");

  return 0;
}


