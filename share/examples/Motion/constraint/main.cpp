#include <Core/util.h>
#include <Motion/motion.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Optim/optimization_benchmarks.h>

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

  OpenGL gl;
  ors::Graph G;
  init(G, gl, MT::getParameter<MT::String>("orsFile"));

  MotionProblem P(&G);
  P.loadTransitionParameters();

  //-- setup the motion problem
  TaskCost *c;
  c = P.addDefaultTaskMap_Bodies("position", posTMT,"endeff",ors::Transformation().setText("<t(0 0 .2)>"));
  P.setInterpolatingCosts(c, MotionProblem::constFinalMid,
                          ARRAY(P.ors->getBodyByName("target")->X.pos), 1e3,
                          ARRAY(0.,0.,0.), 1e-3);
  P.setInterpolatingVelCosts(c, MotionProblem::constFinalMid,
                             ARRAY(0.,-1.,0.), 1e3, //final desired velocity: v_y=-1 (hit ball from behind)
                             ARRAY(0.,0.,0.), 0.);

  c = P.addDefaultTaskMap("collision", collTMT, 0, Transformation_Id, 0, Transformation_Id, ARR(.1));
  P.setInterpolatingCosts(c, MotionProblem::constFinalMid, ARRAY(0.), 1e-0);

  
  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction F(P);

  arr x(F.get_T()+1,F.get_n());
  x.setZero();

  optGaussNewton(x, Convert(F), OPT(verbose=2, stopIters=100, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));

  P.costReport();
  write(LIST<arr>(x),"z.output");
  gnuplot("plot 'z.output' us 1,'z.output' us 2,'z.output' us 3", false, true);
  displayTrajectory(x, 1, G, gl,"planned trajectory");
  
  return 0;
}


