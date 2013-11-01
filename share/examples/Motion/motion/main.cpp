#include <Core/util.h>
#include <Motion/motion.h>
#include <Motion/taskMap_proxy.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>

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
                          ARRAY(0.,-1.,0.), 1e-0, //final desired velocity: v_y=-1 (hit ball from behind)
                          ARRAY(0.,0.,0.), 0.);

//  c = P.addDefaultTaskMap("collision", collTMT, 0, Transformation_Id, 0, Transformation_Id, ARR(.1));
//  P.setInterpolatingCosts(c, MotionProblem::constFinalMid, ARRAY(0.), 1e-0);

//  c = P.addDefaultTaskMap("qitself", qItselfTMT, (int)0, Transformation_Id, 0, Transformation_Id, 0);
//  P.setInterpolatingCosts(   c, MotionProblem::constFinalMid, ARRAY(0.), 1e-4);
//  //P.setInterpolatingVelCosts(c, MotionProblem::constFinalMid, ARRAY(0.), 1e4, ARRAY(0.), 1e-2);

  //-- collisions with other objects
  uintA shapes = ARRAY<uint>(P.ors->getBodyByName("endeff")->shapes(0)->index);
  c = P.addCustomTaskMap("proxyColls", new ProxyTaskMap(allVersusListedPTMT, shapes, .1, true));
  P.setInterpolatingCosts(c, MotionProblem::constFinalMid, ARRAY(0.), 1e-0);

  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction F(P);
  uint T=F.get_T();
  uint k=F.get_k();
  uint n=F.dim_x();
  cout <<"Problem parameters:"
       <<"\n T=" <<T
       <<"\n k=" <<k
       <<"\n n=" <<n
       <<endl;

  //mini evaluation test:
  arr x(T+1,n);
  x.setZero();
  cout <<"fx = " <<evaluateVF(Convert(F), x) <<endl;

  //gradient check
  for(uint k=0;k<0;k++){
    rndUniform(x,-1.,1.);
    checkJacobian(Convert(F), x, 1e-5);
  }
  
  OpenGL costs(STRING("PHI ("<<F.dim_phi(0)<<" tasks)"), 3*T+10, 3*F.dim_phi(0)+10 );
  //-- optimize
  for(uint k=0;k<2;k++){
    optNewton(x, Convert(F), OPT(verbose=2, stopIters=40, useAdaptiveDamping=false, damping=1e-0, maxStep=1.));
    costs.displayRedBlue(~sqr(P.costMatrix), false, 3);
    P.costReport();
    write(LIST<arr>(x),"z.output");
    gnuplot("plot 'z.output' us 1,'z.output' us 2,'z.output' us 3", false, true);
    displayTrajectory(x, 1, G, gl,"planned trajectory");
  }
  
  return 0;
}


