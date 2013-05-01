#include <MT/util.h>
#include <MT/MotionProblem.h>
#include <MT/opengl.h>

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

  MotionProblem P;
  P.loadTransitionParameters();


  //-- setup the control variables (problem definition)
  TaskCost *c;
  c = P.addDefaultTaskMap("position", posTMT,"endeff","<t(0 0 .2)>",0,0,ARR());
  P.setInterpolatingCosts(c, MotionProblem::constFinalMid,
                          ARRAY(P.ors->getBodyByName("target")->X.pos), 1e3,
                          ARRAY(0.,0.,0.), 1e-3);
  P.setInterpolatingVelCosts(c, MotionProblem::constFinalMid,
                          ARRAY(0.,0.,0.), 1e-3,
                          ARRAY(0.,0.,0.), 0.);

  c = P.addDefaultTaskMap("collision", collTMT, 0, Transformation_Id, 0, 0, ARR(.15));
  P.setInterpolatingCosts(c, MotionProblem::constFinalMid, ARRAY(0.), 1e-2);

  c = P.addDefaultTaskMap("qitself", qItselfTMT, (int)0, Transformation_Id, 0, 0, 0);
  P.setInterpolatingCosts(   c, MotionProblem::constFinalMid, ARRAY(0.), 1e-4);
  //P.setInterpolatingVelCosts(c, MotionProblem::constFinalMid, ARRAY(0.), 1e4, ARRAY(0.), 1e-2);

  //-- planning (AICO) to generate an optimal (kinematic) trajectory
  P.setState(P.x0);
  
  MotionProblemFunction F(P);

  //-- print some info on the problem
  uint T=F.get_T();
  uint k=F.get_k();
  uint n=F.get_n();
  cout <<"Problem parameters:"
       <<"\n T=" <<T
       <<"\n k=" <<k
       <<"\n n=" <<n
       <<endl;

  arr x(T+1,n);
  x.setZero();
  cout <<"fx = " <<evaluateVF(Convert(F), x) <<endl;

  //-- gradient check
  //arr x(T+1,n);
  for(uint k=0;k<0;k++){
    rndUniform(x,-1.,1.);
    checkJacobian(Convert(F), x, 1e-5);
  }
  
#if 0
  //-- print some example output
  arr phi,J;
  P.fv(phi, J, x);
  cout <<"x=" <<x <<"\nphi=" <<phi <<"\nJ=" <<J <<endl;
#endif
  
#if 0
  //-- test cost on a simple deterministic trajectory
  for(uint t=0;t<x.d0;t++){ x(t,0) = double(t)/T; x(t,1)=1.; }
  for(uint t=0;t<x.d0;t++){ double tt=double(t)/T;  x(t,1) = 2.*tt; x(t,0) = tt*tt; }
  analyzeTrajectory(sys, x, true, &cout);
  //return 0;
#endif

  OpenGL costs(STRING("PHI ("<<F.get_m(0)<<" tasks)"), 3*T+10, 3*F.get_m(0)+10 );
  //-- optimize
  //rndUniform(x,-10.,-1.);
  for(;;){
    optGaussNewton(x, Convert(F), OPT4(verbose=2, stopIters=20, useAdaptiveDamping=.0, maxStep=1.));
    costs.displayRedBlue(~sqr(F.costMatrix), false, 3);
    write(LIST<arr>(x),"z.output");
    gnuplot("plot 'z.output' us 1,'z.output' us 2,'z.output' us 3", false, true);
    displayTrajectory(x, 1, *P.ors, *P.gl,"planned trajectory");
  }
  
  return 0;
}


