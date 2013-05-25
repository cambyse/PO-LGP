#include <Array/util.h>
#include <Motion/motion.h>
#include <Gui/opengl.h>

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

  OpenGL gl;
  ors::Graph G;
  init(G, gl, MT::getParameter<MT::String>("orsFile"));

  MotionProblem P(&G);
  P.loadTransitionParameters();

  //-- setup the motion problem
  TaskCost *c;
  c = P.addDefaultTaskMap("position", posTMT,"endeff","<t(0 0 .2)>",0,0,ARR());
  P.setInterpolatingCosts(c, MotionProblem::constFinalMid,
                          ARRAY(P.ors->getBodyByName("target")->X.pos), 1e3,
                          ARRAY(0.,0.,0.), 1e-3);
//  P.setInterpolatingVelCosts(c, MotionProblem::constFinalMid,
//                          ARRAY(0.,0.,0.), 1e3, //final desired velocity: v_y=-1 (hit ball from behind)
//                          ARRAY(0.,0.,0.), 0.);

  c = P.addDefaultTaskMap("collision", collTMT, 0, Transformation_Id, 0, Transformation_Id, ARR(.1));
  P.setInterpolatingCosts(c, MotionProblem::constFinalMid, ARRAY(0.), 1e-0);

  c = P.addDefaultTaskMap("qitself", qItselfTMT, (int)0, Transformation_Id, 0, Transformation_Id, 0);
  P.setInterpolatingCosts(   c, MotionProblem::constFinalMid, ARRAY(0.), 1e-4);
  //P.setInterpolatingVelCosts(c, MotionProblem::constFinalMid, ARRAY(0.), 1e4, ARRAY(0.), 1e-2);
  
  //-- create the Optimization problem (of type kOrderMarkov)
  MotionProblemFunction F(P);
  uint T=F.get_T();
  uint k=F.get_k();
  uint n=F.get_n();
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
  
  OpenGL costs(STRING("PHI ("<<F.get_m(0)<<" tasks)"), 3*T+10, 3*F.get_m(0)+10 );
  //-- optimize
  for(;;){
    optGaussNewton(x, Convert(F), OPT4(verbose=2, stopIters=20, useAdaptiveDamping=.0, maxStep=1.));
    costs.displayRedBlue(~sqr(P.costMatrix), false, 3);
    P.costReport();
    write(LIST<arr>(x),"z.output");
    gnuplot("plot 'z.output' us 1,'z.output' us 2,'z.output' us 3", false, true);
    displayTrajectory(x, 1, G, gl,"planned trajectory");
  }
  
  return 0;
}


