#include <Core/util.h>
#include <MT/soc_orsSystem.h>
#include <Gui/opengl.h>

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  OpenGL gl;
  
  double D=MT::getParameter<double>("time_duration",4.);
  uint _T=MT::getParameter<uint>("time_steps",200);
  OrsSystem sys;
  sys.initBasics(NULL, NULL, &gl, _T, D, MT::getParameter<bool>("dynamic",false), NULL);
  sys.os=&std::cout;
 
  //-- setup the control variables (problem definition)
  TaskVariable *pos = new DefaultTaskVariable("position", sys.getOrs(), posTVT,"endeff","<t(0 0 .2)>",0,0,ARR());
  pos->y_target = ARRAY(sys.getOrs().getBodyByName("target")->X.pos);
  
  TaskVariable *col = new DefaultTaskVariable("collision", sys.getOrs(), collTVT,0,0,0,0,ARR(.15));
  col->y_prec=1e-0;
  col->y_target = ARR(0.);

  TaskVariable *qtv = new DefaultTaskVariable("qitself", sys.getOrs(), qItselfTVT, 0, 0, 0, 0, 0);
  qtv->y_target.setZero();

  sys.setTaskVariables(ARRAY(pos,col,qtv));

  //-- planning (AICO) to generate an optimal (kinematic) trajectory
  sys.setTox0();
  pos->setInterpolatedTargetsEndPrecisions(_T, 1e-3, 1e3, 0., 1e-3);
  col->setInterpolatedTargetsConstPrecisions(_T, 1e-2, 0.);
  qtv->setInterpolatedTargetsEndPrecisions(_T, 0., 0., 1e-2, 1e4);
  
  // cost checks
//  arr q,x,xx;
//  MT::load(q, "z.q");
//  getPhaseTrajectory(xx, q, sys.get_tau());
//  analyzeTrajectory(sys, xx, true, &cout);
  
  //-- print some info on the problem
  KOrderMarkovFunction& kom = Convert(sys);
  uint T=kom.get_T();
  uint k=kom.get_k();
  uint n=kom.get_n();
  cout <<"Problem parameters:"
       <<"\n T=" <<T
       <<"\n k=" <<k
       <<"\n n=" <<n
       <<endl;

  arr x(T+1,n);
  x.setZero();
  cout <<"fx = " <<evaluateVF(Convert(sys), x) <<endl;

  //-- gradient check
  //arr x(T+1,n);
  for(uint k=0;k<0;k++){
    rndUniform(x,-1.,1.);
    checkJacobian(Convert(sys), x, 1e-5);
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

  //-- optimize
  //rndUniform(x,-10.,-1.);
  for(;;){
    optNewton(x, Convert(sys), OPT4(verbose=2, stopIters=20, useAdaptiveDamping=.0, maxStep=1.));
    arr xx;
    if(sys.get_xDim()>x.d1) getPhaseTrajectory(xx, x, sys.get_tau()); else xx=x;
    analyzeTrajectory(sys, xx, true, &cout);
    write(LIST<arr>(x),"z.output");
    gnuplot("set term x11 1; plot 'z.output' us 1,'z.output' us 2,'z.output' us 3", true, true);
    displayTrajectory(sys, xx, NULL, 1, "planned trajectory");
    MT::wait();
  }
  
  return 0;
}


