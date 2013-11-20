#include <MT/soc.h>
#include <MT/util.h>
#include <MT/specialTaskVariables.h>
#include <MT/opengl.h>
#include <MT/aico.h>

//===========================================================================

void problem1(){
  cout <<"\n= problem 1: simple kinematic reaching with 1 TV =\n" <<endl;

  //setup the system
  soc::SocSystem_Ors sys;
  OpenGL gl;
  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");
  sys.initBasics(NULL,NULL,&gl,T,3.,false,NULL);
  
  //setup the task
  TaskVariable *pos = new DefaultTaskVariable("position" , *sys.ors, posTVT, "graspCenter", 0, ARR());
  sys.setTaskVariables(ARRAY(pos));
  pos->y_target = arr(sys.ors->getShapeByName("target")->X.pos.p,3);
  pos->setInterpolatedTargetsEndPrecisions(T,1e-2,1e4,0.,0.);

  arr xt;
  sys.getx0(xt);
  sys.testGradientsInCurrentState(xt, 0);
  
  arr q0,x0,x;
  soc::straightTaskTrajectory(sys, q0, 0);
  //soc::getPhaseTrajectory(x0, q0, sys.getTau());
  x=x0=q0;
  conv_VectorChainFunction P2(sys);
  checkJacobian((VectorFunction&)P2, x, 1e-4);
  
  optOptions o;  o.stopTolerance=1e-3;
  eval_cost=0;  x=x0;  optDynamicProgramming(x, P2, (o.stopIters=100, o.initialDamping=1e-4, o.verbose=2, o) );  cout <<"-- evals=" <<eval_cost <<endl;
  eval_cost=0;  x=x0;  optMinSumGaussNewton(x, P2, (o.stopIters=100, o.initialDamping=1e-4, o.verbose=2, o) );  cout <<"-- evals=" <<eval_cost <<endl;
  
  AICO solver;

  cout <<"\n== second test: T step planning ==\n" <<endl;
  T=MT::getParameter<uint>("reachPlanTrajectoryLength");
  sys.setTimeInterval(3.,T);
  sys.setTox0();
  pos->setInterpolatedTargetsEndPrecisions(T,1e-2,1e4,0.,10*1e4);
  solver.init(sys);
  solver.iterate_to_convergence();
  
  for(;;) sys.displayTrajectory(solver.q, NULL, 1, "result");
}

//===========================================================================

void problem2(){
  cout <<"\n= problem 2: dynamic reaching =\n" <<endl;

  //setup the system
  soc::SocSystem_Ors sys;
  OpenGL gl;
  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");
  sys.initBasics(NULL,NULL,&gl,T,4.,true,NULL);
  
  //setup the task
  TaskVariable *pos = new DefaultTaskVariable("position" , *sys.ors, posTVT, "graspCenter", 0, ARR());
  sys.setTaskVariables(ARRAY(pos));
  pos->y_target = arr(sys.ors->getShapeByName("target")->X.pos.p,3);
  pos->setInterpolatedTargetsEndPrecisions(T,
                                           MT::getParameter<double>("reachPlanMidPrec"),
                                           MT::getParameter<double>("reachPlanEndPrec"),
                                           0.,
                                           MT::getParameter<double>("reachPlanEndVelPrec"));

  arr xt;
  sys.getx0(xt);
  sys.testGradientsInCurrentState(xt, 0);
  
  arr q0,x0,x;
  soc::straightTaskTrajectory(sys, q0, 0);
  soc::getPhaseTrajectory(x0, q0, sys.getTau());
  x=x0;
  conv_VectorChainFunction P2(sys);
  checkJacobian((VectorFunction&)P2, x, 1e-4);
  
  optOptions o;  o.stopTolerance=1e-3;
  eval_cost=0;  x=x0;  optDynamicProgramming(x, P2, (o.stopIters=100, o.initialDamping=1e-4, o.verbose=2, o) );  cout <<"-- evals=" <<eval_cost <<endl;
  eval_cost=0;  x=x0;  optMinSumGaussNewton(x, P2, (o.stopIters=100, o.initialDamping=1e-4, o.verbose=2, o) );  cout <<"-- evals=" <<eval_cost <<endl;
  
  AICO solver;
  solver.init(sys);
  solver.iterate_to_convergence();

  for(;;) sys.displayTrajectory(solver.q, NULL, 1, "result");
}


//===========================================================================

void problem3(){
  cout <<"\n= problem 3: dynamic grasping =\n" <<endl;

  //setup the problem
  soc::SocSystem_Ors sys;
  OpenGL gl;
  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");
  sys.initBasics(NULL,NULL,&gl,T,4.,true,NULL);
  
  createStandardRobotTaskVariables(sys);
  setGraspGoals(sys,T,"cyl1");

#if 1
  AICO solver(sys);
  solver.iterate_to_convergence();
  
  for(;;) sys.displayTrajectory(solver.q, NULL, 1, "result");
  return;
#else
  //arr xt;
  //sys.getx0(xt);
  //sys.testGradientsInCurrentState(xt, 0);
  
  arr q0,x0,x;
  soc::straightTaskTrajectory(sys, q0, 0);
  soc::getPhaseTrajectory(x0, q0, sys.getTau());
  x=x0;
  conv_VectorChainFunction P2(sys);
  //checkJacobian((VectorFunction&)P2, x, 1e-4);
  
  optOptions o;  o.stopTolerance=1e-3;
  eval_cost=0;  x=x0;  optDynamicProgramming(x, P2, (o.stopIters=100, o.initialDamping=1e-4, o.verbose=2, o) );  cout <<"-- evals=" <<eval_cost <<endl;
  eval_cost=0;  x=x0;  optMinSumGaussNewton(x, P2, (o.stopIters=100, o.initialDamping=1e-4, o.verbose=2, o) );  cout <<"-- evals=" <<eval_cost <<endl;
#endif
}

//===========================================================================

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  int mode=MT::getParameter<int>("mode");
  switch(mode){
  case 1:  problem1();  break;
  case 2:  problem2();  break;
  case 3:  problem3();  break;
  default: NIY;
  }
  return 0;
}
