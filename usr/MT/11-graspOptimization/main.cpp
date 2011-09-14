#include <MT/soc.h>
#include <MT/util.h>
#include <MT/specialTaskVariables.h>
#include <MT/opengl.h>
#include <MT/aico.h>
#include <DZ/aico_key_frames.h>
#include <SD/graspObjects.h>
#include <SD/potentialTaskVariables.h>
#include <MT/ors.h>

#include "setNewGraspGoals.cpp"
#include "setNewGraspGoals_explorative.cpp"
#include "setOldGraspGoals.cpp"

void interpolateTrajectory(arr& x,uint t1, uint T, const arr& x0, const arr& x1, const arr& xT){
  uint t;
  double a;
  x.resize(T+1,x0.N);
  for(t=0; t<=t1; t++){
    a = (double)t/t1;
    x[t]() = ((double)1.-a)*x0 + a*x1;
  }
  for(t=t1; t<=T; t++){
    a = (double)(t-t1)/(T-t1);
    x[t]() = ((double)1.-a)*x1 + a*xT;
  }
}

void problem1(){
  cout <<"\n= 1-step grasp optimization=\n" <<endl;

  //setup the problem
  soc::SocSystem_Ors sys;
  OpenGL gl;
  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");
  sys.initBasics(NULL,NULL,&gl,T,4.,true,NULL);
  
  createStandardRobotTaskVariables(sys);

  rnd.clockSeed();
  uint side=rnd(3);
  side = 2;

#if 1
  arr b,x0,b1,Binv;
  sys.getx0(x0);
  setNewGraspGoals(sys,T,sys.ors->getShapeByName("target")->index, side, 0);
  OneStepDynamicFull(b, Binv, sys, 4., 1e-1, true);
  sys.displayState(&b, NULL, "posture estimate");
  sys.gl->watch();

  b.subRange(7,13) = ARR(0,-1.,.8,-1.,.8,-1.,.8);
  b1=b;
  sys.setx(b);
  sys.gl->watch();
  
  setNewGraspGoals(sys,T,sys.ors->getShapeByName("target")->index, side, 1);
  OneStepDynamicFull(b, Binv, sys, 4., 1e-1, true, true);
  sys.displayState(&b, NULL, "posture estimate");
  sys.gl->watch();

  arr q,q0;
  sys.getq0(q0);
  interpolateTrajectory(q,4*T/5,T,q0,b1.sub(0,13),b.sub(0,13));
  sys.displayTrajectory(q,NULL,1,"initialization");
  
  AICO solver(sys);
  solver.useBwdMsg=true;
  solver.bwdMsg_v = b;
  solver.bwdMsg_Vinv = Binv + diag(1e2,b.N);
  //solver.init_trajectory(q,1.);
  solver.iterate_to_convergence();
#elif 1
  arr b,Binv;
  setOldGraspGoals(sys,T,sys.ors->getShapeByName("target")->index, side, 1);
  OneStepDynamicFull(b, Binv, sys, 4., 1e-1, true);
  sys.displayState(&b, NULL, "posture estimate");
  sys.gl->watch();

  AICO solver(sys);
  solver.useBwdMsg=true;
  solver.bwdMsg_v = b;
  solver.bwdMsg_Vinv = Binv + diag(1e1,b.N);
  solver.iterate_to_convergence();
#else
  setNewGraspGoals(sys,T,sys.ors->getShapeByName("target")->index, side, 1);
  AICO solver(sys);
  solver.iterate_to_convergence();
#endif
  
}

//===========================================================================

int main(int argn,char **argv){
  MT::initCmdLine(argn,argv);

  int mode=MT::getParameter<int>("mode");
  switch(mode){
  case 1:  problem1();  break;
  //case 2:  problem2();  break;
  //case 3:  problem3();  break;
  default: NIY;
  }
  return 0;
}
