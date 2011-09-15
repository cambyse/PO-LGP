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

void threeStepGraspHeuristic(soc::SocSystem_Ors& sys, uint T, double seconds){
  arr b,x0,b1,Binv;
  sys.getx0(x0);
  arr cost(3),bs(3,x0.N);
  uint side;
  for(side=0;side<3;side++){
    setNewGraspGoals(sys,T,sys.ors->getShapeByName("target")->index, side, 0);
    cost(side) = OneStepDynamicFull(b, Binv, sys, seconds, 1e-1, false);
    sys.displayState(&b, NULL, "posture estimate");
    sys.gl->watch();
    bs[side]() = b;
  }
  side=cost.minIndex();
  b = bs[side];

  b.subRange(7,13) = ARR(0,-1.,.8,-1.,.8,-1.,.8);
  b1=b;
  sys.setx(b);
  sys.gl->watch();
  
  setNewGraspGoals(sys,T,sys.ors->getShapeByName("target")->index, side, 1);
  OneStepDynamicFull(b, Binv, sys, seconds, 1e-1, false, true);
  sys.displayState(&b, NULL, "posture estimate");
  sys.gl->watch();

  AICO solver(sys);
  solver.useBwdMsg=true;
  solver.bwdMsg_v = b;
  solver.bwdMsg_Vinv = Binv + diag(1e2,b.N);
  solver.iterate_to_convergence();

  sys.displayTrajectory(solver.q,NULL,1,"solution");
}

void problem1(){
  cout <<"\n= 1-step grasp optimization=\n" <<endl;
  double seconds = 5.;

  //setup the problem
  soc::SocSystem_Ors sys;
  OpenGL gl;
  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");
  sys.initBasics(NULL,NULL,&gl,T,seconds,true,NULL);
  
  createStandardRobotTaskVariables(sys);

  rnd.clockSeed();
  uint side=rnd(3);
  side = 2;

  for(uint k=0;k<10;k++){
#if 1
    threeStepGraspHeuristic(sys, T, seconds);
#else
    setNewGraspGoals(sys,T,sys.ors->getShapeByName("target")->index, side, 1);
    AICO solver(sys);
    solver.iterate_to_convergence();
    for(;;) sys.displayTrajectory(solver.q,NULL,1,"solution");
#endif

    ors::Shape *s = sys.ors->getShapeByName("target");
    s->rel.pos.setRandom(.3);
    s->rel.rot.setRandom();
  }
  
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
