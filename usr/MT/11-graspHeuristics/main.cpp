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

void threeStepGraspHeuristic(soc::SocSystem_Ors& sys, uint T, uint shapeId, double seconds){
  arr b,x0,b1,Binv;
  sys.getx0(x0);
  arr cost(3),bs(3,x0.N);
  uint side=0;
  if(sys.ors->shapes(shapeId)->type==ors::boxST){
    for(side=0;side<3;side++){
      setNewGraspGoals(sys,T,shapeId, side, 0);
      cost(side) = OneStepDynamicFull(b, Binv, sys, seconds, 1e-1, 1e-2, 0);
      sys.displayState(&b, NULL, "posture estimate", true);
      sys.gl->watch();
      bs[side]() = b;
    }
    side=cost.minIndex();
    b = bs[side];
  }else{
    setNewGraspGoals(sys,T,shapeId, side, 0);
    OneStepDynamicFull(b, Binv, sys, seconds, 1e-1, 1e-2, 0);
    sys.displayState(NULL, NULL, "posture estimate", true);
    sys.gl->watch();
  }

  b.subRange(7,13) = ARR(0,-1.,.8,-1.,.8,-1.,.8);
  b1=b;
  sys.setx(b);
  sys.gl->watch();
  
  setNewGraspGoals(sys,T,shapeId, side, 1);
  OneStepDynamicFull(b, Binv, sys, seconds, 1e-1, 1e-2, 0, true);
  sys.displayState(&b, NULL, "posture estimate", true);
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
  arr W;  W <<"[.1 .1 .2 .2 .2 1 1    .1 .1 .1 .1 .1 .1 .1]";
  sys.initBasics(NULL,NULL,&gl,T,seconds,true,&W);
  
  createStandardRobotTaskVariables(sys);

  rnd.clockSeed();
  uint side=rnd(3);
  side = 2;

  ors::Shape *s = sys.ors->getShapeByName("target1");
  for(uint k=0;k<10;k++){

#if 1
    threeStepGraspHeuristic(sys, T, s->index, seconds);
#else
    setNewGraspGoals(sys,T,s->index, side, 1);
    AICO solver(sys);
    solver.iterate_to_convergence();
    for(;;) sys.displayTrajectory(solver.q,NULL,1,"solution");
#endif

    if(k%2) s=sys.ors->getShapeByName("target1");
    else  s=sys.ors->getShapeByName("target2");
    s->rel.pos.setRandom(.3);
    s->rel.rot.setRandom();
    for(uint l=0;l<3;l++) s->size[l] = rnd.uni(.05,.15);
    s->size[3] = rnd.uni(.02,.07);
    s->mesh.clear();
    sys.swift->init(*sys.ors, sys.swift->cutoff); //reinitShape(*sys.ors, s);

    sys.setx0AsCurrent();
    sys.gl->watch();
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
