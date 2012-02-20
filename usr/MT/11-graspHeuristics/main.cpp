#include <MT/soc.h>
#include <MT/util.h>
#include <MT/specialTaskVariables.h>
#include <MT/opengl.h>
#include <MT/aico.h>
#include <DZ/aico_key_frames.h>
#include <SD/graspObjects.h>
#include <SD/potentialTaskVariables.h>
#include <MT/ors.h>

#include <architecture/MotionPrimitive.h>

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

  //rnd.clockSeed();
  uint side=rnd(3);
  side = 2;

  ors::Shape *s = sys.ors->getShapeByName("target1");
  for(uint k=0;k<10;k++){

#if 1
    arr x,x0;
    sys.getx0(x0);
    threeStepGraspHeuristic(x, sys, x0, s->index, 1);
    
    //set velocities to zero
    if(sys.dynamic) x.subRange(14,-1) = 0.;
    
    AICO solver(sys);
    solver.fix_final_state(x);
    solver.iterate_to_convergence();
    
    sys.displayTrajectory(solver.q,NULL,1,"solution");
#else
    setNewGraspGoals(sys, T, s->index, side, 1);
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

    sys.setx0ToCurrent();
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
