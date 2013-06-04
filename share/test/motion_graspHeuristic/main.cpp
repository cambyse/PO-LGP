#include <MT/aico.h>
#include <Core/util_t.cxx>
#include <Gui/opengl.h>

#include <motion/MotionPlanner.h>

void testGraspHeuristic(){
  cout <<"\n= 1-step grasp optimization=\n" <<endl;
  double seconds = MT::getParameter<double>("reachPlanTrajectoryDuration");

  //setup the problem
  //OrsSystem sys;
  OrsSystem sys;
  OpenGL gl;
  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");
  arr W;  W <<"[.1 .1 .2 .2 .2 1 1    .1 .1 .1 .1 .1 .1 .1]";
  sys.initBasics(NULL,NULL,&gl,T,seconds,true,&W);
  
  //createStandardRobotTaskVariables(sys);

  //rnd.clockSeed();
  //uint side=rnd(3);
  //side = 2;

  ors::Shape *s = sys.getOrs().getShapeByName("target1");
  for(uint k=0;k<10;k++){

#if 1
    arr x,x0;
    sys.get_x0(x0);
    threeStepGraspHeuristic(x, sys, x0, s->index, 2);
    
    //set velocities to zero
    if(!sys.isKinematic()) x.subRange(14,-1) = 0.;
    
    AICO solver(sys);
    solver.fix_final_state(x);
    solver.iterate_to_convergence();
    
    displayTrajectory(sys, solver.b(), NULL, 1, "solution");
#else
    setNewGraspGoals(sys, T, s->index, side, 1);
    AICO solver(sys);
    solver.iterate_to_convergence();
    for(;;) sys.displayTrajectory(solver.q,NULL,1,"solution");
#endif

    MT::save(sys.getOrs(),"z.ors");

    if(k%2) s=sys.getOrs().getShapeByName("target1");
    else  s=sys.getOrs().getShapeByName("target2");
    s->rel.pos.setRandom(.3);
    s->rel.rot.setRandom();
    for(uint l=0;l<3;l++) s->size[l] = rnd.uni(.05,.15);
    s->size[3] = rnd.uni(.02,.07);
    s->mesh.clear();
    sys.getSwift().init(sys.getOrs(), sys.getSwift().cutoff); //reinitShape(*sys.ors, s);

    sys.setx0ToCurrent();
    sys.gl->watch();
  }
  
}

//===========================================================================

int main(int argn,char **argv){
  MT::initCmdLine(argn,argv);

  testGraspHeuristic();
  
  return 0;
}
