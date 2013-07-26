#include <Core/util_t.h>
#include <Gui/opengl.h>

#include <Motion/motionHeuristics.h>

void testGraspHeuristic(){
  cout <<"\n= 1-step grasp optimization=\n" <<endl;
//  double seconds = MT::getParameter<double>("reachPlanTrajectoryDuration");

  //setup the problem
  OpenGL gl;
  ors::Graph G;
  init(G, gl, MT::getParameter<MT::String>("orsFile"));

  MotionProblem P(&G);
  P.loadTransitionParameters();

//  uint T=MT::getParameter<uint>("reachPlanTrajectoryLength");
//  arr W;  W <<"[.1 .1 .2 .2 .2 1 1    .1 .1 .1 .1 .1 .1 .1]";
//  sys.initBasics(NULL,NULL,&gl,T,seconds,true,&W);
  
  //createStandardRobotTaskVariables(sys);

  //rnd.clockSeed();
  //uint side=rnd(3);
  //side = 2;

  ors::Shape *s = G.getShapeByName("target1");
  for(uint k=0;k<10;k++){

#if 1
    arr x, xT, x0=P.x0;
    threeStepGraspHeuristic(xT, P, x0, s->index, 2);

    MotionProblemFunction F(P);

    sineProfile(x, x0, xT, P.T);

    optGaussNewton(x, Convert(F), OPT(verbose=2, stopIters=20, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));
    //costs.displayRedBlue(~sqr(P.costMatrix), false, 3);
    P.costReport();
    write(LIST<arr>(x),"z.output");
    gnuplot("plot 'z.output' us 1,'z.output' us 2,'z.output' us 3", false, true);
    displayTrajectory(x, 1, G, gl,"planned trajectory");
#else
    setNewGraspGoals(sys, T, s->index, side, 1);
    AICO solver(sys);
    solver.iterate_to_convergence();
    for(;;) sys.displayTrajectory(solver.q,NULL,1,"solution");
#endif

    MT::save(G,"z.ors");

    if(k%2) s=G.getShapeByName("target1");
    else  s=G.getShapeByName("target2");
    s->rel.pos.setRandom(.3);
    s->rel.rot.setRandom();
    for(uint l=0;l<3;l++) s->size[l] = rnd.uni(.05,.15);
    s->size[3] = rnd.uni(.02,.07);
    s->mesh.clear();

    P.setx0(P.x_current);
    gl.watch();
  }
  
}

//===========================================================================

int main(int argn,char **argv){
  MT::initCmdLine(argn,argv);

  testGraspHeuristic();
  
  return 0;
}