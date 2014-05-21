#include <Core/util_t.h>
#include <Gui/opengl.h>

#include <Motion/motionHeuristics.h>
#include <Motion/pr2_heuristics.h>

void testGraspHeuristic(){
  cout <<"\n= 1-step grasp optimization=\n" <<endl;

  //setup the problem
  ors::KinematicWorld G(MT::getParameter<MT::String>("orsFile"));
  makeConvexHulls(G.shapes);
  G.watch(true);

  MotionProblem MP(G);
  MP.loadTransitionParameters();
  MP.H_rate_diag = pr2_reasonable_W(G);

  ors::Shape *s = G.getShapeByName("target1");
  for(uint k=0;k<10;k++){

    arr x, xT;

    threeStepGraspHeuristic(xT, MP, s->index, 2);

    MotionProblemFunction F(MP);

    sineProfile(x, MP.x0, xT, MP.T);

    optNewton(x, Convert(F), OPT(verbose=2, stopIters=100, damping=1e-0, stopTolerance=1e-2, maxStep=.5));
    MP.costReport();
    gnuplot("load 'z.costReport.plt'", false, true);

    displayTrajectory(x, 1, G, "planned trajectory");
    displayTrajectory(x, 1, G, "planned trajectory");
    displayTrajectory(x, 1, G, "planned trajectory");

    G >>FILE("z.ors");

    if(k%2) s=G.getShapeByName("target1");
    else    s=G.getShapeByName("target2");
    s->rel.pos.setRandom(.1);
    s->rel.rot.setRandom();
    s->X = s->body->X * s->rel;
    for(uint l=0;l<3;l++) s->size[l] = rnd.uni(.05,.15);
    s->size[3] = rnd.uni(.02,.07);
    s->mesh.clear();

    MP.x0 = x[MP.T-1];
    MP.prefix.clear();
    G.watch(true);
  }
  
}

//===========================================================================

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  testGraspHeuristic();
  
  return 0;
}
