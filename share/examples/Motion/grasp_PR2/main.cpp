#include <Core/util_t.h>
#include <Gui/opengl.h>

#include <Motion/motionHeuristics.h>
#include <Motion/pr2_heuristics.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>

#include <Ors/ors_swift.h>

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

void testPickAndPlace(){
  cout <<"\n= 1-step grasp optimization=\n" <<endl;

  //setup the problem
  ors::KinematicWorld G(MT::getParameter<MT::String>("orsFile"));
  makeConvexHulls(G.shapes);
//  G.watch(true);

  MotionProblem MP(G);
  MP.loadTransitionParameters();
  MP.H_rate_diag = pr2_reasonable_W(G);

  arr x, xT;

  threeStepGraspHeuristic(xT, MP, G.getShapeByName("target1")->index, 2);

  MotionProblemFunction MF(MP);
  sineProfile(x, MP.x0, xT, MP.T);
  optNewton(x, Convert(MF), OPT(verbose=2, stopIters=100, damping=1e-0, stopTolerance=1e-2, maxStep=.5));
  MP.costReport();
  gnuplot("load 'z.costReport.plt'", false, true);

  displayTrajectory(x, -1, G, "planned trajectory");

  G.glueBodies(G.getBodyByName("l_wrist_roll_link"), G.getBodyByName("target1"));
  G.getShapeByName("target1")->cont=false;
  G.getShapeByName("target2")->cont=false;
  G.getShapeByName("target")->cont=false;
  G.swift().initActivations();


  //-- setup new motion problem
  MP.prefix.clear();
  listDelete(MP.taskCosts);
  MP.x0 = x[MP.T-1];

  TaskCost *c;
  c = MP.addTask("position", new DefaultTaskMap(posTMT, G, "target1", ors::Vector(0, 0, 0)));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(MP.world.getShapeByName("target")->X.pos), 1e3);

  c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, G));
  c->map.order=1; //make this a velocity variable!
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, NoArr, 1e1);

  c = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0}, .04));
  MP.setInterpolatingCosts(c, MotionProblem::constant, NoArr, 1e-0);

  //initialize trajectory
  for(uint t=0;t<=MP.T;t++) x[t]() = MP.x0;

  //-- optimize
  optNewton(x, Convert(MF), OPT(verbose=2, stopIters=20, maxStep=1., stepInc=2.));
  MP.costReport();

  displayTrajectory(x, 1, G, "planned trajectory", .1);

  delete G.joints.last();
  G.getShapeByName("target1")->cont=false;
  G.getShapeByName("target2")->cont=false;
  G.getShapeByName("target")->cont=false;
  G.swift().initActivations();

  //-- setup new motion problem
  MP.prefix.clear();
  listDelete(MP.taskCosts);
  MP.x0 = x[MP.T-1];

  c = MP.addTask("position", new DefaultTaskMap(posTMT, G, "graspCenter", ors::Vector(0, 0, 0)));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(MP.world.getShapeByName("target2")->X.pos), 1e3);

  c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, G));
  c->map.order=1; //make this a velocity variable!
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, NoArr, 1e1);

  c = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0}, .04));
  MP.setInterpolatingCosts(c, MotionProblem::constant, NoArr, 1e-0);

  //initialize trajectory
  for(uint t=0;t<=MP.T;t++) x[t]() = MP.x0;

  //-- optimize
  optNewton(x, Convert(MF), OPT(verbose=2, stopIters=20, maxStep=1., stepInc=2.));
  MP.costReport();

  displayTrajectory(x, 1, G, "planned trajectory", .1);

}

//===========================================================================

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

//  testGraspHeuristic();
  testPickAndPlace();

  return 0;
}
