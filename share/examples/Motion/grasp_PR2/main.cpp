#include <Core/util.tpp>
#include <Gui/opengl.h>

#include <Kin/taskMaps.h>

#include <Kin/kin_swift.h>

void TEST(GraspHeuristic){
  cout <<"\n= 1-step grasp optimization=\n" <<endl;

  //setup the problem
  mlr::KinematicWorld G(mlr::getParameter<mlr::String>("orsFile"));
  makeConvexHulls(G.shapes);
  G.watch(true);

  KOMO MP(G);

  mlr::Shape *s = G.getShapeByName("target1");
  for(uint k=0;k<10;k++){

    arr x, xT;

    threeStepGraspHeuristic(xT, MP, s->index, 2);

    Task *c;
    c = MP.addTask("transition", new TaskMap_Transition(G));
    c->map.order=2; //make this an acceleration task!
    c->setCostSpecs(0, MP.T, {0.},1e-2);


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

void TEST(PickAndPlace){
  cout <<"\n= 1-step grasp optimization=\n" <<endl;

  //setup the problem
  mlr::KinematicWorld G(mlr::getParameter<mlr::String>("orsFile"));
  makeConvexHulls(G.shapes);
//  G.watch(true);

  KOMO MP(G);

  arr x, xT;

  threeStepGraspHeuristic(xT, MP, G.getShapeByName("target1")->index, 2);
  Task *t;
  t = MP.addTask("transition", new TaskMap_Transition(G));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, MP.T, {0.},1e-2);



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
  G.swift().initActivations(G);
  listDelete(MF.configurations);


  //-- setup new motion problem
  MP.prefix.clear();
  listDelete(MP.tasks);
  MP.x0 = x[MP.T-1];

  Task *c;
  c = MP.addTask("position", new TaskMap_Default(posTMT, G, "target1", mlr::Vector(0, 0, 0)));
  c->setCostSpecs(MP.T, MP.T, conv_vec2arr(MP.world.getShapeByName("target")->X.pos), 1e3);

  c = MP.addTask("q_vel", new TaskMap_qItself());
  c->map.order=1; //make this a velocity variable!
  c->setCostSpecs(MP.T, MP.T, NoArr, 1e1);

  c = MP.addTask("collision", new TaskMap_Proxy(allPTMT, uintA(0), .04));
  c->setCostSpecs(0, MP.T, NoArr, 1e-0);

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
  G.swift().initActivations(G);

  //-- setup new motion problem
  MP.prefix.clear();
  listDelete(MP.tasks);
  MP.x0 = x[MP.T-1];

  c = MP.addTask("position", new TaskMap_Default(posTMT, G, "graspCenter", mlr::Vector(0, 0, 0)));
  c->setCostSpecs(MP.T, MP.T, conv_vec2arr(MP.world.getShapeByName("target2")->X.pos), 1e3);

  c = MP.addTask("q_vel", new TaskMap_qItself());
  c->map.order=1; //make this a velocity variable!
  c->setCostSpecs(MP.T, MP.T, NoArr, 1e1);

  c = MP.addTask("collision", new TaskMap_Proxy(allPTMT, uintA(0), .04));
  c->setCostSpecs(0, MP.T, NoArr, 1e-0);

  //initialize trajectory
  for(uint t=0;t<=MP.T;t++) x[t]() = MP.x0;

  //-- optimize
  optNewton(x, Convert(MF), OPT(verbose=2, stopIters=20, maxStep=1., stepInc=2.));
  MP.costReport();

  displayTrajectory(x, 1, G, "planned trajectory", .1);

}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

//  testGraspHeuristic();
  testPickAndPlace();

  return 0;
}
