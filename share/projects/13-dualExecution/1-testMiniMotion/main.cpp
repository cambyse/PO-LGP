#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>

arr getSimpleTrajectory(ors::KinematicWorld& G){
  MotionProblem P(G, false);
  P.loadTransitionParameters();

  //-- setup the motion problem
  Task *c;
  c = P.addTask("position",
                   new TaskMap_Default(posTMT, G, "endeff", NoVector));
  c->setCostSpecs(P.T, P.T,
                          conv_vec2arr(P.world.getShapeByName("miniTarget")->X.pos), 1e2);
  c = P.addTask("position",
                   new TaskMap_Default(posTMT, G, "endeff", NoVector));
c->map.order=1;
  c->setCostSpecs(P.T, P.T, {0.,0.,0.}, 1e1);

  MotionProblemFunction MF(P);
  arr x = P.getInitialization();

  optNewton(x, Convert(MF), OPT(verbose=2, stopIters=100, damping=1e-3, maxStep=.5));
  P.costReport();
  return x;
}


int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  ors::KinematicWorld G(mlr::getParameter<mlr::String>("orsFile"));

  arr x = getSimpleTrajectory(G);
  arr x2 = reverseTrajectory(x);
  x.append(x2);

  for(uint i=0;i<3;i++)
    displayTrajectory(x, 1, G, "planned trajectory");

  return 0;
}


