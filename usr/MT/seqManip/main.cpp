#include <Core/util_t.h>
#include <Gui/opengl.h>

#include <Motion/motionHeuristics.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/taskMap_transition.h>

#include <Ors/ors_swift.h>
#include <Motion/taskMap_proxy.h>

//===========================================================================

void testPickAndPlace(){
  cout <<"\n= 1-step grasp optimization=\n" <<endl;

  //setup the problem
  ors::KinematicWorld G("model.kvg");
  G.gl().update();

  MotionProblem MP(G);
//  MP.loadTransitionParameters(); //->move transition costs to tasks!
  MotionProblemFunction MF(MP);

  arr x = replicate(MP.x0, MP.T+1);

  ors::GraphOperator *op;

  op = new ors::GraphOperator();
  op->symbol = ors::GraphOperator::addRigid;
  op->timeOfApplication = MP.T/2+5;
  op->fromId = G.getBodyByName("arm7")->index;
  op->toId = G.getBodyByName("obj1")->index;
  G.operators.append(op);

  op = new ors::GraphOperator();
  op->symbol = ors::GraphOperator::deleteJoint;
  op->timeOfApplication = MP.T/2+5;
  op->fromId = G.getBodyByName("table")->index;
  op->toId = G.getBodyByName("obj1")->index;
  G.operators.append(op);


  //-- setup new motion problem
  TaskCost *c;
  uintA pair = {G.getShapeByName("obj1")->index, G.getShapeByName("endeff")->index};
  c = MP.addTask("pos",
                 new DefaultTaskMap(posTMT, pair(1), NoVector, pair(0), {0,0,.17}));
  c->setCostSpecs(MP.T/2, MP.T/2+10, {0.}, 1e3);

  c = MP.addTask("pos2",
                 new DefaultTaskMap(posTMT, pair(0), NoVector, G.getShapeByName("target")->index, NoVector));
  c->setCostSpecs(MP.T, MP.T, {0.}, 1e3);

  c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, G));
  c->map.order=1; //make this a velocity variable!
  c->setCostSpecs(MP.T, MP.T, {0.}, 1e1);

  c = MP.addTask("transitions", new TransitionTaskMap(G));
  c->map.order=2;
  c->setCostSpecs(0, MP.T, {0.}, 1e0);

//  c = MP.addTask("collision", new ProxyTaskMap(allPTMT, pair, .05));
  c = MP.addTask("collisionConstraints", new CollisionConstraint(.05));
  c->setCostSpecs(0, MP.T, {0.}, 1e1);

  //initialize trajectory
  for(uint t=0;t<=MP.T;t++) x[t]() = MP.x0;

  checkGradient(Convert(MF), x, 1e-4);

  //-- optimize
  for(uint k=0;k<5;k++){
//    optNewton(x, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2.));
    optConstrained(x, NoArr, Convert(MF), OPT(verbose=1, stopIters=100, maxStep=.5, stepInc=2., nonStrictSteps=(!k?15:5)));
  }
  MP.costReport();

  G.gl().update();
  for(;;)
    displayTrajectory(x, 1, G, "planned trajectory", -.1);

}

//===========================================================================

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  testPickAndPlace();

  return 0;
}
