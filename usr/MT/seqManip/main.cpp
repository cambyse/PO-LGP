#include <Core/util_t.h>
#include <Gui/opengl.h>

#include <Motion/motion.h>
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
//  MP.z0 = MP.x0.subRange(-4,-1);
//  MP.x0 = MP.x0.subRange(0,-5);
  arr x = replicate(MP.x0, MP.T+1);
  rndGauss(x,.01,true); //don't initialize at a singular config
  x.append(MP.z0);

  ors::GraphOperator *op1 = new ors::GraphOperator();
  op1->symbol = ors::GraphOperator::addRigid;
  op1->timeOfApplication = MP.T/2+2;
  op1->fromId = G.getBodyByName("graspRef")->index;
  op1->toId = G.getBodyByName("obj1")->index;
  G.operators.append(op1);

  ors::GraphOperator *op2 = new ors::GraphOperator();
  op2->symbol = ors::GraphOperator::deleteJoint;
  op2->timeOfApplication = MP.T/2+2;
  op2->fromId = G.getBodyByName("table")->index;
  op2->toId = G.getBodyByName("obj1")->index;
  G.operators.append(op2);

  //-- setup new motion problem
  TaskCost *c;
  uintA pair = {G.getShapeByName("obj1")->index, G.getShapeByName("graspRef")->index};
  c = MP.addTask("pos",
                 new DefaultTaskMap(posTMT, pair(0), NoVector, pair(1)));
  c->setCostSpecs(MP.T/2, MP.T/2+4, {0.}, 1e3);

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
//  c = MP.addTask("collisionConstraints", new CollisionConstraint(.05));
//  c->setCostSpecs(0, MP.T, {0.}, 1e1);

//  checkJacobian(Convert(MF), x, 1e-4); return;

  //-- optimize
  for(uint k=0;k<5;k++){
    optNewton(x, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., damping=1));
//    optConstrained(x, NoArr, Convert(MF), OPT(verbose=1, stopIters=100, maxStep=.5, stepInc=2., nonStrictSteps=(!k?15:5)));
  }
  MP.costReport();

  for(;;)
    displayTrajectory(x, 1, G, "planned trajectory", -100.);

}

//===========================================================================

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  testPickAndPlace();

  return 0;
}
