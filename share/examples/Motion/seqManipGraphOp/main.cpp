#include <Core/util_t.h>
#include <Gui/opengl.h>

#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>

#include <Ors/ors_swift.h>
#include <Motion/taskMaps.h>

//===========================================================================

void TEST(PickAndPlace){
  cout <<"\n= 1-step grasp optimization=\n" <<endl;

  //setup the problem
  ors::KinematicWorld G("model.kvg");
  G.q += .3;
//  rndGauss(G.q,.01,true); //don't initialize at a singular config
  G.setJointState(G.q);
//  G.joints.last()->Q.rot.setDeg(60,1,0,0);
//  G.calc_q_from_Q();
//  G.gl().update();

  MotionProblem MP(G);
//  MP.loadTransitionParameters(); //->move transition costs to tasks!
  MotionProblemFunction MF(MP);
  MP.z0 = MP.x0.subRange(-4,-1);
  MP.x0 = MP.x0.subRange(0,-5);
  arr x = replicate(MP.x0, MP.T+1);
  rndGauss(x,.01,true); //don't initialize at a singular config
  x.append(MP.z0);

  cout <<"z-init=" <<MP.z0 <<endl;

  ors::GraphOperator *op1 = new ors::GraphOperator();
  op1->symbol = ors::GraphOperator::addRigid;
  op1->timeOfApplication = MP.T/2;
  op1->fromId = G.getBodyByName("graspRef")->index;
  op1->toId = G.getBodyByName("obj1")->index;
  G.operators.append(op1);

  ors::GraphOperator *op2 = new ors::GraphOperator();
  op2->symbol = ors::GraphOperator::deleteJoint;
  op2->timeOfApplication = MP.T/2;
  op2->fromId = G.getBodyByName("table")->index;
  op2->toId = G.getBodyByName("obj1")->index;
  G.operators.append(op2);

  //-- setup new motion problem
  ors::Shape *grasp = G.getShapeByName("graspRef");
  ors::Shape *obj = G.getShapeByName("obj1");
  ors::Shape *tar = G.getShapeByName("target");

  Task *t;
  t = MP.addTask("transitions", new TransitionTaskMap(G));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, {0.}, 1e0);

  t = MP.addTask("pos_mid",  new DefaultTaskMap(posDiffTMT, grasp->index, NoVector, obj->index, NoVector) );
  t->setCostSpecs(MP.T/2-1, MP.T/2, {0.}, 1e3);

  t = MP.addTask("quat", new DefaultTaskMap(quatDiffTMT, grasp->index, NoVector, obj->index) );
  t->setCostSpecs(MP.T/2-1, MP.T/2, {0.}, 1e3);

  t = MP.addTask("q_vel_mid", new TaskMap_qItself());
  t->map.order=1; //make this a velocity variable!
  t->setCostSpecs(MP.T/2-1, MP.T/2, {0.}, 1e1);

  t = MP.addTask("lift", new DefaultTaskMap(posTMT, obj->index));
  t->map.order=1; //make this a velocity variable!
  t->setCostSpecs(MP.T/2+3, MP.T/2+5, {0.,0.,.5}, 1e1);

  //target
  t = MP.addTask("pos", new DefaultTaskMap(posDiffTMT, obj->index, NoVector, tar->index) );
  t->setCostSpecs(MP.T, MP.T, {0.}, 1e3);

  t = MP.addTask("q_vel", new TaskMap_qItself());
  t->map.order=1; //make this a velocity variable!
  t->setCostSpecs(MP.T, MP.T, {0.}, 1e1);

  t = MP.addTask("quat", new DefaultTaskMap(quatDiffTMT, grasp->index, NoVector, tar->index) );
  t->setCostSpecs(MP.T, MP.T, {0.}, 1e3);

////  c = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0}, .05));
  ShapeL shaps = {
    obj, grasp,
    obj, G.getShapeByName("endeff"),
    obj, G.getShapeByName("table")
  };
  t = MP.addTask("collisionConstraints", new ProxyConstraint(allExceptPairsPTMT, shapesToShapeIndices(shaps), .05));
  t->setCostSpecs(0, MP.T, {0.}, 1.);

  shaps = {
    obj, G.getShapeByName("table")
  };
  t = MP.addTask("collisionConstraints2", new ProxyConstraint(pairsPTMT, shapesToShapeIndices(shaps), .05));
  t->setCostSpecs(MP.T/2+10, MP.T, {0.}, 1.);

//  arr y;
//  t->map.phi(y, NoArr, G);
//  cout <<y <<endl;
//  return;

//  checkJacobian(Convert(MF), x, 1e-4); return;
//  checkGradient(Convert(MF), x, 1e-2); return;
//  checkAllGradients(Convert(MF), x, 1e-4); return;
//  checkJacobianCP(Convert(MF), x, 1e-4); return;

  //-- optimize
  for(uint k=0;k<1;k++){
//    optNewton(x, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=.1, stepInc=1.1, stepDec=0.7 , damping=1., allowOverstep=true));
    optConstrained(x, NoArr, Convert(MF), OPT(verbose=1, stopIters=100, maxStep=.1, stepInc=1.1, stepDec=0.7 , aulaMuInc=1.2, damping=1., allowOverstep=true));
  }
  MP.costReport();

  cout <<"z-solution=" <<x.subRange(-4,-1) <<' ' <<sumOfSqr(x.subRange(-4,-1)) <<endl;
  cout <<"z-init=" <<MP.z0 <<' ' <<sumOfSqr(MP.z0) <<endl;

  for(;;)
    displayTrajectory(x, 1, G, "planned trajectory", -100., MF.dim_z());

  MT::wait();

}

//===========================================================================

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  testPickAndPlace();

  return 0;
}
