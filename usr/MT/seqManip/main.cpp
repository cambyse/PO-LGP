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
  rndGauss(G.q,.01,true); //don't initialize at a singular config
  G.setJointState(G.q);
  G.joints.last()->Q.rot.setDeg(60,1,0,0);
  G.calc_q_from_Q();
  G.gl().update();

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
  Task *c;

  c = MP.addTask("pos",
                 new DefaultTaskMap(posTMT, grasp->index) );
  c->setCostSpecs(MP.T/2, MP.T/2, ARRAY(obj->X.pos), 1e3);
  c = MP.addTask("quat",
                 new DefaultTaskMap(quatTMT, grasp->index) );
  c->setCostSpecs(MP.T/2, MP.T/2, ARRAY(obj->X.rot), 1e3);

  c = MP.addTask("pos2",
                 new DefaultTaskMap(posTMT, grasp->index) );
  c->setCostSpecs(MP.T, MP.T, ARRAY(tar->X.pos), 1e3);

  c = MP.addTask("q_vel2", new TaskMap_qItself());
  c->map.order=1; //make this a velocity variable!
  c->setCostSpecs(MP.T/2, MP.T/2, {0.}, 1e1);

  c = MP.addTask("q_vel", new TaskMap_qItself());
  c->map.order=1; //make this a velocity variable!
  c->setCostSpecs(MP.T, MP.T, {0.}, 1e1);

  c = MP.addTask("transitions", new TransitionTaskMap(G));
  c->map.order=2;
  c->setCostSpecs(0, MP.T, {0.}, 1e0);

//  c = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0}, .05));
  c = MP.addTask("collisionConstraints", new CollisionConstraint(.05));
  c->setCostSpecs(0, MP.T, {0.}, 1e0);

//  checkJacobian(Convert(MF), x, 1e-4); return;
//  checkGradient(Convert(MF), x, 1e-2); return;
//  checkAllGradients(Convert(MF), x, 1e-4); return;

  //-- optimize
  for(uint k=0;k<1;k++){
//    optNewton(x, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=.1, stepInc=1.1, stepDec=0.7 , damping=1., allowOverstep=true));
    optConstrained(x, NoArr, Convert(MF), OPT(verbose=1, stopIters=100, maxStep=.1, stepInc=1.1, stepDec=0.7 , damping=1., allowOverstep=true));
  }
  MP.costReport();


  cout <<"z-solution=" <<x.subRange(-4,-1) <<' ' <<sumOfSqr(x.subRange(-4,-1)) <<endl;
  cout <<"z-init=" <<MP.z0 <<' ' <<sumOfSqr(MP.z0) <<endl;

  for(;;)
    displayTrajectory(x, 1, G, "planned trajectory", -100., MF.dim_z());

}

//===========================================================================

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  testPickAndPlace();

  return 0;
}
