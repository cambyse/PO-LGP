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

/*void testPickAndPlace(){
  cout <<"\n= 1-step grasp optimization=\n" <<endl;

  //setup the problem
  ors::KinematicWorld world("scene");

  MotionProblem MP(world);
//  MP.loadTransitionParameters(); //->move transition costs to tasks!
  MotionProblemFunction MF(MP);
  MP.z0 = MP.x0.subRange(-4,-1);
  MP.x0 = MP.x0.subRange(0,-5);
  arr x = replicate(MP.x0, MP.T+2);
  x.setZero();//  rndGauss(x,.01,true); //don't initialize at a singular config
  cout << x << endl;
  x.append(~MP.z0);
  cout << x << endl;

  cout <<"z-init=" <<MP.z0 <<endl;

  ors::GraphOperator *op1 = new ors::GraphOperator();
  op1->symbol = ors::GraphOperator::addRigid;
  op1->timeOfApplication = MP.T/2;
  op1->fromId = world.getBodyByName("graspRef")->index;
  op1->toId = world.getBodyByName("obj1")->index;
  world.operators.append(op1);

  ors::GraphOperator *op2 = new ors::GraphOperator();
  op2->symbol = ors::GraphOperator::deleteJoint;
  op2->timeOfApplication = MP.T/2;
  op2->fromId = world.getBodyByName("table")->index;
  op2->toId = world.getBodyByName("obj1")->index;
  world.operators.append(op2);

  //-- setup new motion problem
  ors::Shape *grasp = world.getShapeByName("graspRef");
  ors::Shape *obj = world.getShapeByName("obj1");
  ors::Shape *tar = world.getShapeByName("target");
  TaskCost *c;

  c = MP.addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  c->setCostSpecs(MP.T/2, MP.T/2, ARRAY(obj->X.pos), 1e3);
  c = MP.addTask("quat", new DefaultTaskMap(quatTMT, grasp->index) );
  c->setCostSpecs(MP.T/2, MP.T/2, ARRAY(obj->X.rot), 1e3);

  c = MP.addTask("pos2", new DefaultTaskMap(posTMT, grasp->index) );
  c->setCostSpecs(MP.T, MP.T, ARRAY(tar->X.pos), 1e3);

  c = MP.addTask("q_vel2", new DefaultTaskMap(qItselfTMT, world));
  c->map.order=1; //make this a velocity variable!
  c->setCostSpecs(MP.T/2, MP.T/2, {0.}, 1e1);

  c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, world));
  c->map.order=1; //make this a velocity variable!
  c->setCostSpecs(MP.T, MP.T, {0.}, 1e1);

  c = MP.addTask("transitions", new TransitionTaskMap(world));
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
  cout << x << endl;

  MP.costReport();


  cout <<"z-solution=" <<x.subRange(-4,-1) <<' ' <<sumOfSqr(x.subRange(-4,-1)) <<endl;
  cout <<"z-init=" <<MP.z0 <<' ' <<sumOfSqr(MP.z0) <<endl;

  for(;;)
    displayTrajectory(x, 1, world, "planned trajectory", -100., MF.dim_z());

}
*/

void testSliding() {
  ors::KinematicWorld world("scene");
  arr q, qdot;
  world.getJointState(q, qdot);
  MotionProblem MP(world,false);
  MP.loadTransitionParameters();
  MP.makeContactsAttractive=false;
  arr refGoal1 = ARRAY(MP.world.getShapeByName("target")->X.pos);

  ors::GraphOperator *op1 = new ors::GraphOperator();
  op1->symbol = ors::GraphOperator::addRigid;
  op1->timeOfApplication = MP.T/2;
  op1->fromId = world.getBodyByName("graspRef")->index;
  op1->toId = world.getBodyByName("obj1")->index;
  world.operators.append(op1);

  ors::GraphOperator *op2 = new ors::GraphOperator();
  op2->symbol = ors::GraphOperator::deleteJoint;
  op2->timeOfApplication = MP.T/2;
  op2->fromId = world.getBodyByName("table")->index;
  op2->toId = world.getBodyByName("obj1")->index;
  world.operators.append(op2);

  //-- setup new motion problem
  ors::Shape *grasp = world.getShapeByName("graspRef");
  ors::Shape *obj = world.getShapeByName("obj1");
  ors::Shape *tar = world.getShapeByName("target");

  TaskCost *c;
  c = MP.addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  c->setCostSpecs(MP.T/2, MP.T/2, ARRAY(obj->X.pos), 1e4);

  c = MP.addTask("quat", new DefaultTaskMap(vecTMT, grasp->index, ors::Vector(0.,0.,1.)) );
  c->setCostSpecs(MP.T/2, MP.T/2, ARRAY(0.,0.,-1.), 1e3);

  c = MP.addTask("pos2", new DefaultTaskMap(posTMT, grasp->index) );
  c->setCostSpecs(MP.T, MP.T, ARRAY(tar->X.pos), 1e3);

  c = MP.addTask("q_vel2", new DefaultTaskMap(qItselfTMT, world));
  c->map.order=1; //make this a velocity variable!
  c->setCostSpecs(MP.T/2, MP.T/2, ARR(0.), 1e2);

  c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, world));
  c->map.order=1; //make this a velocity variable!
  c->setCostSpecs(MP.T, MP.T, ARR(0.), 1e1);


  c = MP.addTask("collisionConstraints", new PairCollisionConstraint(MP.world,"endeff","table"));
  MP.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1.);

  MP.x0 = zeros(world.getJointStateDimension(),1);MP.x0.flatten();
  MP.x0(0) = M_PI_2;

  MotionProblemFunction MPF(MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = MP.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

  arr x(T+1,n); x.setZero();arr lambda(T+1); lambda.setZero();
//  optConstrained(x,lambda,Convert(MPF),OPT(verbose=1,stopTolerance=1e-3, maxStep=1.));
  optConstrained(x, NoArr, Convert(MPF), OPT(verbose=1, stopIters=100, maxStep=.1, stepInc=1.1, stepDec=0.7 , damping=1., allowOverstep=true));
  optConstrained(x, NoArr, Convert(MPF), OPT(verbose=1, stopIters=100, maxStep=.1, stepInc=1.1, stepDec=0.7 , damping=1., allowOverstep=true));
  MP.costReport(true);

  for(;;)
    displayTrajectory(x,T,world,"world");
}

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  //  testPickAndPlace();
  testSliding();
  return 0;
}
