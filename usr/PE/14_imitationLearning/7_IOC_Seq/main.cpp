#include <Core/util.tpp>
#include <Gui/opengl.h>

#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/taskMap_transition.h>

#include <Ors/ors_swift.h>
#include <Motion/taskMap_proxy.h>

void testSliding() {
  ors::KinematicWorld world("scene");
  arr q, qdot;
  world.getJointState(q, qdot);
  makeConvexHulls(world.shapes);
  world.swift();

  MotionProblem MP(world,true);
  MP.loadTransitionParameters();
  MP.makeContactsAttractive=false;
  arr refGoal1 = conv_vec2arr(MP.world.getShapeByName("target")->X.pos);

  ors::KinematicSwitch *op1 = new ors::KinematicSwitch();
  op1->symbol = ors::KinematicSwitch::addRigid;
  op1->timeOfApplication = MP.T/2;
  op1->fromId = world.getBodyByName("graspRef")->index;
  op1->toId = world.getBodyByName("obj1")->index;
  world.operators.append(op1);

  ors::KinematicSwitch *op2 = new ors::KinematicSwitch();
  op2->symbol = ors::KinematicSwitch::deleteJoint;
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
  c->setCostSpecs(MP.T/2, MP.T/2, conv_vec2arr(obj->X.pos), 1e4);

  c = MP.addTask("quat", new DefaultTaskMap(vecTMT, grasp->index, ors::Vector(0.,0.,1.)) );
  c->setCostSpecs(MP.T/2, MP.T/2, {0.,0.,-1.}, 1e3);

  c = MP.addTask("pos2", new DefaultTaskMap(posTMT, grasp->index) );
  c->setCostSpecs(MP.T, MP.T, conv_vec2arr(tar->X.pos), 1e3);

  c = MP.addTask("q_vel2", new DefaultTaskMap(qItselfTMT, world));
  c->map.order=1; //make this a velocity variable!
  c->setCostSpecs(MP.T/2, MP.T/2, ARR(0.), 1e2);

  c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, world));
  c->map.order=1; //make this a velocity variable!
  c->setCostSpecs(MP.T, MP.T, ARR(0.), 1e2);


  c = MP.addTask("collisionConstraints", new PairCollisionConstraint(MP.world,"table","obj1",0.01));
  c->setCostSpecs(0, MP.T,ARR(0.),1e0);

  TaskMap *tm_contact = new PairCollisionConstraint(MP.world,"obj1","table",0.01);
  TaskCost *c4 = MP.addTask("contact_endeff",tm_contact);
  c4->map.constraint = false;
  c4->setCostSpecs(MP.T/2,MP.T, ARR(0.) ,1e3);

  MP.x0 = zeros(world.getJointStateDimension(),1);MP.x0.reshapeFlat();
  MP.x0(0) = M_PI_2;

  MotionProblemFunction MPF(MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = MP.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

  arr x(T+1,n); x.setZero();arr lambda(T+1); lambda.setZero();
  x = repmat(MP.x0,T+1,1);
//  optConstrained(x,lambda,Convert(MPF),OPT(verbose=1,stopTolerance=1e-3, maxStep=1.));
  optConstrained(x, lambda, Convert(MPF), OPT(verbose=1, stopIters=100, maxStep=.1, stepInc=1.1, stepDec=0.7 , damping=1., allowOverstep=true));
  optConstrained(x, lambda, Convert(MPF), OPT(verbose=1, stopIters=100, maxStep=.1, stepInc=1.1, stepDec=0.7 , damping=1., allowOverstep=true));
  MP.costReport(true);
  cout << lambda << endl;
  for(;;)
    displayTrajectory(x,T,world,"world");
}

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  testSliding();
  return 0;
}
