#include <Core/util.tpp>
#include <Gui/opengl.h>

#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>

#include <Motion/komo.h>

#include <Ors/ors_swift.h>

//===========================================================================

void TEST(PickAndPlace){
  cout <<"\n= 1-step grasp optimization=\n" <<endl;

  //setup the problem
  ors::KinematicWorld G("model.kvg");
  G.q += .3;
  G.setJointState(G.q);

  MotionProblem MP(G);
  MotionProblemFunction MF(MP);
  arr x = replicate(MP.x0, MP.T+1);
  rndGauss(x,.01,true); //don't initialize at a singular config

  uint pickTime=MP.T/2, placeTime=MP.T;

  ors::KinematicSwitch *op1 = new ors::KinematicSwitch();
  op1->symbol = ors::KinematicSwitch::addJointZero;
  op1->jointType = ors::JT_fixed;
  op1->timeOfApplication = pickTime;
  op1->fromId = G.getShapeByName("graspRef")->index;
  op1->toId = G.getShapeByName("obj1")->index;
  MP.switches.append(op1);

  ors::KinematicSwitch *op2 = new ors::KinematicSwitch();
  op2->symbol = ors::KinematicSwitch::deleteJoint;
  op2->timeOfApplication = pickTime;
  op2->fromId = G.getShapeByName("table")->index;
  op2->toId = G.getShapeByName("obj1")->index;
  MP.switches.append(op2);

  //-- setup new motion problem
  ors::Shape *grasp = G.getShapeByName("graspRef");
  ors::Shape *obj = G.getShapeByName("obj1");
  ors::Shape *tar = G.getShapeByName("target");

  Task *t;
  t = MP.addTask("_MinSumOfSqr_qItself", new TransitionTaskMap(G));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, {}, 1e0);

  t = MP.addTask("_MinSumOfSqr_posDiff_graspRef_obj1",  new DefaultTaskMap(posDiffTMT, grasp->index, NoVector, obj->index, NoVector) );
  t->setCostSpecs(pickTime-1, pickTime, {}, 1e3);

  t = MP.addTask("_MinSumOfSqr_quatDiff_graspRef_obj1", new DefaultTaskMap(quatDiffTMT, grasp->index, NoVector, obj->index) );
  t->setCostSpecs(pickTime-1, pickTime, {}, 1e3);

  t = MP.addTask("_MinSumOfSqr_qItself", new TaskMap_qItself());
  t->map.order=1; //make this a velocity variable!
  t->setCostSpecs(pickTime-1, pickTime, {}, 1e1);

  t = MP.addTask("_MinSumOfSqr_pos_obj1", new DefaultTaskMap(posTMT, obj->index));
  t->map.order=1; //make this a velocity variable!
  t->setCostSpecs(pickTime+3, pickTime+5, {0.,0.,.5}, 1e1);

  //target
  t = MP.addTask("_MinSumOfSqr_posDiff_obj1_target", new DefaultTaskMap(posDiffTMT, obj->index, NoVector, tar->index) );
  t->setCostSpecs(MP.T, MP.T, {}, 1e3);

  t = MP.addTask("_MinSumOfSqr_qItself", new TaskMap_qItself());
  t->map.order=1; //make this a velocity variable!
  t->setCostSpecs(MP.T, MP.T, {}, 1e1);

  t = MP.addTask("_MinSumOfSqr_quatDiff_obj1_target", new DefaultTaskMap(quatDiffTMT, grasp->index, NoVector, tar->index) );
  t->setCostSpecs(MP.T, MP.T, ARR(1.,0.,0.,0.), 1e3);

  // zero grasp joint motion during holding
  if(MP.z0.N==0){
    ors::Joint *j_grasp = MP.world.getJointByName("graspJoint");
    arr M(j_grasp->qDim(), MP.world.getJointStateDimension());
    M.setZero();
    for(uint i=0;i<j_grasp->qDim();i++) M(i,j_grasp->qIndex+i)=1.;
    t = MP.addTask("_MinSumOfSqr_qItself_graspJoint", new TaskMap_qItself(M));
    t->map.order=1;
    t->prec.resize(MP.T+1).setZero();
    for(uint time=pickTime+1;time<placeTime;time++) t->prec(time)=1e3;
  }


////  c = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0}, .05));
  ShapeL shaps = {
    obj, grasp,
    obj, G.getShapeByName("endeff"),
    obj, G.getShapeByName("table")
  };
  t = MP.addTask("_LowerEqualZero_collisionExceptPairs_obj1_graspRef_obj1_endeff_obj1_table", new ProxyConstraint(allExceptPairsPTMT, shapesToShapeIndices(shaps), .05));
  t->setCostSpecs(0, MP.T, {}, 1.);

  shaps = {
    obj, G.getShapeByName("table")
  };
  t = MP.addTask("_LowerEqualZero_collisionPairs_obj1_table", new ProxyConstraint(pairsPTMT, shapesToShapeIndices(shaps), .05));
  t->setCostSpecs(MP.T/2+10, MP.T, {}, 1.);

//  arr y;
//  t->map.phi(y, NoArr, G);
//  cout <<y <<endl;
//  return;

//  checkJacobian(Convert(MF), x, 1e-4); return;
//  checkGradient(Convert(MF), x, 1e-2); return;
//  checkAllGradients(Convert(MF), x, 1e-4); return;
//  checkJacobianCP(Convert(MF), x, 1e-4); return;

  MP.reportFull();

  //-- optimize
  for(uint k=0;k<1;k++){
    optNewton(x, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=.1, stepInc=1.1, stepDec=0.7 , damping=1., allowOverstep=true));
//    optConstrained(x, NoArr, Convert(MF), OPT(verbose=2, stopEvals=200, maxStep=.1, stepInc=1.1, stepDec=0.7 , aulaMuInc=1.2, damping=1., allowOverstep=true));
  }
  MP.costReport();

  for(;;)
    displayTrajectory(x, 1, G, MP.switches, "planned trajectory", .1);

  mlr::wait();
}

//===========================================================================

void komoVersion(){
  Graph specs("specs.g");
  KOMO komo(specs);
  komo.MP->x0 += .3;
  komo.MP->reportFull(true);
  komo.x = replicate(komo.MP->x0, komo.MP->T+1);
  rndGauss(komo.x,.01,true); //don't initialize at a singular config
  optNewton(komo.x, Convert(*komo.MPF), OPT(verbose=2, stopIters=100, maxStep=.1, stepInc=1.1, stepDec=0.7 , damping=1., allowOverstep=true));
//  komo.run();
  komo.MP->costReport(true);
  for(;;)
    komo.displayTrajectory(-1);
}

//===========================================================================


int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

//  testPickAndPlace();
  komoVersion();

  return 0;
}
