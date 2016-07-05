//===========================================================================

void TEST(PickAndPlace){
  cout <<"\n= 1-step grasp optimization=\n" <<endl;

  //setup the problem
  ors::KinematicWorld G("model.kvg");
  G.q += .3;
  G.setJointState(G.q);

  MotionProblem MP(G);
  arr x = MP.getInitialization();
  rndGauss(x,.01,true); //don't initialize at a singular config

  uint pickTime=MP.T/2, placeTime=MP.T;

  ors::KinematicSwitch *op1 = new ors::KinematicSwitch();
  op1->symbol = ors::KinematicSwitch::addJointZero;
  op1->jointType = ors::JT_rigid;
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
  t = MP.addTask("_MinSumOfSqr_qItself", new TaskMap_Transition(G), sumOfSqrTT);
  t->map.order=2;
  t->setCostSpecs(0, MP.T, {}, 1e0);

  t = MP.addTask("_MinSumOfSqr_posDiff_graspRef_obj1",  new TaskMap_Default(posDiffTMT, grasp->index, NoVector, obj->index, NoVector), sumOfSqrTT);
  t->setCostSpecs(pickTime-1, pickTime, {}, 1e3);

  t = MP.addTask("_MinSumOfSqr_quatDiff_graspRef_obj1", new TaskMap_Default(quatDiffTMT, grasp->index, NoVector, obj->index), sumOfSqrTT);
  t->setCostSpecs(pickTime-1, pickTime, {}, 1e3);

  t = MP.addTask("_MinSumOfSqr_qItself", new TaskMap_qItself(), sumOfSqrTT);
  t->map.order=1; //make this a velocity variable!
  t->setCostSpecs(pickTime-1, pickTime, {}, 1e1);

  t = MP.addTask("_MinSumOfSqr_pos_obj1", new TaskMap_Default(posTMT, obj->index), sumOfSqrTT);
  t->map.order=1; //make this a velocity variable!
  t->setCostSpecs(pickTime+3, pickTime+5, {0.,0.,.5}, 1e1);

  //target
  t = MP.addTask("_MinSumOfSqr_posDiff_obj1_target", new TaskMap_Default(posDiffTMT, obj->index, NoVector, tar->index), sumOfSqrTT);
  t->setCostSpecs(MP.T, MP.T, {}, 1e3);

  t = MP.addTask("_MinSumOfSqr_qItself", new TaskMap_qItself(), sumOfSqrTT);
  t->map.order=1; //make this a velocity variable!
  t->setCostSpecs(MP.T, MP.T, {}, 1e1);

  t = MP.addTask("_MinSumOfSqr_quatDiff_obj1_target", new TaskMap_Default(quatDiffTMT, grasp->index, NoVector, tar->index), sumOfSqrTT);
  t->setCostSpecs(MP.T, MP.T, ARR(1.,0.,0.,0.), 1e3);

  // zero grasp joint motion during holding
  ors::Joint *j_grasp = MP.world.getJointByName("graspJoint");
  arr M(j_grasp->qDim(), MP.world.getJointStateDimension());
  M.setZero();
  for(uint i=0;i<j_grasp->qDim();i++) M(i,j_grasp->qIndex+i)=1.;
  t = MP.addTask("_MinSumOfSqr_qItself_graspJoint", new TaskMap_qItself(M), sumOfSqrTT);
  t->map.order=1;
  t->prec.resize(MP.T+1).setZero();
  for(uint time=pickTime+1;time<placeTime;time++) t->prec(time)=1e3;


////  c = MP.addTask("collision", new TaskMap_Proxy(allPTMT, {0}, .05));
  ShapeL shaps = {
    obj, grasp,
    obj, G.getShapeByName("endeff"),
    obj, G.getShapeByName("table")
  };
  t = MP.addTask("_LowerEqualZero_collisionExceptPairs_obj1_graspRef_obj1_endeff_obj1_table", new ProxyConstraint(allExceptPairsPTMT, shapesToShapeIndices(shaps), .05), ineqTT);
  t->setCostSpecs(0, MP.T, {}, 1.);

  shaps = {
    obj, G.getShapeByName("table")
  };
  t = MP.addTask("_LowerEqualZero_collisionPairs_obj1_table", new ProxyConstraint(pairsPTMT, shapesToShapeIndices(shaps), .05), ineqTT);
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
    optNewton(x, Convert(MP), OPT(verbose=2, stopIters=100, maxStep=.1, stepInc=1.1, stepDec=0.7 , damping=1., allowOverstep=true));
//    optConstrained(x, NoArr, Convert(MF), OPT(verbose=2, stopEvals=200, maxStep=.1, stepInc=1.1, stepDec=0.7 , aulaMuInc=1.2, damping=1., allowOverstep=true));
  }
  MP.costReport();

  for(;;)
    displayTrajectory(x, 1, G, MP.switches, "planned trajectory", .1);

  mlr::wait();
}

