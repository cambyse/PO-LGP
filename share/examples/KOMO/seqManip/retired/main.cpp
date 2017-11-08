//===========================================================================

void TEST(UsingSpecs){
  Graph specs("specsPush.g");
  KOMO komo(specs);
  komo.reset();
//  komo.MP->reportFeatures(true);
  komo.run();
  cout <<komo.getReport(true) <<endl;
  for(;;)
    komo.displayTrajectory();
}

//===========================================================================

void testNonSliderSlide(){
  mlr::KinematicWorld W("model.g");

  KOMO komo;
  komo.setModel(W);

  komo.setTiming(5., 20, 5., 2);
  komo.setFixEffectiveJoints(-1., -1., 1e3);
  komo.setFixSwitchedObjects(-1., -1., 1e3);
  komo.setSquaredQAccelerations();
//  komo.setSquaredQVelocities(-1., -1., 1e-1);

//  komo.setPosition(.5, 2.5, "obj1", "endeffWorkspace", OT_sumOfSqr, NoArr, 1e-1);
//  komo.setTouch(1., 3., "endeff", "obj1", OT_sumOfSqr, {.0}, 1e3);

  komo.setKS_placeOn(2., true, "obj1", "table", true);
//  komo.setKS_slider(2., true, "obj1", "table", true);

  komo.setPosition(3.8, 5., "obj1", "target", OT_sumOfSqr, {}, 1e1);

  komo.setKS_placeOn(4., true, "obj1", "table", false);

  //velocities
//  komo.setTask(2.-.15, 2., new TaskMap_Default(posDiffTMT, W, "endeff"), OT_sumOfSqr, {.1,0,0}, 1e2, 1);
//  komo.setTask(4., 4.+.15, new TaskMap_Default(posDiffTMT, W, "endeff"), OT_sumOfSqr, {-.1,0,0}, 1e2, 1);

  //keep distance
//  komo.setTask(1.5, 4., new TaskMap_LinTrans(new TaskMap_Default(posDiffTMT, W, "endeff", NoVector, "obj1", NoVector),
//                                            true),          OT_sumOfSqr, {.2}, 1e3);
//  komo.setTask(2., 4., new TaskMap_GJK(W, "endeff", "obj1", true, true), OT_eq, {-.15}, 1e2, 0);
  komo.setTask(2., 4., new TaskMap_Default(posDiffTMT, W, "endeff", NoVector, "obj1", NoVector), OT_sumOfSqr, {-.1,0,0}, 1e2);
  //push align
//  komo.setTask(2., 4., new TaskMap_PushConsistent(W, "obj1", "endeff"), OT_sumOfSqr, {0,0,0}, 1e3);

  //no collisions
  komo.setTask(0., 1.9, new TaskMap_Proxy(allPTMT, uintA(), .03), OT_sumOfSqr, NoArr, 1e2);
//  komo.setTask(4.5, -1., new TaskMap_Proxy(allPTMT, uintA(), .03), OT_sumOfSqr, NoArr, 1e2);


  komo.reset();
  komo.run();
//      komo.checkGradients();

  cout <<komo.getReport(true);

  while(komo.displayTrajectory(.1, true));
}

//===========================================================================

void TEST(PickAndPlace){
  cout <<"\n= 1-step grasp optimization=\n" <<endl;

  //setup the problem
  mlr::KinematicWorld G("model.kvg");
  G.q += .3;
  G.setJointState(G.q);

  KOMO MP(G);
  arr x = MP.getInitialization();
  rndGauss(x,.01,true); //don't initialize at a singular config

  uint pickTime=MP.T/2, placeTime=MP.T;

  mlr::KinematicSwitch *op1 = new mlr::KinematicSwitch();
  op1->symbol = mlr::KinematicSwitch::addJointZero;
  op1->jointType = mlr::JT_rigid;
  op1->timeOfApplication = pickTime;
  op1->fromId = G.getShapeByName("graspRef")->index;
  op1->toId = G.getShapeByName("obj1")->index;
  MP.switches.append(op1);

  mlr::KinematicSwitch *op2 = new mlr::KinematicSwitch();
  op2->symbol = mlr::KinematicSwitch::deleteJoint;
  op2->timeOfApplication = pickTime;
  op2->fromId = G.getShapeByName("table")->index;
  op2->toId = G.getShapeByName("obj1")->index;
  MP.switches.append(op2);

  //-- setup new motion problem
  mlr::Shape *grasp = G.getShapeByName("graspRef");
  mlr::Shape *obj = G.getShapeByName("obj1");
  mlr::Shape *tar = G.getShapeByName("target");

  Task *t;
  t = MP.addTask("_MinSumOfSqr_qItself", new TaskMap_Transition(G), OT_sumOfSqr);
  t->map.order=2;
  t->setCostSpecs(0, MP.T, {}, 1e0);

  t = MP.addTask("_MinSumOfSqr_posDiff_graspRef_obj1",  new TaskMap_Default(posDiffTMT, grasp->index, NoVector, obj->index, NoVector), OT_sumOfSqr);
  t->setCostSpecs(pickTime-1, pickTime, {}, 1e3);

  t = MP.addTask("_MinSumOfSqr_quatDiff_graspRef_obj1", new TaskMap_Default(quatDiffTMT, grasp->index, NoVector, obj->index), OT_sumOfSqr);
  t->setCostSpecs(pickTime-1, pickTime, {}, 1e3);

  t = MP.addTask("_MinSumOfSqr_qItself", new TaskMap_qItself(), OT_sumOfSqr);
  t->map.order=1; //make this a velocity variable!
  t->setCostSpecs(pickTime-1, pickTime, {}, 1e1);

  t = MP.addTask("_MinSumOfSqr_pos_obj1", new TaskMap_Default(posTMT, obj->index), OT_sumOfSqr);
  t->map.order=1; //make this a velocity variable!
  t->setCostSpecs(pickTime+3, pickTime+5, {0.,0.,.5}, 1e1);

  //target
  t = MP.addTask("_MinSumOfSqr_posDiff_obj1_target", new TaskMap_Default(posDiffTMT, obj->index, NoVector, tar->index), OT_sumOfSqr);
  t->setCostSpecs(MP.T, MP.T, {}, 1e3);

  t = MP.addTask("_MinSumOfSqr_qItself", new TaskMap_qItself(), OT_sumOfSqr);
  t->map.order=1; //make this a velocity variable!
  t->setCostSpecs(MP.T, MP.T, {}, 1e1);

  t = MP.addTask("_MinSumOfSqr_quatDiff_obj1_target", new TaskMap_Default(quatDiffTMT, grasp->index, NoVector, tar->index), OT_sumOfSqr);
  t->setCostSpecs(MP.T, MP.T, {1.,0.,0.,0.}, 1e3);

  // zero grasp joint motion during holding
  mlr::Joint *j_grasp = MP.world.getJointByName("graspJoint");
  arr M(j_grasp->qDim(), MP.world.getJointStateDimension());
  M.setZero();
  for(uint i=0;i<j_grasp->qDim();i++) M(i,j_grasp->qIndex+i)=1.;
  t = MP.addTask("_MinSumOfSqr_qItself_graspJoint", new TaskMap_qItself(M), OT_sumOfSqr);
  t->map.order=1;
  t->prec.resize(MP.T+1).setZero();
  for(uint time=pickTime+1;time<placeTime;time++) t->prec(time)=1e3;


////  c = MP.addTask("collision", new TaskMap_Proxy(allPTMT, {0}, .05));
  ShapeL shaps = {
    obj, grasp,
    obj, G.getShapeByName("endeff"),
    obj, G.getShapeByName("table")
  };
  t = MP.addTask("_LowerEqualZero_collisionExceptPairs_obj1_graspRef_obj1_endeff_obj1_table", new ProxyConstraint(allExceptPairsPTMT, shapesToShapeIndices(shaps), .05), OT_ineq);
  t->setCostSpecs(0, MP.T, {}, 1.);

  shaps = {
    obj, G.getShapeByName("table")
  };
  t = MP.addTask("_LowerEqualZero_collisionPairs_obj1_table", new ProxyConstraint(pairsPTMT, shapesToShapeIndices(shaps), .05), OT_ineq);
  t->setCostSpecs(MP.T/2+10, MP.T, {}, 1.);

//  arr y;
//  t->map.phi(y, NoArr, G);
//  cout <<y <<endl;
//  return;

//  checkJacobian(Convert(MF), x, 1e-4); return;
//  checkGradient(Convert(MF), x, 1e-2); return;
//  checkAllGradients(Convert(MF), x, 1e-4); return;
//  checkJacobianCP(Convert(MF), x, 1e-4); return;

  MP.reportFeatures();

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

