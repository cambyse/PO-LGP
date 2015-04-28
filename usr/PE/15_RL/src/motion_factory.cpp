#include "motion_factory.h"
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include "../src/cost_weight.h"
#include <iomanip>
#include <iostream>


void MotionFactory::execMotion(Scene &s, arr &x, arr &lambda, arr &_x0, uint vis, uint _verbose, MT::String name) {
  s.MP->prefix.clear();
  s.MP->phiMatrix.clear();
  s.MP->ttMatrix.clear();

  /// set initial state
  if (&_x0) {
    s.MP->x0 = _x0;
  }else {
    s.MP->x0 = s.x0;
  }

  /// optimize motion
  arr xx;
  arr ll;
  MotionProblemFunction MPF(*s.MP);


  if (MT::getParameter<bool>("IMP/loadDemoFromFile") && name.N>0) {
    readDemoFromFile(name,xx,ll);
  } else {
    xx = repmat(~s.MP->x0,s.MP->T+1,1);
    OptOptions o; o.verbose=_verbose; o.stopTolerance=MT::getParameter<double>("FMP/stopTolerance"); o.stopIters=100; o.aulaMuInc=1.1; o.maxStep=1.;

    optConstrainedMix(xx, ll, Convert(MPF), o);
    s.MP->costReport(true);
    writeDemoToFile(name,xx,ll);
  }

  /// visualize
  if (vis==1) {
    displayTrajectory(xx,s.MP->T,s.MP->world,"world"); MT::wait(1.5);
  }else if (vis==2) {
    for(;;) {displayTrajectory(xx,s.MP->T,s.MP->world,"world"); MT::wait(1.5);}
  }

  if (&x) x = xx;
  if (&lambda) lambda = ll;
}

void MotionFactory::writeDemoToFile(const char* name,arr &x,arr &lambda){
  String n;
  n<< "data/";
  n<<String(name);
  n <<"x";
  write(LIST<arr>(x),n);
  n <<"l";
  write(LIST<arr>(lambda),n);
}

void MotionFactory::readDemoFromFile(const char* name,arr &x,arr &lambda){
  String n;
  n<<"data/";
  n<<String(name);
  n <<"x";
  x << FILE(n);
  n <<"l";
  lambda << FILE(n);
  lambda.flatten();
}


void MotionFactory::loadScenarioTest(Scenario &scenario, bool useConstraints) {
  for (uint iS=0;iS<2;iS++) {
    Scene s;

    /// create world and motion problem
    s.world = new ors::KinematicWorld("sceneTest");
    s.world->swift();

    s.world->getBodyByName("targetE")->X.pos += ors::Vector(0.,0.,0.2)*iS;

    s.MP = new MotionProblem(*s.world,false);
    s.MP->useSwift=true;
    s.MP->T = 50;
    s.MP->tau = 0.01;
    s.x0 = s.world->getJointState();

    ors::Shape *grasp = s.world->getShapeByName("endeff");
    ors::Body *targetE = s.world->getBodyByName("targetE");


    arr param = ARRAY(1e-1,1e2);
    param = param/sum(param)*scenario.costScale;

    // transition costs
    Task *t;
    t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
    t->map.order=2;
    t->target = ARRAY(0.);
    ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
    if (iS==0) scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));

    t =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
    t->target = ARRAY(targetE->X.pos);
    if (iS==0) scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T-3,s.MP->T),1,3));

    if (useConstraints) {
      s.optConstraintsParam = true;
      t = s.MP->addTask("contact", new PointEqualityConstraint(*s.world, "endeff",NoVector, "targetC",NoVector));
      t->setCostSpecs(s.MP->T*.5, s.MP->T*.5, ARR(0), 1.);
    } else {
      s.optConstraintsParam = false;
    }

    scenario.scenes.append(s);
    scenario.paramGT = param;
    scenario.setParam(param);

    arr x,lambda;
    execMotion(s,x,lambda,NoArr,MT::getParameter<bool>("IMP/visDemo"));

    scenario.scenes.last().xDem = x;
    scenario.scenes.last().lambdaDem = lambda;
    scenario.setParam(ones(param.d0)); // reset parameters
  }
}

void MotionFactory::loadScenarioTestRbf(Scenario &scenario) {
  Scene s;

  /// create world and motion problem
  s.world = new ors::KinematicWorld("sceneTest");
  s.world->swift();

  s.MP = new MotionProblem(*s.world,false);
  s.MP->useSwift=true;
  s.MP->T = 50;
  s.MP->tau = 0.01;
  s.x0 = s.world->getJointState();

  ors::Shape *grasp = s.world->getShapeByName("endeff");
  ors::Body *targetE = s.world->getBodyByName("targetE");
  ors::Body *targetC = s.world->getBodyByName("targetC");

  arr paramDemo = ARRAY(1e-1,1e2,1e2);
  paramDemo = paramDemo/sum(paramDemo)*scenario.costScale;

  // transition costs
  Task *t;
  t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
  t->map.order=2;
  t->target = ARRAY(0.);
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
  scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));


  t =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  t->target = ARRAY(targetE->X.pos);
  scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T-3,s.MP->T),1,3));

  t =s.MP->addTask("pos2", new DefaultTaskMap(posTMT, grasp->index) );
  t->target = ARRAY(targetC->X.pos);
  scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T*.5-3,s.MP->T*.5+3),1,3));

  s.optConstraintsParam = false;

  scenario.scenes.append(s);
  scenario.setParam(paramDemo);

  arr x,lambda;
  execMotion(s,x,lambda,NoArr,MT::getParameter<bool>("IMP/visDemo"));

  scenario.setParam(ones(paramDemo.d0)); // reset parameters
  scenario.scenes.last().xDem = x;
  scenario.scenes.last().lambdaDem = lambda;


  // change last parametrization to RBF
  uint N_RBF = 40;
  arr paramTest;
  paramTest.append(paramDemo.subRange(0,1));
  paramTest.append(ones(N_RBF));
  scenario.weights.last() = CostWeight(CostWeight::RBF,N_RBF,ARR(N_RBF,0,s.MP->T,.3),s.MP->T,3,ARR(1e-3,1e3));
  scenario.scenes.last().MP->taskCosts.last()->prec = ones(s.MP->T+1);
  scenario.paramGT = paramTest;
}


void MotionFactory::loadScenarioTestFeatSelect(Scenario &scenario) {
  Scene s;

  /// create world and motion problem
  s.world = new ors::KinematicWorld("sceneFeatSelect");
  s.world->swift();

  s.MP = new MotionProblem(*s.world,false);
  s.MP->useSwift=true;
  s.MP->T = 50;
  s.MP->tau = 0.01;
  s.x0 = s.world->getJointState();

  ors::Shape *grasp = s.world->getShapeByName("endeff");

  arr param = ARRAY(1e-1);

  // transition costs
  Task *t;
  t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
  t->map.order=1;
  t->target = ARRAY(0.);
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
  scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));

  param.append(zeros(8*4));
  arr timepoints = linspace(20,s.MP->T,3); timepoints.flatten();
  for (uint i=0;i<timepoints.d0;i++) {
    for (uint j=1;j<9;j++) {
      MT::String str; str << "t" <<j<<i;
      MT::String str2; str2 << "t" <<j;
      t =s.MP->addTask(str, new DefaultTaskMap(posTMT, grasp->index) );
      t->target = ARRAY(s.world->getBodyByName(str2)->X.pos);
      scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(timepoints(i),timepoints(i)),1,3));
      t->active=false;
    }
  }
  s.MP->taskCosts(8*4)->active=true;
  param(8*4)=1e2;
  s.MP->taskCosts(1)->active=true;
  param(1)=1e2;
  s.MP->taskCosts(8+4)->active=true;
  param(8+4)=1e2;
  s.MP->taskCosts(2*8+5)->active=true;
  param(2*8+5)=1e2;

  param = param/sum(param)*scenario.costScale;


  s.optConstraintsParam = false;

  scenario.scenes.append(s);
  scenario.paramGT = param;
  scenario.setParam(param);

  arr x,lambda;
  execMotion(s,x,lambda,NoArr,MT::getParameter<bool>("IMP/visDemo"));

  scenario.scenes.last().xDem = x;
  scenario.scenes.last().lambdaDem = lambda;
  scenario.setParam(ones(param.d0)); // reset parameters
  scenario.scenes.last().MP->activateAllTaskCosts();
}


void MotionFactory::loadDemonstration(arr &x,arr &lambda, MotionProblem &MP) {
  MotionProblem b = MP;
  Task *t;
  t = b.addTask("tra", new TransitionTaskMap(b.world));
  t->map.order=2;
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
  t->setCostSpecs(0.,b.T,ARRAY(0.),1e-2);
  ors::Shape *grasp = b.world.getShapeByName("endeff");
  t = b.addTask("t1", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(20.,20.,ARRAY(b.world.getBodyByName("t1")->X.pos),1e2);
  t = b.addTask("t4", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(30.,30.,ARRAY(b.world.getBodyByName("t4")->X.pos),1e2);
  t = b.addTask("t5", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(40.,40.,ARRAY(b.world.getBodyByName("t5")->X.pos),1e2);
  t = b.addTask("t8", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(50.,50.,ARRAY(b.world.getBodyByName("t8")->X.pos),1e2);
  x = repmat(~b.x0,b.T+1,1);
  MotionProblemFunction MPF(b);
  optConstrainedMix(x, lambda, Convert(MPF), OPT(verbose=0, stopIters=100, maxStep=1., stopTolerance = 1e-5));
  displayTrajectory(x,b.T,b.world,"world");
}

void MotionFactory::loadScenarioTestDemonstrations(Scenario &scenario) {
  arr x,lambda;

  Scene s;

  /// create world and motion problem
  s.world = new ors::KinematicWorld("sceneFeatSelect");
  s.world->swift();

  s.MP = new MotionProblem(*s.world,false);
  s.MP->useSwift=true;
  s.MP->T = 50;
  s.MP->tau = 0.1;
  s.x0 = s.world->getJointState();

  loadDemonstration(x,lambda,*s.MP);

  ors::Shape *grasp = s.world->getShapeByName("endeff");
  arr param = ARRAY(1e-1);
  // transition costs
  Task *t;
  t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
  t->map.order=1;
  t->target = ARRAY(0.);
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
  scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));

  param.append(zeros(8*4));
  arr timepoints = linspace(20,s.MP->T,3); timepoints.flatten();
  for (uint i=0;i<timepoints.d0;i++) {
    for (uint j=1;j<9;j++) {
      MT::String str; str << "t" <<j<<i;
      MT::String str2; str2 << "t" <<j;
      t =s.MP->addTask(str, new DefaultTaskMap(posTMT, grasp->index) );
      t->target = ARRAY(s.world->getBodyByName(str2)->X.pos);
      scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(timepoints(i),timepoints(i)),1,3));
      t->active=false;
    }
  }

  param = param/sum(param)*scenario.costScale;

  s.optConstraintsParam = false;

  scenario.scenes.append(s);
  scenario.paramGT = param;
  scenario.setParam(param);

  scenario.scenes.last().xDem = x;
  scenario.scenes.last().lambdaDem = lambda;
  scenario.setParam(ones(param.d0)); // reset parameters
  scenario.scenes.last().MP->activateAllTaskCosts();
}

void MotionFactory::loadScenarioBoxSliding(Scenario &scenario) {
  arr param, paramL;
  MT::Array<CostWeight> weightsL;

  for (uint iS=0;iS<3;iS++) {

    Scene s;

    /// create world and motion problem
    s.world = new ors::KinematicWorld("sceneBox");
    s.world->swift();

    s.MP = new MotionProblem(*s.world,false);
    s.MP->useSwift=true;
    s.MP->T = 100;
    s.MP->tau = 0.01;
    s.x0 = s.world->getJointState();

    arr ft;
    ft << FILE("targets");
    arr targets = ft[iS];

    // set some visualization properties
    s.world->getJointByName("table_box")->A.pos = ors::Vector(targets.subRange(0,2));
    s.world->getJointByName("table_box")->A.rot.setRad(targets(3)*M_PI/180);
    s.world->getJointByName("table_boxTarget")->A.pos = ors::Vector(targets.subRange(4,6));
    s.world->getJointByName("table_boxTarget")->A.rot.setRad(targets(7)*M_PI/180);
    s.world->getJointByName("table_boxTargetVis")->A = s.world->getJointByName("table_boxTarget")->A;
    s.world->getJointByName("table_boxTargetVis2")->A = s.world->getJointByName("table_boxTarget")->A;
    s.world->getJointByName("table_boxTarget")->A.pos.z += .075;
    s.world->getBodyByName("preContact")->X = s.world->getShapeByName("preContactRef")->X;
    s.world->calc_fwdPropagateFrames();

    if (iS==0){
      param = ARRAY(1e-1);//,1e3,5e3,1e3,1e3,1e2,1e2);
      paramL = ARRAY(1e-1);//,5e3,1e3,1e3,1e2,1e2);
      //  param = param/sum(param)*scenario.costScale;
    }

    // time points
    double conT = s.MP->T/2.;
    uint preT = conT-25;
    uint finT = s.MP->T;

    // transition costs
    Task *t;
    t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
    t->map.order=2;
    t->target = ARRAY(0.);
    ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
    if (iS==0) {
      scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));
      weightsL.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));
    }

    uint N_RBF;

    // task costs
    t =s.MP->addTask("pos", new DefaultTaskMap(posTMT,*s.world,"box",NoVector,"boxTarget",NoVector) );
    t->target = ARRAY(0.);
    if (iS==0) {
      N_RBF = 80;
      scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(finT,finT),1,3));
      weightsL.append(CostWeight(CostWeight::RBF,N_RBF,ARR(N_RBF,finT-25,finT,.125),finT,3));
      paramL.append(ones(N_RBF));
      param.append(ARR(1e3));
    }

    t =s.MP->addTask("vec", new DefaultTaskMap(vecAlignTMT, *s.world, "box", ors::Vector(1.,0.,0), "boxTarget",ors::Vector(1.,0.,0)) );
    t->target = ARRAY(1.);
    if (iS==0) {
      N_RBF = 80;
      scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(finT-1,finT),1,1));
      weightsL.append(CostWeight(CostWeight::RBF,N_RBF,ARR(N_RBF,finT-25,finT,.25),finT,1));
      paramL.append(ones(N_RBF));
      param.append(ARR(1e2));
    }

    t =s.MP->addTask("vecFake1", new DefaultTaskMap(vecAlignTMT, *s.world, "box", ors::Vector(0.,1.,0), "boxTarget",ors::Vector(1.,0.,0)) );
    t->target = ARRAY(1.);
    if (iS==0) {
      N_RBF = 80;
      scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(finT-1,finT),1,1));
      weightsL.append(CostWeight(CostWeight::RBF,N_RBF,ARR(N_RBF,finT-25,finT,.125),finT,1));
      paramL.append(ones(N_RBF));
      param.append(ARR(0.));
    }

    t = s.MP->addTask("posC1", new DefaultTaskMap(posTMT, *s.world, "endeffL", NoVector,"boxP1",NoVector));
    t->target = ARRAY(0.);
    if (iS==0) {
      scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(conT-5,conT-1),1,3));
      weightsL.append(CostWeight(CostWeight::Block,1,ARR(conT-5,conT-1),1,3));
      paramL.append(ARR(1.));
      param.append(ARR(1e2));
    }

    t = s.MP->addTask("posC2", new DefaultTaskMap(posTMT, *s.world, "endeffM", NoVector,"boxP2",NoVector));
    t->target = ARRAY(0.);
    if (iS==0) {
      scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(conT-5,conT-1),1,3));
      weightsL.append(CostWeight(CostWeight::Block,1,ARR(conT-5,conT-1),1,3));
      paramL.append(ARR(1.));
      param.append(ARR(1e2));
    }

    t = s.MP->addTask("posPre", new DefaultTaskMap(posTMT, *s.world, "endeffC", NoVector,"preContact",NoVector));
    t->target = ARRAY(0.);
    if (iS==0) {
      N_RBF = 20;
      scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(preT,preT),1,3));
      weightsL.append(CostWeight(CostWeight::RBF,N_RBF,ARR(N_RBF,preT-10,preT+10,.5),finT,3));
      paramL.append(ones(N_RBF));
      param.append(ARR(4e2));
    }

    t = s.MP->addTask("rotPre", new DefaultTaskMap(vecAlignTMT, *s.world, "endeffC", ors::Vector(0.,0.,1.),"preContact",ors::Vector(1.,0.,0.)));
    t->target = ARRAY(1.);
    if (iS==0) {
      N_RBF = 20;
      scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(preT,preT),1,1));
      weightsL.append(CostWeight(CostWeight::RBF,N_RBF,ARR(N_RBF,preT-10,preT+10,.75),finT,1));
      paramL.append(ones(N_RBF));
      param.append(ARR(1e1));
    }

    //    t = s.MP->addTask("posFake", new DefaultTaskMap(posTMT, *s.world, "table", NoVector,"endeffC",NoVector));
    //    t->target = ARRAY(0.);
    //    if (iS==0) {
    //      scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(conT-3,conT+3),1,3));
    //      weightsL.append(CostWeight(CostWeight::Block,1,ARR(conT-3,conT+3),1,3));
    //      paramL.append(1.);
    //      param.append(0.);
    //    }

    // constraints
    t = s.MP->addTask("contact1", new PointEqualityConstraint(*s.world, "endeffL",NoVector, "boxP1",NoVector));
    t->setCostSpecs(conT, finT, ARR(0), 1.);
    t = s.MP->addTask("contact2", new PointEqualityConstraint(*s.world, "endeffM",NoVector, "boxP2",NoVector));
    t->setCostSpecs(conT, finT, ARR(0), 1.);
    t = s.MP->addTask("box_fix1", new qItselfConstraint(s.world->getJointByName("table_box")->qIndex, s.world->getJointStateDimension()));
    t->setCostSpecs(0.,conT-1, ARR(0.), 1.);
    t = s.MP->addTask("box_fix2", new qItselfConstraint(s.world->getJointByName("table_box")->qIndex+1, s.world->getJointStateDimension()));
    t->setCostSpecs(0.,conT-1, ARR(0.), 1.);
    t = s.MP->addTask("box_fix3", new qItselfConstraint(s.world->getJointByName("table_box")->qIndex+2, s.world->getJointStateDimension()));
    t->setCostSpecs(0.,conT-1, ARR(0.), 1.);

    t = s.MP->addTask("velocity_dir", new VelAlignConstraint(*s.world, "endeffC",NoVector, "box", ors::Vector(1,0,0),.95));
    t->setCostSpecs(conT, finT, ARR(0.), 1.);
    s.optConstraintsParam = true;

//    s.optConstraintsParam = false;
    scenario.scenes.append(s);
    scenario.paramGT = param;
    scenario.setParam(param);

    arr x,lambda;
    MT::String name("boxDemo");
    name << iS;
    execMotion(s,x,lambda,NoArr,MT::getParameter<uint>("IMP/visDemo"),1,name);

    scenario.scenes.last().xDem = x;
    scenario.scenes.last().lambdaDem = lambda;
  }

  scenario.weights = weightsL;
  scenario.paramGT = paramL;
  cout << paramL << endl;
  scenario.setParam(paramL,true); // reset parameters
}


void MotionFactory::loadScenarioParamEval(Scenario &scenario,uint type) {
  Scene s;

  /// create world and motion problem
  s.world = new ors::KinematicWorld("sceneTest");
  s.world->swift();

  s.MP = new MotionProblem(*s.world,false);
  s.MP->useSwift=true;
  s.MP->T = 50;
  s.MP->tau = 0.01;
  s.x0 = s.world->getJointState();

  ors::Shape *grasp = s.world->getShapeByName("endeff");
  ors::Body *targetE = s.world->getBodyByName("targetE");

  arr param;
  if (type == 0) { // single parameter weights
    param = ARRAY(1e-3,1e3,1e3,1e3,1e3,1e3,1e3);
    param = param/sum(param)*scenario.costScale;

    // transition costs
    Task *t;
    t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
    t->map.order=2;
    t->target = ARRAY(0.);
    ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
    scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));

    t =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
    t->target = ARRAY(targetE->X.pos);
    scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(25,25),1,3));

    t =s.MP->addTask("pos2", new DefaultTaskMap(posTMT, grasp->index) );
    t->target = ARRAY(targetE->X.pos);
    scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(26,26),1,3));

    t =s.MP->addTask("pos3", new DefaultTaskMap(posTMT, grasp->index) );
    t->target = ARRAY(targetE->X.pos);
    scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(27,27),1,3));

    t =s.MP->addTask("pos4", new DefaultTaskMap(posTMT, grasp->index) );
    t->target = ARRAY(targetE->X.pos);
    scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(28,28),1,3));

    t =s.MP->addTask("pos5", new DefaultTaskMap(posTMT, grasp->index) );
    t->target = ARRAY(targetE->X.pos);
    scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(29,29),1,3));

    t =s.MP->addTask("pos6", new DefaultTaskMap(posTMT, grasp->index) );
    t->target = ARRAY(targetE->X.pos);
    scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(30,30),1,3));
  } else if (type==1) { // rbf
    param = ARRAY(1e-1);
    uint N_RBF = 30;
    param.append(ones(N_RBF));
    param = param/sum(param)*scenario.costScale;

    // transition costs
    Task *t;
    t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
    t->map.order=2;
    t->target = ARRAY(0.);
    ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
    scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));

    t =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
    t->target = ARRAY(targetE->X.pos);
    scenario.weights.append(CostWeight(CostWeight::RBF,N_RBF,ARR(N_RBF,0,s.MP->T,.8),s.MP->T,3));
  } else if (type==2) { // nonlinear gaussian
    param = ARRAY(1e-1,1e2);

//    param = param/sum(param)*scenario.costScale;

    // transition costs
    Task *t;
    t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
    t->map.order=2;
    t->target = ARRAY(0.);
    ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
    scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));

    param.append(ARR(25.,3.0));
    t =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
    t->target = ARRAY(targetE->X.pos);
    scenario.weights.append(CostWeight(CostWeight::Gaussian,3,ARR(0),s.MP->T,3));
  }
  Task *t;
  s.optConstraintsParam = true;
  t = s.MP->addTask("contact", new PointEqualityConstraint(*s.world, "endeff",NoVector, "targetC",NoVector));
  t->setCostSpecs(s.MP->T, s.MP->T, ARR(0), 1.);

  scenario.scenes.append(s);
  scenario.paramGT = param;
  scenario.setParam(param);

  arr x,lambda;
  execMotion(s,x,lambda,NoArr,MT::getParameter<bool>("IMP/visDemo"),0,MT::String("eval"));

  scenario.scenes.last().xDem = x;
  scenario.scenes.last().lambdaDem = lambda;
  scenario.setParam(ones(param.d0),true); // reset parameters
}
