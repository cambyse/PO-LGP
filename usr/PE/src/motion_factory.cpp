#include "motion_factory.h"
#include "cost_weight.h"
#include <Gui/opengl.h>

void MotionFactory::execMotion(Scene &s, arr &x, arr &lambda, arr &_x0, uint vis, uint _verbose, mlr::String name) {
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


  if (mlr::getParameter<bool>("IMP/loadDemoFromFile") && name.N>0) {
    readDemoFromFile(name,xx,ll);
  } else {
    xx = s.MP->getInitialization();//repmat(~s.MP->x0,s.MP->T+1,1);
    OptOptions o; o.verbose=_verbose; o.stopTolerance=1e-2; o.stopIters=100; o.aulaMuInc=1.01; o.maxStep=1.;
//    optConstrained(xx, ll, Convert(MPF), o);
    o.stopTolerance=mlr::getParameter<double>("MP/stopTolerance");
    optConstrained(xx, ll, Convert(MPF), o);

    s.MP->costReport(false);
    writeDemoToFile(name,xx,ll);
  }

  /// visualize
  s.world->watch();
  s.world->gl().resize(800,800);
  if (vis==1) {
    displayTrajectory(xx,s.MP->T,s.MP->world,"world"); mlr::wait(1.5);
  }else if (vis==2) {
    for(;;) {displayTrajectory(xx,s.MP->T,s.MP->world,"world"); mlr::wait(1.5);}
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


void MotionFactory::loadScenarioSimple(Scenario &scenario, uint nScenes, bool useConstraints) {
  for (uint iS=0;iS<nScenes;iS++) {
    Scene s;

    /// create world and motion problem
    s.world = new mlr::KinematicWorld("sceneTest");
    s.world->swift();

    s.world->getBodyByName("targetE")->X.pos += mlr::Vector(0.,0.,0.2)*iS;

    s.MP = new KOMO(*s.world,false);
    s.MP->useSwift=true;
    s.MP->T = 50;
    s.MP->tau = 0.01;
    s.x0 = s.world->getJointState();

    mlr::Shape *grasp = s.world->getShapeByName("endeff");
    mlr::Body *targetE = s.world->getBodyByName("targetE");


    arr param = ARR(1e-1,1e2);
    param = param/sum(param)*scenario.costScale;

    // transition costs
    Task *t;
    t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
    t->map.order=2;
    t->target = ARR(0.);
    ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
    if (iS==0) scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));

    t =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
    t->target = conv_vec2arr(targetE->X.pos);
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
    execMotion(s,x,lambda,NoArr,mlr::getParameter<uint>("IMP/visDemo"));
    cout << "lambda: " << lambda << endl;

    scenario.scenes.last().xDem = x;
    scenario.scenes.last().lambdaDem = lambda;
    scenario.setParam(ones(param.d0)); // reset parameters
  }
}

void MotionFactory::loadScenarioComplex(Scenario& scenario) {
  for (uint iS=0;iS<1;iS++) {
    Scene s;

    /// create world and motion problem
    s.world = new mlr::KinematicWorld("sceneComplex");
    s.world->swift();

    s.MP = new KOMO(*s.world,false);
    s.MP->useSwift=true;
    s.MP->T = 50;
    s.MP->tau = 0.01;
    s.x0 = s.world->getJointState();

    mlr::Shape *grasp = s.world->getShapeByName("endeff");
    mlr::Body *target0 = s.world->getBodyByName("target0");


    arr param = ARR(1e-1,1e2,1e1);
    param = param/sum(param)*scenario.costScale;

    // transition costs
    Task *t;
    t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
    t->map.order=2;
    t->target = ARR(0.);
    ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
    if (iS==0) scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));



    t =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
    t->target = conv_vec2arr(target0->X.pos);
    if (iS==0) scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T-3,s.MP->T),1,3));

    t =s.MP->addTask("vec", new DefaultTaskMap(vecAlignTMT, grasp->index,mlr::Vector(0.,0.,1.),-1,mlr::Vector(1.,0.,0.)) );
    t->target = ARR(1.);
    if (iS==0) scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T*.5,s.MP->T),1,1));

    s.optConstraintsParam = true;
    uint nC=8;
    for (uint i=1;i<=nC;i++) {
      t = s.MP->addTask(STRING("con"<<i), new PointEqualityConstraint(*s.world, "endeff",NoVector, STRING("target"<<i),NoVector));
      t->setCostSpecs(round(s.MP->T*0.5*(1.+i/double(nC))), round(s.MP->T*0.5*(1.+i/double(nC))), ARR(0), 1.);
    }
    t = s.MP->addTask(STRING("con"<<nC+1), new PointEqualityConstraint(*s.world, "endeff",NoVector, STRING("target"<<nC+1),NoVector));
    t->setCostSpecs(s.MP->T, s.MP->T, ARR(0), 1.);


    scenario.scenes.append(s);
    scenario.paramGT = param;
    scenario.setParam(param);

    arr x,lambda;
    execMotion(s,x,lambda,NoArr,mlr::getParameter<uint>("IMP/visDemo"));
    cout << "lambda: " << lambda << endl;

    scenario.scenes.last().xDem = x;
    scenario.scenes.last().lambdaDem = lambda;
    scenario.setParam(ones(param.d0)); // reset parameters
  }
}

void MotionFactory::loadScenarioTestRbf(Scenario &scenario) {
  Scene s;

  /// create world and motion problem
  s.world = new mlr::KinematicWorld("sceneTest");
  s.world->swift();

  s.MP = new KOMO(*s.world,false);
  s.MP->useSwift=true;
  s.MP->T = 50;
  s.MP->tau = 0.01;
  s.x0 = s.world->getJointState();

  mlr::Shape *grasp = s.world->getShapeByName("endeff");
  mlr::Body *targetE = s.world->getBodyByName("targetE");
  mlr::Body *targetC = s.world->getBodyByName("targetC");

  arr paramDemo = ARR(1e-1,1e2,1e2);
  paramDemo = paramDemo/sum(paramDemo)*scenario.costScale;

  // transition costs
  Task *t;
  t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
  t->map.order=2;
  t->target = ARR(0.);
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
  scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));


  t =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  t->target = conv_vec2arr(targetE->X.pos);
  scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T-3,s.MP->T),1,3));

  t =s.MP->addTask("pos2", new DefaultTaskMap(posTMT, grasp->index) );
  t->target = conv_vec2arr(targetC->X.pos);
  scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T*.5-3,s.MP->T*.5+3),1,3));

  s.optConstraintsParam = false;

  scenario.scenes.append(s);
  scenario.setParam(paramDemo);

  arr x,lambda;
  execMotion(s,x,lambda,NoArr,mlr::getParameter<uint>("IMP/visDemo"));

  scenario.setParam(ones(paramDemo.d0)); // reset parameters
  scenario.scenes.last().xDem = x;
  scenario.scenes.last().lambdaDem = lambda;


  // change last parametrization to RBF
  uint N_RBF = 40;
  arr paramTest;
  paramTest.append(paramDemo.subRange(0,1));
  paramTest.append(ones(N_RBF));
  scenario.weights.last() = CostWeight(CostWeight::RBF,N_RBF,ARR(N_RBF,0,s.MP->T,.3),s.MP->T,3,ARR(1e-3,1e3));
  scenario.scenes.last().MP->tasks.last()->prec = ones(s.MP->T+1);
  scenario.paramGT = paramTest;
}


void MotionFactory::loadScenarioTestFeatSelect(Scenario &scenario) {
  Scene s;

  /// create world and motion problem
  s.world = new mlr::KinematicWorld("sceneFeatSelect");
  s.world->swift();

  s.MP = new KOMO(*s.world,false);
  s.MP->useSwift=true;
  s.MP->T = 50;
  s.MP->tau = 0.01;
  s.x0 = s.world->getJointState();

  mlr::Shape *grasp = s.world->getShapeByName("endeff");

  arr param = ARR(1e-1);

  // transition costs
  Task *t;
  t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
  t->map.order=1;
  t->target = ARR(0.);
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
  scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));

  param.append(zeros(8*4));
  arr timepoints = linspace(20,s.MP->T,3); timepoints.flatten();
  for (uint i=0;i<timepoints.d0;i++) {
    for (uint j=1;j<9;j++) {
      mlr::String str; str << "t" <<j<<i;
      mlr::String str2; str2 << "t" <<j;
      t =s.MP->addTask(str, new DefaultTaskMap(posTMT, grasp->index) );
      t->target = conv_vec2arr(s.world->getBodyByName(str2)->X.pos);
      scenario.weights.append(CostWeight(CostWeight::Block,1,ARRAY(timepoints(i),timepoints(i)),1,3));
      t->active=false;
    }
  }
  s.MP->tasks(8*4)->active=true;
  param(8*4)=1e2;
  s.MP->tasks(1)->active=true;
  param(1)=1e2;
  s.MP->tasks(8+4)->active=true;
  param(8+4)=1e2;
  s.MP->tasks(2*8+5)->active=true;
  param(2*8+5)=1e2;

  param = param/sum(param)*scenario.costScale;


  s.optConstraintsParam = false;

  scenario.scenes.append(s);
  scenario.paramGT = param;
  scenario.setParam(param);

  arr x,lambda;
  execMotion(s,x,lambda,NoArr,mlr::getParameter<uint>("IMP/visDemo"));

  scenario.scenes.last().xDem = x;
  scenario.scenes.last().lambdaDem = lambda;
  scenario.setParam(ones(param.d0)); // reset parameters
  scenario.scenes.last().MP->activateAllTaskCosts();
}


void MotionFactory::loadDemonstration(arr &x,arr &lambda, KOMO &MP) {
  KOMO b = MP;
  Task *t;
  t = b.addTask("tra", new TransitionTaskMap(b.world));
  t->map.order=2;
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
  t->setCostSpecs(0.,b.T,ARR(0.),1e-2);
  mlr::Shape *grasp = b.world.getShapeByName("endeff");
  t = b.addTask("t1", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(20.,20.,conv_vec2arr(b.world.getBodyByName("t1")->X.pos),1e2);
  t = b.addTask("t4", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(30.,30.,conv_vec2arr(b.world.getBodyByName("t4")->X.pos),1e2);
  t = b.addTask("t5", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(40.,40.,conv_vec2arr(b.world.getBodyByName("t5")->X.pos),1e2);
  t = b.addTask("t8", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(50.,50.,conv_vec2arr(b.world.getBodyByName("t8")->X.pos),1e2);
  x = repmat(~b.x0,b.T+1,1);
  MotionProblemFunction MPF(b);
  optConstrained(x, lambda, Convert(MPF), OPT(verbose=0, stopIters=100, maxStep=1., stopTolerance = 1e-5));
  displayTrajectory(x,b.T,b.world,"world");
}

void MotionFactory::loadScenarioTestDemonstrations(Scenario &scenario) {
  arr x,lambda;

  Scene s;

  /// create world and motion problem
  s.world = new mlr::KinematicWorld("sceneFeatSelect");
  s.world->swift();

  s.MP = new KOMO(*s.world,false);
  s.MP->useSwift=true;
  s.MP->T = 50;
  s.MP->tau = 0.1;
  s.x0 = s.world->getJointState();

  loadDemonstration(x,lambda,*s.MP);

  mlr::Shape *grasp = s.world->getShapeByName("endeff");
  arr param = ARR(1e-1);
  // transition costs
  Task *t;
  t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
  t->map.order=1;
  t->target = ARR(0.);
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
  scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));

  param.append(zeros(8*4));
  arr timepoints = linspace(20,s.MP->T,3); timepoints.flatten();
  for (uint i=0;i<timepoints.d0;i++) {
    for (uint j=1;j<9;j++) {
      mlr::String str; str << "t" <<j<<i;
      mlr::String str2; str2 << "t" <<j;
      t =s.MP->addTask(str, new DefaultTaskMap(posTMT, grasp->index) );
      t->target = conv_vec2arr(s.world->getBodyByName(str2)->X.pos);
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
  mlr::Array<CostWeight> weightsL;

  for (uint iS=0;iS<3;iS++) {

    Scene s;

    /// create world and motion problem
    s.world = new mlr::KinematicWorld("sceneBox");
    s.world->swift();

    s.MP = new KOMO(*s.world,false);
    s.MP->useSwift=true;
    s.MP->T = 100;
    s.MP->tau = 0.01;
    s.x0 = s.world->getJointState();

    arr ft;
    ft << FILE("targets");
    arr targets = ft[iS];

    // set some visualization properties
    s.world->getJointByName("table_box")->A.pos = mlr::Vector(targets.subRange(0,2));
    s.world->getJointByName("table_box")->A.rot.setRad(targets(3)*M_PI/180);
    s.world->getJointByName("table_boxTarget")->A.pos = mlr::Vector(targets.subRange(4,6));
    s.world->getJointByName("table_boxTarget")->A.rot.setRad(targets(7)*M_PI/180);
    s.world->getJointByName("table_boxTargetVis")->A = s.world->getJointByName("table_boxTarget")->A;
    s.world->getJointByName("table_boxTargetVis2")->A = s.world->getJointByName("table_boxTarget")->A;
    s.world->getJointByName("table_boxTarget")->A.pos.z += .075;
    s.world->getBodyByName("preContact")->X = s.world->getShapeByName("preContactRef")->X;
    s.world->calc_fwdPropagateFrames();

    if (iS==0){
      param = ARR(1e-1);//,1e3,5e3,1e3,1e3,1e2,1e2);
      paramL = ARR(1e-1);//,5e3,1e3,1e3,1e2,1e2);
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
    t->target = ARR(0.);
    ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
    if (iS==0) {
      scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));
      weightsL.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));
    }

    uint N_RBF;

    // task costs
    t =s.MP->addTask("pos", new DefaultTaskMap(posTMT,*s.world,"box",NoVector,"boxTarget",NoVector) );
    t->target = ARR(0.);
    if (iS==0) {
      N_RBF = 80;
      scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(finT,finT),1,3));
      weightsL.append(CostWeight(CostWeight::RBF,N_RBF,ARR(N_RBF,finT-25,finT,.125),finT,3));
      paramL.append(ones(N_RBF));
      param.append(ARR(1e3));
    }

    t =s.MP->addTask("vec", new DefaultTaskMap(vecAlignTMT, *s.world, "box", mlr::Vector(1.,0.,0), "boxTarget",mlr::Vector(1.,0.,0)) );
    t->target = ARR(1.);
    if (iS==0) {
      N_RBF = 80;
      scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(finT-1,finT),1,1));
      weightsL.append(CostWeight(CostWeight::RBF,N_RBF,ARR(N_RBF,finT-25,finT,.25),finT,1));
      paramL.append(ones(N_RBF));
      param.append(ARR(1e2));
    }

    t =s.MP->addTask("vecFake1", new DefaultTaskMap(vecAlignTMT, *s.world, "box", mlr::Vector(0.,1.,0), "boxTarget",mlr::Vector(1.,0.,0)) );
    t->target = ARR(1.);
    if (iS==0) {
      N_RBF = 80;
      scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(finT-1,finT),1,1));
      weightsL.append(CostWeight(CostWeight::RBF,N_RBF,ARR(N_RBF,finT-25,finT,.125),finT,1));
      paramL.append(ones(N_RBF));
      param.append(ARR(0.));
    }

    t = s.MP->addTask("posC1", new DefaultTaskMap(posTMT, *s.world, "endeffL", NoVector,"boxP1",NoVector));
    t->target = ARR(0.);
    if (iS==0) {
      scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(conT-5,conT-1),1,3));
      weightsL.append(CostWeight(CostWeight::Block,1,ARR(conT-5,conT-1),1,3));
      paramL.append(ARR(1.));
      param.append(ARR(1e2));
    }

    t = s.MP->addTask("posC2", new DefaultTaskMap(posTMT, *s.world, "endeffM", NoVector,"boxP2",NoVector));
    t->target = ARR(0.);
    if (iS==0) {
      scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(conT-5,conT-1),1,3));
      weightsL.append(CostWeight(CostWeight::Block,1,ARR(conT-5,conT-1),1,3));
      paramL.append(ARR(1.));
      param.append(ARR(1e2));
    }

    t = s.MP->addTask("posPre", new DefaultTaskMap(posTMT, *s.world, "endeffC", NoVector,"preContact",NoVector));
    t->target = ARR(0.);
    if (iS==0) {
      N_RBF = 20;
      scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(preT,preT),1,3));
      weightsL.append(CostWeight(CostWeight::RBF,N_RBF,ARR(N_RBF,preT-10,preT+10,.5),finT,3));
      paramL.append(ones(N_RBF));
      param.append(ARR(4e2));
    }

    t = s.MP->addTask("rotPre", new DefaultTaskMap(vecAlignTMT, *s.world, "endeffC", mlr::Vector(0.,0.,1.),"preContact",mlr::Vector(1.,0.,0.)));
    t->target = ARR(1.);
    if (iS==0) {
      N_RBF = 20;
      scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(preT,preT),1,1));
      weightsL.append(CostWeight(CostWeight::RBF,N_RBF,ARR(N_RBF,preT-10,preT+10,.75),finT,1));
      paramL.append(ones(N_RBF));
      param.append(ARR(1e1));
    }

    //    t = s.MP->addTask("posFake", new DefaultTaskMap(posTMT, *s.world, "table", NoVector,"endeffC",NoVector));
    //    t->target = ARR(0.);
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

    t = s.MP->addTask("velocity_dir", new VelAlignConstraint(*s.world, "endeffC",NoVector, "box", mlr::Vector(1,0,0),.95));
    t->setCostSpecs(conT, finT, ARR(0.), 1.);
    s.optConstraintsParam = true;

    //    s.optConstraintsParam = false;
    scenario.scenes.append(s);
    scenario.paramGT = param;
    scenario.setParam(param);

    arr x,lambda;
    mlr::String name("boxDemo");
    name << iS;
    execMotion(s,x,lambda,NoArr,mlr::getParameter<uint>("IMP/visDemo"),1,name);

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
  s.world = new mlr::KinematicWorld("sceneTest");
  s.world->swift();

  s.MP = new KOMO(*s.world,false);
  s.MP->useSwift=true;
  s.MP->T = 50;
  s.MP->tau = 0.01;
  s.x0 = s.world->getJointState();

  mlr::Shape *grasp = s.world->getShapeByName("endeff");
  mlr::Body *targetE = s.world->getBodyByName("targetE");

  arr param;
  if (type == 0) { // single parameter weights
    param = ARR(1e-3,1e3,1e3,1e3,1e3,1e3,1e3);
    param = param/sum(param)*scenario.costScale;

    // transition costs
    Task *t;
    t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
    t->map.order=2;
    t->target = ARR(0.);
    ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
    scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));

    t =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
    t->target = conv_vec2arr(targetE->X.pos);
    scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(25,25),1,3));

    t =s.MP->addTask("pos2", new DefaultTaskMap(posTMT, grasp->index) );
    t->target = conv_vec2arr(targetE->X.pos);
    scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(26,26),1,3));

    t =s.MP->addTask("pos3", new DefaultTaskMap(posTMT, grasp->index) );
    t->target = conv_vec2arr(targetE->X.pos);
    scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(27,27),1,3));

    t =s.MP->addTask("pos4", new DefaultTaskMap(posTMT, grasp->index) );
    t->target = conv_vec2arr(targetE->X.pos);
    scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(28,28),1,3));

    t =s.MP->addTask("pos5", new DefaultTaskMap(posTMT, grasp->index) );
    t->target = conv_vec2arr(targetE->X.pos);
    scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(29,29),1,3));

    t =s.MP->addTask("pos6", new DefaultTaskMap(posTMT, grasp->index) );
    t->target = conv_vec2arr(targetE->X.pos);
    scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(30,30),1,3));
  } else if (type==1) { // rbf
    param = ARR(1e-1);
    uint N_RBF = 30;
    param.append(ones(N_RBF));
    param = param/sum(param)*scenario.costScale;

    // transition costs
    Task *t;
    t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
    t->map.order=2;
    t->target = ARR(0.);
    ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
    scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));

    t =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
    t->target = conv_vec2arr(targetE->X.pos);
    scenario.weights.append(CostWeight(CostWeight::RBF,N_RBF,ARR(N_RBF,0,s.MP->T,.8),s.MP->T,3));
  } else if (type==2) { // nonlinear gaussian
    param = ARR(1e-1,1e2);

    //    param = param/sum(param)*scenario.costScale;

    // transition costs
    Task *t;
    t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
    t->map.order=2;
    t->target = ARR(0.);
    ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
    scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));

    param.append(ARR(25.,3.0));
    t =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
    t->target = conv_vec2arr(targetE->X.pos);
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
  execMotion(s,x,lambda,NoArr,mlr::getParameter<uint>("IMP/visDemo"),0,mlr::String("eval"));

  scenario.scenes.last().xDem = x;
  scenario.scenes.last().lambdaDem = lambda;
  scenario.setParam(ones(param.d0),true); // reset parameters
}


void MotionFactory::loadScenarioButton(Scenario &scenario,mlr::KinematicWorld &world) {
  arr x,lambda;
  Scene s;
  /// create world and motion problem
  mlr::String folder = mlr::getParameter<mlr::String>("folder");
  double duration = mlr::getParameter<double>("duration");

  uint bIdx,id;
  bIdx << FILE(STRING(folder<<"/bestIdx_Dof_CBO.dat"));
  id << FILE(STRING(folder<<"/Dof_id.dat"));
  arr cboX;
  cboX << FILE(STRING(folder<<id<<"_Dof_CBO_X.dat"));

  s.world = &world;
  s.MP = new KOMO(*s.world,false);

  /// load demo from file
  s.xDem = FILE(STRING(folder<<bIdx<<"_Dof_Xplan.dat"));
  s.MP->T = s.xDem.d0-1;
  s.MP->tau = duration/s.MP->T;
  s.x0 = s.xDem[0];

  /// compute feature trajectories
  TrajFactory tf;
  arr posC1,velC1,cp1;
  tf.compFeatTraj(s.xDem,posC1,*s.world,new DefaultTaskMap(posTMT,*s.world,"endeffC1"));
  tf.compFeatTraj(s.xDem,cp1,*s.world,new DefaultTaskMap(posTMT,*s.world,"cp1"));
  getVel(velC1,posC1-cp1,s.MP->tau);

  arr conTime = FILE(STRING(folder<<"constraintTime.dat")); conTime.flatten();
  arr conTime_inv = -1.*(conTime-1.);

  uint b2_b1_idx = s.world->getJointByName("b2_b1")->qIndex;

  arr tmp = s.xDem.col(b2_b1_idx); tmp.flatten();
  double b2_b1Min = tmp.min();
  double b2_b1Max = tmp.max();

  uint mIdx = tmp.minIndex();

  double bParam = -0.01078487 + cboX(bIdx-1,0);//b2_b1Min-0.005;//cboX(bIdx-1,0);// + b2_b1Min;


  mlr::Array<uint> conStart;
  conStart << FILE(STRING(folder<<"conStart.dat"));
  mlr::Array<uint> conEnd = FILE(STRING(folder<<"conEnd.dat"));

  arr param = ARR(1e-1,1e1,1e1,1e1,1e1,1e1,1e0,1e0);
  // transition costs
  Task *t;
  t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
  t->map.order = 2;
  t->target = ARR(0.);
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
  scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension(),ARR(5e-2,5e-2)));

  s.optConstraintsParam = true;
  // add features
  arr tmp1,tmp2,tmp3,tmp4,tmp5,tmp6,tmp7,tmp8,tmp9;

  // orientation at contact start, downwards
  s.world->setJointState(s.xDem[conStart(0)]);
  s.world->kinematicsVec(tmp1,NoArr,s.world->getShapeByName("endeffC1")->body,mlr::Vector(1.,0.,0.));
  tmp1 += 1e-1*randn(3);
  tmp1 = tmp1/length(tmp1);
  t =s.MP->addTask("vecConStart", new DefaultTaskMap(vecAlignTMT, *s.world,"endeffC1",mlr::Vector(1.,0.,0.),NULL,mlr::Vector(tmp1)) );
  t->target = ARR(1.);
  scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(conStart(0),conStart(0)),1,1));
  // orientation at contact end, downwards
  s.world->setJointState(s.xDem[conEnd(0)]);
  s.world->kinematicsVec(tmp2,NoArr,s.world->getShapeByName("endeffC1")->body,mlr::Vector(1.,0.,0.));
  tmp2 += 1e0*randn(3);
  tmp2 = tmp2/length(tmp2);
  t = s.MP->addTask("vecConEnd", new DefaultTaskMap(vecAlignTMT, *s.world,"endeffC1",mlr::Vector(1.,0.,0.),NULL,mlr::Vector(tmp2)) );
  t->target = ARR(1.);
  scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(conEnd(0),conEnd(0)),1,1));
  // pre contact position
  uint preconT = conStart(0)-10;
  s.world->setJointState(s.xDem[preconT]);
  t =s.MP->addTask("posPreCon", new DefaultTaskMap(posTMT, *s.world,"endeffC1") );
  t->map.phi(tmp3,NoArr,*s.world);
  t->target = conv_vec2arr(tmp3+1e-2*randn(3));
  scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(preconT,preconT),1,3));
  // post contact position
  uint postconT = conEnd(0)+10;
  s.world->setJointState(s.xDem[postconT]);
  t =s.MP->addTask("posPostCon", new DefaultTaskMap(posTMT, *s.world,"endeffC1") );
  t->map.phi(tmp4,NoArr,*s.world);
  t->target = conv_vec2arr(tmp4+1e-2*randn(3));
  scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(postconT,postconT),1,3));
  // final position
  s.MP->world.setJointState(s.xDem[s.MP->T]);
  t = s.MP->addTask("posFinal", new DefaultTaskMap(posTMT, *s.world,"endeffC1") );
  t->map.phi(tmp5,NoArr,*s.world);
  t->target = conv_vec2arr(tmp5+1e-2*randn(3));
  scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T-20,s.MP->T),1,3));
  // contact velocity start
  s.MP->world.setJointState(s.xDem[conStart(0)]);
  t =s.MP->addTask("velCStart", new DefaultTaskMap(posTMT, *s.world,"endeffC1",NoVector,"cp1") );
  t->map.order=1;
  t->target = velC1[conStart(0)];//ARR(0.,0.,0.);//conv_vec2arr(tmp6+1e-5*randn(3));
  scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(conStart(0),conStart(0)),1,3));
  // contact velocity end
  s.MP->world.setJointState(s.xDem[mIdx]);
  t =s.MP->addTask("velCEnd", new DefaultTaskMap(posTMT, *s.world,"endeffC1",NoVector,"cp1") );
  t->map.order=1;
  t->target = velC1[mIdx];// ARR(0.,0.,0.);//conv_vec2arr(tmp7+1e-5*randn(3));
  scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(mIdx,mIdx),1,3));


  s.world->setJointState(s.xDem[0]);
  // constraint button target position
  t = s.MP->addTask("dof_target", new qItselfConstraint(b2_b1_idx, s.world->getJointStateDimension()));
  t->setCostSpecs(mIdx-10,mIdx+10, ARR(bParam), 1.);
  // constraint button position before and after manipulation
  t = s.MP->addTask("dof_fixation", new qItselfConstraint(b2_b1_idx, s.world->getJointStateDimension()));
  t->target = ARR(s.xDem(0,b2_b1_idx));
  t->prec = conTime_inv;
  // constraint contact point on button during manipulation
  t = s.MP->addTask("contact", new PointEqualityConstraint(*s.world, "endeffC1",NoVector, "cp1",NoVector));
  t->setCostSpecs(conStart,conEnd, ARR(0.), 1.);
  // dof limits
  t = s.MP->addTask("q_limit", new qItselfLimit(s.world->getJointByName("b2_b1")->qIndex, min(ARR(bParam,b2_b1Min)),max(ARR(bParam,b2_b1Max))));// bParam,s.xDem(0,b2_b1_idx)));
  t->setCostSpecs(conStart,conEnd, ARR(0.), 1.);

  s.world->setJointState(s.xDem[0]);

  scenario.scenes.append(s);
  scenario.paramGT = param;
  scenario.setParam(param);
  //  execMotion(s,x,lambda,NoArr,mlr::getParameter<uint>("IMP/visDemo"),1);

  scenario.setParam(ones(param.d0)); // reset parameters

  param = param/sum(param)*scenario.costScale;
}

void MotionFactory::loadScenarioVelocity(Scenario& scenario) {
  for (uint iS=0;iS<1;iS++) {
    Scene s;

    /// create world and motion problem
    s.world = new mlr::KinematicWorld("sceneTest");
    s.world->swift();

    s.world->getBodyByName("targetE")->X.pos += mlr::Vector(0.,0.,0.2)*iS;

    s.MP = new KOMO(*s.world,false);
    s.MP->useSwift=true;
    s.MP->T = 50;
    s.MP->tau = 0.01;
    s.x0 = s.world->getJointState();

    mlr::Shape *grasp = s.world->getShapeByName("endeff");
    mlr::Body *targetE = s.world->getBodyByName("targetE");


    arr param = ARR(1e-1,1e2,1e1);
    param = param/sum(param)*scenario.costScale;

    // transition costs
    Task *t;
    t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
    t->map.order=2;
    t->target = ARR(0.);
    ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
    if (iS==0) scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));

    t =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
    t->target = conv_vec2arr(targetE->X.pos);
    if (iS==0) scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T-3,s.MP->T),1,3));

    t =s.MP->addTask("vel", new DefaultTaskMap(posTMT, grasp->index) );
    t->map.order = 1;
    t->target = ARR(0.,0.,0.);
    if (iS==0) scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T*.5-5,s.MP->T*.5+5),1,3));

    scenario.scenes.append(s);
    scenario.paramGT = param;
    scenario.setParam(param);

    arr x,lambda;
    execMotion(s,x,lambda,NoArr,mlr::getParameter<uint>("IMP/visDemo"));
    cout << "lambda: " << lambda << endl;

    scenario.scenes.last().xDem = x;
    scenario.scenes.last().lambdaDem = lambda;
    scenario.setParam(ones(param.d0)); // reset parameters
  }
}
