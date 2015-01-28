#include "motion_factory.h"

#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>
#include <Ors/ors_swift.h>
#include <Core/geo.h>
#include <Motion/motion.h>


void MotionFactory::execMotion(IKMO &ikmo,Scene &s, arr param, bool vis, uint verbose, arr &x) {
  param = costScale*param/length(param);
  ikmo.setParam(*s.MP,param);

  // reset motion problem
//  s.world->swift().initActivations(*s.world);
  s.MP->prefix.clear();
  s.MP->phiMatrix.clear();
  s.MP->ttMatrix.clear();

  // add transition costs
  if (!ikmo.optLearnTransParam) {
    Task *t;
    t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
    t->map.order=1;
    t->setCostSpecs(0, s.MP->T, ARR(0.), 1e-1*ikmo.costScale);
  }

  s.MP->x0 = s.xDem[0];
  s.MP->world.setJointState(s.xDem[0]);
//  cout << "x0: " << s.MP->x0 << endl;

  // optimize the motion problem
  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
//  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<s.MP->world.getJointStateDimension()<<endl;
  /*arr x(T+1,n); x.setZero();*/arr lambda;
  x.resize(T+1,n);
  x = repmat(~s.MP->x0,T+1,1);
//  x = randn(T+1,n);
//  x = s.xInit;
  optConstrainedMix(x, lambda, Convert(MPF), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-3));


  // visualize trajectory
  if (vis) {
    if (optConstraintsParam)
//      cout <<  "lambda: " << lambda << endl;
    s.MP->costReport(true);
//    for (;;)
      displayTrajectory(x,s.MP->T,s.MP->world,"world");
  }
}


void MotionFactory::createScenes(uint sID,MT::Array<Scene> &trainScenes, MT::Array<Scene> &testScenes, MT::Array<CostWeight> &weights)
{
  vis = MT::getParameter<uint>("visDemo");
  nS = MT::getParameter<uint>("numScenes");
  optConstraintsParam = MT::getParameter<bool>("optConstraintsParam");

  // create training scenes
   MT::Array<CostWeight> w;
  for (uint i = 0; i < nS; i++) {
    Scene s_train;
    Scene s_test;
    switch(sID) {
      case 0:
        createSceneTest(s_train,w,i);
//        createScene0(s_test,weights,i+nS);
        break;
      case 1:
        createScenePR2(s_train,w,i);
        break;
      case 2:
        createSceneTestRBF(s_train,w,i);
//        createSceneTestNonlinear(s_train,weights,i);
        /// recovery of Gaussian (1 parameter)
//        createScene1(s_train,weights,i);
//        createScene1(s_test,weights,i+nS);
        /// recovery of Gaussian (2 parameter)
//        createScene2(s_train,weights,i);
//        createScene2(s_test,weights,i+nS);
        break;
      case 3:
        /// learning
        createSceneTestGaussian(s_train,w,i);
        break;
      case 4:
        /// pr2 closing a drawer
//        createScene4(s_train,weights,1);
//        createScene4(s_test,weights,3);
        break;
      case 5:
        /// nonlinear simple example
//        createScene5(s_train,weights,i);
//        createScene5(s_test,weights,i+nS);
        break;
      case 6:
        createSceneBoxSliding(s_train,w,i);
        break;
    }
    s_test.xInit = s_train.xDem;
    s_train.xInit = s_train.xDem;

    trainScenes.append(s_train);
//    testScenes.append(s_test);
  }

  weights = w.subRange(0,w.d0/nS-1);

  // compute number of parameters
  numParam = 0;
  for (uint c=0;c<trainScenes(0).MP->taskCosts.N;c++) {
    if (trainScenes(0).MP->taskCosts(c)->map.type==sumOfSqrTT) {
      numParam++;
    }
  }
  cout << "IOC Number of Tasks: " << numParam << endl;
}


void MotionFactory::createSceneTest(Scene &s, MT::Array<CostWeight> &weights, uint i) {
  uint optTestParam = MT::getParameter<uint>("optTestParam");

  s.world = new ors::KinematicWorld("sceneTest");
  arr q, qdot;
  s.world->getJointState(q, qdot);
  makeConvexHulls(s.world->shapes);
  s.world->swift().setCutoff(10.);

  s.world->swift();
  s.MP = new MotionProblem(*s.world,false);
  s.MP->useSwift=true;
  s.MP->T = 50;
  s.MP->tau = 0.01;

  //-- setup new motion problem
  s.world->getBodyByName("target")->X.pos += double(i)*ARR(0.,0.2,0.);

  ors::Shape *grasp = s.world->getShapeByName("endeff");
  ors::Body *tar = s.world->getBodyByName("target");

  /// Set task costs
  arr param = ARRAY(.5,1e3,1e3,1e3);
  param=param.subRange(0,1+optTestParam);
  param = param/length(param)*costScale;

  uint pC = 0;
  // transition costs
  Task *t;
  t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, s.MP->T, ARR(0.), param(pC));
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
  weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));
  pC++;

  // task costs
  t =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(s.MP->T-10,s.MP->T,ARRAY(tar->X.pos),param(pC));
  weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T-10,s.MP->T),1,3));
  pC++;


  if (optTestParam>0) {
    t =s.MP->addTask("vec", new DefaultTaskMap(vecTMT, grasp->index,ors::Vector(1.,0.,0)) );
    t->setCostSpecs(s.MP->T,s.MP->T,ARRAY(0.,0.,-1.),param(pC));
    weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T),1,3));
    pC++;
    t->active=true;
  }

  if (optTestParam>1) {
    t =s.MP->addTask("pos2", new DefaultTaskMap(posTMT, grasp->index) );
    t->setCostSpecs(s.MP->T-10,s.MP->T-10,ARRAY(tar->X.pos)+ARR(0.,0.05,0),param(pC));
    weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T-10),1,3));
    pC++;
  }

  // task costs
  if (optConstraintsParam) {
    t = s.MP->addTask("contact", new PointEqualityConstraint(*s.world, "endeff",NoVector, "contactPoint",NoVector));
    t->setCostSpecs(s.MP->T*.5, s.MP->T*.5, {0.}, 1.);
  }

  s.MP->x0 = zeros(s.world->getJointStateDimension(),1);s.MP->x0.flatten();
  s.MP->x0(0) = M_PI_2;

  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<endl;
  arr x(T+1,n); x.setZero();arr lambda(T+1,1); lambda.setZero();
  x = repmat(~s.MP->x0,T+1,1);
  optConstrainedMix(x, lambda, Convert(MPF), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-4));

  s.MP->costReport(false);
  if (vis) {
    if (optConstraintsParam)
      cout <<"lambda: " << lambda << endl;
    displayTrajectory(x,s.MP->T,s.MP->world,"t");
  }

  // set all costs in s.MP equal to 0 or 1
  for (uint c=0;c < s.MP->taskCosts.N;c++) {
    if (s.MP->taskCosts(c)->map.type==sumOfSqrTT) {
      s.MP->taskCosts(c)->prec /= (s.MP->taskCosts(c)->prec+1e-12);
      s.MP->taskCosts(c)->active=true;
      cout << s.MP->taskCosts(c)->prec << endl;
    }
  }

  s.xDem = x;
  s.lambdaRef = lambda;
  s.paramRef = param;

}

void MotionFactory::createSceneBoxSliding(Scene &s, MT::Array<CostWeight> &weights, uint i) {
  s.world = new ors::KinematicWorld("sceneBox");
  arr q, qdot;

  arr ft;
  ft << FILE("targets");

  arr targets = ft[i];
  cout << ft << endl;
  // set some visualization properties
  s.world->getJointByName("table_box")->A.pos = ors::Vector(targets.subRange(0,2));
  s.world->getJointByName("table_box")->A.rot.setRad(targets(3)*M_PI/180);
  s.world->getJointByName("table_boxTarget")->A.pos = ors::Vector(targets.subRange(4,6));
  s.world->getJointByName("table_boxTarget")->A.rot.setRad(targets(7)*M_PI/180);
  s.world->getJointByName("table_boxTargetVis")->A = s.world->getJointByName("table_boxTarget")->A;
  s.world->getJointByName("table_boxTargetVis2")->A = s.world->getJointByName("table_boxTarget")->A;
  s.world->calc_fwdPropagateFrames();

//  s.world->watch(true);
  s.world->getJointState(q, qdot);
//  makeConvexHulls(s.world->shapes);
  s.world->swift().setCutoff(10.);

  s.world->swift();
  s.MP = new MotionProblem(*s.world,false);
  s.MP->useSwift=true;
  s.MP->T = 100;
  s.MP->tau = 0.01;

  /// setup new motion problem
  //-- parameter
  arr param = ARRAY(1e-1,1e3,5e3,1e3,1e3,1e2,1e2);
  param = ARRAY(0.372539, 4438.11,7240.43, 3952.09, 3449.62, 369.766, 473.039);

  param = param/length(param)*costScale;

  //-- time points
  double conT = s.MP->T/2.;
  uint pre = conT-25;
  uint pC = 0;

  Task *t;

  // transition costs
  t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, s.MP->T, ARR(0.), param(pC));
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
  weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension(),ARRAY(1e-4,1e1)));
  pC++;

  // position task maps
  t = s.MP->addTask("posT", new DefaultTaskMap(posTMT, *s.world, "box", NoVector, "boxTarget",NoVector));
  t->setCostSpecs(s.MP->T,s.MP->T, {0.}, param(pC));
  weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T),1,3));
  pC++;
  t = s.MP->addTask("vecT", new DefaultTaskMap(vecAlignTMT, *s.world, "box", ors::Vector(0.,1.,0), "boxTarget",ors::Vector(0.,1.,0)));
  t->setCostSpecs(s.MP->T,s.MP->T, {1.}, param(pC));
  weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T),1,1));
  pC++;

  t = s.MP->addTask("posC1", new DefaultTaskMap(posTMT, *s.world, "endeffL", NoVector));
  t->setCostSpecs(conT-5,conT, ARRAY(s.world->getShapeByName("boxP1")->X.pos), param(pC));
  weights.append(CostWeight(CostWeight::Block,1,ARR(conT-5,conT),1,3));
  pC++;
  t = s.MP->addTask("posC2", new DefaultTaskMap(posTMT, *s.world, "endeffM", NoVector));
  t->setCostSpecs(conT-5,conT, ARRAY(s.world->getShapeByName("boxP2")->X.pos), param(pC));
  weights.append(CostWeight(CostWeight::Block,1,ARR(conT-5,conT),1,3));
  pC++;
  t = s.MP->addTask("posPre", new DefaultTaskMap(posTMT, *s.world, "endeffM", NoVector));
  t->setCostSpecs(pre,pre, ARRAY(s.world->getShapeByName("preContact")->X.pos), param(pC));
  weights.append(CostWeight(CostWeight::Block,1,ARR(pre,pre),1,3));
  pC++;
  t = s.MP->addTask("rotPre", new DefaultTaskMap(vecAlignTMT, *s.world, "endeffC", ors::Vector(0.,0.,1.),"preContact",ors::Vector(1.,0.,0.)));
  t->setCostSpecs(pre,pre, ARRAY(1.), param(pC));
  weights.append(CostWeight(CostWeight::Block,1,ARR(pre,pre),1,1));
  pC++;

  // equality constraints
  t = s.MP->addTask("contact1", new PointEqualityConstraint(*s.world, "endeffL",NoVector, "boxP1",NoVector));
  t->setCostSpecs(conT, s.MP->T, {0.}, 1.);
  t = s.MP->addTask("contact2", new PointEqualityConstraint(*s.world, "endeffM",NoVector, "boxP2",NoVector));
  t->setCostSpecs(conT, s.MP->T, {0.}, 1.);

  t = s.MP->addTask("box_fix1", new qItselfConstraint(s.world->getJointByName("table_box")->qIndex, s.world->getJointStateDimension()));
  t->setCostSpecs(0.,conT-1, ARR(0.), 1.);
  t = s.MP->addTask("box_fix2", new qItselfConstraint(s.world->getJointByName("table_box")->qIndex+1, s.world->getJointStateDimension()));
  t->setCostSpecs(0.,conT-1, ARR(0.), 1.);
  t = s.MP->addTask("box_fix3", new qItselfConstraint(s.world->getJointByName("table_box")->qIndex+2, s.world->getJointStateDimension()));
  t->setCostSpecs(0.,conT-1, ARR(0.), 1.);


  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<endl;
  arr x = s.MP->getInitialization();
  arr lambda;
  x = repmat(~s.MP->x0,T+1,1);

  optConstrainedMix(x, lambda, Convert(MPF), OPT(verbose=2, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-3));

//  x = x+randn(x.d0,x.d1)*1e-4;

  s.MP->costReport(true);
  if (vis) {
    cout << "lambda: " <<lambda << endl;
    displayTrajectory(x,s.MP->T,s.MP->world,"t");
  }
//  t->active=false;
  // set all costs in s.MP equal to 0 or 1
  for (uint c=0;c < s.MP->taskCosts.N;c++) {
    if (s.MP->taskCosts(c)->map.type==sumOfSqrTT) {
      s.MP->taskCosts(c)->prec /= (s.MP->taskCosts(c)->prec+1e-12);
      cout << s.MP->taskCosts(c)->prec << endl;
    }
  }

  s.xDem = x;
  s.lambdaRef = lambda;
  s.paramRef = param;
}

void MotionFactory::createSceneTestRBF(Scene &s, MT::Array<CostWeight> &weights, uint i) {
  uint optTestParam = MT::getParameter<uint>("optTestParam");

  s.world = new ors::KinematicWorld("sceneTest");
  arr q, qdot;
  s.world->getJointState(q, qdot);


  s.world->swift();
  s.MP = new MotionProblem(*s.world,false);
  s.MP->T = 50;
  s.MP->tau = 0.01;

  //-- setup new motion problem
  s.world->getBodyByName("target")->X.pos += double(i)*ARR(0.,0.2,0.);

  ors::Shape *grasp = s.world->getShapeByName("endeff");
  ors::Body *tar = s.world->getBodyByName("target");
  ors::Body *tar2 = s.world->getBodyByName("target2");

  /// Set task costs
  arr param = ARRAY(1e-2,1e5);
  uint N_RBF = 100;
  param.append(ones(N_RBF));

  uint pC = 0;
  // transition costs
  Task *t;
  t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, s.MP->T, ARR(0.), param(pC));
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
  weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));
  pC++;

  // task costs 2
  t =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(s.MP->T-3,s.MP->T,ARRAY(tar2->X.pos),param(pC));
  weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T-3,s.MP->T),s.MP->T,3));
  pC++;

  // task costs
  t =s.MP->addTask("pos2", new DefaultTaskMap(posTMT, grasp->index) );
  s.MP->setInterpolatingCosts(t,MotionProblem::constant,ARRAY(tar->X.pos),0.);
  t->prec.subRange(s.MP->T-25,s.MP->T-20) = 1e5;


  weights.append(CostWeight(CostWeight::RBF,N_RBF,ARR(N_RBF,0,s.MP->T,.5),s.MP->T,3,ARR(1e-3,1e3)));
  arr w;
  weights.last().compWeights(w,NoArr,NoArr,param.subRange(pC,pC+N_RBF-1),true);
  cout << w << endl;
  pC = pC+N_RBF;



  s.MP->x0 = zeros(s.world->getJointStateDimension(),1);s.MP->x0.flatten();
  s.MP->x0(0) = M_PI_2;

  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<endl;
  arr x(T+1,n); x.setZero();arr lambda(T+1,1); lambda.setZero();
  x = repmat(~s.MP->x0,T+1,1);
  optConstrainedMix(x, lambda, Convert(MPF), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-6));

  s.MP->costReport(false);
  if (vis) {
    if (optConstraintsParam)
      cout <<"lambda: " << lambda << endl;
    displayTrajectory(x,s.MP->T,s.MP->world,"t");
  }

  // set all costs in s.MP equal to 0 or 1
  for (uint c=0;c < s.MP->taskCosts.N;c++) {
    if (s.MP->taskCosts(c)->map.type==sumOfSqrTT) {
      if (weights(c).type == CostWeight::RBF) {
        s.MP->taskCosts(c)->prec =1.;
      }else{
        s.MP->taskCosts(c)->prec /= (s.MP->taskCosts(c)->prec+1e-12);
      }
      cout << s.MP->taskCosts(c)->prec << endl;
    }
  }

  s.xDem = x;
  s.lambdaRef = lambda;
  s.paramRef = param;
}

void MotionFactory::createSceneTestGaussian(Scene &s, MT::Array<CostWeight> &weights, uint i) {
  uint optTestParam = MT::getParameter<uint>("optTestParam");

  s.world = new ors::KinematicWorld("sceneTest");
  arr q, qdot;
  s.world->getJointState(q, qdot);


  s.world->swift();
  s.MP = new MotionProblem(*s.world,false);
  s.MP->T = 50;
  s.MP->tau = 0.01;

  //-- setup new motion problem
  s.world->getBodyByName("target")->X.pos += double(i)*ARR(0.,0.2,0.);

  ors::Shape *grasp = s.world->getShapeByName("endeff");
  ors::Body *tar = s.world->getBodyByName("target");
  ors::Body *tar2 = s.world->getBodyByName("target2");

  /// Set task costs
  arr param = ARRAY(1e-2,1e4,1e4,27.);

  uint pC = 0;
  // transition costs
  Task *t;
  t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
  t->map.order=2; //make this an acceleration task!
  t->setCostSpecs(0, s.MP->T, ARR(0.), param(pC));
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
  weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));
  pC++;


  t =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(s.MP->T-3,s.MP->T,ARRAY(tar2->X.pos),param(pC));
  weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T-3,s.MP->T),s.MP->T,3));
  pC++;

  // task costs
  t =s.MP->addTask("pos2", new DefaultTaskMap(posTMT, grasp->index) );
  s.MP->setInterpolatingCosts(t,MotionProblem::constant,ARRAY(tar->X.pos),0.);
  t->prec.subRange(s.MP->T-25,s.MP->T-20) = 1e4;

  arr w;
  weights.append(CostWeight(CostWeight::Gaussian,2,ARR(3.),s.MP->T,3));
  weights.last().compWeights(w,NoArr,NoArr,ARR(param(pC),param(pC+1)),true);
  cout << w << endl;
//  t->prec=w;
  pC++;pC++;

  s.MP->x0 = zeros(s.world->getJointStateDimension(),1);s.MP->x0.flatten();
  s.MP->x0(0) = M_PI_2;

  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<endl;
  arr x(T+1,n); x.setZero();arr lambda(T+1,1); lambda.setZero();
  x = repmat(~s.MP->x0,T+1,1);
  optConstrainedMix(x, lambda, Convert(MPF), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-6));

  s.MP->costReport(false);
  if (vis) {
    if (optConstraintsParam)
      cout <<"lambda: " << lambda << endl;
    displayTrajectory(x,s.MP->T,s.MP->world,"t");
  }

  // set all costs in s.MP equal to 0 or 1
  for (uint c=0;c < s.MP->taskCosts.N;c++) {
    if (s.MP->taskCosts(c)->map.type==sumOfSqrTT) {
      if (weights(c).type == CostWeight::RBF || weights(c).type == CostWeight::Gaussian) {
        s.MP->taskCosts(c)->prec =1.;
      }else{
        s.MP->taskCosts(c)->prec /= (s.MP->taskCosts(c)->prec+1e-12);
      }
      cout << s.MP->taskCosts(c)->prec << endl;
    }
  }

  s.xDem = x;
  s.lambdaRef = lambda;
  s.paramRef = param;
}




void MotionFactory::createScenePR2(Scene &s, MT::Array<CostWeight> &weights, uint i) {
  s.world = new ors::KinematicWorld("model.kvg");

//    s.world->watch(true);
  cout << "Joints: " << endl;
  for (uint i = 0;i<s.world->joints.d0;i++){
    if (s.world->joints(i)->type != 10)
      cout << s.world->joints(i)->name << " " << s.world->joints(i)->type << " "  << s.world->joints(i)->qIndex<< endl;
  }

  cout << "###########################################################" << endl;
  /// load demonstrations data
  MT::String demoPath;
  switch (i) {
    case 0:
      demoPath = MT::getParameter<MT::String>("demoPath0");
      break;
    case 1:
      demoPath = MT::getParameter<MT::String>("demoPath1");
      break;
  }
  cout << "Demo: " << demoPath << endl;
  // load joint trajectory
  arr xDem; xDem << FILE(STRING(demoPath<<"/pr2_joints").p);

  arr options; options << FILE(STRING(demoPath<<"/options").p); options.flatten();

  arr marker_pose = zeros(20,7);
  arr markerDem; markerDem << FILE(STRING(demoPath<<"/pr2_marker0").p);
  marker_pose[4].subRange(0,2) = markerDem[0];
  marker_pose[11].subRange(0,2) = markerDem[1];
  marker_pose[15].subRange(0,2) = markerDem[2];
  marker_pose[17].subRange(0,2) = markerDem[3];

  arr refFrame = ARRAY(s.world->getBodyByName("torso_lift_link")->X.pos);
  s.world->getBodyByName("marker4")->X.pos = refFrame + marker_pose[4].subRange(0,2);
  s.world->getBodyByName("marker11")->X.pos = refFrame + marker_pose[11].subRange(0,2);
  s.world->getBodyByName("marker15")->X.pos = refFrame + marker_pose[15].subRange(0,2);
  s.world->getBodyByName("marker17")->X.pos = refFrame + marker_pose[17].subRange(0,2);

  // compute door angle
  arr v1 = ARRAY(s.world->getBodyByName("marker4")->X.pos - s.world->getBodyByName("marker15")->X.pos);
  v1(2)=0.;
  v1 = v1/sqrt(sumOfSqr(v1));
  arr v2 = ARRAY(1.,0.,0.);
  double alpha = acos(sum(v1%v2));

  s.world->getJointByName("world_door")->A.rot.setRadZ(M_PI_2-alpha);
  s.world->getJointByName("world_door")->A.pos = ARRAY(s.world->getBodyByName("marker15")->X.pos)+ s.world->getJointByName("world_door")->A.rot.getArr()*ARRAY(0.,0.6205,0.2305);
  s.world->getJointByName("world_door")->A.pos.z = .99;

  s.world->calc_fwdPropagateFrames();


  arr x0 = xDem[0];
  cout << "x0: " << x0 << endl;

  double N = s.world->getJointStateDimension();
  cout << "Number of Joints: " << N << endl;
  if (vis) { displayTrajectory(xDem,xDem.d0*0.10,*s.world,"");}

  arr xDemTaskPos, xDemTaskVec;
  arr tmpDist;
  // compute task spaces of demonstrations
  for (uint t =0;t<xDem.d0;t++) {
    s.world->setJointState(xDem[t]);
    arr tmpPos,tmpVec;
    s.world->kinematicsPos(tmpPos,NoArr,s.world->getBodyByName("l_wrist_roll_link"),&s.world->getShapeByName("endeffL")->rel.pos);
    xDemTaskPos.append(~tmpPos);
    ors::Vector vv = ors::Vector(0.,1.,0.);
    s.world->kinematicsVec(tmpVec,NoArr,s.world->getShapeByName("endeffL")->body,&vv);
    xDemTaskVec.append(~tmpVec);
    //          cout << t<< endl;
    ors::Vector tmp = tmpPos-s.world->getShapeByName("target1")->X.pos;
    //          cout << tmp.length() << endl;
    tmpDist.append(tmp.length());
    //          cout << tmpVec << endl;
  }
  cout << tmpDist.minIndex() << endl;

  s.MP = new MotionProblem(*s.world,false);
  s.MP->T = xDem.d0-1;
  s.MP->tau = 0.05;
  s.MP->x0 = x0;
  s.world->setJointState(x0);

  /// Set task costs
  //    arr param = ARRAY(5e-1,5e1,1e1,1e1,1e2/*,1e2,1e2,1e2*/);
//  arr param = ARRAY(0.1,12.2487,71.0821,43.5132,53.9258);
//  arr param = ARRAY(0.1,0.0028404,8.77416,4.52686,1.58995);
  arr param = ARRAY(0.1,0.00281488,9.7258,2.20189,0.749742);
//  param = param/length(param)*costScale;
  param.subRange(1,param.d0-1) = param.subRange(1,param.d0-1)/length(param.subRange(1,param.d0-1))*costScale;

  uint pC = 0;
  Task *t;


  // transition costs
  t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
  t->map.order=1;
  t->setCostSpecs(0, s.MP->T, ARR(0.), param(pC));
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
  weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,N,ARR(0.1,1e2)));
  pC++;

  // time points
  uint P = options(0);
  uint C = options(1);
  uint U = options(2);
  uint F = s.MP->T;

  /// tasks
//  arr posCTarget = sum(xDemTaskPos.subRange(P,P+2),1)/3.;
//  cout << posCTarget << endl;

  // first contact with door
  t =s.MP->addTask("posC", new DefaultTaskMap(posTMT, *s.world, "endeffL",NoVector));
  t->setCostSpecs(P, P, ARRAY(s.world->getShapeByName("target1")->X.pos), param(pC));
  weights.append(CostWeight(CostWeight::Block,1,ARR(P,P),s.MP->T,3));
  pC++;

  t =s.MP->addTask("vecC", new DefaultTaskMap(vecAlignTMT, *s.world, "endeffL",ors::Vector(1.,0.,0.),"handle",ors::Vector(-1.,0.,0.)));
  t->setCostSpecs(C,C, ARRAY(1.), param(pC));
  weights.append(CostWeight(CostWeight::Block,1,ARR(C,C),s.MP->T,1));
  pC++;

  ors::Vector vecTarget = ors::Vector(0., 0.9939, -0.1104);
  vecTarget.normalize();
  t =s.MP->addTask("vecUF", new DefaultTaskMap(vecAlignTMT, *s.world, "endeffL",ors::Vector(0.,1.,0.),"handle",vecTarget));
  t->setCostSpecs(U, F, ARRAY(1.), param(pC));
  weights.append(CostWeight(CostWeight::Block,1,ARR(U,F),s.MP->T,1));
  pC++;

  t = s.MP->addTask("door_joint", new TaskMap_qItself(s.world->getJointByName("frame_door")->qIndex, N));
  t->setCostSpecs(F-10, F, ARRAY(-.47), param(pC));
  weights.append(CostWeight(CostWeight::Block,1,ARR(F-10,F),s.MP->T,1));
  pC++;

  // constraints
  t =s.MP->addTask("torso_fixation", new qItselfConstraint(s.world->getJointByName("torso_lift_joint")->qIndex,N));
  t->setCostSpecs(0,F, ARR(0.), 1.);
  t->map.order=1;

  t =s.MP->addTask("door_fixation", new qItselfConstraint(s.world->getJointByName("frame_door")->qIndex,N));
  t->setCostSpecs(0,U, ARR(0.), 1.);
  t->map.order=1;

  t = s.MP->addTask("contact", new PointEqualityConstraint(*s.world, "endeffL",NoVector, "target",NoVector));
  t->setCostSpecs(C, F, ARR(0.), 1.);

//  t = s.MP->addTask("qLimits", new LimitsConstraint());
//  t->setCostSpecs(0., F, ARR(0.), 1.);



  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<endl;
  arr x(T+1,n); x.setZero();
  arr lambda;
  x = repmat(~s.MP->x0,T+1,1);
//  /*
  optConstrainedMix(x, NoArr, Convert(MPF), OPT(verbose=1, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=1.1,stopTolerance = 1e-3));
  if (vis) {
    s.MP->costReport(true);
    displayTrajectory(x,s.MP->T*2,s.MP->world,"t");
  }//*/
  s.world->setJointState(x0);

  // set all costs in s.MP equal to 0 or 1
  for (uint c=0;c < s.MP->taskCosts.N;c++) {
    if (s.MP->taskCosts(c)->map.type==sumOfSqrTT) {
      if (weights(c).type == CostWeight::RBF) {
        s.MP->taskCosts(c)->prec = 1;
      } else {
        s.MP->taskCosts(c)->prec /= (s.MP->taskCosts(c)->prec+1e-12);
      }
      cout << s.MP->taskCosts(c)->prec << endl;
    }
  }

  s.MP->prefix.clear();
  s.MP->postfix.clear();
  s.MP->phiMatrix.clear();
  s.MP->ttMatrix.clear();
  s.MP->dualMatrix.clear();
  s.MP->z0.clear();

  s.xDem = xDem;
  s.lambdaRef = lambda;
  s.paramRef = param;
}

/*
void MotionFactory::createScenePR2(Scene &s, MT::Array<CostWeight> &weights, uint i) {
  s.world = new ors::KinematicWorld("model.kvg");

  cout << "Joints: " << endl;
  for (uint i = 0;i<s.world->joints.d0;i++){
    if (s.world->joints(i)->type != 10)
      cout << s.world->joints(i)->name << " " << s.world->joints(i)->type << " "  << s.world->joints(i)->qIndex<< endl;
  }

  cout << "###########################################################" << endl;
  /// load demonstrations data
  MT::String demoPath = MT::getParameter<MT::String>("demoPath");
  cout << "Demo: " << demoPath << endl;
  // load joint trajectory
  arr xDem; xDem << FILE(STRING(demoPath<<"/pr2_joints").p);

  arr marker_pose = zeros(20,7);
  arr markerDem; markerDem << FILE(STRING(demoPath<<"/pr2_marker0").p);
  marker_pose[4].subRange(0,2) = markerDem[0];
  marker_pose[11].subRange(0,2) = markerDem[1];
  marker_pose[15].subRange(0,2) = markerDem[2];
  marker_pose[17].subRange(0,2) = markerDem[3];

  arr refFrame = ARRAY(s.world->getBodyByName("torso_lift_link")->X.pos);
  s.world->getBodyByName("marker4")->X.pos = refFrame + marker_pose[4].subRange(0,2);
  s.world->getBodyByName("marker11")->X.pos = refFrame + marker_pose[11].subRange(0,2);
  s.world->getBodyByName("marker15")->X.pos = refFrame + marker_pose[15].subRange(0,2);
  s.world->getBodyByName("marker17")->X.pos = refFrame + marker_pose[17].subRange(0,2);

  // compute door angle
  arr v1 = ARRAY(s.world->getBodyByName("marker4")->X.pos - s.world->getBodyByName("marker15")->X.pos);
  v1(2)=0.;
  v1 = v1/sqrt(sumOfSqr(v1));
  arr v2 = ARRAY(1.,0.,0.);
  double alpha = acos(sum(v1%v2));

  s.world->getJointByName("world_door")->A.rot.setRadZ(M_PI_2-alpha);
  s.world->getJointByName("world_door")->A.pos = ARRAY(s.world->getBodyByName("marker15")->X.pos)+ s.world->getJointByName("world_door")->A.rot.getArr()*ARRAY(0.,0.6205,0.2305);
  s.world->getJointByName("world_door")->A.pos.z = .99;

  s.world->calc_fwdPropagateFrames();


  arr x0 = xDem[0];
  cout << "x0: " << x0 << endl;

  double N = s.world->getJointStateDimension();
  cout << "Number of Joints: " << N << endl;
  if (vis) { displayTrajectory(xDem,xDem.d0*.10,*s.world,"");}

//  arr xDemTaskPos, xDemTaskVec;
//  // compute task spaces of demonstrations
//  for (uint t =0;t<xDem.d0;t++) {
//    s.world->setJointState(xDem[t]);
//    arr tmpPos,tmpVec;
//    s.world->kinematicsPos(tmpPos,NoArr,s.world->getBodyByName("l_wrist_roll_link"),&s.world->getShapeByName("endeffL")->rel.pos);
//    xDemTaskPos.append(~tmpPos);
//    ors::Vector vv = ors::Vector(0.,0.,1.);
//    s.world->kinematicsVec(tmpVec,NoArr,s.world->getShapeByName("endeffL")->body,&vv);
//    xDemTaskVec.append(~tmpVec);
//  }

//  s.world->watch(true);

  s.MP = new MotionProblem(*s.world,false);
  s.MP->T = xDem.d0-1;
  s.MP->tau = 0.05;
  s.MP->x0 = x0;
  s.world->setJointState(x0);

  /// Set task costs
  arr param = ARRAY(5e-1,5e1,1e1,1e2,1e2,1e2);
//  param.append(ones(20));
  param = param/length(param)*costScale;
  uint pC = 0;
  // transition costs
  Task *t;
  t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
  t->map.order=1;
  t->setCostSpecs(0, s.MP->T, ARR(0.), param(pC));
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
  weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,N,ARR(0.1,1e2)));
  pC++;

  // time points
  uint F = s.MP->T;
  uint C = round(80/199.*F);
  uint U = round(125/199.*F);


  /// tasks
  // first contact with door
  t =s.MP->addTask("posC", new DefaultTaskMap(posTMT, *s.world, "endeffL",NoVector));
  t->setCostSpecs(C, C+5, ARRAY(s.world->getShapeByName("target")->X.pos), param(pC));
  weights.append(CostWeight(CostWeight::Block,1,ARR(C,C+5),s.MP->T,3));
  pC++;

  t =s.MP->addTask("vecC", new DefaultTaskMap(vecAlignTMT, *s.world, "endeffL",ors::Vector(0.,1.,0.),"door",ors::Vector(0.,0.,-1.)));
  t->setCostSpecs(C, C, ARRAY(1.), param(pC));
  weights.append(CostWeight(CostWeight::Block,1,ARR(C,C),s.MP->T,1));
  pC++;

  t =s.MP->addTask("vecUF", new DefaultTaskMap(vecAlignTMT, *s.world, "endeffL",ors::Vector(0.,1.,0.),"handle",ors::Vector(0.,1.,0.)));
  t->setCostSpecs(U, F, ARRAY(1.), param(pC));
  weights.append(CostWeight(CostWeight::Block,1,ARR(U,F),s.MP->T,1));
  pC++;

//  s.MP->setInterpolatingCosts(t, MotionProblem::constant,ARRAY(1.),0.);
//  weights.append(CostWeight(CostWeight::RBF,20,ARR(20,U,F,4),s.MP->T,1,ARR(1e0,1e2)));
//  arr w;
//  weights.last().compWeights(w,NoArr,NoArr,param.subRange(pC,pC+19),true);
//  t->prec = w;
//  cout << w << endl;
//  pC=pC+20;


  t =s.MP->addTask("vecU", new DefaultTaskMap(vecAlignTMT, *s.world, "endeffL",ors::Vector(1.,0.,0.),"handle",ors::Vector(-1.,0.,0.)));
  t->setCostSpecs(C, C, ARRAY(1.), param(pC));
  weights.append(CostWeight(CostWeight::Block,1,ARR(C,C),s.MP->T,1));
  pC++;

  t = s.MP->addTask("door_joint", new TaskMap_qItself(s.world->getJointByName("frame_door")->qIndex, N));
  t->setCostSpecs(F-10, F, ARRAY(-.52), param(pC));
  weights.append(CostWeight(CostWeight::Block,1,ARR(F-10,F),s.MP->T,1));
  pC++;

  /// constraints
  t = s.MP->addTask("contact", new PointEqualityConstraint(*s.world, "endeffL",NoVector, "target",NoVector));
  t->setCostSpecs(C, F, {0.}, 1.);

  // door fixation
  t =s.MP->addTask("door_fixation", new qItselfConstraint(s.world->getJointByName("frame_door")->qIndex,N));
  t->setCostSpecs(0,U, {0.}, 1.);
  t->map.order=1;

  t =s.MP->addTask("torso_fixation", new qItselfConstraint(s.world->getJointByName("torso_lift_joint")->qIndex,N));
  t->setCostSpecs(0,F, {0.}, 1.);
  t->map.order=1;

  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<endl;
  arr x(T+1,n); x.setZero();
  arr lambda;
  x = repmat(~s.MP->x0,T+1,1);
//  optConstrainedMix(x, NoArr, Convert(MPF), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-4));

//  s.MP->costReport(false);
  if (vis) {
      displayTrajectory(x,s.MP->T,s.MP->world,"t");
  }
  s.world->setJointState(x0);

  // set all costs in s.MP equal to 0 or 1
  for (uint c=0;c < s.MP->taskCosts.N;c++) {
    if (s.MP->taskCosts(c)->map.type==sumOfSqrTT) {
      if (weights(c).type == CostWeight::RBF) {
        s.MP->taskCosts(c)->prec = 1;
      } else {
        s.MP->taskCosts(c)->prec /= (s.MP->taskCosts(c)->prec+1e-12);
      }
      cout << s.MP->taskCosts(c)->prec << endl;
    }
  }

  s.MP->prefix.clear();
  s.MP->postfix.clear();
  s.MP->phiMatrix.clear();
  s.MP->ttMatrix.clear();
  s.MP->dualMatrix.clear();
  s.MP->z0.clear();

  s.xDem = xDem;
  s.lambdaRef = lambda;
  s.paramRef = param;
}

/*
void MotionFactory::createScene1(Scene &s, MT::Array<CostWeight> &weights, uint i) {
  s.world = new ors::KinematicWorld("scene");
  arr q, qdot;
  s.world->getJointState(q, qdot);
  makeConvexHulls(s.world->shapes);
  s.world->swift().setCutoff(10.);

  s.world->swift();
  s.MP = new MotionProblem(*s.world,false);
  s.MP->useSwift=true;
  s.MP->loadTransitionParameters();
  s.MP->makeContactsAttractive=false;
  s.MP->T = 100;
  s.MP->tau = 0.05;

  //-- setup new motion problem
  s.world->getBodyByName("target")->X.pos += double(i)*ARR(0.,0.1,0.);

  ors::Shape *grasp = s.world->getShapeByName("endeff");
  ors::Body *tar = s.world->getBodyByName("target");

  /// Set task costs
  arr param = ARRAY(1.,1e2);
  arr w;
  uint pC = 0;
  // transition costs
  s.MP->H_rate_diag = param(pC);
  weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension(),ARR(1.,1e2)));
  pC++;

  // task costs
  TaskCost *c;
  c =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  s.MP->setInterpolatingCosts(c, MotionProblem::constant,ARRAY(tar->X.pos),0.);
  weights.append(CostWeight(CostWeight::Gaussian,1,ARR(s.MP->T,0.5),s.MP->T,3));
  weights.last().compWeights(w,NoArr,NoArr,ARR(param(pC)),true);
  c->prec = w;
  pC++;

  s.MP->x0 = zeros(s.world->getJointStateDimension(),1);s.MP->x0.flatten();
  s.MP->x0(0) = M_PI_2;

  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<endl;
  arr x(T+1,n); x.setZero();arr lambda(T+1,1); lambda.setZero();
  x = repmat(~s.MP->x0,T+1,1);
  optConstrained(x, NoArr, Convert(MPF), OPT(verbose=0,stopTolerance=1e-5, allowOverstep=true));
  optConstrained(x, NoArr, Convert(MPF), OPT(verbose=0,stopTolerance=1e-6,  allowOverstep=true));


  arr grad;
  ConstrainedProblem & v = Convert(MPF);
  double costs = v.fc(grad,NoArr,NoArr,NoArr,x);
//  cout << grad << endl;
//  cout << ~grad*grad << endl;
//  cout << costs << endl;
//  cout << v.Jy << endl;
//  cout << v.y << endl;

  s.MP->costReport(false);
  if (vis) {
    displayTrajectory(x,s.MP->T/2.,s.MP->world,"t");
  }

  // set all costs in s.MP equal to 0 or 1
  s.MP->H_rate_diag = 1.;
  for (uint c=0;c < s.MP->taskCosts.N;c++) {
    if (!s.MP->taskCosts(c)->map.constraint) {
      if (weights(c+1).type == CostWeight::Block) {
        s.MP->taskCosts(c)->prec /= (s.MP->taskCosts(c)->prec+1e-12);
      } else {
        s.MP->taskCosts(c)->prec = 1.;
      }
      cout << s.MP->taskCosts(c)->prec << endl;
    }
  }

  s.xDem = x;
  s.lambdaRef = lambda;
  s.paramRef = param;
}

void MotionFactory::createScene2(Scene &s, MT::Array<CostWeight> &weights, uint i) {
  s.world = new ors::KinematicWorld("scene");
  arr q, qdot;
  s.world->getJointState(q, qdot);
  makeConvexHulls(s.world->shapes);
  s.world->swift().setCutoff(10.);

  s.world->swift();
  s.MP = new MotionProblem(*s.world,false);
  s.MP->useSwift=true;
  s.MP->loadTransitionParameters();
  s.MP->makeContactsAttractive=false;
  s.MP->T = 100;
  s.MP->tau = 0.05;

  //-- setup new motion problem
  s.world->getBodyByName("target")->X.pos += double(i)*ARR(0.,0.1,0.);

  ors::Shape *grasp = s.world->getShapeByName("endeff");
  ors::Body *tar = s.world->getBodyByName("target");

  /// Set task costs
//  arr param = ARRAY(1.,1e3,80.);
  arr param = ARRAY(1.);
  param.append(zeros(20));
  param(19) = 1e0;
  param(18) = 1e1;
  param(17) = 1e0;
  arr w;
  uint pC = 0;
  // transition costs
  s.MP->H_rate_diag = param(pC);
  weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension(),ARR(1.,1e2)));
  pC++;

  // task costs
//  TaskCost *c;
//  c =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
//  s.MP->setInterpolatingCosts(c, MotionProblem::constant,ARRAY(tar->X.pos),0.);
//  weights.append(CostWeight(CostWeight::Gaussian,2,ARR(1.),s.MP->T,3,ARR(1e0,1e3)));
//  weights.last().compWeights(w,NoArr,NoArr,ARR(param(pC),param(pC+1)),true);
//  c->prec = w;
//  cout << w << endl;
//  pC=pC+2;

  TaskCost *c;
  c =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  s.MP->setInterpolatingCosts(c, MotionProblem::constant,ARRAY(tar->X.pos),0.);
  weights.append(CostWeight(CostWeight::RBF,20,ARR(20,70.,s.MP->T,0.05),s.MP->T,3,ARR(1e0,1e2)));
  weights.last().compWeights(w,NoArr,NoArr,param.subRange(pC,pC+19),true);
  c->prec = w;
  cout << w << endl;
  pC=pC+20;



  s.MP->x0 = zeros(s.world->getJointStateDimension(),1);s.MP->x0.flatten();
  s.MP->x0(0) = M_PI_2;

  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<endl;
  arr x(T+1,n); x.setZero();arr lambda(T+1,1); lambda.setZero();
  x = repmat(~s.MP->x0,T+1,1);
  optConstrained(x, NoArr, Convert(MPF), OPT(verbose=0,stopTolerance=1e-5, allowOverstep=true));
//  optConstrained(x, NoArr, Convert(MPF), OPT(verbose=0,stopTolerance=1e-7,  allowOverstep=true));


  arr grad;
  ConstrainedProblem & v = Convert(MPF);
  double costs = v.fc(grad,NoArr,NoArr,NoArr,x);
//  cout << grad << endl;
  cout << ~grad*grad << endl;
//  cout << costs << endl;
//  cout << v.Jy << endl;
//  cout << v.y << endl;

  s.MP->costReport(false);
  if (vis) {
    displayTrajectory(x,s.MP->T/2.,s.MP->world,"t");
  }

  // set all costs in s.MP equal to 0 or 1
  s.MP->H_rate_diag = 1.;
  for (uint c=0;c < s.MP->taskCosts.N;c++) {
    if (!s.MP->taskCosts(c)->map.constraint) {
      if (weights(c+1).type == CostWeight::Block) {
        s.MP->taskCosts(c)->prec /= (s.MP->taskCosts(c)->prec+1e-12);
      } else {
        s.MP->taskCosts(c)->prec = 1.;
      }
      cout << s.MP->taskCosts(c)->prec << endl;
    }
  }

  s.xDem = x;
  s.lambdaRef = lambda;
  s.paramRef = param;
}

void MotionFactory::createScene3(Scene &s, MT::Array<CostWeight> &weights, uint i)
{
  s.world = new ors::KinematicWorld("scene");
  arr q, qdot;
  s.world->getJointState(q, qdot);
  makeConvexHulls(s.world->shapes);
  s.world->swift().setCutoff(10.);

  s.world->swift();
  s.MP = new MotionProblem(*s.world,false);
  s.MP->useSwift=true;
  s.MP->loadTransitionParameters();
  s.MP->makeContactsAttractive=false;
  s.MP->T = 100;
  s.MP->tau = 0.05;

  //-- setup new motion problem
  s.world->getBodyByName("target")->X.pos += double(i)*ARR(0.,0.1,0.);

  ors::Shape *grasp = s.world->getShapeByName("endeff");
  ors::Body *tar = s.world->getBodyByName("target");

  /// Set task costs
  arr param = ARRAY(1.,1e2,90.,1.5);
  arr w;
  uint pC = 0;
  // transition costs
  s.MP->H_rate_diag = param(pC);
  weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension(),ARR(1.,1e2)));
  pC++;

  // task costs
  TaskCost *c;
  c =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  s.MP->setInterpolatingCosts(c, MotionProblem::constant,ARRAY(tar->X.pos),0.);
//  c->setCostSpecs(s.MP->T,s.MP->T,ARRAY(tar->X.pos),param(pC));
  c->prec.subRange(param(pC+1),param(pC+1)+3)=param(pC);
  weights.append(CostWeight(CostWeight::Gaussian,3,ARR(0),s.MP->T,3));
  pC=pC+3;

  s.MP->x0 = zeros(s.world->getJointStateDimension(),1);s.MP->x0.flatten();
  s.MP->x0(0) = M_PI_2;

  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<endl;
  arr x(T+1,n); x.setZero();arr lambda(T+1,1); lambda.setZero();
  x = repmat(~s.MP->x0,T+1,1);
  optConstrained(x, NoArr, Convert(MPF), OPT(verbose=0,stopTolerance=1e-5, allowOverstep=true));
  optConstrained(x, NoArr, Convert(MPF), OPT(verbose=0,stopTolerance=1e-7,  allowOverstep=true));

  s.MP->costReport(false);
  if (vis) {
    displayTrajectory(x,s.MP->T/2.,s.MP->world,"t");
  }

  // set all costs in s.MP equal to 0 or 1
  s.MP->H_rate_diag = 1.;
  for (uint c=0;c < s.MP->taskCosts.N;c++) {
    if (!s.MP->taskCosts(c)->map.constraint) {
      if (weights(c+1).type == CostWeight::Block) {
        s.MP->taskCosts(c)->prec /= (s.MP->taskCosts(c)->prec+1e-12);
      } else {
        s.MP->taskCosts(c)->prec = 1.;
      }
      cout << s.MP->taskCosts(c)->prec << endl;
    }
  }

  s.xDem = x;
  s.lambdaRef = lambda;
  s.paramRef = param;
}

void MotionFactory::createScene4(Scene &s, MT::Array<CostWeight> &weights, uint i)
{
  s.world = new ors::KinematicWorld("scene");
  arr q, qdot;
  s.world->getJointState(q, qdot);
  makeConvexHulls(s.world->shapes);
  s.world->swift().setCutoff(10.);
  s.world->swift();

  //- replay demonstrations
  arr xDem,markerDem,contactIdxDem,contactDem,markerQuatDem,dtDem,tDem;
  FILE("data/push2m2_joints") >> xDem;
  FILE("data/push2m2_marker") >> markerDem;
  FILE("data/push2m2_markerQuat") >> markerQuatDem;
  FILE("data/push2m2_contact") >> contactDem;
  FILE("data/push2m2_dt") >> dtDem;
  FILE("data/push2m2_t") >> tDem;
  FILE("data/push2m2_contact_idx") >> contactIdxDem;

  arr refFrame = ARRAY(s.world->getBodyByName("torso_lift_link")->X.pos);
  ors::Quaternion refFrameQuat = s.world->getBodyByName("torso_lift_link")->X.rot;

  // compute mean orientation of object
  arr mObjQuat = sum(fabs(markerQuatDem),0)/double(markerQuatDem.d0);
  mObjQuat = mObjQuat %(markerQuatDem[0]/fabs(markerQuatDem[0]));
  ors::Quaternion objQuat = refFrameQuat*ors::Quaternion(mObjQuat);//markerQuatDem[0]);

  arr markerOffset = objQuat.getArr()*ARRAY(-0.126,0.0,0.0415);

  arr q0 = xDem[0];
  arr objPos0 = refFrame + markerOffset + markerDem[0];
  arr objPosT = refFrame + markerOffset + markerDem[markerDem.d0-1];

  s.world->getBodyByName("drawer1")->X.rot =  objQuat;
  if (i==2){
    objPosT = objPosT - ARRAY(0.,0.33+0.024,0.);
    objPos0 = objPos0 - ARRAY(0.,0.33+0.024,0.);
    s.world->getBodyByName("drawer1")->X.pos =  objPos0;
  } else if (i==3) {
    objPosT = objPosT - ARRAY(0.,0.,0.163+0.005);
    objPos0 = objPos0 - ARRAY(0.,0.,0.163+0.005);
    s.world->getBodyByName("drawer1")->X.pos =  objPos0;
    q0(1)+=0.5;
    q0(0)-=0.2;
  } else if (i==4) {
    objPosT = objPosT - ARRAY(0.,0.33+0.024,0.163+0.005);
    objPos0 = objPos0 - ARRAY(0.,0.33+0.024,0.163+0.005);
    s.world->getBodyByName("drawer1")->X.pos =  objPos0;
  }

  s.world->getBodyByName("wall1")->X.pos =  objPosT+ARRAY(-0.1,0.165+0.026+0.03,-0.);
  s.world->getBodyByName("wall2")->X.pos =  objPosT+ARRAY(-0.1,0.,0.0815+0.07);

  s.world->getBodyByName("goalDrawer1")->X.rot = objQuat;
  s.world->getBodyByName("goalDrawer1")->X.pos = objPosT;
  s.world->calc_fwdPropagateShapeFrames();

  arr xDemTaskPos,xDemTaskVec,tmpPos;
  for (uint t =0;t<xDem.d0;t++) {
    s.world->setJointState(xDem[t]);
    arr tmpPos,tmpVec;
    s.world->kinematicsPos(tmpPos,NoArr,s.world->getBodyByName("endeffR"));
    xDemTaskPos.append(~tmpPos);
    ors::Vector vv = ors::Vector(1.,0.,0.);
    s.world->kinematicsVec(tmpVec,NoArr,s.world->getBodyByName("endeffR"),&vv);
    xDemTaskVec.append(~tmpVec);

    if (t%18==0) {
      s.world->getBodyByName("drawer1")->X.pos = refFrame + markerOffset + markerDem[t];
      s.world->getBodyByName("targetRef")->X.pos = refFrame + markerDem[t];
//      s.world->gl().update(STRING(t));
    }
    if (contactDem(t,0)>0) {
      memmove(s.world->getBodyByName("drawer1")->shapes(0)->color,ARR(1, 0, 0).p, 3*sizeof(double));
    }else{
      memmove(s.world->getBodyByName("drawer1")->shapes(0)->color,ARR(0, 0, 1).p, 3*sizeof(double));
    }
  }

  arr vec;
  s.world->kinematicsVec(vec,NoArr,s.world->getBodyByName("drawer1"));

  s.MP = new MotionProblem(*s.world,false);
  s.MP->useSwift=true;
  s.MP->loadTransitionParameters();
  s.MP->makeContactsAttractive=false;
  s.MP->T = xDem.d0-1;
  s.MP->tau = dtDem(0,0);

  uint contactTime = contactIdxDem(0,0);//round(s.MP->T/2.);
  cout << "contactTime: " << contactTime << endl;

  s.MP->x0 = q0;
  s.world->getBodyByName("drawer1")->X.pos = objPos0;

  ors::Shape *grasp = s.world->getShapeByName("endeffR");
  ors::Body *tar = s.world->getBodyByName("drawer1");

  // add graph operator
  ors::GraphOperator *op1 = new ors::GraphOperator();
  op1->symbol = ors::GraphOperator::addRigid;
  op1->timeOfApplication = contactTime;
  op1->fromId = s.world->getBodyByName("endeffR")->index;
  op1->toId = s.world->getBodyByName("drawer1")->index;
  s.world->operators.append(op1);

  arr param = ARR(s.MP->H_rate_diag(0));
  param.append(ARR(1e2,1e2,1e0,1e0,1e0,1e0,1e0,1e0));
  param = param/sqrt(sumOfSqr(param))*1e4;
  s.MP->H_rate_diag = param(0);//param.subRange(0,s.MP->H_rate_diag.d0-1);
  cout << "Parameter: " << param << endl;
  uint N = 1;//s.world->getJointStateDimension();

  uint stay = 5;
  TaskCost *c;
  c =s.MP->addTask("posT", new DefaultTaskMap(posTMT, grasp->index) );
  c->setCostSpecs(s.MP->T,s.MP->T, objPosT, param(N++));

  c =s.MP->addTask("posC", new DefaultTaskMap(posTMT, grasp->index) );
  c->setCostSpecs(contactTime,contactTime, objPos0, param(N++));

  c =s.MP->addTask("vecT", new DefaultTaskMap(vecTMT, grasp->index,ors::Vector(1.,0.,0)) );
  c->setCostSpecs(s.MP->T,s.MP->T, xDemTaskVec[s.MP->T], param(N++));
  cout << "xDemTaskVec[s.MP->T]: "<< xDemTaskVec[s.MP->T] << endl;

  c =s.MP->addTask("vecC", new DefaultTaskMap(vecTMT, grasp->index,ors::Vector(1.,0.,0)) );
  c->setCostSpecs(contactTime,contactTime, xDemTaskVec[s.MP->T], param(N++));

  c =s.MP->addTask("velT", new DefaultTaskMap(posTMT, grasp->index) );
  c->map.order = 1;
  c->setCostSpecs(s.MP->T-stay,s.MP->T, zeros(3), param(N++));

  c =s.MP->addTask("velC", new DefaultTaskMap(posTMT, grasp->index) );
  c->map.order = 1;
  c->setCostSpecs(contactTime-stay,contactTime+stay, zeros(3), param(N++));

  TaskMap *tm_contact = new PairCollisionConstraint(*s.world,"wall1","drawer1",0.02);
  TaskCost *c4 = s.MP->addTask("contact_endeff",tm_contact);
  c4->map.constraint = false;
  c4->setCostSpecs(contactTime,s.MP->T, ARR(0.) ,param(N++));

  TaskMap *tm_contact2 = new PairCollisionConstraint(*s.world,"wall2","drawer1",0.02);
  TaskCost *c5 = s.MP->addTask("contact_endeff2",tm_contact2);
  c5->map.constraint = false;
  c5->setCostSpecs(contactTime,s.MP->T, ARR(0.) ,param(N++));

  c = s.MP->addTask("collisionConstraints1", new PairCollisionConstraint(*s.world,"wall1","drawer1",0.03));
  s.MP->setInterpolatingCosts(c, MotionProblem::constant,ARR(0.),1e0);



  c = s.MP->addTask("collisionConstraints2", new PairCollisionConstraint(*s.world,"wall2","drawer1",0.03));
  s.MP->setInterpolatingCosts(c, MotionProblem::constant,ARR(0.),1e0);


  //  /*arr costGrid = linspace(50.,double(xDem.d0-1),5); costGrid.flatten();
  //  cout << "costGrid"<<costGrid << endl;
  //  c =s.MP->addTask("posC", new DefaultTaskMap(posTMT, grasp->index) );
  //  c->setCostSpecs(contactTime,contactTime+3, objPosT, param(N++));
  //  c =s.MP->addTask("vel", new DefaultTaskMap(qItselfTMT,*s.world) );
  //  c->map.order = 1.;
  //  c->setCostSpecs(s.MP->T, s.MP->T, zeros(s.world->getJointStateDimension()), param(N++));
  //  for (uint idx=0;idx<costGrid.d0;idx++) {
  //    c = s.MP->addTask(STRING("pos"<<idx),new DefaultTaskMap(posTMT, grasp->index) );
  //    c->setCostSpecs(costGrid(idx),costGrid(idx), xDemTaskPos[costGrid(idx)], 1e2);
  //    param.append(ARR(1.));
  //    c->active=true;

  //    c =s.MP->addTask(STRING("vec"<<idx), new DefaultTaskMap(vecTMT, grasp->index,ors::Vector(1.,0.,0)) );
  //    c->setCostSpecs(costGrid(idx),costGrid(idx), xDemTaskVec[costGrid(idx)], 1e2);
  //    param.append(ARR(1.));
  //    c->active=true;
  //  }

  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" << N<<endl;
  arr x(T+1,n); x.setZero();arr lambdaTrain(2*T+1); lambdaTrain.setZero();
  x = repmat(~s.MP->x0,T+1,1);
//    x = xDem;
  if (i==1 && !MT::getParameter<bool>("evalOnly")) {

    optConstrained(x, lambdaTrain, Convert(MPF), OPT(verbose=0,stopTolerance=1e-3));//, allowOverstep=true));
//    optConstrained(x, lambdaTrain, Convert(MPF), OPT(verbose=0,stopTolerance=1e-4,  allowOverstep=true));

    if (vis) {
//      cout << "lambdaTrain" << lambdaTrain << endl;
      s.MP->costReport(true);
      displayTrajectory(x,s.MP->T,s.MP->world,"t");
    }
  }

  // set all costs equal to 1 or 0
  s.MP->H_rate_diag = 1.;
  for (uint c=0;c < s.MP->taskCosts.N;c++) {
    if (!s.MP->taskCosts(c)->map.constraint) {
      s.MP->taskCosts(c)->active = true;
      s.MP->taskCosts(c)->prec = s.MP->taskCosts(c)->prec/(s.MP->taskCosts(c)->prec+1e-12);

    }
  }

  arr lambda(2*(T+1));lambda.setZero();
//  lambda.subRange(2*contactIdxDem(0,0),lambda.d0-1) = 0.01;

  arr aDem;
  getAcc(aDem,xDem,dt);
  cout << "sum aDem" << sum(sum((aDem%aDem),0))*0.5 << endl;

  s.xInit = xDem;
  s.xDem = xDem;
  s.lambdaRef = lambda;
  s.paramRef = param;
  s.contactTime = contactTime;
}



void MotionFactory::createScene5(Scene &s, MT::Array<CostWeight> &weights, uint i) {
  s.world = new ors::KinematicWorld("scene");
  arr q, qdot;
  s.world->getJointState(q, qdot);
  makeConvexHulls(s.world->shapes);
  s.world->swift().setCutoff(10.);

  s.world->swift();
  s.MP = new MotionProblem(*s.world,false);
  s.MP->useSwift=true;
  s.MP->loadTransitionParameters();
  s.MP->makeContactsAttractive=false;
  s.MP->T = 100;
  s.MP->tau = 0.01;

  //-- setup new motion problem
  s.world->getBodyByName("target")->X.pos += double(i)*ARR(0.,0.1,0.);

  ors::Shape *grasp = s.world->getShapeByName("endeff");
  ors::Body *tar = s.world->getBodyByName("target");

  /// Set task costs
  arr param = ARRAY(1.,1e3);
  uint pC = 0;
  // transition costs
  s.MP->H_rate_diag = param(pC);
  weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));
  pC++;

  // task costs
  TaskCost *c;
  c =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  c->setCostSpecs(s.MP->T,s.MP->T,ARRAY(tar->X.pos),param(pC));
  weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T),1,3));
  pC++;

//  c =s.MP->addTask("pos2", new DefaultTaskMap(posTMT, grasp->index) );
//  s.MP->setInterpolatingCosts(c, MotionProblem::constant,ARRAY(tar->X.pos),0.);
  //  c->setCostSpecs(s.MP->T*0.5,s.MP->T*0.5, ARRAY(tar->X.pos), param(N));
//  c->prec(s.MP->T*0.5) = param(N);

  s.MP->x0 = zeros(s.world->getJointStateDimension(),1);s.MP->x0.flatten();
  s.MP->x0(0) = M_PI_2;

  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<endl;
  arr x(T+1,n); x.setZero();arr lambda(T+1,1); lambda.setZero();
  x = repmat(~s.MP->x0,T+1,1);
  optConstrained(x, NoArr, Convert(MPF), OPT(verbose=0,stopTolerance=1e-5, allowOverstep=true));
  optConstrained(x, NoArr, Convert(MPF), OPT(verbose=0,stopTolerance=1e-8,  allowOverstep=true));

  s.MP->costReport(false);
  if (vis) {
    displayTrajectory(x,s.MP->T/2.,s.MP->world,"t");
  }

  // set all costs in s.MP equal to 0 or 1
  s.MP->H_rate_diag = 1.;
  for (uint c=0;c < s.MP->taskCosts.N;c++) {
    if (!s.MP->taskCosts(c)->map.constraint) {
      s.MP->taskCosts(c)->prec /= (s.MP->taskCosts(c)->prec+1e-12);
      cout << s.MP->taskCosts(c)->prec << endl;
    }
  }

  s.xDem = x;
  s.lambdaRef = lambda;
  s.paramRef = param;
}
*/


/*
  // replay motion
  cout << objQuat << endl;
  for (uint t =0;t<xDem.d0;t+=10) {
    s.world->setJointState(xDem[t]);
    s.world->getBodyByName("target")->X.rot =  refFrameRot*objQuat;
    s.world->getBodyByName("target")->X.pos = refFrame + markerDem[t]+s.world->getBodyByName("target")->X.rot.getArr()*ARRAY(-0.1,0.,0.05);
    s.world->getBodyByName("targetRef")->X.pos = refFrame+ markerDem[t];

    s.world->gl().update(STRING(t));
  }
  */

/*
// quaternion test
arr marker0; marker0 << FILE(STRING(demoPath<<"_marker0").p);
arr markerQuat0; markerQuat0 << FILE(STRING(demoPath<<"_markerQuat0").p);
arr refFrame = ARRAY(s.world->getBodyByName("torso_lift_link")->X.pos);


ors::Quaternion torso_rot = s.world->getBodyByName("torso_lift_link")->X.rot;
s.world->getBodyByName("marker")->X.pos = ARRAY(1.,1.,0.1);
s.world->getBodyByName("marker")->X.rot = torso_rot;

ors::Quaternion world_rot = s.world->getBodyByName("world")->X.rot;
s.world->getBodyByName("marker2")->X.pos = ARRAY(1.,1.,.5);
s.world->getBodyByName("marker2")->X.rot = world_rot;

ors::Quaternion obj_rot = s.world->getBodyByName("door")->X.rot;
s.world->getBodyByName("marker3")->X.pos = ARRAY(1.,1.,1.);
s.world->getBodyByName("marker3")->X.rot = obj_rot;

ors::Quaternion door1_rot = ors::Quaternion(markerQuat0[1]);
s.world->getBodyByName("marker4")->X.pos = ARRAY(1.,1.,1.5);
s.world->getBodyByName("marker4")->X.rot = door1_rot;

ors::Quaternion door1_trans = torso_rot*door1_rot;
s.world->getBodyByName("marker5")->X.pos = ARRAY(1.,1.,2.);
s.world->getBodyByName("marker5")->X.rot = door1_trans;

ors::Quaternion door1_xRot; door1_xRot.setRad(M_PI_2,ors::Vector(1.,0.,0.));
ors::Quaternion door1_trans2 = door1_xRot*door1_trans;
s.world->getBodyByName("marker6")->X.pos = ARRAY(1.,1.,2.5);
s.world->getBodyByName("marker6")->X.rot = door1_trans2;

ors::Quaternion door1_xRot2; door1_xRot2.setRad(M_PI_2,door1_trans2.getY());
ors::Quaternion door1_trans3 = door1_xRot2*door1_trans2;
ors::Quaternion door1_zRot; door1_zRot.setRad(M_PI_2,door1_trans3.getZ());
door1_trans3 =door1_trans3*door1_zRot;
s.world->getBodyByName("marker7")->X.pos = ARRAY(1.,1.,3.);
s.world->getBodyByName("marker7")->X.rot = door1_trans3;

s.world->getJointByName("world_door")->A.pos = refFrame + marker0[1] + door1_trans3.getArr()*ARR(0.,0.375*2,0.);
s.world->getJointByName("world_door")->A.pos.z = .99;
s.world->getJointByName("world_door")->A.rot = door1_trans3;


s.world->getBodyByName("marker17")->X.pos = marker0[0]+refFrame;
s.world->getBodyByName("marker17")->X.rot = door1_trans3;
s.world->getBodyByName("marker4")->X.pos = marker0[1]+refFrame;
s.world->getBodyByName("marker4")->X.rot = door1_trans3;

s.world->calc_fwdPropagateFrames();
*/
