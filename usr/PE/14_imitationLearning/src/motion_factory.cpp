#include "motion_factory.h"

#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>
#include <Ors/ors_swift.h>
#include <Geo/geo.h>
#include <Motion/motion.h>


void MotionFactory::execMotion(IKMO &ikmo,Scene &s, arr param, bool vis, uint verbose) {

  param = costScale*param/length(param);
  ikmo.setParam(*s.MP,param);

  // reinit swift interface
//  s.world->swift().initActivations(*s.world);
  s.MP->prefix.clear();
  s.MP->x0 = s.xDem[0];
  s.MP->world.setJointState(s.xDem[0]);
  cout << "x0: " << s.MP->x0 << endl;

//  Task *t;
//  t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
//  t->map.order=1;
//  t->setCostSpecs(0, s.MP->T, ARR(0.), 1.);
//  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;

  s.MP->phiMatrix.clear();
  s.MP->ttMatrix.clear();

  // optimize the motion problem
  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<s.MP->world.getJointStateDimension()<<endl;
  arr x(T+1,n); x.setZero();arr lambda(T+1); lambda.setZero();
  x = repmat(~s.MP->x0,T+1,1);
//  x = randn(T+1,n);
//  x = s.xInit;
  optConstrained(x, lambda, Convert(MPF), OPT(verbose=verbose,stopTolerance=1e-5));
//  optConstrained(x, lambda, Convert(MPF), OPT(verbose=verbose,stopTolerance=1e-6));


  // visualize trajectory
  if (vis) {
    cout <<  "lambda: " << lambda << endl;
    s.MP->costReport(true);
    for (;;)
      displayTrajectory(x,s.MP->T/2.,s.MP->world,"world");
  }
}


void MotionFactory::createScenes(uint sID,mlr::Array<Scene> &trainScenes, mlr::Array<Scene> &testScenes, mlr::Array<CostWeight> &weights)
{
  vis = mlr::getParameter<uint>("visDemo");
  nS = mlr::getParameter<uint>("numScenes");
  optConstraintsParam = mlr::getParameter<bool>("optConstraintsParam");

  // create training scenes
  for (uint i = 0; i < nS; i++) {
    Scene s_train;
    Scene s_test;
    switch(sID) {
      case 0:
        /// recovery of Dirac (1 parameter)
        createScene0(s_train,weights,i);
//        createScene0(s_test,weights,i+nS);
        break;
      case 1:
        createScenePR2(s_train,weights,i);
        break;
      case 2:
        /// recovery of Gaussian (1 parameter)
//        createScene1(s_train,weights,i);
//        createScene1(s_test,weights,i+nS);
        /// recovery of Gaussian (2 parameter)
//        createScene2(s_train,weights,i);
//        createScene2(s_test,weights,i+nS);
        break;
      case 3:
        /// learning of Gaussian (3 parameter) for Dirac Ground Truth
//        createScene3(s_train,weights,i);
//        createScene3(s_test,weights,i+nS);
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
    }
    s_test.xInit = s_train.xDem;
    s_train.xInit = s_train.xDem;

    trainScenes.append(s_train);
//    testScenes.append(s_test);
  }

  // compute number of parameters
  numParam = 0;
  numParam += 1; //transition costs
  for (uint c=0;c<trainScenes(0).MP->taskCosts.N;c++) {
    if (!trainScenes(0).MP->taskCosts(c)->map.type==sumOfSqrTT) {
      numParam++;
    }
  }
  cout << "IOC Number of Tasks: " << numParam << endl;
}


void MotionFactory::createScene0(Scene &s, mlr::Array<CostWeight> &weights, uint i) {
  s.world = new ors::KinematicWorld("scene");
  arr q, qdot;
  s.world->getJointState(q, qdot);
  makeConvexHulls(s.world->shapes);
  s.world->swift().setCutoff(10.);

  s.world->swift();
  s.MP = new MotionProblem(*s.world,false);
  s.MP->useSwift=true;
  s.MP->T = 100;
  s.MP->tau = 0.01;

  //-- setup new motion problem
  s.world->getBodyByName("target")->X.pos += double(i)*ARR(0.,0.1,0.);

  ors::Shape *grasp = s.world->getShapeByName("endeff");
  ors::Body *tar = s.world->getBodyByName("target");

  /// Set task costs
  arr param = {1.,1e3};
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
  t->setCostSpecs(s.MP->T,s.MP->T,conv_vec2arr(tar->X.pos),param(pC));
  weights.append(CostWeight(CostWeight::Dirac,1,ARR(s.MP->T),1,3));
  pC++;

//  param.append(1e2);
//  t =s.MP->addTask("vec", new DefaultTaskMap(vecTMT, grasp->index,ors::Vector(1.,0.,0)) );
//  t->setCostSpecs(s.MP->T,s.MP->T,{0.,0.,-1.},param(pC));
//  weights.append(CostWeight(CostWeight::Dirac,1,ARR(s.MP->T),1,3));
//  pC++;


  // task costs
  if (optConstraintsParam) {
    param.append(5e2);
    TaskMap *tm_contact = new PairCollisionConstraint(*s.world,"table","endeff",0.01);
    Task *c1 = s.MP->addTask("contact_endeff",tm_contact);
    c1->map.type = sumOfSqrTT;
    c1->setCostSpecs(s.MP->T/2.,s.MP->T/2, ARR(0.) ,param(pC));
    weights.append(CostWeight(CostWeight::Dirac,1,ARR(s.MP->T/2),1,1));
    pC++;

    Task *c2 = s.MP->addTask("collisionConstraints", new PairCollisionConstraint(*s.world,"endeff","table",0.01));
    s.MP->setInterpolatingCosts(c2, MotionProblem::constant,ARR(0.),1e0);
  }

  s.MP->x0 = zeros(s.world->getJointStateDimension(),1);s.MP->x0.flatten();
  s.MP->x0(0) = M_PI_2;

  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<endl;
  arr x(T+1,n); x.setZero();arr lambda(T+1,1); lambda.setZero();
  x = repmat(~s.MP->x0,T+1,1);
  optConstrained(x, lambda, Convert(MPF), OPT(verbose=0,stopTolerance=1e-5, allowOverstep=true));
  optConstrained(x, lambda, Convert(MPF), OPT(verbose=0,stopTolerance=1e-8,  allowOverstep=true));

  s.MP->costReport(false);
  if (vis) {
    cout << lambda << endl;
    displayTrajectory(x,s.MP->T,s.MP->world,"t");
  }

//  ConstrainedProblem v = Convert(MPF);
//  arr PHI,JP;
//  TermTypeA tt;
//  v(PHI,JP,tt,x);
//  arr grad = 2.*comp_At_x(JP,PHI);
//  cout << "costs: " <<~PHI*PHI << endl;
//  cout << "grad: " << grad << endl;
//  cout << "grad x grad: " << ~grad*grad << endl;
//  cout << costs << endl;
//  cout << v.Jy << endl;
//  cout << v.y << endl;

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

void MotionFactory::createScenePR2(Scene &s, mlr::Array<CostWeight> &weights, uint i) {
  s.world = new ors::KinematicWorld("model.kvg");

  cout << "Joints: " << endl;
  for (uint i = 0;i<s.world->joints.d0;i++){
    if (s.world->joints(i)->type != 10)
      cout << s.world->joints(i)->name << " " << s.world->joints(i)->type << " "  << s.world->joints(i)->qIndex<< endl;
  }

  cout << s.world->getJointByName("torso_lift_joint")->axis << endl;
  s.world->getJointByName("torso_lift_joint")->A.pos.z +=0.29398;

  s.world->calc_fwdPropagateFrames();
  // endeffL pos: (0.55445 0.240455 0.977503)

  cout << "###########################################################" << endl;
  /// load demonstrations data
  mlr::String demoPath = mlr::getParameter<mlr::String>("demoPath");
  cout << "Demo: " << demoPath << endl;
  // load joint trajectory
  arr xDem; xDem << FILE(STRING(demoPath<<"_joints").p);
  arr x0 = xDem[0];
  cout << "x0: " << x0 << endl;
  s.world->setJointState(x0);

  cout << "endeffL pos: " << s.world->getShapeByName("endeffL")->X.pos << endl;

  // set door position
  arr marker0; marker0 << FILE(STRING(demoPath<<"_marker0").p);
  arr markerQuat0; markerQuat0 << FILE(STRING(demoPath<<"_markerQuat0").p);
  arr refFrame = conv_vec2arr(s.world->getBodyByName("torso_lift_link")->X.pos);

  cout << marker0 << endl;
  cout << markerQuat0 << endl;
  ors::Quaternion door_rot = ors::Quaternion(markerQuat0[1]);
  ors::Quaternion trans = s.world->getBodyByName("torso_lift_link")->X.rot;
  door_rot = trans*door_rot;
  trans.setRad(M_PI_2,ors::Vector(1.,0.,0.));
  door_rot = trans*door_rot;
  trans.setRad(M_PI_2,door_rot.getY());
  door_rot = trans*door_rot;
  trans.setRad(M_PI_2,door_rot.getZ());
  door_rot = trans*door_rot;

  s.world->getJointByName("world_door")->A.pos = refFrame + marker0[1] + door_rot.getArr()*ARR(0.015,0.375*2,0.);
  s.world->getJointByName("world_door")->A.pos.z = .99;
  s.world->getJointByName("world_door")->A.rot = door_rot;

  s.world->getBodyByName("marker17")->X.pos = marker0[0]+refFrame;
  s.world->getBodyByName("marker17")->X.rot = door_rot;
  s.world->getBodyByName("marker4")->X.pos = marker0[1]+refFrame;
  s.world->getBodyByName("marker4")->X.rot = door_rot;

  arr xDemTaskPos, xDemTaskVec;
  // compute task spaces of demonstrations
  for (uint t =0;t<xDem.d0;t++) {
    s.world->setJointState(xDem[t]);
    arr tmpPos,tmpVec;
    s.world->kinematicsPos(tmpPos,NoArr,s.world->getBodyByName("l_wrist_roll_link"),&s.world->getShapeByName("endeffL")->rel.pos);
    xDemTaskPos.append(~tmpPos);
    ors::Vector vv = ors::Vector(0.,0.,1.);
    s.world->kinematicsVec(tmpVec,NoArr,s.world->getShapeByName("endeffL")->body,&vv);
    xDemTaskVec.append(~tmpVec);
  }

//  s.world->watch(true);

  double N = s.world->getJointStateDimension();
  cout << "Number of Joints: " << N << endl;
  if (vis) { displayTrajectory(xDem,xDem.d0*.5,*s.world,"");}

  s.MP = new MotionProblem(*s.world,false);
  s.MP->T = xDem.d0-1;
  s.MP->tau = 0.05;
  s.MP->x0 = x0;
  s.world->setJointState(x0);

  /// Set task costs
//  arr param = {1.,1e2,1e2,1e2,1e2};
  arr param = {.1,1e2,1e2,1e2,1e2,1e2,1e1};
//  arr param = {0.0423369, 96.5584, 7.91922, 33.2956};
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
  uint C = 90;
  uint U = 140;
  uint F = s.MP->T;

  /// tasks
  // first contact with door
  t = s.MP->addTask("posC", new DefaultTaskMap(posTMT, *s.world, "endeffL",NoVector));
  t->setCostSpecs(C, C, conv_vec2arr(s.world->getShapeByName("handle")->X.pos), param(pC));
  weights.append(CostWeight(CostWeight::Dirac,1,ARR(C),1,3));
  pC++;

  t = s.MP->addTask("vecC", new DefaultTaskMap(vecAlignTMT, *s.world, "endeffL", ors::Vector(0.,1.,0.),"handle",ors::Vector(0.,0.,1.)));
  t->setCostSpecs(C, C, ARR(1.), param(pC));
  weights.append(CostWeight(CostWeight::Dirac,1,ARR(C),1,1));
  pC++;

  t = s.MP->addTask("vecC2", new DefaultTaskMap(vecAlignTMT, *s.world, "endeffL", ors::Vector(0.,1.,0.),"handle",ors::Vector(0.,0.,1.)));
  t->setCostSpecs(C-10, C-10, ARR(1.), param(pC));
  weights.append(CostWeight(CostWeight::Dirac,1,ARR(C-10),1,1));
  pC++;

  t = s.MP->addTask("vecC3", new DefaultTaskMap(vecAlignTMT, *s.world, "endeffL", ors::Vector(0.,1.,0.),"handle",ors::Vector(0.,0.,1.)));
  t->setCostSpecs(C+10, C+10, ARR(1.), param(pC));
  weights.append(CostWeight(CostWeight::Dirac,1,ARR(C+10),1,1));
  pC++;

  ors::Vector dir = ors::Vector(0.,-.7,0.2); dir.normalize();
  t = s.MP->addTask("vecF", new DefaultTaskMap(vecAlignTMT, *s.world, "endeffL", ors::Vector(0.,1.,0.),"handle",dir));
  t->setCostSpecs(U, U, ARR(1.), param(pC));
  weights.append(CostWeight(CostWeight::Dirac,1,ARR(U),1,1));
  pC++;

//  // handle down
////  t = s.MP->addTask("vecU", new DefaultTaskMap(vecTMT, *s.world, "endeffL", ors::Vector(0.,0.,1.)));
////  t->setCostSpecs(U, U, xDemTaskVec[U], param(pC));
////  weights.append(CostWeight(CostWeight::Dirac,1,ARR(U),1,3));
////  pC++;
  // door final position
  t = s.MP->addTask("door_joint", new TaskMap_qItself(s.world->getJointByName("frame_door")->qIndex, s.world->getJointStateDimension()));
  t->setCostSpecs(F, F, ARR(xDem(F,0)), param(pC));
  weights.append(CostWeight(CostWeight::Dirac,1,ARR(F),1,1));
  pC++;

//  t = s.MP->addTask("posC2", new DefaultTaskMap(posTMT, *s.world, "endeffL",NoVector));
//  t->setCostSpecs(U, U, xDemTaskPos[0]*3., param(pC));
//  weights.append(CostWeight(CostWeight::Dirac,1,ARR(C),1,3));
//  pC++;

  /// constraints
  // hold contact endeffector-handle
  t = s.MP->addTask("contact", new PointEqualityConstraint(*s.world, "endeffL",NoVector, "target",NoVector));
  t->setCostSpecs(U+1, F, {0.}, 1.);
  // door fixation
  t = s.MP->addTask("door_fixation", new qItselfConstraint(s.world->getJointByName("frame_door")->qIndex, s.world->getJointStateDimension()));
  t->setCostSpecs(0,U-1, {0.}, 1.);
  t->map.order=1;

  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<endl;
  arr x(T+1,n); x.setZero();arr lambda(T+1,1); lambda.setZero();
  x = repmat(~s.MP->x0,T+1,1);
//  optConstrained(x, lambda, Convert(MPF), OPT(verbose=1,stopTolerance=1e-3));
//  optConstrained(x, lambda, Convert(MPF), OPT(verbose=0,stopTolerance=1e-8,  allowOverstep=true));

//  s.MP->costReport(false);
  if (vis) {
//    cout << lambda << endl;
//    for (;;)
      displayTrajectory(x,s.MP->T,s.MP->world,"t");
  }
  s.world->setJointState(x0);

  // set all costs in s.MP equal to 0 or 1
  for (uint c=0;c < s.MP->taskCosts.N;c++) {
    if (s.MP->taskCosts(c)->map.type==sumOfSqrTT) {
      s.MP->taskCosts(c)->prec /= (s.MP->taskCosts(c)->prec+1e-12);
      cout << s.MP->taskCosts(c)->prec << endl;
    }
  }

  s.MP->prefix.clear();
  s.MP->postfix.clear();
  s.xDem = xDem;
  s.lambdaRef = lambda;
  s.paramRef = param;
}

/*
void MotionFactory::createScene1(Scene &s, mlr::Array<CostWeight> &weights, uint i) {
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
  arr param = {1.,1e2};
  arr w;
  uint pC = 0;
  // transition costs
  s.MP->H_rate_diag = param(pC);
  weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension(),ARR(1.,1e2)));
  pC++;

  // task costs
  TaskCost *c;
  c =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  s.MP->setInterpolatingCosts(c, MotionProblem::constant,conv_vec2arr(tar->X.pos),0.);
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
      if (weights(c+1).type == CostWeight::Dirac) {
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

void MotionFactory::createScene2(Scene &s, mlr::Array<CostWeight> &weights, uint i) {
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
//  arr param = {1.,1e3,80.};
  arr param = {1.};
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
//  s.MP->setInterpolatingCosts(c, MotionProblem::constant,conv_vec2arr(tar->X.pos),0.);
//  weights.append(CostWeight(CostWeight::Gaussian,2,ARR(1.),s.MP->T,3,ARR(1e0,1e3)));
//  weights.last().compWeights(w,NoArr,NoArr,ARR(param(pC),param(pC+1)),true);
//  c->prec = w;
//  cout << w << endl;
//  pC=pC+2;

  TaskCost *c;
  c =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  s.MP->setInterpolatingCosts(c, MotionProblem::constant,conv_vec2arr(tar->X.pos),0.);
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
      if (weights(c+1).type == CostWeight::Dirac) {
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

void MotionFactory::createScene3(Scene &s, mlr::Array<CostWeight> &weights, uint i)
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
  arr param = {1.,1e2,90.,1.5};
  arr w;
  uint pC = 0;
  // transition costs
  s.MP->H_rate_diag = param(pC);
  weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension(),ARR(1.,1e2)));
  pC++;

  // task costs
  TaskCost *c;
  c =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  s.MP->setInterpolatingCosts(c, MotionProblem::constant,conv_vec2arr(tar->X.pos),0.);
//  c->setCostSpecs(s.MP->T,s.MP->T,conv_vec2arr(tar->X.pos),param(pC));
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
      if (weights(c+1).type == CostWeight::Dirac) {
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

void MotionFactory::createScene4(Scene &s, mlr::Array<CostWeight> &weights, uint i)
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

  arr refFrame = conv_vec2arr(s.world->getBodyByName("torso_lift_link")->X.pos);
  ors::Quaternion refFrameQuat = s.world->getBodyByName("torso_lift_link")->X.rot;

  // compute mean orientation of object
  arr mObjQuat = sum(fabs(markerQuatDem),0)/double(markerQuatDem.d0);
  mObjQuat = mObjQuat %(markerQuatDem[0]/fabs(markerQuatDem[0]));
  ors::Quaternion objQuat = refFrameQuat*ors::Quaternion(mObjQuat);//markerQuatDem[0]);

  arr markerOffset = objQuat.getArr()*{-0.126,0.0,0.0415};

  arr q0 = xDem[0];
  arr objPos0 = refFrame + markerOffset + markerDem[0];
  arr objPosT = refFrame + markerOffset + markerDem[markerDem.d0-1];

  s.world->getBodyByName("drawer1")->X.rot =  objQuat;
  if (i==2){
    objPosT = objPosT - {0.,0.33+0.024,0.};
    objPos0 = objPos0 - {0.,0.33+0.024,0.};
    s.world->getBodyByName("drawer1")->X.pos =  objPos0;
  } else if (i==3) {
    objPosT = objPosT - {0.,0.,0.163+0.005};
    objPos0 = objPos0 - {0.,0.,0.163+0.005};
    s.world->getBodyByName("drawer1")->X.pos =  objPos0;
    q0(1)+=0.5;
    q0(0)-=0.2;
  } else if (i==4) {
    objPosT = objPosT - {0.,0.33+0.024,0.163+0.005};
    objPos0 = objPos0 - {0.,0.33+0.024,0.163+0.005};
    s.world->getBodyByName("drawer1")->X.pos =  objPos0;
  }

  s.world->getBodyByName("wall1")->X.pos =  objPosT+{-0.1,0.165+0.026+0.03,-0.};
  s.world->getBodyByName("wall2")->X.pos =  objPosT+{-0.1,0.,0.0815+0.07};

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
  ors::KinematicSwitch *op1 = new ors::KinematicSwitch();
  op1->symbol = ors::KinematicSwitch::addRigid;
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
  if (i==1 && !mlr::getParameter<bool>("evalOnly")) {

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



void MotionFactory::createScene5(Scene &s, mlr::Array<CostWeight> &weights, uint i) {
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
  arr param = {1.,1e3};
  uint pC = 0;
  // transition costs
  s.MP->H_rate_diag = param(pC);
  weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));
  pC++;

  // task costs
  TaskCost *c;
  c =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  c->setCostSpecs(s.MP->T,s.MP->T,conv_vec2arr(tar->X.pos),param(pC));
  weights.append(CostWeight(CostWeight::Dirac,1,ARR(s.MP->T),1,3));
  pC++;

//  c =s.MP->addTask("pos2", new DefaultTaskMap(posTMT, grasp->index) );
//  s.MP->setInterpolatingCosts(c, MotionProblem::constant,conv_vec2arr(tar->X.pos),0.);
  //  c->setCostSpecs(s.MP->T*0.5,s.MP->T*0.5, conv_vec2arr(tar->X.pos), param(N));
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
    s.world->getBodyByName("target")->X.pos = refFrame + markerDem[t]+s.world->getBodyByName("target")->X.rot.getArr()*{-0.1,0.,0.05};
    s.world->getBodyByName("targetRef")->X.pos = refFrame+ markerDem[t];

    s.world->gl().update(STRING(t));
  }
  */

/*
// quaternion test
arr marker0; marker0 << FILE(STRING(demoPath<<"_marker0").p);
arr markerQuat0; markerQuat0 << FILE(STRING(demoPath<<"_markerQuat0").p);
arr refFrame = conv_vec2arr(s.world->getBodyByName("torso_lift_link")->X.pos);


ors::Quaternion torso_rot = s.world->getBodyByName("torso_lift_link")->X.rot;
s.world->getBodyByName("marker")->X.pos = {1.,1.,0.1};
s.world->getBodyByName("marker")->X.rot = torso_rot;

ors::Quaternion world_rot = s.world->getBodyByName("world")->X.rot;
s.world->getBodyByName("marker2")->X.pos = {1.,1.,.5};
s.world->getBodyByName("marker2")->X.rot = world_rot;

ors::Quaternion obj_rot = s.world->getBodyByName("door")->X.rot;
s.world->getBodyByName("marker3")->X.pos = {1.,1.,1.};
s.world->getBodyByName("marker3")->X.rot = obj_rot;

ors::Quaternion door1_rot = ors::Quaternion(markerQuat0[1]);
s.world->getBodyByName("marker4")->X.pos = {1.,1.,1.5};
s.world->getBodyByName("marker4")->X.rot = door1_rot;

ors::Quaternion door1_trans = torso_rot*door1_rot;
s.world->getBodyByName("marker5")->X.pos = {1.,1.,2.};
s.world->getBodyByName("marker5")->X.rot = door1_trans;

ors::Quaternion door1_xRot; door1_xRot.setRad(M_PI_2,ors::Vector(1.,0.,0.));
ors::Quaternion door1_trans2 = door1_xRot*door1_trans;
s.world->getBodyByName("marker6")->X.pos = {1.,1.,2.5};
s.world->getBodyByName("marker6")->X.rot = door1_trans2;

ors::Quaternion door1_xRot2; door1_xRot2.setRad(M_PI_2,door1_trans2.getY());
ors::Quaternion door1_trans3 = door1_xRot2*door1_trans2;
ors::Quaternion door1_zRot; door1_zRot.setRad(M_PI_2,door1_trans3.getZ());
door1_trans3 =door1_trans3*door1_zRot;
s.world->getBodyByName("marker7")->X.pos = {1.,1.,3.};
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
