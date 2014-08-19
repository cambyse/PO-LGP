#include "motion_factory.h"

#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/taskMap_transition.h>
#include <Ors/ors_swift.h>


void MotionFactory::execMotion(Scene &s, arr cost_param, bool vis) {
  cost_param = cost_param/sqrt(sumOfSqr(cost_param))*costScale;

  // set cost_param
  for (uint c=0;c<s.MP->H_rate_diag.N;c++) {
    s.MP->H_rate_diag(c) = cost_param(c);
  }
  for (uint c=0;c<s.MP->taskCosts.N;c++) {
    if (!s.MP->taskCosts(c)->map.constraint) {
      s.MP->taskCosts(c)->prec *= cost_param(c+s.MP->H_rate_diag.N);
    }
  }

  // reinit swift interface
  s.world->swift().initActivations(*s.world);

  // optimize the motion problem
  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<s.MP->world.getJointStateDimension()<<endl;
  arr x(T+1,n); x.setZero();arr lambda(T+1); lambda.setZero();
//  x = repmat(~s.MP->x0,T+1,1);
  x = s.xInit;
  optConstrained(x, lambda, Convert(MPF), OPT(verbose=0,stopTolerance=1e-4, allowOverstep=true));
  optConstrained(x, lambda, Convert(MPF), OPT(verbose=0,stopTolerance=1e-4,  allowOverstep=true));

  // visualize trajectory
  if (vis) {
    cout <<  "lambda: " << lambda << endl;
    s.MP->costReport(true);
    displayTrajectory(x,T,s.MP->world,"world");
  }

  // reset cost_param to 0-1 values
  for (uint c=0;c<s.MP->taskCosts.N;c++) {
    if (!s.MP->taskCosts(c)->map.constraint) {
      s.MP->taskCosts(c)->prec /= cost_param(c+s.MP->H_rate_diag.N);
    }
  }
}


void MotionFactory::createScenes(uint sID,MT::Array<Scene> &trainScenes, MT::Array<Scene> &testScenes, uint numScenes, bool vis)
{
  costScale = 1e2;

  // create training scenes
  for (uint i = 0; i < numScenes; i++) {
    Scene s_train;
    Scene s_test;
    switch(sID) {
      case 1:
        createScene1(s_train,i,vis);
        createScene1(s_test,i+numScenes,vis);
        break;
      case 2:
        createScene2(s_train,i,vis);
        createScene2(s_test,i+numScenes,vis);
        break;
      case 3:
        createScene3(s_train,i,vis);
        createScene3(s_test,i+numScenes,vis);
        break;
    }
    s_test.xInit = s_train.xRef;
    s_train.xInit = s_train.xRef;

    trainScenes.append(s_train);
    testScenes.append(s_test);
  }

  // compute number of parameters
  numParam = 0;
  numParam += trainScenes(0).MP->H_rate_diag.d0;
  for (uint c=0;c<trainScenes(0).MP->taskCosts.N;c++) {
    if (!trainScenes(0).MP->taskCosts(c)->map.constraint) {
      numParam++;
    }
  }
  cout << "IOC Number of Parameter: " << numParam << endl;
}

void MotionFactory::createScene1(Scene &s, uint i, bool vis) {
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

  //-- setup new motion problem
  s.world->getBodyByName("target")->X.pos += double(i)*ARR(0.,0.1,0.);

  ors::Shape *grasp = s.world->getShapeByName("endeff");
  ors::Body *tar = s.world->getBodyByName("target");


  // only work with normalized task costs (for comparison)
  arr param = s.MP->H_rate_diag;
  param.append(ARR(1e3,1e2));
  param = param/sqrt(sumOfSqr(param))*costScale;
  s.MP->H_rate_diag = param.subRange(0,s.MP->H_rate_diag.d0-1);
  cout << param << endl;
  uint N = s.world->getJointStateDimension();

  TaskCost *c;
  c =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  c->setCostSpecs(s.MP->T,s.MP->T, ARRAY(tar->X.pos), param(N));

  TaskMap *tm_contact = new PairCollisionConstraint(*s.world,"table","endeff",0.01);
  TaskCost *c4 = s.MP->addTask("contact_endeff",tm_contact);
  c4->map.constraint = false;
  c4->setCostSpecs(s.MP->T/2-2.,s.MP->T/2.+2, ARR(0.) ,param(N+1));

  c = s.MP->addTask("collisionConstraints", new PairCollisionConstraint(*s.world,"endeff","table",0.01));
  s.MP->setInterpolatingCosts(c, MotionProblem::constant,ARR(0.),1e0);

  c = s.MP->addTask("collisionConstraints2", new PairCollisionConstraint(*s.world,"table","R_LOWER_WRIST",0.05));
  s.MP->setInterpolatingCosts(c, MotionProblem::constant,ARR(0.),1e0);


  s.MP->x0 = zeros(s.world->getJointStateDimension(),1);s.MP->x0.flatten();
  s.MP->x0(0) = M_PI_2;

  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" << N<<endl;
  arr x(T+1,n); x.setZero();arr lambda(T+1,2); lambda.setZero();
  x = repmat(~s.MP->x0,T+1,1);
  optConstrained(x, lambda, Convert(MPF), OPT(verbose=0,stopTolerance=1e-4, allowOverstep=true));
  optConstrained(x, lambda, Convert(MPF), OPT(verbose=0,stopTolerance=1e-4,  allowOverstep=true));

  if (vis) {
    cout << "Lambda" << lambda << endl;
    s.MP->costReport(true);
    displayTrajectory(x,s.MP->T/2.,s.MP->world,"t");
  }

  // set all costs equal to 1 or 0
  s.MP->H_rate_diag = 1.;
  for (uint c=0;c < s.MP->taskCosts.N;c++) {
    if (!s.MP->taskCosts(c)->map.constraint) {
      s.MP->taskCosts(c)->prec = s.MP->taskCosts(c)->prec/(s.MP->taskCosts(c)->prec+1e-12);
    }
  }

  s.xRef = x;
  s.lambdaRef = lambda;
  s.paramRef = param;
}

void MotionFactory::createScene2(Scene &s, uint i, bool vis) {
  s.world = new ors::KinematicWorld("scene2");
  arr q, qdot;
  s.world->getJointState(q, qdot);

  makeConvexHulls(s.world->shapes);
  s.world->swift().setCutoff(10.);
  s.world->swift();
  s.MP = new MotionProblem(*s.world,false);
  s.MP->useSwift=true;
  s.MP->loadTransitionParameters();
  s.MP->makeContactsAttractive=false;

  // add graph operator
  ors::GraphOperator *op1 = new ors::GraphOperator();
  op1->symbol = ors::GraphOperator::addRigid;
  op1->timeOfApplication = s.MP->T/2;
  op1->fromId = s.world->getBodyByName("endeff")->index;
  op1->toId = s.world->getBodyByName("obj1")->index;
  s.world->operators.append(op1);

  ors::GraphOperator *op2 = new ors::GraphOperator();
  op2->symbol = ors::GraphOperator::deleteJoint;
  op2->timeOfApplication = s.MP->T/2;
  op2->fromId = s.world->getBodyByName("table")->index;
  op2->toId = s.world->getBodyByName("obj1")->index;
  s.world->operators.append(op2);


  //-- setup new motion problem
  s.world->getJointByName("table_target")->B.pos += double(i)*ARR(0.0,0.05,.0);
  s.world->calc_Q_from_q();
  s.world->calc_fwdPropagateFrames();

  ors::Shape *grasp = s.world->getShapeByName("endeff");
  ors::Body *obj = s.world->getBodyByName("obj1");
  ors::Body *tar = s.world->getBodyByName("target");

  // only work with normalized task costs (for comparison)
  arr param = s.MP->H_rate_diag;
  param.append(ARR(1e5,1e3,1e3,1e2,1e2,1e2));
  param = param/sqrt(sumOfSqr(param))*costScale;
  s.MP->H_rate_diag = param.subRange(0,s.MP->H_rate_diag.d0-1);
  cout << param << endl;
  uint N = s.world->getJointStateDimension();

  TaskCost *c;
  c =s.MP->addTask("pos1", new DefaultTaskMap(posTMT, obj->shapes(0)->index) );
  c->setCostSpecs(s.MP->T,s.MP->T, ARRAY(tar->X.pos), param(N++));

  c =s.MP->addTask("pos2", new DefaultTaskMap(posTMT, grasp->index) );
  c->setCostSpecs(s.MP->T/2-2.,s.MP->T/2+2., ARRAY(obj->X.pos), param(N++));

  c =s.MP->addTask("rot1", new DefaultTaskMap(quatTMT, grasp->index) );
  c->setCostSpecs(s.MP->T/2-2, s.MP->T/2+2, ARRAY(obj->X.rot), param(N++));

  c =s.MP->addTask("rot2", new DefaultTaskMap(quatTMT, grasp->index) );
  c->setCostSpecs(s.MP->T, s.MP->T, ARRAY(tar->X.rot), param(N++));

  c =s.MP->addTask("vel", new DefaultTaskMap(qItselfTMT) );
  c->map.order = 1.;
  c->setCostSpecs(s.MP->T/2-2, s.MP->T/2+2, zeros(s.world->getJointStateDimension()), param(N++));

  TaskMap *tm_contact = new PairCollisionConstraint(*s.world,"obj1","table",0.01);
  TaskCost *c4 = s.MP->addTask("contact_endeff",tm_contact);
  c4->map.constraint = false;
  c4->setCostSpecs(s.MP->T/2.,s.MP->T, ARR(0.) ,param(N++));

  c = s.MP->addTask("collisionConstraints", new PairCollisionConstraint(*s.world,"obj1","table",0.01));
  s.MP->setInterpolatingCosts(c, MotionProblem::constant,ARR(0.),1e0);
  s.MP->x0 = zeros(s.world->getJointStateDimension(),1);s.MP->x0.flatten();
  s.MP->x0 = ARR( 0.188712, -0.866221, 0.447127, 0.387585, -0.471564, -4.3981, -0.689431);

  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" << N<<endl;
  arr x(T+1,n); x.setZero();arr lambda(T+1); lambda.setZero();
  x = repmat(~s.MP->x0,T+1,1);
  optConstrained(x, lambda, Convert(MPF), OPT(verbose=0,stopTolerance=1e-4, allowOverstep=true));
  optConstrained(x, lambda, Convert(MPF), OPT(verbose=0,stopTolerance=1e-4,  allowOverstep=true));

  if (vis) {
    cout << "Lambda" << lambda << endl;
    s.MP->costReport(true);
    displayTrajectory(x,s.MP->T/2.,s.MP->world,"t");
  }

  // set all costs equal to 1 or 0
  s.MP->H_rate_diag = 1.;
  for (uint c=0;c < s.MP->taskCosts.N;c++) {
    if (!s.MP->taskCosts(c)->map.constraint) {
      s.MP->taskCosts(c)->prec = s.MP->taskCosts(c)->prec/(s.MP->taskCosts(c)->prec+1e-12);
    }
  }


  s.xRef = x;
  s.lambdaRef = lambda;
  s.paramRef = param;
}

void MotionFactory::createScene3(Scene &s, uint i, bool vis)
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

  //-- setup new motion problem
  s.world->getBodyByName("target")->X.pos += double(i)*ARR(0.,0.1,0.);

  ors::Shape *grasp = s.world->getShapeByName("endeff");
  ors::Body *tar = s.world->getBodyByName("target");


  // only work with normalized task costs (for comparison)
  arr param = s.MP->H_rate_diag;
  param.append(ARR(1e3,1e2));
  param = param/sqrt(sumOfSqr(param))*costScale;
  s.MP->H_rate_diag = param.subRange(0,s.MP->H_rate_diag.d0-1);
  cout << param << endl;
  uint N = s.world->getJointStateDimension();

  TaskCost *c;
  c =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  c->setCostSpecs(s.MP->T,s.MP->T, ARRAY(tar->X.pos), param(N));

  TaskMap *tm_contact = new PairCollisionConstraint(*s.world,"table","endeff",0.01);
  TaskCost *c4 = s.MP->addTask("contact_endeff",tm_contact);
  c4->map.constraint = false;
  c4->setCostSpecs(s.MP->T/2-2.,s.MP->T/2.+2, ARR(0.) ,param(N+1));

  // add more arbitrary task costs
  for (uint idx=0;idx<500;idx++) {
    c = s.MP->addTask(STRING("pos"<<idx),new DefaultTaskMap(posTMT, grasp->index) );
    c->setCostSpecs(s.MP->T/(idx+2.),s.MP->T/(idx+2.), ARRAY(tar->X.pos), 1.);
    param.append(ARR(1.));
    c->active=false;
  }

  // add more arbitrary task costs
  for (uint idx=0;idx<10;idx++) {
    c = s.MP->addTask(STRING("rot1"<<idx),new DefaultTaskMap(quatTMT, grasp->index) );
    c->setCostSpecs(s.MP->T/(idx+1.),s.MP->T/(idx+1.), ARRAY(tar->X.rot), 1.);
    param.append(ARR(1.));
    c->active=false;
  }

  c = s.MP->addTask("collisionConstraints", new PairCollisionConstraint(*s.world,"endeff","table",0.01));
  s.MP->setInterpolatingCosts(c, MotionProblem::constant,ARR(0.),1e0);

  c = s.MP->addTask("collisionConstraints2", new PairCollisionConstraint(*s.world,"table","R_LOWER_WRIST",0.05));
  s.MP->setInterpolatingCosts(c, MotionProblem::constant,ARR(0.),1e0);

  s.MP->x0 = zeros(s.world->getJointStateDimension(),1);s.MP->x0.flatten();
  s.MP->x0(0) = M_PI_2;

  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" << N<<endl;
  arr x(T+1,n); x.setZero();arr lambda(T+1,2); lambda.setZero();
  x = repmat(~s.MP->x0,T+1,1);
  optConstrained(x, lambda, Convert(MPF), OPT(verbose=0,stopTolerance=1e-4, allowOverstep=true));
  optConstrained(x, lambda, Convert(MPF), OPT(verbose=0,stopTolerance=1e-4,  allowOverstep=true));

  if (vis) {
    cout << "Lambda" << lambda << endl;
    s.MP->costReport(true);
    displayTrajectory(x,s.MP->T/2.,s.MP->world,"t");
  }



  // set all costs equal to 1 or 0
  s.MP->H_rate_diag = 1.;
  for (uint c=0;c < s.MP->taskCosts.N;c++) {
    if (!s.MP->taskCosts(c)->map.constraint) {
      s.MP->taskCosts(c)->active = true;
      s.MP->taskCosts(c)->prec = s.MP->taskCosts(c)->prec/(s.MP->taskCosts(c)->prec+1e-12);
    }
  }

  s.xRef = x;
  s.lambdaRef = lambda;
  s.paramRef = param;
}

