#include "motion_factory.h"

#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/taskMap_transition.h>
#include <Ors/ors_swift.h>
#include <Core/geo.h>


void MotionFactory::execMotion(Scene &s, arr cost_param, bool vis, uint verbose) {
  cost_param = cost_param/sqrt(sumOfSqr(cost_param))*costScale;

  // set cost_param
  s.MP->H_rate_diag = cost_param(0);

  for (uint c=0;c<s.MP->taskCosts.N;c++) {
    if (!s.MP->taskCosts(c)->map.constraint) {
      s.MP->taskCosts(c)->prec *= cost_param(c+1);
    }
  }

  // reinit swift interface
  s.world->swift().initActivations(*s.world);

  // optimize the motion problem
  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<s.MP->world.getJointStateDimension()<<endl;
  arr x(T+1,n); x.setZero();arr lambda(T+1); lambda.setZero();
  x = repmat(~s.MP->x0,T+1,1);
//  x = randn(T+1,n);
//  x = s.xInit;
  optConstrained(x, lambda, Convert(MPF), OPT(verbose=verbose,stopTolerance=1e-2));
  optConstrained(x, lambda, Convert(MPF), OPT(verbose=verbose,stopTolerance=1e-4, allowOverstep=true));


  // visualize trajectory
  if (vis) {
    cout <<  "lambda: " << lambda << endl;
    s.MP->costReport(true);
    displayTrajectory(x,T,s.MP->world,"world");
  }

  s.MP->H_rate_diag=1.;
  // reset cost_param to 0-1 values
  for (uint c=0;c<s.MP->taskCosts.N;c++) {
    if (!s.MP->taskCosts(c)->map.constraint) {
      s.MP->taskCosts(c)->prec /= (cost_param(c+1)+1e-12);
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
      case 0:
        /// scene with one position task map
        createScene0(s_train,i,vis);
        createScene0(s_test,i+numScenes,vis);
        break;
      case 1:
        /// scene with simple table contact
        createScene1(s_train,i,vis);
        createScene1(s_test,i+numScenes,vis);
        break;
      case 2:
        /// scene for sliding a box
        createScene2(s_train,i,vis);
        createScene2(s_test,i+numScenes,vis);
        break;
      case 3:
        /// scene with many arbitrary task maps
        createScene3(s_train,i,vis);
        createScene3(s_test,i+numScenes,vis);
        break;
      case 4:
        /// pr2 closing a drawer
        createScene4(s_train,1,vis);
        createScene4(s_test,3,vis);
        break;
      case 5:
        /// nonlinear simple example
        createScene5(s_train,1,vis);
        createScene5(s_test,3,vis);
        break;
    }
    s_test.xInit = s_train.xRef;
    s_train.xInit = s_train.xRef;

    trainScenes.append(s_train);
    testScenes.append(s_test);
  }

  // compute number of parameters
  numParam = 0;
  numParam += 1; //transition costs
  for (uint c=0;c<trainScenes(0).MP->taskCosts.N;c++) {
    if (!trainScenes(0).MP->taskCosts(c)->map.constraint) {
      numParam++;
    }
  }
  cout << "IOC Number of Parameter: " << numParam << endl;
}


void MotionFactory::createScene0(Scene &s, uint i, bool vis) {
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
  s.MP->T = 20;
  s.MP->tau = 0.01;

  //-- setup new motion problem
  s.world->getBodyByName("target")->X.pos += double(i)*ARR(0.,0.1,0.);

  ors::Shape *grasp = s.world->getShapeByName("endeff");
  ors::Body *tar = s.world->getBodyByName("target");

  // only work with normalized task costs (for comparison)
  arr param = ARR(s.MP->H_rate_diag(0));
  param.append(ARR(1e3));
  param = param/sqrt(sumOfSqr(param))*costScale;
  cout << param << endl;
  s.MP->H_rate_diag = param(0);

  uint N = 1;

  TaskCost *c;
  c =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  c->setCostSpecs(s.MP->T-1,s.MP->T, ARRAY(tar->X.pos), param(N));

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
  arr param = ARR(s.MP->H_rate_diag(0));
  param.append(ARR(1e3,1e2));
  param = param/sqrt(sumOfSqr(param))*costScale;
  s.MP->H_rate_diag = param(0);
  cout << param << endl;
  uint N = 1;

  TaskCost *c;
  c =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  c->setCostSpecs(s.MP->T,s.MP->T, ARRAY(tar->X.pos), param(N));

  TaskMap *tm_contact = new PairCollisionConstraint(*s.world,"table","endeff",0.01);
  TaskCost *c4 = s.MP->addTask("contact_endeff",tm_contact);
  c4->map.constraint = false;
  c4->setCostSpecs(s.MP->T/2-2.,s.MP->T/2.+2, ARR(0.) ,param(N+1));

  c = s.MP->addTask("collisionConstraints", new PairCollisionConstraint(*s.world,"endeff","table",0.01));
  s.MP->setInterpolatingCosts(c, MotionProblem::constant,ARR(0.),1e0);

//  c = s.MP->addTask("collisionConstraints2", new PairCollisionConstraint(*s.world,"table","R_LOWER_WRIST",0.05));
//  s.MP->setInterpolatingCosts(c, MotionProblem::constant,ARR(0.),1e0);

  s.MP->x0 = zeros(s.world->getJointStateDimension(),1);s.MP->x0.flatten();
  s.MP->x0(0) = M_PI_2;

  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" << N<<endl;
  arr x(T+1,n); x.setZero();arr lambda(T+1,1); lambda.setZero();
  x = repmat(~s.MP->x0,T+1,1);
  optConstrained(x, lambda, Convert(MPF), OPT(verbose=0,stopTolerance=1e-4, allowOverstep=true));
  optConstrained(x, lambda, Convert(MPF), OPT(verbose=0,stopTolerance=1e-4,  allowOverstep=true));

  if (vis) {
    cout << "Lambda" << lambda << endl;
    s.MP->costReport(true);
    displayTrajectory(x,s.MP->T,s.MP->world,"t");
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
//  arr param = s.MP->H_rate_diag;
//  param.append(ARR(1e5,1e3,1e3,1e2,1e2,1e2));
//  param = param/sqrt(sumOfSqr(param))*costScale;
//  s.MP->H_rate_diag = param.subRange(0,s.MP->H_rate_diag.d0-1);
//  cout << param << endl;
//  uint N = s.world->getJointStateDimension();

  arr param = ARR(s.MP->H_rate_diag(0));
  param.append(ARR(1e3,1e2,1e0,1e3,1e1,1e1));
  param = param/sqrt(sumOfSqr(param))*costScale;
  s.MP->H_rate_diag = param(0);//param.subRange(0,s.MP->H_rate_diag.d0-1);
  cout << "Parameter: " << param << endl;
  uint N = 1;//s.world->getJointStateDimension();


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

//  c = s.MP->addTask("collisionAvoidConstraints", new PairCollisionConstraint(*s.world,"R_HAND_R_FINGER_KNUCKLE_3","table",0.01));
//  s.MP->setInterpolatingCosts(c, MotionProblem::constant,ARR(0.),1e0);

  s.MP->x0 = zeros(s.world->getJointStateDimension(),1);s.MP->x0.flatten();
  s.MP->x0 = ARR( 0.188712, -0.866221, 0.447127, 0.387585, -0.471564, -4.3981, -0.689431);

  MotionProblemFunction MPF(*s.MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt = s.MP->tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" << N<<endl;
  arr x(T+1,n); x.setZero();arr lambda(T+1); lambda.setZero();
  x = repmat(~s.MP->x0,T+1,1);
  optConstrained(x, lambda, Convert(MPF), OPT(verbose=0,stopTolerance=1e-4, allowOverstep=true));
  optConstrained(x, lambda, Convert(MPF), OPT(verbose=0,stopTolerance=1e-7,  allowOverstep=true));

  if (vis) {
    cout << "Lambda" << lambda << endl;
//    s.MP->costReport(true);
    displayTrajectory(x,s.MP->T,s.MP->world,"t");
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

void MotionFactory::createScene4(Scene &s, uint i, bool vis)
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
      s.world->gl().update(STRING(t));
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
  //  cout << "costGrid"<<costGrid << endl;*/
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
  s.xRef = xDem;
  s.lambdaRef = lambda;
  s.paramRef = param;
  s.contactTime = contactTime;
}



void MotionFactory::createScene5(Scene &s, uint i, bool vis) {
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
  s.MP->T = 20;
  s.MP->tau = 0.1;

  //-- setup new motion problem
  s.world->getBodyByName("target")->X.pos += double(i)*ARR(0.,0.1,0.);

  ors::Shape *grasp = s.world->getShapeByName("endeff");
  ors::Body *tar = s.world->getBodyByName("target");

  // only work with normalized task costs (for comparison)
  arr param = ARR(s.MP->H_rate_diag(0));
  param.append(ARR(1e3));
  param = param/sqrt(sumOfSqr(param))*costScale;
  cout << param << endl;
  s.MP->H_rate_diag = param(0);

  uint N = 1;

  TaskCost *c;
  c =s.MP->addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  s.MP->setInterpolatingCosts(c, MotionProblem::constant,ARRAY(tar->X.pos),0.);
//  c->setCostSpecs(s.MP->T*0.5,s.MP->T*0.5, ARRAY(tar->X.pos), param(N));
  c->prec(s.MP->T*0.5) = param(N);

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
//    s.MP->costReport(true);
    displayTrajectory(x,s.MP->T/2.,s.MP->world,"t");
  }

  // set all costs equal to 1 or 0
  s.MP->H_rate_diag = 1.;
  for (uint c=0;c < s.MP->taskCosts.N;c++) {
    if (!s.MP->taskCosts(c)->map.constraint) {
      s.MP->taskCosts(c)->prec = 1.;
    }
  }

  s.xRef = x;
  s.lambdaRef = lambda;
  s.paramRef = param;
}



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
