#include "task_manager.h"
#include "traj_factory.h"

#include "plotUtil.h"

void GraspTask::addConstraints(KOMO *MP, const arr &X)
{
  TrajFactory tf;
  arr yC1,yC2;
  tf.compFeatTraj(X,yC1,MP->world,new DefaultTaskMap(posTMT,MP->world,"endeffC1"));
  tf.compFeatTraj(X,yC2,MP->world,new DefaultTaskMap(posTMT,MP->world,"endeffC2"));

  arr prec = constraintTime*1e3;
  Task *t;
  t = MP->addTask("posC1", new DefaultTaskMap(posTMT,MP->world,"endeffC1"));
  t->target = yC1;
  t->prec = prec;
  t = MP->addTask("posC2", new DefaultTaskMap(posTMT,MP->world,"endeffC2"));
  t->target = yC2;
  t->prec = prec;

}

void GraspTask::updateVisualization(mlr::KinematicWorld &world,arr &X, arr &Y) {
  drawLine(world,X,Pdemo1f,"endeffC1",0,0,constraintCP(0));
  drawLine(world,X,Pdemo1c,"endeffC1",2,constraintCP(0),constraintCP(1));
  drawLine(world,X,Pdemo2f,"endeffC2",0,0,constraintCP(0));
  drawLine(world,X,Pdemo2c,"endeffC2",2,constraintCP(0),constraintCP(1));

  /// draw demo
}

void GraspTask::computeConstraintTime(const arr &F,const arr &X) {
  constraintTime = zeros(F.d0); constraintTime.flatten();
  uint qIdx = world->getJointByName("l_gripper_joint")->qIndex;
  for (uint t=0;t<F.d0;t++){
    if(fabs(X(t,qIdx)) < mlr::getParameter<double>("grasp_threshold")) {
      constraintTime(t) = 1.;
      constraintTime.subRange(t-5,t) = 1.;
//      break;
    }
  }
  constraintTime(constraintTime.d0-1) = 1.;
  constraintCP = ARR(constraintTime.findValue(1.),F.d0); constraintCP.flatten();

  cout << "constraintTime: " << constraintTime << endl;
  cout << "constraintCP: " << constraintCP << endl;
}

bool GraspTask::success(const arrA& X, const arr &Y) {
//  cout << catCol(X,Y) << endl;
  return true;//length(X[X.d0-1] - Y[Y.d0-1])<0.06;
}

void GraspTask::getParamLimit(arr& paramLimit)
{

}

bool GraspTask::transformTrajectory(arr &Xn, const arr &theta, arr &Xdemo){
  arr C1demo,C2demo,Gdemo;
  TrajFactory tf;
  tf.compFeatTraj(Xdemo,C1demo,*world,new DefaultTaskMap(posTMT,*world,"endeffC1"));
  tf.compFeatTraj(Xdemo,C2demo,*world,new DefaultTaskMap(posTMT,*world,"endeffC2"));
  tf.compFeatTraj(Xdemo,Gdemo,*world,new TaskMap_qItself(*world,"l_gripper_joint"));

  arr C1trans = C1demo;
  arr C2trans = C2demo;
  arr CP1, CP2;

  world->gl().resize(800,800);
  updateVisualization(*world,Xdemo);

  for (uint t=0;t<Xdemo.d0;t++) {
    world->setJointState(Xdemo[t]);
    mlr::Body *handle = world->getBodyByName("handle");
    mlr::Shape *ec1 = world->getShapeByName("endeffC1");
    mlr::Shape *ec2 = world->getShapeByName("endeffC2");

    handle->X.pos = (C1demo[t]+C2demo[t])/2.;
    handle->X.rot = ec1->X.rot;
    double d = length(conv_vec2arr(ec1->X.pos-ec2->X.pos));
    double h = handle->shapes(0)->size[2];
    double w = ec1->rel.pos(0)*2.;

    double alpha = asin(w/d)*180./M_PI;
    double beta = asin(h/d)*180./M_PI;
    handle->X.addRelativeRotationDeg(90.,0.,1.,0.);
    handle->X.addRelativeRotationDeg(-alpha-beta,1.,0.,0.);

    double trans = sqrt(d*d-h*h)*0.5;
    world->getShapeByName("cp1")->rel.pos.y = -trans - theta(0) + theta(1);
    world->getShapeByName("cp2")->rel.pos.y = trans +  theta(1);

    CP1.append(~conv_vec2arr(world->getShapeByName("cp1")->X.pos));
    CP2.append(~conv_vec2arr(world->getShapeByName("cp2")->X.pos));

    world->gl().update(STRING(t));
  }

  arr offsetC1 = CP1[constraintCP(0)] - C1trans[constraintCP(0)];
  arr offsetC2 = CP2[constraintCP(0)] - C2trans[constraintCP(0)];

  double s;
  for (uint t=0;t<constraintCP(0);t++) {
    s = t/constraintCP(0);
    C1trans[t] = C1trans[t] +s*offsetC1;
    C2trans[t] = C2trans[t] +s*offsetC2;
  }
  for (uint t=constraintCP(0);t<constraintCP(1);t++) {
    C1trans[t] = CP1[t];
    C2trans[t] = CP2[t];
  }

  KOMO MP(*world,false);
  MP.T = Xdemo.d0-1;
  MP.tau = mlr::getParameter<double>("duration")/MP.T;
  MP.x0 = Xdemo[0];

  //--tasks
  Task *t;
  t = MP.addTask("tra", new TransitionTaskMap(*world));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e-1);
  ((TransitionTaskMap*)&t->map)->H_rate_diag = world->getHmetric();

  t =MP.addTask("posC1", new DefaultTaskMap(posTMT,*world,"endeffC1"));
  t->setCostSpecs(0,MP.T, C1trans, 1e3);
  t =MP.addTask("posC2", new DefaultTaskMap(posTMT,*world,"endeffC2"));
  t->setCostSpecs(0,MP.T, C2trans, 1e3);

  /// fix unused joints
//  t = MP.addTask("worldTranslationRotation", new qItselfConstraint(world->getJointByName("worldTranslationRotation")->qIndex, world->getJointStateDimension()));
//  t->setCostSpecs(0.,MP.T, ARR(0.), 1.);
//  t = MP.addTask("worldTranslationRotation1", new qItselfConstraint(world->getJointByName("worldTranslationRotation")->qIndex+1, world->getJointStateDimension()));
//  t->setCostSpecs(0.,MP.T, ARR(0.), 1.);
//  t = MP.addTask("worldTranslationRotation2", new qItselfConstraint(world->getJointByName("worldTranslationRotation")->qIndex+2, world->getJointStateDimension()));
//  t->setCostSpecs(0.,MP.T, ARR(0.), 1.);
//  t = MP.addTask("torso", new qItselfConstraint(world->getJointByName("torso_lift_joint")->qIndex, world->getJointStateDimension()));
//  t->setCostSpecs(0.,MP.T, ARR(0.), 1.);
//  t->map.order = 1;


  MotionProblemFunction MPF(MP);
  Xn = Xdemo;
  OptOptions o;
  o.stopTolerance = 1e-3; o.constrainedMethod=anyTimeAula; o.verbose=0;
  optConstrained(Xn, NoArr, Convert(MPF), o);

  // augment gripper joints
  for (uint t=0;t<Xn.d0;t++) {
    Xn(t,24) = Xn(t,22)*0.1743;
//    Xn(t,24) = sin(Xn(t,22))*0.182681221532615 + cos(Xn(t,22))*0.011489460574348 - Xn(t,22)*0.00004369554688;
  }

  arr Pn1,Pn2;
//  drawPoints(*world,Xn,Pn1,"endeffC1",1);
//  drawPoints(*world,Xn,Pn2,"endeffC2",1);

  // check if gripper limits are exceeded
  cout << " parameter min=" << Xn.col(24).min() << " max=" << Xn.col(24).max() << endl;
  cout << " parameter lim upper=" << world->getJointByName("l_gripper_joint")->limits(1) << " lower=" << world->getJointByName("l_gripper_joint")->limits(0) << endl;

  if (Xn.col(24).max() > world->getJointByName("l_gripper_joint")->limits(1) || Xn.col(24).min() < world->getJointByName("l_gripper_joint")->limits(0)){
    cout << "limit reached" << endl;
    return false;
  }else{
    return true;
  }
}

double GraspTask::reward(const arr &Z){
  return exp(-.2*sumOfAbs(Z)/Z.d0);
}

double GraspTask::cost(const arr& Z){
  arr tmp;
  getAcc(tmp,Z,1.);
  return sumOfSqr(tmp);
}
