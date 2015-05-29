#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Gui/plot.h>
#include <Algo/spline.h>
#include <GL/glu.h>
#include <Gui/opengl.h>
#include <Optim/search.h>

#include "../src/motion_factory.h"
#include "../src/cost_weight.h"
#include "../src/inverse_motion.h"
#include "../src/plotUtil.h"
#include "../src/traj_factory.h"

void testPathIMP() {
  /// create reference motion
  ors::KinematicWorld world("scene");
  arr q, qdot;
  world.getJointState(q, qdot);
  world.swift();
  MotionProblem MP(world,false);
  MP.T = 50;
  MP.tau = 0.05;
  ors::Shape *grasp = world.getShapeByName("endeff");
  ors::Body *target1 = world.getBodyByName("target1");
  ors::Body *target2 = world.getBodyByName("target2");
  Task *t;
  t = MP.addTask("tra", new TransitionTaskMap(world));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e-2);
  t =MP.addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(MP.T-3,MP.T,ARRAY(target1->X.pos),1e2);
  MotionProblemFunction MPF(MP);
  arr x(MP.T+1,q.N); x.setZero();
  optConstrainedMix(x, NoArr, Convert(MPF), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-3));

  /// convert to task space traj
  arr y(MP.T+1,3);
  for (uint t=0;t<y.d0;t++) {
    arr tmp;
    world.setJointState(x[t]);
    world.kinematicsPos(tmp,NoArr,grasp->body,&grasp->rel.pos);
    y[t] = tmp;
  }

  /// transform traj
  MT::Path P(y,2);
  world.gl().add(drawGreenPoints,&(P.points));
  MT::Path P2(y,2);
  arr expl = ARR(0.,0.,0.1);
  P2.transform_CurrentFixed_EndBecomes(y[y.d0-1]+expl,0.);
  world.gl().add(drawGreenPoints,&(P2.points));

  /// convert back into joint space
  MP.taskCosts.clear();
  t = MP.addTask("tra", new TransitionTaskMap(world));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e-2);
  t =MP.addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(0,MP.T,P2.points,1e2);
  arr x2(MP.T+1,q.N); x2.setZero();
  MP.setState(x[0]);
  MotionProblemFunction MPF2(MP);
  optConstrainedMix(x2, NoArr, Convert(MPF2), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-3));

  /// IMP on transformed traj
  MotionFactory* mf = new MotionFactory();
  Scenario scenario;
  scenario.costScale = 1e2;
  Scene s;
  s.world = &world;
  s.MP = &MP;
  s.MP->taskCosts.clear();
  s.optConstraintsParam = false;
  s.xDem = x2;
  s.x0 = x[0];

  arr param = ARRAY(1e-2,1e2,0.);
  param = param/sum(param)*scenario.costScale;
  t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
  t->map.order=2;
  t->target = ARRAY(0.);
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
  scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));

  t =s.MP->addTask("pos2", new DefaultTaskMap(posTMT, grasp->index) );
  t->target = ARRAY(target2->X.pos);
  scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T-3,s.MP->T),1,3));
  t =s.MP->addTask("pos1", new DefaultTaskMap(posTMT, grasp->index) );
  t->target = ARRAY(target1->X.pos);
  scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T-3,s.MP->T),1,3));

  scenario.scenes.append(s);
  scenario.paramGT = param;
  scenario.setParam(ones(param.d0));

  InverseMotionProblem IMP(scenario);
  param = IMP.initParam(InverseMotionProblem::ONES);
  arr param0 =param;
  checkAllGradients(IMP,param0,1e-2);
  optConstrained(param,NoArr,IMP,OPT(verbose=0,stopTolerance=1e-5,stepInc=2,aulaMuInc=1,maxStep=-1., constrainedMethod=augmentedLag, stopIters=1000,dampingInc=1.));
  IMP.costReport(param,param0);

  scenario.setParam(param);
  mf->execMotion(scenario.scenes(0),NoArr,NoArr,NoArr,1);


  /// plotting
  plotClear();
  plotFunction(P.points.col(0));
  plotFunction(P.points.col(1));
  plotFunction(P.points.col(2));
  plotFunction(P2.points.col(0));
  plotFunction(P2.points.col(1));
  plotFunction(P2.points.col(2));
  plot(true);

  world.watch(true);
  displayTrajectory(x,MP.T,world,"world");
  displayTrajectory(x2,MP.T,world,"world");
}

void testPathBBO() {
  /// create reference motion
  ors::KinematicWorld world("scene");
  arr q, qdot;
  world.getJointState(q, qdot);
  world.swift();
  MotionProblem MP(world,false);
  MP.T = 50;
  MP.tau = 0.05;
  ors::Shape *grasp = world.getShapeByName("endeff");
  ors::Body *target1 = world.getBodyByName("target1");
  ors::Body *target2 = world.getBodyByName("target2");
  Task *t;
  t = MP.addTask("tra", new TransitionTaskMap(world));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e-2);
  t =MP.addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(MP.T-3,MP.T,ARRAY(target1->X.pos),1e2);
  MotionProblemFunction MPF(MP);
  arr x(MP.T+1,q.N); x.setZero();
  optConstrainedMix(x, NoArr, Convert(MPF), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-3));

  /// convert to task space traj
  arr y(MP.T+1,3);
  for (uint t=0;t<y.d0;t++) {
    arr tmp;
    world.setJointState(x[t]);
    world.kinematicsPos(tmp,NoArr,grasp->body,&grasp->rel.pos);
    y[t] = tmp;
  }

  /// transform traj
  MT::Path P(y,2);
  world.gl().add(drawGreenPoints,&(P.points));
  MT::Path P2(y,2);
  arr expl = ARR(0.,0.,0.1);
  P2.transform_CurrentFixed_EndBecomes(y[y.d0-1]+expl,0.);

  SearchCMA cma;
  int cmaD = 1;
  int cmaMu = -1;
  int cmaLambda = -1;
//  double cmaLo = 20.;
//  double cmaHi = 30.;

  arr param = ARR(0.);
  arr paramStd = ARR(.2);
  cma.init(cmaD,cmaMu,cmaLambda,param,paramStd);

  arr samples,values;
  for(uint t=0;t<50;t++){
    cma.step(samples, values);

    for(uint i=0;i<samples.d0;i++) {
      param = samples[i];
      P2 = P;
      P2.transform_CurrentFixed_EndBecomes(y[y.d0-1]+ARR(0.,0.,param(0)),0.);
      values(i) = fabs(target2->X.pos(2)-P2.points(P2.points.d0-1,2));
    }
    cout << samples << endl;
    cout << values << endl;
  }
  world.gl().add(drawRedLine,&(P2.points));


  /// plotting
  plotClear();
  plotFunction(P.points.col(0));
  plotFunction(P.points.col(1));
  plotFunction(P.points.col(2));
  plotFunction(P2.points.col(0));
  plotFunction(P2.points.col(1));
  plotFunction(P2.points.col(2));
  plot(true);


  world.watch(true);
  displayTrajectory(x,MP.T,world,"world");
}

void testSplineExploration() {
  /// create reference motion
  ors::KinematicWorld world("scene");
  arr q, qdot;
  world.getJointState(q, qdot);
  world.swift();
  MotionProblem MP(world,false);
  MP.T = 50;
  MP.tau = 0.05;
  ors::Shape *grasp = world.getShapeByName("endeff");
  ors::Body *target1 = world.getBodyByName("target1");
  Task *t;
  t = MP.addTask("tra", new TransitionTaskMap(world));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e-2);
  t =MP.addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(MP.T-3,MP.T,ARRAY(target1->X.pos),1e2);
  MotionProblemFunction MPF(MP);
  arr x(MP.T+1,q.N); x.setZero();
  optConstrainedMix(x, NoArr, Convert(MPF), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-3));

  /// convert to task space traj
  arr y(MP.T+1,3);
  for (uint t=0;t<y.d0;t++) {
    arr tmp;
    world.setJointState(x[t]);
    world.kinematicsPos(tmp,NoArr,grasp->body,&grasp->rel.pos);
    y[t] = tmp;
  }

  /// transform traj
//  world.gl().add(drawGreenPoints,&(y));

//  arr ys = y.sub(0,-1,linspace(0,y.d0-1,y.d0-1/2));
  MT::Spline S(y.d0,y,1);
  arr m = linspace(0,1,20); m.flatten();
  arr z;
  for (uint i=0;i<m.d0;i++) {
    z.append(~S.eval(m(i)));
  }
  cout << m << endl;

  MT::Spline S2(z.d0,z,2);
  arr e = S2.eval();
  arr n = linspace(0,1,50);n.flatten();

  /// add exploration
  MT::Spline S3(z.d0,z,2);
  S3.points[3] = S3.points[3] + ARRAY(0.01,0.01,0.01);

  arr l2,l3;
  for (uint i=0;i<n.d0;i++) {
    l2.append(~S2.eval(n(i)));
    l3.append(~S3.eval(n(i)));
  }
//  world.gl().add(drawRedPoints,&(e));
  world.gl().add(drawBlueLine,&(S.points));
  world.gl().add(drawRedLine,&(l2));
  world.gl().add(drawRedPoints,&(S2.points));
  world.gl().add(drawGreenLine,&(l3));
  world.gl().add(drawGreenPoints,&(S3.points));

  world.gl().resize(800,800);
  world.watch(true);
  displayTrajectory(x,MP.T,world,"world");
}

double reward(const arr &x, ors::KinematicWorld &world) {
  double r = 0.;
  world.setJointState(x[x.d0-1]);

  // distance to goal
  arr y = ARRAY(world.getShapeByName("endeff")->X.pos);
  arr g = ARRAY(world.getBodyByName("target1")->X.pos);
  r += sum(fabs(y-g));

  // length of motion
  arr v;
  getVel(v,x,1.);
  r += sum(fabs(v));
  return r;
}


void TEST(SimpleWorld) {
  /// create starting motion
  ors::KinematicWorld world("scene");
  world.gl().resize(800,800);
  arr q, qdot;
  world.getJointState(q, qdot);
  world.swift();
  MotionProblem MP(world,false);
  MP.T = 50;
  MP.tau = 0.05;
  ors::Shape *grasp = world.getShapeByName("endeff");
  ors::Body *target1 = world.getBodyByName("target1");
  ors::Body *target2 = world.getBodyByName("target2");
  Task *t;
  t = MP.addTask("tra", new TransitionTaskMap(world));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e-2);
  t =MP.addTask("pos1", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(MP.T-3,MP.T,ARRAY(target1->X.pos),1e2);
  t =MP.addTask("pos2", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(MP.T*.5-3,MP.T*.5,ARRAY(target2->X.pos),1e1);
  MotionProblemFunction MPF(MP);
  arr x(MP.T+1,q.N); x.setZero();
  optConstrainedMix(x, NoArr, Convert(MPF), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-3));

  TrajFactory tf;

  /// transform joint trajectory into feature space
  arr y;
  tf.compFeatTraj(x,y,world,new DefaultTaskMap(posTMT,grasp->index));

  /// back transform feature trajectory into joint space
  arr xNew;
  tf.compJointTraj(x,y,xNew,MP,new DefaultTaskMap(posTMT,grasp->index));

  cout << "transformation error: " << length(xNew-x) << endl;
  cout << "reward: " << reward(x,world) << endl;
  cout << "reward xNew: " << reward(xNew,world) << endl;

  arr y_trans;
  tf.transform(y,ARRAY(0.01,0.01,0.01),y_trans,0.5,.04);
  world.gl().add(drawRedLine,&(y_trans));
  world.gl().add(drawBlueLine,&(y));

  /// apply CMA on splines
  SearchCMA cma;
  arr param,paramStd;

  arr s_list = ARR(0.5,0.75,0.25,0.5,0.75,0.25,0.5,0.75,0.25);
  arr y2 = y;
  arr samples,values;
  for (uint r = 0;r<s_list.d0;r++) {
    param = zeros(3);
    param.append(log(ARR(0.1)));
    paramStd = ones(param.d0)*0.05;
    cma.init(param.d0,-1,-1,param,paramStd);
    samples.clear();values.clear();

    for(uint t=0;t<10;t++){
      cma.step(samples, values);

      for(uint i=0;i<samples.d0;i++) {
        param = samples[i].subRange(0,2);
        double std_i = exp(samples(i,3));
        tf.transform(y2,param,y_trans,s_list(r),std_i);

        arr x_i;
        tf.compJointTraj(x,y_trans,x_i,MP,new DefaultTaskMap(posTMT,grasp->index));
        values(i) = reward(x_i,world);
        world.gl().update();
      }

      cout << samples << endl;
      cout << values << endl;
    }
    arr best_sample;
    cma.getBestSample(best_sample);
    param = best_sample.subRange(0,2);
    double std_i = exp(best_sample(3));
    tf.transform(y2,param,y_trans,s_list(r),std_i);
    y2=y_trans;
  }

  world.watch(true);
  displayTrajectory(x,MP.T,world,"world");
  displayTrajectory(xNew,MP.T,world,"world");
}

int main(int argc,char **argv){
  rnd.seed(3);
  MT::initCmdLine(argc,argv);
  TEST(SimpleWorld);
  return 0;
}


// subsampled splines
//MT::Spline S(y.d0,y,1);
//arr m = linspace(0,1,10); m.flatten();
//arr z;
//for (uint i=0;i<m.d0;i++) {
//  z.append(~S.eval(m(i)));
//}

//MT::Spline S2(z.d0,z,2);
//arr n = linspace(0,1,50);n.flatten();

///// add exploration
//MT::Spline S3(z.d0,z,2);
//S3.points[3] = S3.points[3] + ARRAY(0.01,0.01,0.01);

//arr l2,l3;
//for (uint i=0;i<n.d0;i++) {
//  l2.append(~S2.eval(n(i)));
//  l3.append(~S3.eval(n(i)));
//}
