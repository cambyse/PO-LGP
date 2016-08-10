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

void drawTraj(uint color, arr& p, uint lineStyle) {
  glColor(color);
  glPointSize(4.0f);
  glLineWidth(2);
  if (lineStyle == 1) {
    glBegin(GL_POINTS);
  } else {
    glBegin(GL_LINES);
  }
  glVertex3f(p(0,0),p(0,1),p(0,2));
  uint i;
  for (i = 1; i<p.d0-1; i++) {
    glVertex3f(p(i,0),p(i,1),p(i,2));
    glVertex3f(p(i,0),p(i,1),p(i,2));
  }
  glVertex3f(p(i,0),p(i,1),p(i,2));
  glEnd();
//  GLUquadric *style=gluNewQuadric();
//  gluDisk(style, 0.15, 0.2, 30, 1);
//  gluDeleteQuadric(style);
  glLineWidth(1);
}

void drawRedLine(void* classP){  arr *p = (arr*)classP;  drawTraj(2,*p,2); }
void drawRedPoints(void* classP){  arr *p = (arr*)classP;  drawTraj(2,*p,1); }
void drawGreenLine(void* classP){  arr *p = (arr*)classP;  drawTraj(5,*p,2); }
void drawGreenPoints(void* classP){  arr *p = (arr*)classP;  drawTraj(5,*p,1); }
void drawBlueLine(void* classP){  arr *p = (arr*)classP;  drawTraj(0,*p,2); }
void drawBluePoints(void* classP){  arr *p = (arr*)classP;  drawTraj(0,*p,1); }

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
  t->setCostSpecs(MP.T-3,MP.T,ARR(target1->X.pos),1e2);
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
  mlr::Path P(y,2);
  world.gl().add(drawGreenPoints,&(P.points));
  mlr::Path P2(y,2);
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

  arr param = ARR(1e-2,1e2,0.);
  param = param/sum(param)*scenario.costScale;
  t = s.MP->addTask("tra", new TransitionTaskMap(*s.world));
  t->map.order=2;
  t->target = ARR(0.);
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
  scenario.weights.append(CostWeight(CostWeight::Transition,1,ARR(0.),s.MP->T,s.world->getJointStateDimension()));

  t =s.MP->addTask("pos2", new DefaultTaskMap(posTMT, grasp->index) );
  t->target = ARR(target2->X.pos);
  scenario.weights.append(CostWeight(CostWeight::Block,1,ARR(s.MP->T-3,s.MP->T),1,3));
  t =s.MP->addTask("pos1", new DefaultTaskMap(posTMT, grasp->index) );
  t->target = ARR(target1->X.pos);
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
  mf->execMotion(scenario.scenes(0),NoArr,NoArr,NoArr,true);


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
  t->setCostSpecs(MP.T-3,MP.T,ARR(target1->X.pos),1e2);
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
  mlr::Path P(y,2);
  world.gl().add(drawGreenPoints,&(P.points));
  mlr::Path P2(y,2);
  arr expl = ARR(0.,0.,0.1);
  P2.transform_CurrentFixed_EndBecomes(y[y.d0-1]+expl,0.);

  SearchCMA cma;
  int cmaD = 1;
  int cmaMu = -1;
  int cmaLambda = -1;
  double cmaLo = 20.;
  double cmaHi = 30.;

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
  t->setCostSpecs(MP.T-3,MP.T,ARR(target1->X.pos),1e2);
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
  mlr::Spline S(y.d0,y,1);
  arr m = linspace(0,1,20); m.flatten();
  arr z;
  for (uint i=0;i<m.d0;i++) {
    z.append(~S.eval(m(i)));
  }
  cout << m << endl;

  mlr::Spline S2(z.d0,z,2);
  arr e = S2.eval();
  arr n = linspace(0,1,50);n.flatten();

  /// add exploration
  mlr::Spline S3(z.d0,z,2);
  S3.points[3] = S3.points[3] + ARR(0.01,0.01,0.01);

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

int main(int argc,char **argv){
  rnd.seed(3);
  mlr::initCmdLine(argc,argv);
//  TEST(PathIMP);
  TEST(PathBBO);
//  TEST(SplineExploration);
  return 0;
}
