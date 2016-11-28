#include <Motion/motion.h>
#include <Motion/pr2_heuristics.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Gui/opengl.h>

#include "../../src/motion_factory.h"
#include "../../src/traj_factory.h"
#include "../../src/cost_weight.h"
#include "../../src/inverse_motion.h"
#include "../../src/task_manager.h"

void dofExploration() {
  mlr::String folder = mlr::getParameter<mlr::String>("folder");
  double duration = mlr::getParameter<double>("duration");

  mlr::KinematicWorld G(STRING(folder<<"modelaug.kvg"));
  TaskManager *task = new ButtonTask(G);

  arr Xbase,FLbase,Mbase;
  Xbase << FILE(STRING(folder<<"Xaug.dat"));
  FLbase << FILE(STRING(folder<<"FLaug.dat"));
  Mbase << FILE(STRING(folder<<"Maug.dat"));

  G.gl().update(); G.gl().resize(800,800);
  task->computeConstraintTime(FLbase,Xbase);
  task->updateVisualization(G,Xbase);

  arr param0 = ARR(Xbase(Xbase.d0-1,G.getJointByName("b2_b1")->qIndex));

  MotionProblem *MP = new MotionProblem(G,false);
  MP->T = Xbase.d0-1;
  MP->tau = duration/MP->T;
  MP->x0 = Xbase[0];

  Task *t;
  t = MP->addTask("tra", new TransitionTaskMap(G));
  ((TransitionTaskMap*)&t->map)->H_rate_diag = pr2_reasonable_W(G);
  t->map.order=2;
  t->setCostSpecs(0, MP->T, ARR(0.), 1e-1);

  arr prec = task->constraintTime;
  arr prec_inv = -1.*(prec-1.);

  /// homing trajectory
  t = MP->addTask("homing_traj", new TaskMap_qItself());
  t->target = Xbase;
  t->prec = prec_inv*1e-2;
  /// add contact constraint
  t = MP->addTask("b2_b1_con", new PointEqualityConstraint(MP->world,"endeffL",NoVector,"b1_shape"));
  t->target = zeros(prec.d0,3);
  t->prec = prec*1.;
  /// final position constraint
  t = MP->addTask("b2_b1_T", new qItselfConstraint(G.getJointByName("b2_b1")->qIndex,G.getJointStateDimension()));
  t->setCostSpecs(MP->T-3,MP->T,param0+.05,1.);
  /// joint fixation constraint
  t = MP->addTask("b2_b1_fix", new qItselfConstraint(G.getJointByName("b2_b1")->qIndex,G.getJointStateDimension()));
  t->target = ARR(0.);
  t->prec = prec_inv;

  MotionProblemFunction MF(*MP);
  arr X = Xbase;
  arr lambda;

  optConstrainedMix(X, lambda, Convert(MF), OPT(verbose=0, constrainedMethod=anyTimeAula,stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-4));
  MP->costReport();

  for(;;) displayTrajectory(X,X.d0,G,"X");

  G.watch(true);
}

void initExploration() {
  for (;;) {
    mlr::String folder = mlr::getParameter<mlr::String>("folder");
    double duration = mlr::getParameter<double>("duration");

    mlr::KinematicWorld G(STRING(folder<<"modelaug.kvg"));
    TaskManager *task = new ButtonTask(G);

    arr Xbase,FLbase,Mbase;
    Xbase << FILE(STRING(folder<<"Xaug.dat"));
    FLbase << FILE(STRING(folder<<"FLaug.dat"));
    Mbase << FILE(STRING(folder<<"Maug.dat"));

    G.gl().update(); G.gl().resize(800,800);
    task->computeConstraintTime(FLbase,Xbase);


    /// compute new initial position
    arr q0 = randn(Xbase.d1)*1e-1 + Xbase[0];
    q0(G.getJointByName("b2_b1")->qIndex) = Xbase(0,G.getJointByName("b2_b1")->qIndex);

    G.setJointState(Xbase[0]);
    mlr::Vector yBase0 = G.getShapeByName("endeffL")->X.pos;
    G.setJointState(q0);
    mlr::Vector y0 = conv_vec2arr(G.getShapeByName("endeffL")->X.pos);

    arr offsetC1 = conv_vec2arr(y0 - yBase0);
    cout << "offset" << y0-yBase0 << endl;

    TrajFactory tf;
    arr C1trans, C1demo;
    tf.compFeatTraj(Xbase,C1demo,G,new DefaultTaskMap(posTMT,G,"endeffL"));
    C1trans = C1demo;

    double s;
    for (uint t=0;t<task->constraintCP(0);t++) {
      s = 1.-t/task->constraintCP(0);
      C1trans[t] = C1trans[t] + s*offsetC1;
    }


    MotionProblem *MP = new MotionProblem(G,false);
    MP->T = Xbase.d0-1;
    MP->tau = duration/MP->T;
    MP->x0 = q0;

    Task *t;
    t = MP->addTask("tra", new TransitionTaskMap(G));
    ((TransitionTaskMap*)&t->map)->H_rate_diag = pr2_reasonable_W(G);
    t->map.order=2;
    t->setCostSpecs(0, MP->T, ARR(0.), 1e-1);

    arr prec = task->constraintTime;
    arr prec_inv = -1.*(prec-1.);

    /// homing trajectory
    t = MP->addTask("homing_traj", new qItselfConstraint());
    t->target = Xbase;
    t->prec = prec;
    /// add contact constraint
    t = MP->addTask("b2_b1_con", new DefaultTaskMap(posTMT,G,"endeffL"));
    t->target = C1trans;
    t->prec = prec_inv;
    /// joint fixation constraint
    t = MP->addTask("b2_b1_fix", new qItselfConstraint(G.getJointByName("b2_b1")->qIndex,G.getJointStateDimension()));
    t->target = ARR(0.);
    t->prec = prec_inv;

    MotionProblemFunction MF(*MP);
    arr X = Xbase;
    arr lambda;

    optConstrainedMix(X, lambda, Convert(MF), OPT(verbose=0, constrainedMethod=anyTimeAula,stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-4));
    MP->costReport();

    task->updateVisualization(G,Xbase,X);

    //  for(;;)
    displayTrajectory(X,X.d0,G,"X");
    G.watch(true);
  }
}

void initdofExploration() {
  for(;;) {
    mlr::String folder = mlr::getParameter<mlr::String>("folder");
    double duration = mlr::getParameter<double>("duration");

    mlr::KinematicWorld G(STRING(folder<<"modelaug.kvg"));
    TaskManager *task = new ButtonTask(G);

    arr Xbase,FLbase,Mbase;
    Xbase << FILE(STRING(folder<<"Xaug.dat"));
    FLbase << FILE(STRING(folder<<"FLaug.dat"));
    Mbase << FILE(STRING(folder<<"Maug.dat"));
    G.gl().update(); G.gl().resize(800,800);
    task->computeConstraintTime(FLbase,Xbase);


    arr param0 = ARR(Xbase(Xbase.d0-1,G.getJointByName("b2_b1")->qIndex));

    /// compute new initial position
    arr q0 = randn(Xbase.d1)*1e-1 + Xbase[0];
    q0(G.getJointByName("b2_b1")->qIndex) = Xbase(0,G.getJointByName("b2_b1")->qIndex);

    G.setJointState(Xbase[0]);
    mlr::Vector yBase0 = G.getShapeByName("endeffL")->X.pos;
    G.setJointState(q0);
    mlr::Vector y0 = conv_vec2arr(G.getShapeByName("endeffL")->X.pos);

    arr offsetC1 = conv_vec2arr(y0 - yBase0);
    cout << "offset" << y0-yBase0 << endl;

    TrajFactory tf;
    arr C1trans, C1demo;
    tf.compFeatTraj(Xbase,C1demo,G,new DefaultTaskMap(posTMT,G,"endeffL"));
    C1trans = C1demo;

    double s;
    for (uint t=0;t<task->constraintCP(0);t++) {
      s = 1.-t/task->constraintCP(0);
      C1trans[t] = C1trans[t] + s*offsetC1;
    }

    MotionProblem *MP = new MotionProblem(G,false);
    MP->T = Xbase.d0-1;
    MP->tau = duration/MP->T;
    MP->x0 = q0;

    Task *t;
    t = MP->addTask("tra", new TransitionTaskMap(G));
    ((TransitionTaskMap*)&t->map)->H_rate_diag = pr2_reasonable_W(G);
    t->map.order=2;
    t->setCostSpecs(0, MP->T, ARR(0.), 1e-1);

    arr prec = task->constraintTime;
    arr prec_inv = -1.*(prec-1.);

    /// joint fixation constraint
    t = MP->addTask("b2_b1_fix", new qItselfConstraint(G.getJointByName("b2_b1")->qIndex,G.getJointStateDimension()));
    t->target = ARR(0.);
    t->prec = prec_inv;
    /// final position constraint
    t = MP->addTask("b2_b1_T", new qItselfConstraint(G.getJointByName("b2_b1")->qIndex,G.getJointStateDimension()));
    t->setCostSpecs(MP->T-3,MP->T,param0+.05,1.);
    /// add contact1
    t = MP->addTask("b2_b1_con1", new DefaultTaskMap(posTMT,G,"endeffL"));
    t->target = C1trans;
    t->prec = prec_inv;
    /// add contact2
    t = MP->addTask("b2_b1_con2", new PointEqualityConstraint(MP->world,"endeffL",NoVector,"b1_shape"));
    t->target = zeros(prec.d0,3);
    t->prec = prec;
    /// homing trajectory
    t = MP->addTask("homing_traj", new TaskMap_qItself());
    t->target = Xbase;
    t->prec = ones(Xbase.d0)*1e-3;

    MotionProblemFunction MF(*MP);
    arr X = Xbase;
    arr lambda;

    optConstrainedMix(X, lambda, Convert(MF), OPT(verbose=0, constrainedMethod=anyTimeAula,stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-4));
    MP->costReport();
    task->updateVisualization(G,Xbase,X);

    displayTrajectory(X,X.d0,G,"X");

    G.watch(true);
  }
}

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
//    dofExploration();
  //  initExploration();
  initdofExploration();
  return 0;
}
