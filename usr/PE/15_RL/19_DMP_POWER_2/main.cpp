#include <Algo/spline.h>
#include <Core/array.h>
#include <Core/array_t.h>
#include <Gui/plot.h>
#include <Motion/motion.h>
#include <Motion/pr2_heuristics.h>
#include <Motion/taskMaps.h>
#include <Motion/dynamicMovementPrimitives.h>
#include <Optim/optimization.h>
#include <Ors/ors.h>
#include <pr2/roscom.h>
#include <System/engine.h>
#include "../17_FINAL/src/motion_interface.h"
#include "../17_FINAL/src/task_manager.h"
#include "../src/plotUtil.h"
#include "../src/traj_factory.h"

struct Power_DMP {

  DynamicMovementPrimitives *dmp;
  arr xDemo, constraintCP;
  arr y_ref,y_refC,y_refF;
  arr basis,goal;
  arr base_weight;
  uint n_base,D,N,n_param,n_iter, iter,n_rfs, n_goal_param;
  arr Q,Return,s_Return,param,variance,current_param;
  double duration;
  uintA s_ReturnI;
  double dt;
  ors::KinematicWorld* world;

  Power_DMP(arr &xDemo_, double duration_,uint n_base_,double var_,ors::KinematicWorld &world_, arr &Xn, arr &constraintCP_) {
    xDemo = xDemo_;
    duration = duration_;
    n_base = n_base_;
    world = new ors::KinematicWorld(world_);
    constraintCP = constraintCP_;

    // compute endeffector trajectory
    arr C1demo,C2demo;
    TrajFactory tf;
    tf.compFeatTraj(xDemo,C1demo,*world,new DefaultTaskMap(posTMT,*world,"endeffC1"));
    tf.compFeatTraj(xDemo,C2demo,*world,new DefaultTaskMap(posTMT,*world,"endeffC2"));
    y_ref = catCol(C1demo,C2demo);

    y_refF = y_ref.subRange(0,constraintCP(0)-1);
    y_refC = y_ref.subRange(constraintCP(0),constraintCP(1)-1);

    dt = duration_/xDemo.d0;
    D = y_ref.d1;
    n_rfs = n_base*D;
    n_param = n_rfs + D;
    n_iter = 300;
    N = y_refF.d0;

    dmp = new DynamicMovementPrimitives(y_refF,n_base,dt,1e-5);
    dmp->trainDMP();

    basis = dmp->PHI%(1./repmat(sum(dmp->PHI,1),1,n_base));
    basis = repmat(basis,1,D);
    base_weight = dmp->weights;
    variance = var_*ones(n_rfs,1); variance.flatten();
    goal = dmp->goal;
    n_goal_param = goal.N;

    Q.resize(n_iter+1,N).setZero();
    Return.resize(1,n_iter+1).setZero(); Return.flatten();
    s_Return.resize(n_iter+1,2).setZero();
    s_ReturnI.resize(n_iter+1).setZero();
    param.resize(n_iter+1,n_param).setZero();

    current_param = param[0];

    for (uint t=0;t<N-1;t++) {
      dmp->iterate();
    }

    /// convert task space trajectory into joint space trajectory
    arr yC1 = dmp->y_bk.cols(0,3);
    arr yC2 = dmp->y_bk.cols(3,6);
    yC1.append(y_refC.cols(0,3));
    yC2.append(y_refC.cols(3,6));
    MotionProblem *MP = new MotionProblem(*world,false);
    MP->T = xDemo.d0-1;
    MP->tau =duration/MP->T;
    MP->x0 = xDemo[0];

    Task *t;
    t = MP->addTask("tra", new TransitionTaskMap(*world));
    ((TransitionTaskMap*)&t->map)->H_rate_diag = pr2_reasonable_W(*world);
    t->map.order=2;
    t->setCostSpecs(0, MP->T, ARR(0.), 1e-1);

    // final position constraint
    t = MP->addTask("qT", new TaskMap_qItself());
    t->setCostSpecs(MP->T,MP->T,xDemo[xDemo.d0-1],1e2);

    t = MP->addTask("posC1", new DefaultTaskMap(posTMT,MP->world,"endeffC1"));
    t->setCostSpecs(0,MP->T,yC1,5e1);

    t = MP->addTask("posC2", new DefaultTaskMap(posTMT,MP->world,"endeffC2"));
    t->setCostSpecs(0,MP->T,yC2,5e1);

    OptOptions o;
    o.stopTolerance = 1e-3; o.verbose=0;

    MotionProblemFunction MPF(*MP);
    Xn = xDemo;
    optConstrainedMix(Xn, NoArr, Convert(MPF), o);

    for (uint t=0;t<Xn.d0;t++) {
      Xn(t,24) = Xn(t,22)*0.1743;
    }

    iter = 0;
  }

  /// ---------------------------------------------------- ///
  void addDataPoint(arr &X, arr &costs, bool result) {
    dmp->reset();

    /// compute Q function of rollout
    arr q(N); q.setZero();
    arr xdd; getAcc(xdd,X,1.);
    xdd = sum(fabs(xdd),1); xdd.flatten();
    arr goal2 = ARR(0.794022, 0.354064, 1.05302, 0.815959, 0.384794, 1.05458);
    for (uint t=0;t<N;t++) {
      dmp->iterate();
      double r = exp(-20.*xdd(t));
//      double r = exp(-10.*sumOfSqr(dmp->Y-goal2));

      q.subRange(0,t) = q.subRange(0,t) + r;
    }

    q = q/(double)N;
    q = q/6.;
    cout << "acceleration reward: " << q(0) << endl;

    /// add force costs
    q = q+exp(-0.1*sum(costs)/costs.d0);

    cout << "force reward: " << exp(-0.1*sum(costs)/costs.d0) << endl;
    cout << "full reward: " << q(0) << endl;
    cout << "result: " << result << endl;
    cout << "////////////////////////\n" << q(0) << endl;

    if(!result) { q = q*0.+1e-3;/*q-0.4;*/ }

    Q[iter] = q;

    Return(iter) = Q(iter,0);
    s_Return[0] = ARR(Return(iter),iter);
    for (uint l=0;l<=n_iter;l++){s_ReturnI(l)=l;}

    std::sort(s_ReturnI.begin(), s_ReturnI.end(),[&](const int& a, const int& b) {
              return (s_Return(a,0) < s_Return(b,0)); });

    s_Return.permuteRows(s_ReturnI);
}

/// ---------------------------------------------------- ///
void evaluate(arr &Xn){
  dmp->reset();

  arr param_nom(n_param); param_nom.setZero();
  arr param_dnom(n_param); param_nom.setZero();

  for (uint i=0; i<min(ARR(iter+1,20)); i++) {
    uint j = s_Return(s_Return.d0-1-i,1);
    arr temp_W = ~(basis%basis%(1./repmat(sum(basis%basis%repmat(~variance,N,1),1),1,n_rfs)));
    temp_W.append(ones(n_goal_param,N));
    arr temp_explore = repmat(param[j]-current_param,1,N);
    arr temp_Q = repmat(~Q[j],n_param,1);

    param_nom = param_nom + sum(temp_W%temp_explore%temp_Q,1);
    param_dnom = param_dnom + sum(temp_W%temp_Q,1);
  }

  param[iter+1] = current_param + param_nom%(1./(param_dnom+1e-10));
  current_param = param[iter+1];

  if (iter!=n_iter-2) {
    arr expl = pow(variance,.5)%randn(n_rfs);
    for (uint i=0;i<expl.d0;i++) {
      expl(i) = max(ARR(min(ARR(100.,expl(i))),-100.));
    }
    expl.append(0.01*randn(n_goal_param,1));
    param[iter+1] = param[iter+1] + expl;
  }

  arr tmp = param[iter+1];
  /// set shape parameter
  arr weight_diff = tmp.sub(0,tmp.N-n_goal_param-1);
  weight_diff.reshape(D,n_base); weight_diff = ~weight_diff;
  dmp->weights = base_weight + weight_diff;

  /// set goal parameter
  tmp = param[iter+1];
  arr goal_diff = tmp.subRange(tmp.d0-n_goal_param,tmp.d0-1);
  dmp->goal = goal + goal_diff;

  for (uint t=0;t<N;t++) {
    dmp->iterate();
  }

  iter++;

  /// convert task space trajectory into joint space trajectory
  arr yC1 = dmp->y_bk.cols(0,3);
  arr yC2 = dmp->y_bk.cols(3,6);
  cout << "yC1[yC1.d0-1]: " <<dmp->y_bk[dmp->y_bk.d0-1] << endl;
  cout << "y_refC + repmat(~goal_diff,y_refC.d0,1): " <<y_refC[0] + goal_diff << endl;
  yC1.append(y_refC.cols(0,3) +repmat(~goal_diff.sub(0,2),y_refC.d0,1));
  yC2.append(y_refC.cols(3,6) +repmat(~goal_diff.sub(3,5),y_refC.d0,1));
  MotionProblem *MP = new MotionProblem(*world,false);
  MP->T = xDemo.d0-1;
  MP->tau =duration/MP->T;
  MP->x0 = xDemo[0];

  Task *t;
  t = MP->addTask("tra", new TransitionTaskMap(*world));
  ((TransitionTaskMap*)&t->map)->H_rate_diag = pr2_reasonable_W(*world);
  t->map.order=2;
  t->setCostSpecs(0, MP->T, ARR(0.), 1e-1);

  t = MP->addTask("qT", new TaskMap_qItself());
  t->setCostSpecs(0,MP->T,xDemo,1e-2);


  // final position constraint
  t = MP->addTask("posC1", new DefaultTaskMap(posTMT,MP->world,"endeffC1"));
  t->setCostSpecs(0,MP->T,yC1,5e1);

  t = MP->addTask("posC2", new DefaultTaskMap(posTMT,MP->world,"endeffC2"));
  t->setCostSpecs(0,MP->T,yC2,5e1);

  OptOptions o;
  o.stopTolerance = 1e-3; o.verbose=0;

  MotionProblemFunction MPF(*MP);
  Xn = xDemo;
  optConstrainedMix(Xn, NoArr, Convert(MPF), o);

  for (uint t=0;t<Xn.d0;t++) {
    Xn(t,24) = Xn(t,22)*0.1743;
  }
}

void load(mlr::String folder) {
  Q << FILE(STRING(folder<<"Q.dat"));
  Return << FILE(STRING(folder<<"Return.dat")); Return.flatten();
  s_Return << FILE(STRING(folder<<"s_Return.dat"));
  iter << FILE(STRING(folder<<"iter.dat"));
  param << FILE(STRING(folder<<"param.dat"));
  current_param << FILE(STRING(folder<<"current_param.dat"));
}

void save(mlr::String folder,uint count){
  write(LIST<arr>(Q),STRING(folder<<"Q.dat"));
  write(LIST<arr>(Return),STRING(folder<<"Return.dat"));
  write(LIST<arr>(s_Return),STRING(folder<<"s_Return.dat"));
  write(LIST<arr>(ARR(iter)),STRING(folder<<"iter.dat"));
  write(LIST<arr>(param),STRING(folder<<"param.dat"));
  write(LIST<arr>(current_param),STRING(folder<<"current_param.dat"));
  write(LIST<arr>(dmp->y_ref),STRING(folder<<"y_ref.dat"));
  write(LIST<arr>(dmp->y_bk),STRING(folder<<"y_bk"<<count<<".dat"));
}
};


int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  bool useRos = mlr::getParameter<bool>("useRos");
  bool visualize = mlr::getParameter<bool>("visualize");
  double duration = mlr::getParameter<double>("duration");
  mlr::String folder = mlr::getParameter<mlr::String>("folder");

  ors::KinematicWorld world(STRING("model.kvg"));
  DoorTask *task = new DoorTask(world);
  Motion_Interface *mi;
  if (useRos) mi = new Motion_Interface(world);

  arr Xreverse;
  uint count;

  /// ----- load demonstration ---------------------------
  arr Xdemo,Fdemo,Mdemo;
  if (mlr::getParameter<bool>("loadDemoFromFile")) {
    Xdemo << FILE(STRING(folder<<"/Xdemo.dat"));
    Fdemo << FILE(STRING(folder<<"/Fdemo.dat"));
    Mdemo << FILE(STRING(folder<<"/Mdemo.dat"));
  } else {
    mi->recordDemonstration(Xdemo,duration);
    if (visualize) displayTrajectory(Xdemo,-1,world,"demonstration ");
    world.watch(true);
    mi->gotoPosition(Xdemo[0]);
    mi->executeTrajectory(Xdemo,duration,true);

    Xdemo = mi->Xact;
    Fdemo = mi->FLact;
    Mdemo = mi->Mact;
    write(LIST<arr>(Xdemo),STRING(folder<<"/Xdemo.dat"));
    write(LIST<arr>(Fdemo),STRING(folder<<"/Fdemo.dat"));
    write(LIST<arr>(Mdemo),STRING(folder<<"/Mdemo.dat"));

    if (useRos) {Xreverse = Xdemo; Xreverse.reverseRows(); mi->executeTrajectory(Xreverse,duration);}
  }

  /// compute contact phase
  task->computeConstraintTime(Fdemo,Xdemo);


  arr x0 = Xdemo[0];
  arr X = Xdemo;
  arr Xn = Xdemo;

  /// setup visualization

  world.gl().resize(800,800);
  task->updateVisualization(world,Xdemo);

  if (visualize) displayTrajectory(Xdemo,-1,world,"demonstration ");


  Power_DMP *pdmp = new Power_DMP(X,duration,15,100.,world,Xn,task->constraintCP);
  if (visualize) displayTrajectory(Xdemo,-1,world,"Xn ");
  world.watch(true);

  arr costs = sum(fabs(Fdemo),1); costs.flatten();
  if (mlr::getParameter<bool>("MF_restartLearning")) {
   if (useRos) {
     mi->gotoPosition(x0);
     mi->executeTrajectory(Xn,duration,true);
     Fdemo = mi->FLact;
   }

   costs = sum(fabs(Fdemo),1); costs.flatten();
   pdmp->addDataPoint(Xn,costs,true);
   cout << "costs: "<< sum(costs) <<endl;

   if (useRos) {Xreverse = X; Xreverse.reverseRows(); mi->executeTrajectory(Xreverse,duration);}
   count = 0;
  } else {
    pdmp->load(STRING(folder<<"/"));
    count = pdmp->iter;
  }


  // learning loop
  for (;;) {
    cout << "iteration: " << count << endl;
    /// goto iniitial position
    if (useRos) mi->gotoPosition(x0);

    pdmp->evaluate(Xn);

    if (visualize) {
      task->updateVisualization(world,Xn,Xdemo);
//      world.watch(true,"press enter to visualize trajectory");
      displayTrajectory(Xn,Xn.d0,world,"");
//      world.watch(true,"press enter to execute trajectory");
    }


    /// evaluate cost function
    bool result;
    if (useRos) {
      mi->executeTrajectory(Xn,duration,true);

      result = task->success(mi->Mact, Mdemo);
      costs = sum(fabs(mi->FLact),1); costs.flatten();

      mi->logging(STRING(folder<<"/mf"),count);

      /// logging
      if (result) {
        Xreverse = Xn; Xreverse.reverseRows(); mi->executeTrajectory(Xreverse,duration);
      }else {
//        mi->stopMotion();
      }
    }else{
      /// simulation dummys
      write(LIST<arr>(Xn),STRING(folder<<"/dmpX.dat"));
      write(LIST<arr>(Xn),STRING(folder<<"/dmpX"<<count<<".dat"));
      result = true;
      costs = 0.;
    }
    cout << "costs: "<< sum(costs) <<endl;

    pdmp->addDataPoint(Xn,costs,result);

    pdmp->save(STRING(folder<<"/"),count);

    count++;
  }

  mi->~Motion_Interface();
  return 0;
}

