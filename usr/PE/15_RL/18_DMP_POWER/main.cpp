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
  arr xDemo;
  arr y_ref,basis;
  arr base_weight;
  uint n_base,D,N,n_param,n_iter, iter;
  arr Q,Return,s_Return,param,variance,current_param;
  double duration;
  uintA s_ReturnI;
  double dt;
  ors::KinematicWorld* world;

  Power_DMP(arr &xDemo_, double duration_,uint n_base_,double var_,ors::KinematicWorld &world_, arr &Xn) {
    xDemo = xDemo_;
    duration = duration_;
    n_base = n_base_;
    world = new ors::KinematicWorld(world_);

    // compute endeffector trajectory
    arr C1demo,C2demo;
    TrajFactory tf;
    tf.compFeatTraj(xDemo,C1demo,*world,new DefaultTaskMap(posTMT,*world,"endeffC1"));
    tf.compFeatTraj(xDemo,C2demo,*world,new DefaultTaskMap(posTMT,*world,"endeffC2"));
    y_ref = catCol(C1demo,C2demo);

    dt = duration_/xDemo_.d0;
    D = y_ref.d1;
    n_param = n_base*D;
    n_iter = 100;
    N = floor(duration_/dt);

    dmp = new DynamicMovementPrimitives(y_ref,n_base,dt);
    dmp->trainDMP();

    basis = dmp->PHI%(1./repmat(sum(dmp->PHI,1),1,n_base));
    basis = repmat(basis,1,D);
    base_weight = dmp->weights;
    variance = var_*ones(n_param,1); variance.flatten();


    Q.resize(n_iter+1,N).setZero();
    Return.resize(1,n_iter+1).setZero(); Return.flatten();
    s_Return.resize(n_iter+1,2).setZero();
    s_ReturnI.resize(n_iter+1).setZero();
    param.resize(n_iter+1,n_param).setZero();

    current_param = param[0];

    for (uint t=0;t<N;t++) {
      dmp->iterate();
    }

    /// convert task space trajectory into joint space trajectory
    arr yC1 = dmp->y_bk.cols(0,3);
    arr yC2 = dmp->y_bk.cols(3,6);
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
    for (uint t=0;t<N;t++) {
      dmp->iterate();
      q.subRange(0,t) = q.subRange(0,t) + exp(-.5*costs(t)); // + exp(-.5*xdd(t));
    }

    if(!result) {
      q = q*0.;
    }

    q = q/(double)N;
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
    arr temp_W = ~(basis%basis%(1./repmat(sum(basis%basis%repmat(~variance,N,1),1),1,n_param)));
    arr temp_explore = repmat(param[j]-current_param,1,N);
    arr temp_Q = repmat(~Q[j],n_param,1);

    param_nom = param_nom + sum(temp_W%temp_explore%temp_Q,1);
    param_dnom = param_dnom + sum(temp_W%temp_Q,1);
  }

  param[iter+1] = current_param + param_nom%(1./(param_dnom+1e-10));
  current_param = param[iter+1];

  if (iter!=n_iter-2) {param[iter+1] = param[iter+1] + pow(variance,.5)%randn(n_param,1);}
  arr tmp = param[iter+1]; tmp.reshape(D,n_base); tmp = ~tmp;
  dmp->weights = base_weight + tmp;

  for (uint t=0;t<N;t++) {
    dmp->iterate();
  }

  iter++;

  /// convert task space trajectory into joint space trajectory
  arr yC1 = dmp->y_bk.cols(0,3);
  arr yC2 = dmp->y_bk.cols(3,6);
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
}

void load(MT::String folder) {
  Q << FILE(STRING(folder<<"Q.dat"));
  Return << FILE(STRING(folder<<"Return.dat")); Return.flatten();
  s_Return << FILE(STRING(folder<<"s_Return.dat"));
  iter << FILE(STRING(folder<<"iter.dat"));
  param << FILE(STRING(folder<<"param.dat"));
  current_param << FILE(STRING(folder<<"current_param.dat"));
}

void save(MT::String folder){
  write(LIST<arr>(Q),STRING(folder<<"Q.dat"));
  write(LIST<arr>(Return),STRING(folder<<"Return.dat"));
  write(LIST<arr>(s_Return),STRING(folder<<"s_Return.dat"));
  write(LIST<arr>(ARR(iter)),STRING(folder<<"iter.dat"));
  write(LIST<arr>(param),STRING(folder<<"param.dat"));
  write(LIST<arr>(current_param),STRING(folder<<"current_param.dat"));
}
};


int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);
  bool useRos = MT::getParameter<bool>("useRos");
  bool visualize = MT::getParameter<bool>("visualize");
  double duration = MT::getParameter<double>("duration");
  MT::String folder = MT::getParameter<MT::String>("folder");

  ors::KinematicWorld world(STRING("model.kvg"));
  DoorTask *task = new DoorTask(world);
  Motion_Interface *mi;
  if (useRos) mi = new Motion_Interface(world);

  arr Xreverse;
  uint count;

  /// ----- load demonstration ---------------------------
  arr Xdemo,Fdemo,Mdemo;
  if (MT::getParameter<bool>("loadDemoFromFile")) {
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
  world.watch(true);


  Power_DMP *pdmp = new Power_DMP(X,duration,15,4000.,world,Xn);
  if (visualize) displayTrajectory(Xdemo,-1,world,"Xn ");

  arr costs = sum(fabs(Fdemo),1); costs.flatten();
  if (MT::getParameter<bool>("MF_restartLearning")) {
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


  for (;;) {
    cout << "iteration: " << count << endl;
    /// goto iniitial position
    if (useRos) mi->gotoPosition(x0);

    pdmp->evaluate(Xn);

    if (visualize) {
      task->updateVisualization(world,Xn);
      world.watch(true,"press enter to visualize trajectory");
      displayTrajectory(Xn,Xn.d0,world,"");
      world.watch(true,"press enter to execute trajectory");
    }


    /// evaluate cost function
    bool result;
    if (useRos) {
      mi->executeTrajectory(Xn,duration,true);

      result = task->success(mi->Mact, Mdemo);
      costs = sum(fabs(mi->FLact),1); costs.flatten();
      cout << "costs: "<< sum(costs) <<endl;

      mi->logging(STRING(folder<<"/mf"),count);

      /// logging
      if (result) {
        X = Xn;
        if (useRos) {Xreverse = X; Xreverse.reverseRows(); mi->executeTrajectory(Xreverse,duration);}
      }else {
        if (useRos) {mi->sendZeroGains();}
      }
    }else{
      write(LIST<arr>(X),STRING(folder<<"/dmpX.dat"));
      write(LIST<arr>(X),STRING(folder<<"/dmpX"<<count<<".dat"));
      result = true;
      costs = costs + randn(costs.d0);
    }
    pdmp->addDataPoint(X,costs,result);

    pdmp->save(STRING(folder<<"/"));

    count++;
  }

  mi->~Motion_Interface();
  return 0;
}


/*  arr tmp = pdmp->dmp->y_bk.cols(0,3);
arr tmp2 = pdmp->dmp->y_bk.cols(3,6);
drawPoints(world,tmp,1);
drawPoints(world,tmp2,1);
world.watch(true);
arr K1,K2,tmp3,tmp4;
  pdmp->evaluate(Xn);
  tmp3 = pdmp->dmp->y_bk.cols(0,3);
  tmp4 = pdmp->dmp->y_bk.cols(3,6);
  drawPoints(world,tmp3,2);
  drawPoints(world,tmp4,2);
  drawLine(world,Xn,K1,"endeffC1",2,0,0);
  drawLine(world,Xn,K2,"endeffC2",2,0,0);
  world.watch(true);
  displayTrajectory(Xn,-1,world,"Xn");
  world.watch(true);
*/
