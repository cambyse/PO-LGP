#include <Algo/spline.h>
#include <Core/array.h>
#include <Gui/plot.h>
#include <Gui/opengl.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Optim/blackbox.h>
#include <Ors/ors.h>
#include <pr2/roscom.h>
#include <Motion/pr2_heuristics.h>
#include <pr2/trajectoryInterface.h>

#include "../../src/plotUtil.h"
#include "../../src/traj_factory.h"
#include "../src/task_manager.h"


struct CMA_RL {
  uint nParam;
  arr paramLim;
  arr X,Y,YS;
  SearchCMA cma;
  double muStd;
  arr samples,values;

  CMA_RL(uint nParam_, arr &paramLim_, mlr::String taskName) {
    nParam = nParam_;
    paramLim = paramLim_;
    muStd = 0.05;
  }

  void addDatapoint(arr x,arr y, arr ys) {

    if (X.N==0) {
      cma.init(nParam,-1,-1,x,muStd);
      Y.append(~y);
    }else{
      samples.append(~x);
      if (ys(0)>0) {
        values.append(y(0));
        Y.append(~y);
      }else{
        values.append(0.9);
        Y.append(~(y*0.+0.9));
      }
    }

    X.append(~x);
    YS.append(~ys);
  }

  void evaluate(arr &param) {
    cma.step(samples, values);
    param = samples;

    /// clear samples + values for next iteration
    samples.clear();values.clear();
  }

  void load(mlr::String folder){
    X << FILE(STRING(folder<<"X.dat"));
    Y << FILE(STRING(folder<<"Y.dat"));
    YS << FILE(STRING(folder<<"YS.dat"));
    samples << FILE(STRING(folder<<"samples.dat"));
    values << FILE(STRING(folder<<"values.dat"));

    arr mean,stdv;
    mean << FILE(STRING(folder<<"mean1.dat"));
    stdv << FILE(STRING(folder<<"stdv1.dat"));
    mean.flatten(); stdv.flatten();
    cma.init(nParam,-1,-1,mean,stdv);
  }

  void save(mlr::String folder){
    write(LIST<arr>(X),STRING(folder<<"X.dat"));
    write(LIST<arr>(Y),STRING(folder<<"Y.dat"));
    write(LIST<arr>(YS),STRING(folder<<"YS.dat"));
    write(LIST<arr>(samples),STRING(folder<<"samples.dat"));
    write(LIST<arr>(values),STRING(folder<<"values.dat"));

    arr mean;
    cma.getMean(mean);
    write(LIST<arr>(mean),STRING(folder<<"mean1.dat"));
  }

};

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  bool useRos = mlr::getParameter<bool>("useRos");
  bool visualize = mlr::getParameter<bool>("visualize");
  double duration = mlr::getParameter<double>("duration");
  mlr::String folder = mlr::getParameter<mlr::String>("folder");
  mlr::String taskName = mlr::getParameter<mlr::String>("taskName");

  mlr::KinematicWorld world(STRING("../model.kvg"));
  TrajectoryInterface *mi;
  DoorTask *task = new DoorTask(world);
  if (useRos) mi = new TrajectoryInterface(world);

  arr Xdemo,FLdemo,Mdemo;
  Xdemo << FILE(STRING(folder<<"/phaseX.dat"));
  FLdemo << FILE(STRING(folder<<"/phaseFLact.dat"));
  Mdemo << FILE(STRING(folder<<"/Mdemo.dat"));

  task->computeConstraintTime(FLdemo,Xdemo);

  cout << Xdemo[0] << endl;
  world.gl().resize(800,800);
  task->updateVisualization(world,Xdemo);
  if (visualize) displayTrajectory(Xdemo,-1,world,"demonstration");

  /// initialize model free strategy
  arr paramLim;
  paramLim.append(~ARR(-0.06,0.06)); // hand opening
  paramLim.append(~ARR(-0.12,0.12)); // hand position
  CMA_RL *mfs = new CMA_RL(2,paramLim,taskName);
  arr x0 = Xdemo[0];

  arr Xreverse, param, Xn;
  double y, ys;

  uint count;
  if (mlr::getParameter<bool>("MF_restartLearning")) {
    param = ARR(0.0,0.); param.flatten();

    task->transformTrajectory(Xn,param,Xdemo);
    if (useRos) {
      mi->gotoPosition(x0); mi->executeTrajectory(Xn,duration,true);
      y = task->reward(mi->logFLact);
      ys = 1.;
      Xreverse = Xn; Xreverse.reverseRows(); mi->executeTrajectory(Xreverse,duration);
    } else {
      /// dummy values for debugging
      arr Xd; getAcc(Xd,Xn,0.05);
      y = task->reward(Xn); ys = 1.;
    }

    y = -y+1.;
    mfs->addDatapoint(param,ARR(y),ARR(ys));
    cout <<"x = "<< param <<"   | y = "<<y<<"   | ys = "<<ys << endl;

    count = 0;
  } else {
    mfs->load(STRING(folder<<"/"));
    count = mfs->X.d0-1;
  }

  arr param_list;
  for(;;) {
    cout << "\n Iteration: " << count << endl;
    /// choose next datapoint
    mfs->evaluate(param_list);


    for (uint i=0;i<param_list.d0;i++) {
      /// transform trajectory
      param = param_list[i];

      bool result = task->transformTrajectory(Xn,param,Xdemo);

      if (visualize) {
        task->updateVisualization(world,Xn);
//        world.watch(true,"press enter to visualize trajectory");
        displayTrajectory(Xn,Xn.d0,world,"");
//        world.watch(true,"press enter to execute candidate");
      }
      x0 = Xn[0];

      if (useRos&&result) {
        /// goto iniitial position
        mi->gotoPosition(x0);
        /// execute trajectory on robot
        mi->executeTrajectory(Xn,duration,true);
        /// evaluate cost functions
        if (result) result = task->success(mi->logMact, Mdemo);
        y = task->reward(mi->logFLact);
        ys = (result) ?(1.):(-1.);

      } else {
        /// dummy values for debugging
        arr Xd; getAcc(Xd,Xn,0.05);
        y = task->reward(Xd);
        ys = (result) ?(1.):(-1.);
      }

      y = -y + 1.;
      mfs->addDatapoint(param,ARR(y),ARR(ys));
      cout <<"param:  "<<param << endl;
      cout <<"x = "<< param <<" y = "<<y<<" ys = "<<ys << endl;


      /// logging
      mfs->save(STRING(folder<<"/"));
      if (useRos && result) { mi->logging(STRING(folder<<"/cma"),count); }

      if (result) {
        if (useRos) {Xreverse = Xn; Xreverse.reverseRows(); mi->executeTrajectory(Xreverse,duration);}
      } else {
//        if (useRos) {mi->stopMotion();}
      }

      count++;
    }
  }

  mi->~TrajectoryInterface();
  return 0;
}
