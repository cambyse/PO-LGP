#include <Algo/spline.h>
#include <Core/array.h>
#include <Gui/plot.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Ors/ors.h>
#include <pr2/roscom.h>
#include <System/engine.h>
#include "../12_MBMF_LEARNING/task_manager.h"
#include "../12_MBMF_LEARNING/mf_strategy.h"
#include "../src/plotUtil.h"

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);
  TaskManager *tm = new DonutTask();
  ors::KinematicWorld world("donut.ors");
  arr Xdemo;
  tm->createSynthethicDemonstration(Xdemo,world);
  //  displayTrajectory(Xdemo,-1,world,"Xdemo");
  arr X=Xdemo;
  arr A;
  MT::timerStart(true);
  arr P1,P2; drawPoints(world,Xdemo,P1,"endeff",1);
  MF_strategy *mfs = new MF_strategy(1,*tm);

  mfs->recomputeCosts(*tm,Xdemo);

  cout << mfs->X_grid << endl;
  cout << mfs->F_grid << endl;

  arr param,data_param,data_result,data_cost;
  data_param.resize(1,1);
  data_param[0] = 0.;
  data_result.append(1.);
  data_cost.append(tm->rewardFunction(Xdemo));

  for (uint i = 0;i<10;i++){
    X=Xdemo;
    mfs->evaluate(param,data_param,data_result,data_cost);
    cout<<"param: "<< param << endl;
    tm->applyModelFreeExploration(X,Xdemo,param);
    data_param.append(param);
    data_result.append(1.);
    data_cost.append(tm->rewardFunction(X));
    drawPoints(world,X,P1,"endeff",0);
    displayTrajectory(X,-1,world,"X");

    MT::wait(2.);
  }
  cout << MT::timerRead(true) << endl;
  return 0;
}
