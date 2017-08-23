#include <Kin/kin.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>

void forsyth(double& f, double& df_dx, double x, double a){
  double x2=x*x;
  double a2=a*a;
  f = x2/(x2+a2);
  df_dx = 2.*x/(x2+a2) - 2.*x*x2/((x2+a2)*(x2+a2));
}

struct MyTaskMap : TaskMap{

  MyTaskMap() {}

  virtual void phi(arr& y, arr& J, const WorldL& Ks, double tau, int t=-1);
  virtual void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t=-1){ NIY }
  virtual uint dim_phi(const mlr::KinematicWorld& G){ return 3; }
  virtual mlr::String shortTag(const mlr::KinematicWorld& G){ return STRING("MyTaskMap"); }
};

void MyTaskMap::phi(arr &y, arr &J, const WorldL &Ks, double tau, int t){
  TaskMap *gravVel = new TaskMap_Default(posTMT, *Ks.last(), "block");
  gravVel->order = 1;
  gravVel->phi(y, J, Ks, tau, t);

  y -= ARR(0.,0.,-.4);


#if 0
  TaskMap *pos = new TaskMap_Default(posTMT, *Ks.last(), "block");
  pos->order = 0;
  arr y_pos, J_pos;
  gravVel->phi(y_pos, J_pos, *Ks.first());
  double alpha = y_pos(2)-1.1;
#else
  TaskMap *distance = new TaskMap_GJK(*Ks.first(), "table1", "block", true, true);
  arr y_dist, J_dist;
  distance->phi(y_dist, J_dist, *Ks.first());
  double dist = y_dist.scalar()+.08;
#endif
  double f, df;
  forsyth(f, df, dist, .5);
  cout <<t <<' ' <<dist <<endl;
  y *= f;
  if(&J){
    arr J_pos_all = zeros(J.dim());
    J_pos_all.setMatrixBlock(J_dist, 0, 0);
    J = f*J;
    J += y^(df*J_pos_all[2]);
  }
}

int MAIN(int argc,char **argv){
  mlr::initCmdLine(argc, argv);

  mlr::KinematicWorld K("gravity.g");
  KOMO komo;
  komo.setModel(K, true);

  komo.setPathOpt(2.);

  MyTaskMap map;

  komo.setTask(-1.,-1., &map, OT_eq, {}, 1e3, 1);


//  komo.setTask(-1.,-1., new TaskMap_GJK(K, "block", "table1", true, true), OT_ineq, {});

  komo.setTask(-1.,-1., new TaskMap_Proxy(allPTMT, uintA(), .05), OT_sumOfSqr, NoArr, 1e2);
//  komo.setTask(-1.,-1., new TaskMap_GJK(K, "table1", "block", true, true), OT_ineq, ARR(-.02), 1e2, 0);

//  komo.setTask(-1.,-1., new TaskMap_ProxyConstraint(allPTMT, uintA(), .03), OT_ineq, NoArr, 1e2);

//  komo.setSlowAround(1.,2.);

  komo.reset();
  komo.run();
  komo.checkGradients();
  cout <<komo.getReport(true);

  while( komo.displayTrajectory(.1, true) );

  return 0;
}

