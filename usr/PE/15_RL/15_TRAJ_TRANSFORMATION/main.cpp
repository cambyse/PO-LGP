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
#include "../12_MBMF_LEARNING/motion_interface.h"
#include "../src/traj_factory.h"
#include "../src/plotUtil.h"
#include <pr2/roscom.h>
#include <System/engine.h>

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);
  bool useRos = MT::getParameter<bool>("useRos");


  TaskManager *tm = new DonutTask();
  ors::KinematicWorld world("donut.ors");
  arr X;
  tm->createSynthethicDemonstration(X,world);
  world.gl().resize(800,800);
  arr Pendeff,P;
  TrajFactory tf;
  tf.compFeatTraj(X,P,world,new DefaultTaskMap(posTMT,world,"endeff"));

  arr offset = ARRAY(0.,0.,0.05);

  arr yT;
  yT.resizeAs(P);
  /// interpolation in task space
  for(uint t=0;t<X.d0;t++){
    double s = t/double(X.d0);
    yT[t] = P[t]+offset*s;
  }

  arr Q;
  Q.resizeAs(X);
  arr xf = X[X.d0-1];
  arr yf,Jf;// = P[P.d0-1];

  world.setJointState(xf);

  ors::Vector v(0.,0.,0.4);
  world.kinematicsPos(yf,Jf,world.getShapeByName("endeff")->body,&v);

  arr C = eye(3);
  arr W = eye(4)*1e-3;


  arr yTarget = yf + offset;
  arr q_offset = inverse_SymPosDef(~Jf*C*Jf + W)*~Jf*C*(yTarget-yf);
  cout<<yTarget << endl;
  cout << q_offset << endl;
  /// interpolation in joint space
  for(uint t=0;t<X.d0;t++){
    double s = t/double(X.d0);
    Q[t] = X[t]+q_offset*s;
  }
  arr qT;
  tf.compFeatTraj(Q,qT,world,new DefaultTaskMap(posTMT,world,"endeff"));

  cout << qT[qT.d0-1] << endl;
  world.gl().add(drawRedPoints,&(qT));
  world.gl().add(drawBluePoints,&(P));
  world.gl().add(drawGreenPoints,&(yT));

  world.watch(true);
  displayTrajectory(X,-1,world,"X");
  displayTrajectory(Q,-1,world,"X");




  return 0;

}
