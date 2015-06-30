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
#include "../src/plotUtil.h"
#include <pr2/roscom.h>
#include <System/engine.h>

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);
  bool useRos = MT::getParameter<bool>("useRos");


  TaskManager *tm = new DonutTask();
  ors::KinematicWorld world("model.kvg");

  for (uint i=0;i<world.joints.d0;i++){
    cout << world.joints(i)->name << world.joints(i)->qIndex << world.joints(i)->qDim() << endl;
  }
  cout << world.getJointByName("l_gripper_r_finger_joint")->qIndex << endl;
/*
  Motion_Interface *mi = new Motion_Interface(world);

  arr X;
  mi->recordDemonstration(X,10.);
  write(LIST<arr>(X),"data/Xdemo.dat");
//  X << FILE("data/Xdemo.dat");

  arr x0 = X[0];
  mi->gotoPosition(x0);
  mi->executeTrajectory(X,10.);
  mi->~Motion_Interface();

*/
  return 0;

}
