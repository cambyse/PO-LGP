#include <Roopi/roopi.h>
#include <Motion/komo.h>
#include <Control/taskControl.h>

//===============================================================================

void TEST(LimitsCollisions) {
  {
    Roopi R(false);

    R.setKinematics("model-coll.g");
    R.startTaskController();
    R.startTweets();
    //  R.taskController().verbose(1);
    R.collisions(true);

    for(uint k=0;k<10;k++){
      double box=.5;
      auto posL = R.newCtrlTask(new TaskMap_Default(posTMT, R.getKinematics(), "endeffL"));
      posL.set()->PD().setTarget(posL.y0 + box*rand(3) - (box/2.));

      auto posR = R.newCtrlTask(new TaskMap_Default(posTMT, R.getKinematics(), "endeffR"));
      posR.set()->PD().setTarget(posR.y0 + box*rand(3) - (box/2.));


      R.hold(false);
      R.wait({&posL});
      R.hold(true);
    }
  }
  cout <<"LEFT OVER REGISTRY:\n" <<registry() <<endl;
}

//===============================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  testLimitsCollisions();

  return 0;
}
