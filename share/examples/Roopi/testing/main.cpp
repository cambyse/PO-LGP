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

    auto posL = R.newCtrlTask(new TaskMap_Default(posTMT, R.getKinematics(), "endeffL"), {1., .8});
    auto posR = R.newCtrlTask(new TaskMap_Default(posTMT, R.getKinematics(), "endeffR"), {1., .8});
    for(uint k=0;k<100;k++){
      double box=1.;
      posL.set()->PD().setTarget(posL.y0 + box*randn(3));

      posR.set()->PD().setTarget(posR.y0 + box*randn(3));


//      R.hold(false);
      R.wait({&posL, &posR});
//      R.hold(true);
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
