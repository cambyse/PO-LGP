#include <Roopi/roopi.h>
#include <KOMO/komo.h>
#include <Control/taskControl.h>
#include <Kin/frame.h>

//===============================================================================

void TEST(LimitsCollisions) {
  {
    Roopi R(false);

    R.setKinematics("model-coll.g");
    R.startTaskController();
    R.startTweets();
    //  R.taskController().verbose(1);
    R.collisions(true);
    auto limit = R.newLimitAvoidance();
    R.hold(false);

    arr tL = R.getK()->getFrameByName("endeffL")->X.pos.getArr();
    arr tR = R.getK()->getFrameByName("endeffR")->X.pos.getArr();
//    arr tL=ARR(.5, .1, 1.);
//    arr tR=ARR(.5, -.1, 1.);
    auto L = R.newMarker("targetL", tL);
    auto Re = R.newMarker("targetR", tR);

    auto posL = R.newCtrlTask(new TaskMap_Default(posDiffTMT, R.getK(), "endeffL", NoVector, "targetL"), {}, {}, {1e-1});
    auto posR = R.newCtrlTask(new TaskMap_Default(posDiffTMT, R.getK(), "endeffR", NoVector, "targetR"), {}, {}, {1e-1});

    for(uint k=0;k<100;k++){
      double box=.5;
#if 0
      posL->set()->setTarget(box*randn(3));
      posR->set()->setTarget(box*randn(3));
#else
      posL->stop();
      posR->stop();
      L->X.pos = tL + box*randn(3);
      Re->X.pos = tL + box*randn(3);
      posL->set()->setTarget({}); //box*randn(3));
      posR->set()->setTarget({}); //box*randn(3));
      posL->resetStatus();
      posR->resetStatus();
      posL->start();
      posR->start();
#endif

//      R.hold(false);
      R.wait(posL+posR);
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
