#include <Core/util.h>
#include "../interface/myBaxter.h"
#include <Control/taskControl.h>

// =================================================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  {
    MyBaxter baxter;

//    auto pos = baxter.task("endeffL",
//                           posTMT, "endeffL", NoVector, "obj1", NoVector,
//                           ARR(0., 0., .3),
//                           1., .8, 1., 1.);

    auto posL   = baxter.task(GRAPH("map=pos ref1=endeffL ref2=base_footprint target=[0.6 0.6 1.3] PD=[1., .8, 1., 1.]"));
    auto posR   = baxter.task(GRAPH("map=pos ref1=endeffR ref2=base_footprint target=[0.6 -0.6 1.3] PD=[1., .8, 1., 1.]"));
    auto alignL = baxter.task(GRAPH("map=vecAlign ref1=endeffL ref2=obj1 vec1=[1 0 0] vec2=[0 0 -1] target=[1] PD=[1., .8, 1., 1.]"));
    auto alignR = baxter.task(GRAPH("map=vecAlign ref1=endeffR ref2=obj1 vec1=[1 0 0] vec2=[0 0 -1] target=[1] PD=[1., .8, 1., 1.]"));

    baxter.waitConv({posL, posR, alignL, alignR});
    mlr::wait(5.);

    baxter.reportPerceptionObjects();

    arr torques = baxter.getEfforts();
    std::cout << torques << std::endl;
    mlr::wait(2.);

    baxter.disablePosControl();
    baxter.enableTotalTorqueMode();

    // Send it 0 torques for 4 seconds
    for (uint i = 0; i < 400; i++)
    {
      baxter.publishTorque(torques, "right_");
      mlr::wait(0.01);
    }
    baxter.disableTotalTorqueMode();
    baxter.enablePosControl();

    auto home = baxter.task(GRAPH(" map=qItself PD=[.5, 1., .2, 10.]"));
    baxter.modifyTarget(home, baxter.q0());
    baxter.stop({posL, posR, alignR, alignL});
    baxter.waitConv({home});

  }

  cout <<"bye bye" <<endl;
  return 0;
}
