#include <Core/util.h>
#include "../interface/myBaxter.h"

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
    auto align = baxter.task(GRAPH("map=vecAlign ref1=endeffL ref2=obj1 vec1=[1 0 0] vec2=[0 0 -1] target=[1] PD=[1., .8, 1., 1.]"));
    auto grip  = baxter.task(GRAPH("map=qItself ref1=l_gripper_l_finger_joint target=[.1] PD=[1., .8, 1., 1.]"));
    baxter.waitConv({posL, posR, align});

    mlr::wait(3.);

    baxter.reportPerceptionObjects();
    ors::Vector closest_vec = baxter.closestCluster();
    arr closest = ARR(closest_vec.x, closest_vec.y, closest_vec.z);

    baxter.modify(posL, GRAPH("PD=[3., 1., 1., 1.]"));
    baxter.modifyTarget(posL, closest + ARR(0.02, 0.05, .3));
    baxter.waitConv({posL, align});

    baxter.modifyTarget(posL, closest + ARR(0.02, 0.05, .03));
    baxter.waitConv({posL, align});

    baxter.modifyTarget(grip, {0.});
    baxter.waitConv({posL, align, grip});

    baxter.modifyTarget(posL, closest + ARR(0., 0., .3));
    baxter.waitConv({posL, align});

    baxter.modifyTarget(posL, closest + ARR(0., 0.5, .3));
    baxter.waitConv({posL, align});

    baxter.modifyTarget(grip, {.1});
    baxter.waitConv({posL, align, grip});

    auto home = baxter.task(GRAPH(" map=qItself PD=[.5, 1., .2, 10.]"));
    baxter.modifyTarget(home, baxter.q0());
    baxter.stop({posL, posR, align, grip});
    baxter.waitConv({home});
  }

  cout <<"bye bye" <<endl;
  return 0;
}
