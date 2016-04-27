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

    auto pos   = baxter.task(GRAPH("map=pos ref1=endeffL ref2=obj1 target=[0 0 .3] PD=[1., .8, 1., 1.]"));
    auto align = baxter.task(GRAPH("map=vecAlign ref1=endeffL ref2=obj1 vec1=[1 0 0] vec2=[0 0 -1] target=[1] PD=[1., .8, 1., 1.]"));
    auto grip  = baxter.task(GRAPH("map=qItself ref1=l_gripper_l_finger_joint target=[.1] PD=[1., .8, 1., 1.]"));
    baxter.waitConv({pos, align});

    mlr::wait(3.);
    baxter.reportPerceptionObjects();

    baxter.modify(pos, GRAPH("target=[0 0 .1] PD=[3., 1., 1., 1.]"));
    baxter.waitConv({pos, align});

    baxter.modifyTarget(grip, {0.});
    baxter.waitConv({pos, align, grip});

    baxter.modifyTarget(pos, {0., 0., .3});
    baxter.waitConv({pos, align});

    baxter.modifyTarget(pos, {0., .5, .3,});
    baxter.waitConv({pos, align});

    baxter.modifyTarget(grip, {.1});
    baxter.waitConv({pos, align, grip});

    auto home = baxter.task(GRAPH(" map=qItself PD=[.5, 1., .2, 10.]"));
    baxter.modifyTarget(home, baxter.q0());
    baxter.stop({pos, align, grip});
    baxter.waitConv({home});
  }

  cout <<"bye bye" <<endl;
  return 0;
}
