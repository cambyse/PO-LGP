#include <Core/util.h>
#include "../interface/myBaxter.h"
#include <Control/taskController.h>

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
    auto grip  = baxter.task(GRAPH("map=qItself ref1=l_gripper_l_finger_joint target=[.1] PD=[1., .8, 1., 1.]"));

    baxter.waitConv({posL, posR, alignL, alignR});

    mlr::wait(3.);

    baxter.reportPerceptionObjects();
    ors::Vector closest_vec = baxter.arPose();

    if (closest_vec.length() == 0)
      closest_vec.set(0.5, 0.6, 1);

    arr closest = ARR(closest_vec.x, closest_vec.y, closest_vec.z);

    baxter.modify(posR, GRAPH("PD=[3., 1., 1., 1.]"));
    baxter.modifyTarget(posR, closest + ARR(0.02, 0.05, .3));
    baxter.waitConv({posR, alignR});

    baxter.modifyTarget(posR, closest + ARR(0.02, 0.05, -.07));
    baxter.waitConv({posR, alignR});

    mlr::wait(3.);
    baxter.modifyTarget(grip, {0.});
    baxter.waitConv({posR, alignR, grip});

    baxter.modifyTarget(posR, closest + ARR(0., 0., .3));
    baxter.waitConv({posR, alignR});

    baxter.modifyTarget(posR, closest + ARR(0., -0.5, .3));
    baxter.waitConv({posR, alignR});

    baxter.modifyTarget(grip, {.1});
    baxter.waitConv({posR, alignR, grip});

    auto home = baxter.task(GRAPH(" map=qItself PD=[.5, 1., .2, 10.]"));
    baxter.modifyTarget(home, baxter.q0());
    baxter.stop({posR, posR, alignR, alignL, grip});
    baxter.waitConv({home});
  }

  cout <<"bye bye" <<endl;
  return 0;
}
