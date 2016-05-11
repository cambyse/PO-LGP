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

    baxter.modifyTarget(posR, closest + ARR(0.1, 0.12, -.07));
    baxter.waitConv({posR, alignR});

    mlr::wait(2.);

    baxter.modifyTarget(posR, closest + ARR(0.1, 0.12, -.08));
    baxter.waitConv({posR, alignR});

    baxter.reportJointState();
    baxter.disablePosControl();

    //-- compute torques
    arr y,J;
    posR->map.phi(y, J, baxter.getKinematicWorld());

    // Send it 0 torques for 1 second
    for (uint i = 0; i < 200; i++) {
      baxter.publishTorque(~J * ARR(0.,0.,-10.));
      mlr::wait(0.01);
    }
    baxter.enablePosControl();

    auto home = baxter.task(GRAPH(" map=qItself PD=[.5, 1., .2, 10.]"));
    baxter.modifyTarget(home, baxter.q0());
    baxter.stop({posL, posR, alignR, alignL});
    baxter.waitConv({home});

  }

  cout <<"bye bye" <<endl;
  return 0;
}
