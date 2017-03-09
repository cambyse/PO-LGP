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

    ofstream fil("z.data");

    auto posL   = baxter.task(GRAPH("map=pos ref1=endeffL ref2=base_footprint target=[0.6 0.6 1.3] PD=[1., .8, 1., 1.]"));
    auto posR   = baxter.task(GRAPH("map=pos ref1=endeffR ref2=base_footprint target=[0.6 -0.6 1.3] PD=[1., .8, 1., 1.]"));
    auto align = baxter.task(GRAPH("map=vecAlign ref1=endeffL ref2=obj1 vec1=[1 0 0] vec2=[0 0 -1] target=[1] PD=[1., .8, 1., 1.]"));
    auto grip  = baxter.task(GRAPH("map=qItself ref1=l_gripper_l_finger_joint target=[.1] PD=[1., .8, 1., 1.]"));

    while (! posL->isConverged() ) {
      cout <<"limits=" <<baxter.getJointLimits();
      cout <<"collision=" <<baxter.getCollisionScalar();
      mlr::wait(0.05);
    }

    baxter.waitConv({posR, align});

    mlr::wait(3.);

    arr q,qdot,u;
    cout <<"COLLECT" <<endl;
    for(uint i=0;i<100;i++){
      baxter.getState(q,qdot,u);
      fil <<q <<' ' <<qdot <<' ' <<u <<endl;
      mlr::wait(.01);
    }
    cout <<"STOP" <<endl;

    auto home = baxter.task(GRAPH(" map=qItself PD=[.5, 1., .2, 10.]"));
    baxter.modifyTarget(home, baxter.q0());
    baxter.stop({posL, posR, align, grip});
    baxter.waitConv({home});

    fil.close();
  }


  cout <<"bye bye" <<endl;
  return 0;
}
