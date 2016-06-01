#include <Core/util.h>
#include "interface/myBaxter.h"
#include "lockbox/lockbox.h"
#include <Control/taskController.h>

// =================================================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  {
    MyBaxter baxter;

    mlr::wait(3.);
    auto home = baxter.task(GRAPH(" map=qItself PD=[1., 0.8, 1.0, 10.]"));
    baxter.modifyTarget(home, baxter.q0());
    baxter.waitConv({home});
    baxter.stop({home});

    baxter.grip(0);


    auto endL   = baxter.task(GRAPH("map=pos ref1=endeffL ref2=base_footprint target=[0.7 0.2 1.6] PD=[1., .8, 1., 1.]"));
    auto endR   = baxter.task(GRAPH("map=pos ref1=endeffR ref2=base_footprint target=[0.6 -0.8 1.5] PD=[1., .8, 1., 1.]"));
    auto alignT = baxter.task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[1 0 0] vec2=[1 0 0] target=[1] PD=[1., .8, 1., 1.]"));
    auto alignRT = baxter.task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 1 0] vec2=[0 1 0] target=[1] PD=[1., .8, 1., 1.]"));
    auto wristT   = baxter.task(GRAPH("map=vecAlign ref1=wristL ref2=base_footprint vec1=[0 1 0] vec2=[0 0 1] target=[1] PD=[1., .8, 1., 1.]"));
    auto elbowL   = baxter.task(GRAPH("map=vecAlign ref1=elbowL ref2=base_footprint vec1=[0 1 0] vec2=[0 0 1] target=[1] PD=[1., .8, 1., 5.]"));

    endL->prec = ARR(75.);
    baxter.waitConv({alignRT, alignT, endR, endL});
    mlr::wait(3.);

    baxter.stop({wristT, elbowL, alignT, endR, endL});

    arr js = baxter.getJointState();
    Lockbox lockbox;
    mlr::wait(3.);

    ors::Transformation tf;
    while(1)
    {
      for(uint joint = 1; joint < 6; ++joint)
      {
        std::cout << "Joint: " << joint << std::endl;
        if (lockbox.getAbsoluteJointTransform(joint, tf))
        {
          auto alignR = baxter.task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[1 0 0] vec2=[1 0 0] target=[1] PD=[1., .8, 1., 1.]"));
          auto wristR   = baxter.task(GRAPH("map=vecAlign ref1=wristL ref2=base_footprint vec1=[0 1 0] vec2=[0 0 1] target=[1] PD=[1., .8, 1., 1.]"));

          ors::Vector pos = tf.pos;
          //pos += ors::Vector(lockbox.offsets.at(joint)(0),lockbox.offsets.at(joint)(1), lockbox.offsets.at(joint)(2));


          baxter.grip(0);
          mlr::String pd = STRING("PD=[1., .8, 1., 1.]");
          mlr::String str = STRING(
                              "map=pos ref1=endeffL ref2=base_footprint target=["
                              << pos.x - 0.2 << ' ' << pos.y << ' ' << pos.z
                              << "] " << pd);


          std::cout << "\t" << str << std::endl;

          auto posR = baxter.task(GRAPH(str));
          posR->prec = ARR(75.);
          baxter.waitConv({posR, alignR});
          mlr::wait(2.);

          ors::Vector off = pos + ors::Vector(lockbox.offsets.at(joint)(0),lockbox.offsets.at(joint)(1), lockbox.offsets.at(joint)(2));

          baxter.modifyTarget(posR, ARR(off.x - 0.1, off.y, off.z));
          baxter.waitConv({posR, alignR});

          mlr::wait(2.);

          baxter.modifyTarget(posR, ARR(off.x - 0.03, off.y, off.z));
          baxter.waitConv({posR, alignR});

          baxter.grip(1);
          baxter.grip(0);

          mlr::wait(2.);
          baxter.modifyTarget(posR, ARR(off.x - 0.1, off.y, off.z));
          baxter.waitConv({posR, alignR});

          baxter.stop({posR, alignR, wristR});

          auto homer = baxter.task(GRAPH(" map=qItself PD=[.5, 1., .2, 10.]"));

          baxter.modifyTarget(homer, js);
          baxter.waitConv({homer});
          baxter.stop({homer});
          mlr::wait(3.);
        }
        mlr::wait(1.);
      }
    }
  }

  cout <<"bye bye" <<endl;
  return 0;
}
