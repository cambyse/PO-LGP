#include <Core/util.h>
#include "interface/myBaxter.h"
#include "lockbox/lockbox.h"
#include <Control/taskController.h>

// =================================================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  {
    MyBaxter baxter;

    mlr::wait(2.);
    auto home = baxter.task(GRAPH(" map=qItself PD=[.5, 1., .2, 10.]"));

    baxter.modifyTarget(home, baxter.q0());
    baxter.waitConv({home});
    baxter.stop({home});

    auto endR   = baxter.task(GRAPH("map=pos ref1=endeffR ref2=base_footprint target=[0.6 -0.6 1.3] PD=[1., .8, 1., 1.]"));
    auto endL   = baxter.task(GRAPH("map=pos ref1=endeffL ref2=base_footprint target=[0.6 0.6 1.3] PD=[1., .8, 1., 1.]"));
    baxter.waitConv({endR, endL});
    baxter.stop({endR, endL});

    mlr::wait(2.);
    arr js = baxter.getJointState();

    Lockbox lockbox;

    ors::Transformation tf;
    while(1){
      for (uint joint = 1; joint < 6; ++joint)
      {
        std::cout << "Joint: " << joint << std::endl;
        if (lockbox.getAbsoluteJointTransform(joint, tf))
        {
          auto alignR = baxter.task(GRAPH("map=vecAlign ref1=endeffR ref2=obj1 vec1=[1 0 0] vec2=[1 0 0] target=[1] PD=[1., .8, 1., 1.]"));
          auto elbowR   = baxter.task(GRAPH("map=vecAlign ref1=wristR ref2=base_footprint vec1=[0 1 0] vec2=[0 0 -1] target=[1] PD=[1., .8, 1., 1.]"));

          ors::Vector pos = tf.pos;
          pos += ors::Vector(lockbox.offsets.at(joint)(0),lockbox.offsets.at(joint)(1), lockbox.offsets.at(joint)(2));


          mlr::String pd = STRING("PD=[1., .8, 1., 1.]");
          mlr::String str = STRING(
                              "map=pos ref1=endeffR ref2=base_footprint target=["
                              << pos.x - 0.1 << ' ' << pos.y << ' ' << pos.z
                              << "] " << pd);


          std::cout << "\t" << str << std::endl;

          auto posR = baxter.task(GRAPH(str));
          baxter.waitConv({posR, alignR});

          mlr::wait(3.);
          baxter.modifyTarget(posR, ARR(pos.x + 0.03, pos.y, pos.z));
          baxter.waitConv({posR, alignR});

          mlr::wait(3.);
          baxter.modifyTarget(posR, ARR(pos.x - 0.1, pos.y, pos.z));
          baxter.waitConv({posR, alignR});

          baxter.stop({posR, alignR, elbowR});

          auto homer = baxter.task(GRAPH(" map=qItself PD=[.5, 1., .2, 10.]"));

          baxter.modifyTarget(homer, js);
          baxter.waitConv({homer});
          baxter.stop({homer});
        }
        mlr::wait(1.);
      }
    }
  }

  cout <<"bye bye" <<endl;
  return 0;
}
