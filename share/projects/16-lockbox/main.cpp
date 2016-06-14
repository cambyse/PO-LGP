#include <Core/util.h>
#include "interface/myBaxter.h"
#include "lockbox/lockbox.h"
#include <Control/taskController.h>

// =================================================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  MyBaxter baxter;

//  mlr::wait(5.);

  //    auto home = baxter.task(GRAPH(" map=qItself PD=[1., 0.8, 1.0, 10.]"));
  //    baxter.modifyTarget(home, baxter.q0());
  //    baxter.waitConv({home});
  //    baxter.stop({home});


  auto endL   = baxter.task(GRAPH("map=pos ref1=endeffL ref2=base_footprint target=[0.6 0.45 1.4] PD=[1., .8, 1., 1.]"));
  auto endR   = baxter.task(GRAPH("map=pos ref1=endeffR ref2=base_footprint target=[0.6 -0.8 1.5] PD=[1., .8, 1., 1.]"));
  auto alignT = baxter.task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[1 0 0] vec2=[1 0 0] target=[1] PD=[1., .8, 1., 1.]"));
  auto alignRT = baxter.task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 1 0] vec2=[0 0 -1] target=[1] PD=[1., .8, 1., 1.]"));
  auto wristT   = baxter.task(GRAPH("map=vecAlign ref1=wristL ref2=base_footprint vec1=[0 1 0] vec2=[0 0 1] target=[1] PD=[1., .8, 1., 1.]"));
  auto elbowL   = baxter.task(GRAPH("map=vecAlign ref1=elbowL ref2=base_footprint vec1=[0 1 0] vec2=[0 0 1] target=[1] PD=[1., .8, 1., 5.]"));

  baxter.grip(0);

  baxter.waitConv({alignRT, alignT, endR, endL});
  baxter.stop({alignT, alignRT, wristT, elbowL, endR, endL});

  mlr::wait(3.);

  arr js = baxter.getJointState();
  std::cout << js << std::endl;

  Lockbox lockbox(&baxter);

  mlr::wait(5.);

  for (uint joint = 1; joint < 6; ++joint)
  {
    ors::Transformation tf; tf.setZero();
    tf.addRelativeTranslation(-0.1, 0.05, 0);
    tf.appendTransformation(lockbox.joint_origins.at(joint));
    lockbox.moveAbsolute(tf);
//    lockbox.moveToJoint(joint, tf);
    baxter.disablePosControl();
    mlr::wait(2.);
    lockbox.update = false;
    lockbox.joint_origins.at(joint) = lockbox.joint_tfs.at(joint);
    cout << "Joint: " << joint << endl;
    cout << "\t" << lockbox.joint_origins.at(joint);
    lockbox.update = true;
    baxter.enablePosControl();

    lockbox.moveRelative(ors::Vector(-0.1, 0, 0), 1, 100);

  }



//  for (uint joint = 1; joint < 6; ++joint)
//  {
//    auto alignZ = baxter.task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[1 0 0] vec2=[1 0 0] target=[1] PD=[1., .8, 1., 1.] prec=[50]"));
//    auto alignZ2 = baxter.task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 1 0] vec2=[0 0 -1] target=[1] PD=[1., .8, 1., 1.] prec=[50]"));
//    auto alignZ3 = baxter.task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 0 1] vec2=[0 1 0] target=[1] PD=[1., .8, 1., 1.] prec=[50]"));

//    ors::Transformation tf;
//    tf.setZero();
//    tf.addRelativeTranslation(-0.1, 0.05, 0);
//    lockbox.update = false;
//    lockbox.moveAbsolute(lockbox.joint_origins.at(joint).pos + tf.pos, 1, 50.);
//    //lockbox.moveToAlvar(joint, tf, 1, 50.);
//    baxter.stop({alignZ, alignZ2, alignZ3});
//    lockbox.update = true;
//    mlr::wait(1.);
//    baxter.disablePosControl();

//    lockbox.getAbsoluteJointTransform(joint, tf);
//    lockbox.joint_origins.at(joint) = tf;

//    baxter.enablePosControl();
//    mlr::wait(1.);
//    lockbox.moveRelative(ors::Vector(-0.1, 0, 0), 1, 100);
//  }

  auto homer = baxter.task(GRAPH(" map=qItself PD=[1, 1., 1, 5.] prec=[100.]"));
  baxter.modifyTarget(homer, js);
  baxter.waitConv({homer});
  baxter.stop({homer});

  while(1)
  {
    for(uint joint = 1; joint < 6; ++joint)
    {
      double position;

      if (lockbox.getJointPosition(joint, position))
      {
        std::cout << joint << ": " << position << std::endl;

        baxter.grip(0);

        lockbox.moveJointToPosition(joint, 1);
        /*
        auto alignR = baxter.task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[1 0 0] vec2=[0 0 1] target=[0] PD=[1., .8, 1., 5.] prec=[1]"));
        auto alignR2 = baxter.task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 1 0] vec2=[0 0 -1] target=[1] PD=[1., .8, 1., 5.] prec=[1]"));
        auto alignR3 = baxter.task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 0 1] vec2=[1 0 0] target=[0] PD=[1., .8, 1., 5.] prec=[1]"));

        ors::Transformation offset_tf;
        offset_tf.setZero();
        offset_tf.addRelativeTranslation(-0.2, 0.05, 0);
        lockbox.moveToAlvar(joint, offset_tf, 1.2, 50);
        baxter.waitConv({alignR, alignR2, alignR3});

        baxter.disablePosControl();
        mlr::wait(1.);
        lockbox.update = false;
        baxter.enablePosControl();
        mlr::wait(1.);

        offset_tf.setZero();
        offset_tf.addRelativeTranslation(-0.08, 0, 0);
        lockbox.moveToJointStub(joint, offset_tf, 1, 50);
        lockbox.moveToJointStub(joint, offset_tf, 1, 0.1);
        baxter.waitConv({alignR, alignR2, alignR3});


        offset_tf.setZero();
        lockbox.moveToJointStub(joint, offset_tf, 1, 50);
        lockbox.moveToJointStub(joint, offset_tf, 1, 1);
        baxter.waitConv({alignR, alignR2, alignR3});



//        lockbox.moveRelative(-offset_tf.pos, 1, 1);
//        lockbox.moveRelative(-offset_tf.pos, 1, 0.1);
//        mlr::wait(2.);

        baxter.grip(1);

        baxter.stop({alignR, alignR2, alignR3});

        ors::Quaternion rot = lockbox.end_orientations.at(joint);
        ors::Quaternion initial; initial.setRpy(-MLR_PI/2, 0, 0);
        rot = rot * initial;

        ors::Vector xVec = rot.getMatrix() * ors::Vector(1, 0, 0);
        ors::Vector yVec = rot.getMatrix() * ors::Vector(0, 1, 0);
        ors::Vector zVec = rot.getMatrix() * ors::Vector(0, 0, 1);


        auto alignX = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[1 0 0] vec2=["
                                               << xVec.x << ' ' << xVec.y << ' ' << xVec.z
                                               << "] target=[1] PD=[1., 1., .8, 1.] prec=[1000]")));

        auto alignY = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 1 0] vec2=["
                                               << yVec.x << ' ' << yVec.y << ' ' << yVec.z
                                               << "] target=[1] PD=[1., 1., .8, 1.] prec=[1000]")));

        auto alignZ = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 0 1] vec2=["
                                               << zVec.x << ' ' << zVec.y << ' ' << zVec.z
                                               << "] target=[1] PD=[1., 1., .8, 1.] prec=[1000]")));

        lockbox.moveRelative(lockbox.end_offsets.at(joint), 0.8, 1000);

   //     lockbox.moveRelative(lockbox.end_offsets.at(joint), 0.8, 30);
        mlr::wait(2.);

        alignX->prec = ARR(50);
        alignY->prec = ARR(50);
        alignZ->prec = ARR(50);
        baxter.waitConv({alignX, alignY, alignZ});

        baxter.grip(0);
//        mlr::wait(2.);

        ors::Vector away = rot * ors::Vector(-.1, 0, 0);
        lockbox.moveRelative(away, 0.8, 100);
        mlr::wait(2.);
        baxter.stop({alignX, alignY, alignZ});
        */

        auto homer = baxter.task(GRAPH(" map=qItself PD=[1, 1., .2, 5.] prec=[100.]"));
        baxter.modifyTarget(homer, js);
        baxter.waitConv({homer});
        baxter.stop({homer});

        lockbox.update = true;
      }
    }
    std::cout << std::endl;
    mlr::wait(0.25);
  }

  cout <<"bye bye" <<endl;
  return 0;
}
