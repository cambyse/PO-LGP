#include <Core/util.h>
#include "interface/myBaxter.h"
#include "lockbox/lockbox.h"
#include <Control/taskControl.h>
#include <Motion/komo.h>

// =================================================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  bool useRos = mlr::getParameter<bool>("useRos", false);
  cout << (useRos ? "Using ROS." : "Simulating.") << endl;
  rnd.clockSeed();

  MyBaxter baxter;
  Lockbox lockbox(&baxter);

  threadOpenModules(true);

  lockbox.initializeJoints(); // Currently keeping this separate, since it doesn't wait for all modules to be really started...

  lockbox.moveHome(true);

  if (useRos)
  {
    baxter.disablePosControl();
    mlr::wait(5.);
    lockbox.update();
    baxter.enablePosControl();

    mlr::Transformation tf = baxter.getModelWorld().getShapeByName("alvar_10")->X;


    // Position task
    mlr::String str;
    mlr::Vector target = tf*mlr::Vector(-0.05, 0, 0.25);
    str << "map=pos ref1=endeffL ref2=base_footprint vec2=[" << target.x << ", " << target.y << ", " << target.z << "] PD=[1., 1.2, .2, 1.]";

//    mlr::Vector target = mlr::Vector(0, 0, 0.25);
//    str << "map=pos ref1=endeffL ref2=alvar_10 vec2=[" << target.x << ", " << target.y << ", " << target.z << "] PD=[.5, .8, .2, 1.]";

    auto approach = baxter.task(GRAPH(str));
//    baxter.waitConv({approach});

    // Alignment tasks.
//    auto alignX = baxter.task("alignX", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=alvar_10 vec1=[1 0 0] vec2=[0 0 -1] target=[1] prec=[1000]  PD=[.5, .8, .2, 1.]")));
//    auto alignY = baxter.task("alignY", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=alvar_10 vec1=[0 1 0] vec2=[1 0 0] target=[1] prec=[1000] PD=[.5, .8, .2, 1.]")));
//    auto alignZ = baxter.task("alignZ", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=alvar_10 vec1=[0 0 1] vec2=[0 -1 0] target=[1] prec=[1000] PD=[.5, .8, .2, 1.]")));

    mlr::Vector vecx = tf.rot * mlr::Vector(0,0,-1); vecx.normalize();
    mlr::Vector vecy = tf.rot * mlr::Vector(1,0,0); vecy.normalize();
    mlr::Vector vecz = tf.rot * mlr::Vector(0,-1, 0); vecz.normalize();
    auto alignX = baxter.task("alignX", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[1 0 0] vec2=[" << vecx.x << ' ' << vecx.y << ' ' << vecx.z << "] target=[1] prec=[1000]  PD=[1., 1.2, .2, 1.]")));
    auto alignY = baxter.task("alignY", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 1 0] vec2=[" << vecy.x << ' ' << vecy.y << ' ' << vecy.z << "] target=[1] prec=[1000] PD=[1., 1.2, .2, 1.]")));
    auto alignZ = baxter.task("alignZ", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 0 1] vec2=[" << vecz.x << ' ' << vecz.y << ' ' << vecz.z << "] target=[1] prec=[1000] PD=[1., 1.2, .2, 1.]")));

    baxter.waitConv({approach, alignX, alignY, alignZ});
    baxter.testRealConv({approach, alignX, alignY, alignZ}, 10);

    // Now we are 15 cm away and aligned with the handle.
    baxter.stop({approach, alignX, alignY, alignZ});

    baxter.disablePosControl();
    mlr::wait(5.);
    lockbox.update();
    baxter.enablePosControl();

    lockbox.moveHome(true);
  }
  else
  {
    lockbox.update();
  }

  lockbox.readyToTest = true;

  moduleShutdown().waitForStatusGreaterThan(0);




////  for (uint joint = 1; joint <= 5; ++joint)
////  {
////    double max = baxter.getKinematicWorld().getJointByName(lockbox.joint_to_ors_joint.at(joint))->limits(1);
////    lockbox.moveJoint(joint, max );// / (3-i));
////  }

//  //Hack to ignore the 5th joint for now...
////  lockbox.locked_joints.removeValue(5);

//  lockbox.locked_joints.removeValue(1);
//  lockbox.locked_joints.removeValue(2);
//  lockbox.locked_joints.removeValue(3);
//  lockbox.locked_joints.removeValue(4);


//  mlr::Array<uint> joints_to_test = mlr::Array<uint>(lockbox.locked_joints);

//  std::srand(std::time(NULL));
//  while (joints_to_test.N > 0)
//  {
//    const uint joint = joints_to_test(std::rand() % joints_to_test.N);
//    cout << "Testing: " << joint << endl;
//    bool unlock = lockbox.testJoint(joint);

//    if (unlock)
//    {
//      joints_to_test = lockbox.locked_joints;
//    }
//    else
//    {
//      joints_to_test.removeValue(joint);
//    }
//    cout << "Joints to test: " << joints_to_test << endl;
//    cout << "Locked joints: " << lockbox.locked_joints << endl;
//  }

  cout <<"bye bye" <<endl;
  return 0;
}
