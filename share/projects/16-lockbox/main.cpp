#define SIM 1
#include <Core/util.h>
#include "interface/myBaxter.h"
#include "lockbox/lockbox.h"
#include <Control/taskController.h>
#include <Motion/komo.h>

// =================================================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  ors::KinematicWorld lockbox_world;
  lockbox_world.init(mlr::mlrPath("data/baxter_model/baxter-lockbox.ors").p);

  KOMO komo;
  komo.setModel(lockbox_world);
  komo.setTiming(1, 1, 5., 1, true);
  komo.setSquaredQVelocities();
  komo.setCollisions(true);
  komo.setLimits(true);

  arr posR = 3.*randn(3);
 // ors::Joint* world_joint = baxter_world.getJointByName("lockbox_world_joint");
//  posR += baxter_world.getShapeByName("lockbox_marker6")->X.pos.getArr();

//  ors::Shape* marker = baxter_world.getShapeByName("lockbox_marker6")->X.pos;
  //cout << "Joint position: " << world_joint->A.pos.getArr() << endl;
  //cout << "Posr: " << posR << endl;

  komo.setPosition(1., 1., "lockbox_marker6", NULL, sumOfSqrTT, posR);
  komo.reset();
  komo.run();

  // New Q vector is in komo.x
  lockbox_world.setJointState(komo.x);

  ors::Transformation lockboxPos = lockbox_world.getBodyByName("lockbox")->X;

  MyBaxter baxter;

  baxter.updateLockbox(lockboxPos);
  // Get updated shape position for the


  cout << "Done." << endl;
//  mlr::wait();

//  KOMO komo;
//  ors::KinematicWorld lockbox_world;
//  lockbox_world.init(mlr::mlrPath("data/lockboxNew.ors").p);

//  komo.setModel(lockbox_world);
//  komo.setTiming(1, 1, 5., 1, true);
//  komo.setSquaredQVelocities();
//  komo.setCollisions(true);
//  komo.setLimits(true);
//  komo.setPosition(-1., -1., "lockbox_marker6", NULL, sumOfSqrTT, posR);
//  komo.reset();
//  komo.run();

//  Graph result = komo.getReport();
////    cout <<result <<endl;
//  double cost = result.get<double>({"total","sqrCosts"});
//  double constraints = result.get<double>({"total","constraints"});

//  if(constraints<.1 && cost<5.){
//    komo.x.refRange(0,2)=0.;
//    W.setJointState(komo.x);
//  }else{
//    return getPose(qInit);
//  }
//  W.watch(false);
//  arr qT = komo.x;


#if SIM
#else
  mlr::wait(2.);
#endif

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

  baxter.waitConv({alignRT, alignT, endR, endL});
  arr js = baxter.getTaskControllerModule().ctrl_q_ref.get();
  baxter.stop({alignT, alignRT, wristT, elbowL, endR, endL});

  baxter.grip(0, SIM);

  mlr::wait(5.);
  Lockbox lockbox(&baxter);
  threadOpenModules(true);
  lockbox.initializeJoints();

  const uint start = 1;

  for (uint joint = start; joint <= 5; ++joint)
  {
    mlr::String handle = lockbox.joint_to_handle.at(joint);
    mlr::String name = lockbox.joint_to_ors_joint.at(joint);
    mlr::String str;
    str << "map=pos ref1=endeffL ref2=" << handle << " target=[0.05 0 0]";

    cout << "Fixing position of joint: " << name << endl;

    auto hold = baxter.task("hold", new qItselfConstraint(baxter.getKinematicWorld().getJointByName(name)->qIndex, baxter.getKinematicWorld().getJointStateDimension()), 1, .8, 1, 1);

    cout << "Aligning. " << endl;
    auto alignX = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << handle << " vec1=[1 0 0] vec2=[0 0 -1] target=[1]")));
    auto alignY = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << handle << " vec1=[0 1 0] vec2=[1 0 0] target=[1]")));
    auto alignZ = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << handle << " vec1=[0 0 1] vec2=[0 -1 0] target=[1]")));

    cout << "Approach. " << endl;
    auto approach = baxter.task(GRAPH(str));
    baxter.waitConv({approach, alignX, alignY, alignZ});

    cout << "Pregrip." << endl;
    baxter.modifyTarget(approach, arr(-0.05, 0, 0));

    baxter.waitConv({approach});

    cout << "Grip. " << endl;
    auto gripTask = baxter.task("grip", new PointEqualityConstraint(baxter.getKinematicWorld(), "endeffL", NoVector, handle, NoVector), 1, 1, 5, 5);
    baxter.stop({hold, approach});

    // For the desired articulation:
    // - q_ref(lockbox_joint) = lockbox_joint->limits(1)

    auto move_joint = baxter.task("move_joint", new qItselfConstraint(baxter.getKinematicWorld().getJointByName(name)->qIndex, baxter.getKinematicWorld().getJointStateDimension()), 1, .8, 1, 1);

    cout << "Moving lockbox joint." << endl;
    baxter.modifyTarget(move_joint, ARR(baxter.getKinematicWorld().getJointByName(name)->limits(1)));

    baxter.waitConv({move_joint, gripTask});

    baxter.stop({move_joint, gripTask});
    cout << "Done, fixing lockbox joint again." << endl;
    mlr::wait(1.);

    auto hold2 = baxter.task("hold2", new qItselfConstraint(baxter.getKinematicWorld().getJointByName(name)->qIndex, baxter.getKinematicWorld().getJointStateDimension()), 1, .8, 1, 1);
    baxter.modifyTarget(hold2, ARR(baxter.getKinematicWorld().getJointByName(name)->limits(1)));

    str << "map=pos ref1=endeffL ref2=" << handle << " target=[-0.05 0 0]";
    auto retract = baxter.task(GRAPH(str));
    baxter.waitConv({retract, alignX, alignY, alignZ});
    baxter.stop({retract, alignX, alignY, alignZ});


    cout << "Done retract. Going home." << endl;
    auto home = baxter.task(GRAPH(" map=qItself PD=[1., 0.8, 1.0, 10.] prec=[1]"));
    js(baxter.getKinematicWorld().getJointByName(name)->qIndex) = baxter.getKinematicWorld().getJointByName(name)->limits(1);
    baxter.modifyTarget(home, js);
    baxter.waitConv({home});
    baxter.stop({home, hold2});
  }

  cout <<"bye bye" <<endl;
  return 0;
}
