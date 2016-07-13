#define SIM 1
#include <Core/util.h>
#include "interface/myBaxter.h"
#include "lockbox/lockbox.h"
#include <Control/taskController.h>

// =================================================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  MyBaxter baxter;

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


  Lockbox lockbox(&baxter);
  threadOpenModules(true);
  lockbox.initializeJoints();

  const uint start = 1;

  for (uint joint = start; joint <= 5; ++joint)
  {
    ors::KinematicWorld kw = baxter.getKinematicWorld();
    mlr::String handle = lockbox.joint_to_handle.at(joint);
    mlr::String name = lockbox.joint_to_ors_joint.at(joint);
    mlr::String str;
    str << "map=pos ref1=endeffL ref2=" << handle << " target=[0 0 0.2]";

    cout << "Fixing position of lockbox." << endl;

    auto hold = baxter.task("hold", new qItselfConstraint(kw.getJointByName(name)->qIndex, kw.getJointStateDimension()), 1, .8, 1, 1);

    cout << "Aligning. " << endl;
    auto alignX = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << handle << " vec1=[1 0 0] vec2=[0 0 -1] target=[1]")));
    auto alignY = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << handle << " vec1=[0 1 0] vec2=[1 0 0] target=[1]")));
    auto alignZ = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << handle << " vec1=[0 0 1] vec2=[0 -1 0] target=[1]")));

    cout << "Approach. " << endl;
    auto approach = baxter.task(GRAPH(str));
    baxter.waitConv({approach, alignX, alignY, alignZ});

    cout << "Pregrip." << endl;
    baxter.modifyTarget(approach, arr(0, 0, 0.05));

    baxter.waitConv({approach});

    cout << "Grip. " << endl;
    CtrlTask* gripTask = baxter.task("grip", new PointEqualityConstraint(kw, "endeffL", NoVector, handle, NoVector), 1, 1, 5, 5);
    baxter.stop({hold});

    // For the desired articulation:
    // - q_ref(lockbox_joint) = lockbox_joint->limits(1)

    auto move_joint = baxter.task("move_joint", new qItselfConstraint(kw.getJointByName(name)->qIndex, kw.getJointStateDimension()), 1, .8, 1, 1);
  //  baxter.modify(move_joint, GRAPH("prec=[1]"));

    //baxter.task(GRAPH("map=qItself PD=[1., 0.8, 1.0, 10.]"));

    arr new_js = kw.q;
    new_js(kw.getJointByName(name)->qIndex) = kw.getJointByName(name)->limits(1);

    cout << "Moving lockbox joint." << endl;
    baxter.modifyTarget(move_joint, ARR(kw.getJointByName(name)->limits(1)));


    baxter.waitConv({move_joint, gripTask});

    baxter.stop({move_joint, gripTask, approach});
    cout << "Done, fixing lockbox joint again." << endl;

    auto hold2 = baxter.task("hold", new qItselfConstraint(kw.getJointByName(name)->qIndex, kw.getJointStateDimension()), 1, .8, 1, 1);
    baxter.modifyTarget(hold2, ARR(kw.getJointByName(name)->limits(1)));

    str << "map=pos ref1=endeffL ref2=" << handle << " target=[0 0 0.05]";
    auto retract = baxter.task(GRAPH(str));
    baxter.waitConv({retract, alignX, alignY, alignZ});
    baxter.stop({retract, alignX, alignY, alignZ});


    cout << "Done retract. Going home." << endl;
    auto home = baxter.task(GRAPH(" map=qItself PD=[1., 0.8, 1.0, 10.] prec=[1]"));
    js(kw.getJointByName(name)->qIndex) = kw.getJointByName(name)->limits(1);
    baxter.modifyTarget(home, js);
    baxter.waitConv({home});
    baxter.stop({home});
  }

  cout <<"bye bye" <<endl;
  return 0;
}
