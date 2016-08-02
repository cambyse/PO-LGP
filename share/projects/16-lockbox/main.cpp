#include <Core/util.h>
#include "interface/myBaxter.h"
#include "lockbox/lockbox.h"
#include <Control/taskController.h>
#include <Motion/komo.h>

// =================================================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  bool useRos = mlr::getParameter<bool>("useRos", false);
  cout << (useRos ? "Using ROS." : "Simulating.") << endl;
  rnd.clockSeed();

  MyBaxter baxter;

  auto endL   = baxter.task(GRAPH("map=pos ref1=endeffL ref2=base_footprint target=[0.65 0.45 1.4] PD=[1., .8, 1., 1.]"));
  auto endR   = baxter.task(GRAPH("map=pos ref1=endeffR ref2=base_footprint target=[0.6 -0.8 1.5] PD=[1., .8, 1., 1.]"));
  auto alignT = baxter.task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[1 0 0] vec2=[1 0 0] target=[1] PD=[1., .8, 1., 1.]"));
  auto alignRT = baxter.task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 1 0] vec2=[0 0 -1] target=[1] PD=[1., .8, 1., 1.]"));
  auto wristT   = baxter.task(GRAPH("map=vecAlign ref1=wristL ref2=base_footprint vec1=[0 1 0] vec2=[0 0 1] target=[1] PD=[1., .8, 1., 1.]"));
  auto elbowL   = baxter.task(GRAPH("map=vecAlign ref1=elbowL ref2=base_footprint vec1=[0 1 0] vec2=[0 0 1] target=[1] PD=[1., .8, 1., 5.]"));

  baxter.waitConv({alignRT, alignT, endR, endL});
  baxter.grip(0, !useRos);
  arr js = baxter.getTaskControllerModule().ctrl_q_ref.get();
  baxter.stop({alignT, alignRT, wristT, elbowL, endR, endL});

  /*
   *  Home position acquired. Now find the lockbox.
   */

  cout << "Made it to the position." << endl;

  Lockbox lockbox(&baxter);
  threadOpenModules(true);
  lockbox.initializeJoints();

  cout << "Disabling position control." << endl;
  baxter.disablePosControl();
  if (useRos)
    mlr::wait(10.);

  cout << "Updating lockbox" << endl;
  lockbox.update();
  baxter.enablePosControl();

  /*
   * We should now have a better estimate of the lockbox position. Move to the estimated marker10.
   */
/*
  mlr::String marker_ref = "alvar_10";

  ors::Shape* sh = baxter.getModelWorld().getShapeByName(marker_ref);

  ors::Vector target = sh->rel*ors::Vector(0,0,.3);
  auto near10 = baxter.task(GRAPH(STRING("map=pos ref1=endeffL ref2=" << marker_ref << " target=["
                                        << target.x << ' ' << target.y << ' ' << target.z << "] PD=[1., .8, 1., 1.]")));

  auto align10X = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << marker_ref << " vec1=[1 0 0] vec2=[0 0 -1] target=[1]")));
  auto align10Y = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << marker_ref << " vec1=[0 1 0] vec2=[1 0 0] target=[1]")));
  auto align10Z = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << marker_ref << " vec1=[0 0 1] vec2=[0 -1 0] target=[1]")));

  baxter.waitConv({near10, align10X, align10Y, align10Z});
  baxter.stop({near10, align10X, align10Y, align10Z});

  baxter.disablePosControl();
  if (useRos)
    mlr::wait(5.);

  lockbox.update();
  baxter.enablePosControl();
*/


  for (uint joint = 1; joint <= 5; ++joint)
  {
    // Move the joint to the limit

    mlr::String handle = lockbox.joint_to_handle.at(joint);
    mlr::String name = lockbox.joint_to_ors_joint.at(joint);
    mlr::String str;

    cout << baxter.getModelWorld().getShapeByName(handle)->rel  << endl;

    ors::Vector target = ors::Vector(0, 0, 0.15);
    str << "map=pos ref1=endeffL ref2=" << handle << " vec2=[" << target.x << ", " << target.y << ", " << target.z << "] PD=[1., 1, 1., 1.]";

    cout << "Fixing position of joint: " << name << endl;

    auto hold = baxter.task("hold", new qItselfConstraint(baxter.getModelWorld().getJointByName(name)->qIndex, baxter.getModelWorld().getJointStateDimension()), 1, 1., 1, 1);

    cout << "Aligning. " << endl;
    cout << "Handle: " << handle << endl;
    cout << "Position task: " << str << endl;
    auto alignX = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << handle << " vec1=[1 0 0] vec2=[0 0 -1] target=[1]")));
    auto alignY = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << handle << " vec1=[0 1 0] vec2=[1 0 0] target=[1]")));
    auto alignZ = baxter.task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << handle << " vec1=[0 0 1] vec2=[0 -1 0] target=[1]")));

    cout << "Approach. " << endl;
    auto approach = baxter.task(GRAPH(str));
    baxter.waitConv({approach, alignX, alignY, alignZ});
    cout << "Moving closer. " << endl;

    char cont = ' ';
    while ((cont != 'y') && (cont != 'n'))
    {
      cout << "Continue (y/n): ";
      std::cin >> cont;
      cout << endl;
    }
    baxter.stop({approach});

    if (cont == 'n')
    {
      baxter.stop({alignX, alignY, alignZ, hold});
      auto home = baxter.task(GRAPH(" map=qItself PD=[1., 0.8, 1.0, 10.] prec=[1]"));
      js(baxter.getKinematicWorld().getJointByName(name)->qIndex) = baxter.getKinematicWorld().getJointByName(name)->limits(1);
      baxter.modifyTarget(home, js);
      baxter.waitConv({home});
      baxter.stop({home});
      continue;
    }


    str.clear();
    target = /*handle_tf * */ors::Vector(0., 0., 0.05);
    str << "map=pos ref1=endeffL ref2=" << handle << " vec2=["<< target.x << ", " << target.y << ", " << target.z << "] PD=[1., 1, 1., 1.]";
    approach = baxter.task(GRAPH(str));

    baxter.waitConv({approach, alignX, alignY, alignZ});
    cout << "Close." << endl;

    if (useRos)
      mlr::wait(3.);

    cont = ' ';
    while ((cont != 'y') && (cont != 'n'))
    {
      cout << "Continue (y/n): ";
      std::cin >> cont;
      cout << endl;
    }
    baxter.stop({approach});


    if (cont == 'n')
    {
      baxter.stop({alignX, alignY, alignZ, hold});
      auto home = baxter.task(GRAPH(" map=qItself PD=[1., 0.8, 1.0, 10.] prec=[1]"));
      js(baxter.getKinematicWorld().getJointByName(name)->qIndex) = baxter.getKinematicWorld().getJointByName(name)->limits(1);
      baxter.modifyTarget(home, js);
      baxter.waitConv({home});
      baxter.stop({home});
      continue;
    }


    cout << "Grip. " << endl;
    baxter.grip(true, useRos);

    auto gripTask = baxter.task("grip", new PointEqualityConstraint(baxter.getKinematicWorld(), "endeffL", NoVector, handle, target), .5, 1, 5, 5);
    baxter.stop({hold});

    // For the desired articulation:
    // - q_ref(lockbox_joint) = lockbox_joint->limits(1)

    auto move_joint = baxter.task("move_joint", new qItselfConstraint(baxter.getKinematicWorld().getJointByName(name)->qIndex, baxter.getKinematicWorld().getJointStateDimension()), 1, 1, 1, 1);

    cout << "Moving lockbox joint." << endl;

    double current = baxter.getKinematicWorld().q(baxter.getKinematicWorld().getJointByName(name)->qIndex);
    double desired = baxter.getKinematicWorld().getJointByName(name)->limits(1);

    const double steps = 10;
    for (double i = 1; i<=steps; i++)
    {
      double target = (desired - current) * (i/steps) + current;
      cout << "Step: " << i << " target: " << target << endl;
      baxter.modifyTarget(move_joint, ARR(target));
      baxter.waitConv({move_joint, gripTask, alignX, alignY, alignZ});
    }

    baxter.stop({move_joint, gripTask});

    baxter.grip(false, useRos);

    cout << "Done, fixing lockbox joint again." << endl;

    auto hold2 = baxter.task("hold2", new qItselfConstraint(baxter.getKinematicWorld().getJointByName(name)->qIndex, baxter.getKinematicWorld().getJointStateDimension()), 1, .8, 1, 1);
    baxter.modifyTarget(hold2, ARR(baxter.getModelWorld().q(baxter.getModelWorld().getJointByName(name)->qIndex)));

    str.clear();
    str << "map=pos ref1=endeffL ref2=" << handle << " vec2=[0 0 0.1] PD=[1., 1, 1., 1.]";
    cout << "Retracting. " << endl;
    auto retract = baxter.task(GRAPH(str));
    baxter.waitConv({retract, alignX, alignY, alignZ});
    baxter.stop({retract, alignX, alignY, alignZ});

    str.clear();
    str << "map=pos ref1=endeffL ref2=" << handle << " vec2=[0 0 0.2] PD=[1., 1, 1., 1.]";
    cout << "Retracting. " << endl;
    retract = baxter.task(GRAPH(str));
    baxter.waitConv({retract});
    baxter.stop({retract});

    baxter.disablePosControl();
    mlr::wait(2.);
    arr new_q;
    lockbox.updatedJointPose(joint, new_q);
    baxter.setRealWorld(new_q);
    baxter.enablePosControl();


    cout << "Done retract. Going home." << endl;
    auto home = baxter.task(GRAPH(" map=qItself PD=[1., 0.8, 1.0, 10.] prec=[1]"));
    js(baxter.getKinematicWorld().getJointByName(name)->qIndex) = baxter.getKinematicWorld().getJointByName(name)->limits(1);
    baxter.modifyTarget(home, js);
    baxter.waitConv({home});
    baxter.stop({home, hold2});


    mlr::wait(1.);
    cout << "After: " << baxter.getKinematicWorld().q << endl;

  }

  cout <<"bye bye" <<endl;
  return 0;
}
