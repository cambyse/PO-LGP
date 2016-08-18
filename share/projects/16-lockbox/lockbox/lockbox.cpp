#define REPORT 1
#define SIM 1

#include "lockbox.h"
#include <unordered_map>
#include <Motion/komo.h>


Lockbox::Lockbox(MyBaxter* baxter) : Module("lockbox", -1),
    object_database(this, "object_database", true),
    data_collector(!mlr::getParameter<bool>("useRos", false))
{
  myBaxter = baxter;
  usingRos = mlr::getParameter<bool>("useRos", false);
  threadOpenModules(true);
}

Lockbox::~Lockbox(){
  threadCloseModules();
}


void Lockbox::initializeJoints()
{
  lockbox_world.init(mlr::mlrPath("data/baxter_model/baxter-lockbox.ors").p);

  joint_to_ors_joint.clear();
  joint_to_handle.clear();
  joint_name.clear();

  joint_name.insert(std::make_pair(1, "door"));
  joint_name.insert(std::make_pair(2, "bar"));
  joint_name.insert(std::make_pair(3, "wheel"));
  joint_name.insert(std::make_pair(4, "bar2"));
  joint_name.insert(std::make_pair(5, "door2"));

  for (uint i = 1; i <= 5; ++i)
  {
    mlr::String name = joint_name.at(i);
    joint_to_handle.insert(std::make_pair(i, STRING(name << "_handle")));
    joint_to_ors_joint.insert(std::make_pair(i, STRING("lockbox_" << name)));
    locked_joints.append(i);
  }
}

void Lockbox::fixJoint(const uint joint, const bool toFix)
{
  mlr::String name = joint_to_ors_joint.at(joint);

  uint qIndex = myBaxter->getModelWorld().getJointByName(name)->qIndex;

  cout << (toFix ? "Fixing " : "Releasing ") << joint << "  " << name << endl;

  if (toFix)
  {
    // First, see if it is already fixed.
    for (CtrlTask* task : joint_fixed_tasks)
    {
      if (task->name == name)
        return;
    }

    CtrlTask* hold = myBaxter->task(name, new qItselfConstraint(qIndex, myBaxter->getModelWorld().getJointStateDimension()), 1, .8, 10, 10);
    myBaxter->modifyTarget(hold, ARR(myBaxter->getModelWorld().q(myBaxter->getModelWorld().getJointByName(name)->qIndex)));
    joint_fixed_tasks.append(hold);
    return;
  }


  if (!toFix)
  {
    for (CtrlTask* task : joint_fixed_tasks)
    {
      if (task->name == name)
      {
        myBaxter->stop({task});
        joint_fixed_tasks.removeValue(task);
        return;
      }
    }
  }
}

void Lockbox::moveJoint(const uint joint, const double position)
{
  // First, fix the joint in the model world.
  fixJoint(joint, true);


  // Move and align relative to the alvar in the model world.

  mlr::String handle = joint_to_handle.at(joint);
  mlr::String marker_name = STRING(joint_name.at(joint) << "_marker");


  // Alignment tasks.
  auto alignX = myBaxter->task("alignX", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << handle << " vec1=[1 0 0] vec2=[0 0 -1] target=[1] prec=[1000]")));
  auto alignY = myBaxter->task("alignY", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << handle << " vec1=[0 1 0] vec2=[1 0 0] target=[1] prec=[1000]")));
  auto alignZ = myBaxter->task("alignZ", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << handle << " vec1=[0 0 1] vec2=[0 -1 0] target=[1] prec=[1000]")));

  // Position task
  mlr::String str;
  ors::Vector target = ors::Vector(0, 0, 0.15);
  str << "map=pos ref1=endeffL ref2=" << handle << " vec2=[" << target.x << ", " << target.y << ", " << target.z << "] PD=[1., 1, 1., 1.]";

  auto approach = myBaxter->task(GRAPH(str));
  myBaxter->waitConv({approach, alignX, alignY, alignZ});

  // Now we are 15 cm away and aligned with the handle.
  myBaxter->stop({approach});


  mlr::wait(2.0);

  // Move closer.
  str.clear();
  target = ors::Vector(0., 0., 0.05);
  str << "map=pos ref1=endeffL ref2=" << handle << " vec2=["<< target.x << ", " << target.y << ", " << target.z << "] PD=[1., 1, 1., 1.]";
  approach = myBaxter->task("approach", GRAPH(str));

  myBaxter->waitConv({approach, alignX, alignY, alignZ});


  // Grip
  myBaxter->grip(true, usingRos);

  fixJoint(joint, false);

  // For the desired articulation:
  // - q_ref(lockbox_joint) = lockbox_joint->limits(1)

  // Create a task for moving the joint in the modelWorld.
  auto move_joint = myBaxter->task("move_joint", new qItselfConstraint(myBaxter->getModelWorld().getJointByName(joint_to_ors_joint.at(joint))->qIndex, myBaxter->getModelWorld().getJointStateDimension()), 1, 1, 1, 1);


  double current = myBaxter->getModelWorld().q(myBaxter->getModelWorld().getJointByName(joint_to_ors_joint.at(joint))->qIndex);
  double desired = position;

  const double steps = 10;
  for (double i = 1; i<=steps; i++)
  {
    double target = (desired - current) * (i/steps) + current;
    cout << "Step: " << i << " target: " << target << endl;
    myBaxter->modifyTarget(move_joint, ARR(target));
    myBaxter->waitConv({move_joint, approach, alignX, alignY, alignZ});
  }

  myBaxter->stop({move_joint, approach});

  myBaxter->grip(false, usingRos);


//    auto hold2 = baxter.task("hold2", new qItselfConstraint(baxter.getKinematicWorld().getJointByName(name)->qIndex, baxter.getKinematicWorld().getJointStateDimension()), 1, .8, 1, 1);
//    baxter.modifyTarget(hold2, ARR(baxter.getModelWorld().q(baxter.getModelWorld().getJointByName(name)->qIndex)));

  // Fixing the joint in the model world.
  fixJoint(joint, true);

  // Retracting with alignment.
  cout << "Retracting with alignment. " << endl;

  str.clear();
  str << "map=pos ref1=endeffL ref2=" << handle << " vec2=[0 0 0.15] PD=[1., 1, 1., 1.]";
  auto retract = myBaxter->task("retract", GRAPH(str));
  myBaxter->waitConv({retract, alignX, alignY, alignZ});
  myBaxter->stop({retract});//, alignX, alignY, alignZ});

  cout << "Retraced from handle. Moving to above marker." << endl;

  str.clear();
  str << "map=pos ref1=endeffL ref2=" << marker_name << " vec2=[0 0 0.2] PD=[1., 1, 1., 1.]";
//    ors::Vector marker_pos = baxter.getModelWorld().getShapeByName(marker_name)->X.pos;
//    str << "map=pos ref1=endeffL ref2=base_footprint  vec2=[" << marker_pos.x << ' ' << marker_pos.y << ' ' << marker_pos.z << "] PD=[1., 1, 1., 1.]";

  // Alignment tasks.
//    auto alignXmarker = baxter.task("alignX", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << marker_name << " vec1=[1 0 0] vec2=[0 0 -1] target=[1] prec=[10]")));
//    auto alignYmarker = baxter.task("alignY", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << marker_name << " vec1=[0 1 0] vec2=[1 0 0] target=[1] prec=[10]")));
//    auto alignZmarker = baxter.task("alignZ", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << marker_name << " vec1=[0 0 1] vec2=[0 -1 0] target=[1] prec=[10]")));
  auto marker = myBaxter->task("marker", GRAPH(str));
  myBaxter->waitConv({marker, alignX, alignY, alignZ});//});//, alignXmarker, alignYmarker, alignZmarker});
  myBaxter->stop({marker, alignX, alignY, alignZ});//});//, alignXmarker, alignYmarker, alignZmarker});

  cout << "Retracted, updating modelworld. " << endl;

  // Update the position
  myBaxter->disablePosControl();
  mlr::wait(5.);
  arr new_q;
  updatedJointPose(joint, new_q);
  myBaxter->setRealWorld(new_q);
  myBaxter->enablePosControl();

  // Retracting without alignment.
  str.clear();
  str << "map=pos ref1=endeffL ref2=" << marker_name << " vec2=[0 0 0.2] PD=[1., 1, 1., 1.]";
  cout << "Retracting. " << endl;
  retract = myBaxter->task(GRAPH(str));
  myBaxter->waitConv({retract});
  myBaxter->stop({retract});

  // Done, move home.
  moveHome(true);
}

bool Lockbox::testJoint(const uint joint)
{
  // First, fix the joint in the model world.
  fixJoint(joint, true);


  // Move and align relative to the alvar in the model world.

  mlr::String handle = joint_to_handle.at(joint);
  mlr::String marker_name = STRING(joint_name.at(joint) << "_marker");


  // Alignment tasks.
  auto alignX = myBaxter->task("alignX", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << handle << " vec1=[1 0 0] vec2=[0 0 -1] target=[1] prec=[1000]")));
  auto alignY = myBaxter->task("alignY", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << handle << " vec1=[0 1 0] vec2=[1 0 0] target=[1] prec=[1000]")));
  auto alignZ = myBaxter->task("alignZ", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << handle << " vec1=[0 0 1] vec2=[0 -1 0] target=[1] prec=[1000]")));

  // Position task
  mlr::String str;
  ors::Vector target = ors::Vector(0, 0, 0.15);
  str << "map=pos ref1=endeffL ref2=" << handle << " vec2=[" << target.x << ", " << target.y << ", " << target.z << "] PD=[1., 1, 1., 1.]";

  auto approach = myBaxter->task(GRAPH(str));
  myBaxter->waitConv({approach, alignX, alignY, alignZ});

  // Now we are 15 cm away and aligned with the handle.
  myBaxter->stop({approach});


  mlr::wait(2.0);

  // Move closer.
  str.clear();
  target = ors::Vector(0., 0., 0.05);
  str << "map=pos ref1=endeffL ref2=" << handle << " vec2=["<< target.x << ", " << target.y << ", " << target.z << "] PD=[1., 1, 1., 1.]";
  approach = myBaxter->task("approach", GRAPH(str));

  myBaxter->waitConv({approach, alignX, alignY, alignZ});


  // Grip
  myBaxter->grip(true, usingRos);

  fixJoint(joint, false);

  // For the desired articulation:
  // - q_ref(lockbox_joint) = lockbox_joint->limits(1)

  // Create a task for moving the joint in the modelWorld.
  auto move_joint = myBaxter->task("move_joint", new qItselfConstraint(myBaxter->getModelWorld().getJointByName(joint_to_ors_joint.at(joint))->qIndex, myBaxter->getModelWorld().getJointStateDimension()), 1, 1, 1, 1);


  double current = myBaxter->getModelWorld().q(myBaxter->getModelWorld().getJointByName(joint_to_ors_joint.at(joint))->qIndex);
  double desired = myBaxter->getModelWorld().getJointByName(joint_to_ors_joint.at(joint))->limits(1);

  const double steps = 10;
  for (double i = 1; i<=steps; i++)
  {
    double target = (desired - current) * (i/steps) + current;
    cout << "Step: " << i << " target: " << target << endl;

    if (!usingRos)
    {
      // see if we can actually unlock the joint
      if ( joint != locked_joints.min() )
      {
        fixJoint(joint, true);
      }
    }

    myBaxter->modifyTarget(move_joint, ARR(target));
    bool success = myBaxter->testConv({move_joint, approach, alignX, alignY, alignZ});

    cout << "Success: " << success << endl;
    // If not movement success, decide if it is failure, or if joint is locked.
    if (!success && i>= 3)
    {
      // Assuming joint is locked if steps is >= 5
      cout << "Joint is locked." << endl;
      myBaxter->stop({move_joint, approach});
      myBaxter->grip(false, usingRos);
      fixJoint(joint, true);
      str.clear();
      ors::Vector point = ors::Vector(0., 0., 0.1);
      str << "map=pos ref1=endeffL ref2=" << handle << " vec2=["<< point.x << ", " << point.y << ", " << point.z << "] PD=[1., 1, 1., 1.]";
      auto retract = myBaxter->task("retract", GRAPH(str));
      myBaxter->waitConv({retract, alignX, alignY, alignZ});
      myBaxter->stop({retract, alignX, alignY, alignZ});
      moveHome(true);
      return false;
    }
  }

  myBaxter->stop({move_joint, approach});

  myBaxter->grip(false, usingRos);


//    auto hold2 = baxter.task("hold2", new qItselfConstraint(baxter.getKinematicWorld().getJointByName(name)->qIndex, baxter.getKinematicWorld().getJointStateDimension()), 1, .8, 1, 1);
//    baxter.modifyTarget(hold2, ARR(baxter.getModelWorld().q(baxter.getModelWorld().getJointByName(name)->qIndex)));

  // Fixing the joint in the model world.
  fixJoint(joint, true);

  // Retracting with alignment.
  cout << "Retracting with alignment. " << endl;

  str.clear();
  str << "map=pos ref1=endeffL ref2=" << handle << " vec2=[0 0 0.15] PD=[1., 1, 1., 1.]";
  auto retract = myBaxter->task("retract", GRAPH(str));
  myBaxter->waitConv({retract, alignX, alignY, alignZ});
  myBaxter->stop({retract});//, alignX, alignY, alignZ});

  cout << "Retraced from handle. Moving to above marker." << endl;

  str.clear();
  str << "map=pos ref1=endeffL ref2=" << marker_name << " vec2=[0 0 0.2] PD=[1., 1, 1., 1.]";
//    ors::Vector marker_pos = baxter.getModelWorld().getShapeByName(marker_name)->X.pos;
//    str << "map=pos ref1=endeffL ref2=base_footprint  vec2=[" << marker_pos.x << ' ' << marker_pos.y << ' ' << marker_pos.z << "] PD=[1., 1, 1., 1.]";

  // Alignment tasks.
//    auto alignXmarker = baxter.task("alignX", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << marker_name << " vec1=[1 0 0] vec2=[0 0 -1] target=[1] prec=[10]")));
//    auto alignYmarker = baxter.task("alignY", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << marker_name << " vec1=[0 1 0] vec2=[1 0 0] target=[1] prec=[10]")));
//    auto alignZmarker = baxter.task("alignZ", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << marker_name << " vec1=[0 0 1] vec2=[0 -1 0] target=[1] prec=[10]")));
  auto marker = myBaxter->task("marker", GRAPH(str));
  myBaxter->waitConv({marker, alignX, alignY, alignZ});//});//, alignXmarker, alignYmarker, alignZmarker});
  myBaxter->stop({marker, alignX, alignY, alignZ});//});//, alignXmarker, alignYmarker, alignZmarker});

  cout << "Retracted, updating modelworld. " << endl;

  // Update the position
  myBaxter->disablePosControl();
  mlr::wait(5.);
  arr new_q;
  updatedJointPose(joint, new_q);
  myBaxter->setRealWorld(new_q);
  myBaxter->enablePosControl();

  // Retracting without alignment.
  str.clear();
  str << "map=pos ref1=endeffL ref2=" << marker_name << " vec2=[0 0 0.2] PD=[1., 1, 1., 1.]";
  cout << "Retracting. " << endl;
  retract = myBaxter->task(GRAPH(str));
  myBaxter->waitConv({retract});
  myBaxter->stop({retract});

  // Done, move home.
  moveHome(true);

  locked_joints.removeValue(joint);
  return true;
}

void Lockbox::moveHome(const bool stopAllOtherTasks)
{
  if (stopAllOtherTasks)
  {
    cout << "Stopping all other tasks. " << endl;
    joint_fixed_tasks.clear();
    myBaxter->stopAll();
//    CtrlTask* limits = myBaxter->task("limits", new LimitsConstraint());
//    limits->setGainsAsNatural(1, 1);
  }

  if (q0.N == 0)
  {
    myBaxter->grip(0, !mlr::getParameter<bool>("useRos", false));
    auto endL   = myBaxter->task(GRAPH("map=pos ref1=endeffL ref2=base_footprint target=[0.675 0.45 1.4] PD=[1., .8, 1., 1.]"));
    auto endR   = myBaxter->task(GRAPH("map=pos ref1=endeffR ref2=base_footprint target=[0.6 -0.8 1.5] PD=[1., .8, 1., 1.]"));
    auto alignX = myBaxter->task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[1 0 0] vec2=[1 0 0] target=[1] PD=[1., 1., 1., 1.]"));
    auto alignY = myBaxter->task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 1 0] vec2=[0 0 -1] target=[1] PD=[1., 1., 1., 1.]"));
    auto alignZ = myBaxter->task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 0 1] vec2=[0 1 0] target=[1] PD=[1., 1., 1., 1.]"));
    auto wrist   = myBaxter->task(GRAPH("map=vecAlign ref1=wristL ref2=base_footprint vec1=[0 1 0] vec2=[0 0 1] target=[1] PD=[1., 1., 1., 1.]"));
    auto elbow   = myBaxter->task(GRAPH("map=vecAlign ref1=elbowL ref2=base_footprint vec1=[0 1 0] vec2=[0 0 1] target=[1] PD=[1., 1., 1., 1.]"));

    myBaxter->waitConv({alignX, alignY, alignZ, endR, endL});
    myBaxter->stop({alignX, alignY, alignZ, wrist, elbow, endR, endL});
    q0 = myBaxter->getModelWorld().q;
  }
  else
  {
    auto home = myBaxter->task(GRAPH(" map=qItself PD=[.5, 1., .2, 10.]"));
    myBaxter->modifyTarget(home, q0);
    myBaxter->waitConv({home});
    myBaxter->stop({home});
  }


  // Fix all joints.
  for (uint i = 1; i <= 5; i++)
  {
    fixJoint(i, true);
  }

}

bool Lockbox::queryContinue()
{
  char cont = ' ';
  while ((cont != 'y') && (cont != 'n'))
  {
    cout << "Continue (y/n): ";
    std::cin >> cont;
    cout << endl;
  }
  return (cont == 'y') ? true : false;
}

void Lockbox::update()
{
  cout << "Updating Lockbox position." << endl;
  KOMO komo;
  komo.setModel(lockbox_world);
  komo.setTiming(1, 1, 5., 1, false);
  komo.setSquaredQVelocities();
  komo.setCollisions(true);
  komo.setLimits(true);

  for (auto i : {6, 7, 9, 10})
  {
    mlr::String alvar_name = STRING("alvar_" << i);
    ors::Shape* alvar = myBaxter->getModelWorld().getShapeByName(alvar_name, false);

    if (!alvar)
     continue;

    cout << "Alvar: " << i << " seen." << endl;

    mlr::String lockbox_marker_name = STRING("lockbox_marker" << i);
    komo.setPosition(1., 1., lockbox_marker_name, NULL, sumOfSqrTT, alvar->X.pos.getArr());
  }

  komo.reset();
  komo.run();

  // New Q vector is in komo.x
  lockbox_world.setJointState(komo.x);

  ors::Transformation lockboxPos = lockbox_world.getBodyByName("lockbox")->X;

  myBaxter->updateLockbox(lockboxPos);
}

bool Lockbox::updatedJointPose(const uint joint_num, arr& new_q)
{

  ors::KinematicWorld my_copy(myBaxter->getKinematicWorld());

  KOMO komo;
  komo.setModel(my_copy);
  komo.setTiming(1, 1, 10., 1, false);
  komo.setCollisions(true);
  komo.setLimits(true);

  mlr::String lockbox_marker_name = STRING(joint_name.at(joint_num) << "_marker");

  mlr::String alvar_name;

  if (usingRos)
    alvar_name = STRING("alvar_" << joint_num);
  else
    alvar_name = lockbox_marker_name;


  ors::Shape* alvar = myBaxter->getModelWorld().getShapeByName(alvar_name);
  ors::Joint* jt = my_copy.getJointByName(STRING("lockbox_" << joint_name.at(joint_num)));

  //ors::Joint* model_jt = myBaxter->getModelWorld().getJointByName(STRING("lockbox_" << joint_name.at(joint_num)));

  if (!alvar)
    return false;

//  cout << "Name: " << alvar_name << endl;
//  cout << "Desired tf: " << alvar->X << endl;
//  cout << "Desired Q: " << myBaxter->getModelWorld().q(model_jt->qIndex) << endl;
//  cout << "Current tf: " << my_copy.getShapeByName(lockbox_marker_name)->X << endl;
//  cout << "Current Q: " << my_copy.q(jt->qIndex) << endl;

//  cout << "Full Q before: " << my_copy.q << endl;
//  cout << "Model world Q: " << myBaxter->getModelWorld().q << endl;

  komo.setPosition(1., 1., lockbox_marker_name, NULL, sumOfSqrTT, alvar->X.pos.getArr(), 10000);
  komo.setAlign(1., 1., lockbox_marker_name, ARR(1, 0, 0), NULL, alvar->X.rot.getX().getArr(), sumOfSqrTT);
  komo.setAlign(1., 1., lockbox_marker_name, ARR(0, 1, 0), NULL, alvar->X.rot.getY().getArr(), sumOfSqrTT);
  komo.setAlign(1., 1., lockbox_marker_name, ARR(0, 0, 1), NULL, alvar->X.rot.getZ().getArr(), sumOfSqrTT);

  komo.reset();
  komo.run();

//  cout << "Full Q after: " << komo.x << endl;

  // New Q vector is in komo.x
  new_q = myBaxter->getKinematicWorld().q;


  new_q(jt->qIndex) = komo.x(jt->qIndex);
  q0(jt->qIndex) = komo.x(jt->qIndex);

//  cout << "Q with modified joint: " << new_q << endl;

//  cout << "Joint. Original: " << myBaxter->getKinematicWorld().q(jt->qIndex)
//       << " Modelworld: " << myBaxter->getModelWorld().q(jt->qIndex)
//       << " min: " << jt->limits(0) << "\tCurrent: " << komo.x(jt->qIndex) << "\t Max: " << jt->limits(1) << endl;


  return true;
}
