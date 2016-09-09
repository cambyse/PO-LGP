#define REPORT 1
#define SIM 1

#include "lockbox.h"
#include <unordered_map>
#include <Motion/komo.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

Lockbox::Lockbox(MyBaxter* baxter) : Module("lockbox", -1),
    test_joint(this, "test_joint", true),
    get_joint_position(this, "get_joint_position", true),
    object_database(this, "object_database", true),
    data_collector(!mlr::getParameter<bool>("useRos", false))
{
  usingRos = mlr::getParameter<bool>("useRos", false);

  if (usingRos)
  {
    nh = new ros::NodeHandle;
    joint_position_publisher = nh->advertise<std_msgs::Float64>("/lockbox/joint_position_result", 1, true);
    test_joint_publisher = nh->advertise<std_msgs::Bool>("/lockbox/test_joint_result", 1, true);
  }

  myBaxter = baxter;
  test_joint_revision = test_joint.readAccess();
  test_joint.deAccess();
  joint_position_revision = get_joint_position.readAccess();
  get_joint_position.deAccess();
//  threadOpenModules(true);
}

double Lockbox::getJointPosition(const uint joint)
{
//  ors::Joint* jt = myBaxter->getModelWorld().getJointByName(joint_to_ors_joint.at(joint));
//  return 100.0 * myBaxter->getModelWorld().q(jt->qIndex) / jt->limits(1);
    return 100.0 * joint_positions.at(joint);
}

bool Lockbox::testJoint(const uint joint)
{
  char res = 'a';
  while ((res != 'y') && (res != 'n'))
  {
    cout << "Is joint: " << joint << " movable (y/n): ";
    std::cin >> res;
    cout << endl;
  }
  if (res == 'n')
  {
    cout << "Returning false." << endl;
    return false;
  }
  else
  {
    cout << "Moving the joint." << endl;
    return moveJoint(joint);
  }

}

Lockbox::~Lockbox(){
  if (nh)
    delete nh;
  //  threadCloseModules();
}


void Lockbox::step()
{
  if (readyToTest)
  {
    int rev = test_joint.var->revisionNumber();
    if (rev > test_joint_revision)
    {
      uint joint = test_joint.get()().data;
      std_msgs::Bool result;
      result.data = this->testJoint(joint);
      test_joint_publisher.publish(result);
      test_joint_revision = rev;
    }
    rev = get_joint_position.var->revisionNumber();
    if (rev > joint_position_revision)
    {
       uint joint = get_joint_position.get()().data;
       std_msgs::Float64 result; result.data = getJointPosition(joint);
       joint_position_publisher.publish(result);
       joint_position_revision = rev;
    }
  mlr::wait(0.1);
  }
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
    joint_positions.insert(std::make_pair(i, 0.0));
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

void Lockbox::grip(const bool toGrip)
{
//  arr q = myBaxter->getModelWorld().q;
  ors::Joint *j = myBaxter->getModelWorld().getJointByName("l_gripper_l_finger_joint");

  toGrip ? q0(j->qIndex) = j->limits(0) : q0(j->qIndex) = j->limits(1);

  std::cout << "Gripping: " << toGrip << std::endl;

  for (CtrlTask* task : joint_fixed_tasks)
  {
    if (task->name == "grip")
    {
      myBaxter->stop({task});
      joint_fixed_tasks.removeValue(task);
    }
  }

  CtrlTask* gripTask = myBaxter->task("grip", new qItselfConstraint(j->qIndex, myBaxter->getModelWorld().getJointStateDimension()), 1, .8, 10, 10);
  myBaxter->modifyTarget(gripTask, ARR(q0(j->qIndex)));
  gripTask->prec = ARR(10000);

//  auto gripTask = myBaxter->task(GRAPH("map=qItself PD=[1., 1, 3., 2.] prec=[100.]"));
//  gripTask->name = "grip";
//  myBaxter->modifyTarget(gripTask, q);

  joint_fixed_tasks.append(gripTask);

  cout << "Here." << endl;
  // Special wait-conv, since it won't converge if it is gripping.
  cout << (myBaxter->testConv({gripTask}, 5) ? "Grip converged. " : "Grip timed out." )<< endl;
//  uint count = 0;
//  arr pos;
//  do
//  {
//    count++;
//    if (count > 500)
//      break;
//    mlr::wait(0.01);

//    pos = myBaxter->getModelWorld().q;

//  } while( std::abs(pos(j->qIndex) - q0(j->qIndex)) > 0.001);
}


bool Lockbox::moveJoint(const uint joint)
{
  // First, fix the joint in the model world.
  fixJoint(joint, true);

  grip(false);
//  myBaxter->grip(false, usingRos);


  // Move and align relative to the alvar in the model world.

  mlr::String handle = joint_to_handle.at(joint);
  mlr::String marker_name = STRING(joint_name.at(joint) << "_marker");


  // Position task
  mlr::String str;
  ors::Vector target = ors::Vector(0, 0, 0.15);
  str << "map=pos ref1=endeffL ref2=" << handle << " vec2=[" << target.x << ", " << target.y << ", " << target.z << "] PD=[1., 1.2, .2, 10.]";

  cout << "Moving above handle." << endl;
  auto approach = myBaxter->task("approach", GRAPH(str));

  myBaxter->waitConv({approach});
//  myBaxter->testRealConv({approach}, 10);

  cout << "At handle, aligning" << endl;
  // Alignment tasks.
  auto alignX = myBaxter->task("alignX", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << handle << " vec1=[1 0 0] vec2=[0 0 -1] target=[1] prec=[1000] PD=[1, 1.2, 4, .2]")));
  auto alignY = myBaxter->task("alignY", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << handle << " vec1=[0 1 0] vec2=[1 0 0] target=[1] prec=[1000] PD=[1, 1.2, 4, .2]")));
  auto alignZ = myBaxter->task("alignZ", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << handle << " vec1=[0 0 1] vec2=[0 -1 0] target=[1] prec=[1000] PD=[1, 1.2, 4, .2]")));

  if (usingRos)
    myBaxter->testRealConv({approach, alignX, alignY, alignZ}, 10);
  else
    myBaxter->waitConv({approach, alignX, alignY, alignZ});

  mlr::wait(2.);
  cout << "Aligned with handle." << endl;
  // Now we are 15 cm away and aligned with the handle.
  myBaxter->stop({approach});

  // Move closer.
  str.clear();
  target = ors::Vector(0., 0., 0.05);
  str << "map=pos ref1=endeffL ref2=" << handle << " vec2=["<< target.x << ", " << target.y << ", " << target.z << "] PD=[1., 1, 2, .5]";
  approach = myBaxter->task("approach", GRAPH(str));

  if (usingRos)
    myBaxter->testRealConv({approach, alignX, alignY, alignZ}, 10);
  else
    myBaxter->waitConv({approach, alignX, alignY, alignZ});


  // Grip
//  myBaxter->grip(true, usingRos);
  grip(true);

  fixJoint(joint, false);

  // For the desired articulation:
  // - q_ref(lockbox_joint) = lockbox_joint->limits(1)

  // Create a task for moving the joint in the modelWorld.
  CtrlTask* move_joint = myBaxter->task("move_joint", new qItselfConstraint(myBaxter->getModelWorld().getJointByName(joint_to_ors_joint.at(joint))->qIndex, myBaxter->getModelWorld().getJointStateDimension()), 1, 1, 1, 1);



  double current = myBaxter->getModelWorld().q(myBaxter->getModelWorld().getJointByName(joint_to_ors_joint.at(joint))->qIndex);
  double desired = ( joint_positions.at(joint) == 0 ) ? myBaxter->getModelWorld().getJointByName(joint_to_ors_joint.at(joint))->limits(1) : myBaxter->getModelWorld().getJointByName(joint_to_ors_joint.at(joint))->limits(0);

  joint_positions.at(joint) = 1 - joint_positions.at(joint);

  // Simulate locked joints, if not using ROS
  if (!usingRos)
  {
    // see if we can actually unlock the joint
    if ( joint != locked_joints.min() )
    {
      fixJoint(joint, true);
    }
  }

  const double steps = 20;
  uint num_failed = 0;
  for (double i = 1; i<=steps; i++)
  {
    double target = (desired - current) * (i/steps) + current;

    myBaxter->modifyTarget(move_joint, ARR(target));

    // Wait for the model world to converge
//    myBaxter->testConv({move_joint}, 5);

    // Update the real world position of the joint
    arr sim_q = myBaxter->getKinematicWorld().q;
    sim_q(myBaxter->getModelWorld().getJointByName(joint_to_ors_joint.at(joint))->qIndex) = target;
    myBaxter->setRealWorld(sim_q);

    // Test if the real world converged
    bool success;
#if 0
    if (usingRos)
      success = myBaxter->testRealConv({approach, alignX, alignY, alignZ}, 5);
    else
      success = myBaxter->testConv({move_joint}, 3);
#endif
    success = myBaxter->testConv({move_joint}, 5);

    // If not movement success, decide if it is failure, or if joint is locked.
    if (!success)
    {
      num_failed++;
      cout << "Step: " << i << " target: " << target << " failed. Num failed: " << num_failed << endl;
    }
    else
      num_failed=0;

    if (num_failed > 3)
    {
      // Assuming joint is locked if steps is >= 5
      cout << "Joint is locked." << endl;
      myBaxter->modifyTarget(move_joint, ARR(current));
      myBaxter->waitConv({move_joint});
      mlr::wait(1.);
      myBaxter->stop({move_joint, approach});
      grip(false);
      fixJoint(joint, true);
      str.clear();
      ors::Vector point = ors::Vector(0., 0., 0.3);
      str << "map=pos ref1=endeffL ref2=" << handle << " vec2=["<< point.x << ", " << point.y << ", " << point.z << "] PD=[1., 1, 1., 1.]";
      auto retract = myBaxter->task("retract", GRAPH(str));
      myBaxter->waitConv({retract, alignX, alignY, alignZ});
      myBaxter->stop({retract, alignX, alignY, alignZ});
      moveHome(true);
      sim_q = myBaxter->getKinematicWorld().q;
      sim_q(myBaxter->getModelWorld().getJointByName(joint_to_ors_joint.at(joint))->qIndex) = current;
      myBaxter->setRealWorld(sim_q);
      return false;
    }
  }

  myBaxter->stop({move_joint, approach});

//  myBaxter->grip(false, usingRos);
  grip(false);

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

//  cout << "Retraced from handle. Moving to above marker." << endl;

//  str.clear();
//  str << "map=pos ref1=endeffL ref2=" << marker_name << " vec2=[0 0 0.2] PD=[1., 1, 1., 1.]";

////    ors::Vector marker_pos = baxter.getModelWorld().getShapeByName(marker_name)->X.pos;
////    str << "map=pos ref1=endeffL ref2=base_footprint  vec2=[" << marker_pos.x << ' ' << marker_pos.y << ' ' << marker_pos.z << "] PD=[1., 1, 1., 1.]";

//  // Alignment tasks.
////    auto alignXmarker = baxter.task("alignX", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << marker_name << " vec1=[1 0 0] vec2=[0 0 -1] target=[1] prec=[10]")));
////    auto alignYmarker = baxter.task("alignY", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << marker_name << " vec1=[0 1 0] vec2=[1 0 0] target=[1] prec=[10]")));
////    auto alignZmarker = baxter.task("alignZ", GRAPH(STRING("map=vecAlign ref1=endeffL ref2=" << marker_name << " vec1=[0 0 1] vec2=[0 -1 0] target=[1] prec=[10]")));
//  CtrlTask* marker = myBaxter->task("marker", GRAPH(str));
//  myBaxter->waitConv({marker, alignX, alignY, alignZ});//});//, alignXmarker, alignYmarker, alignZmarker});
//  myBaxter->stop({marker, alignX, alignY, alignZ});//});//, alignXmarker, alignYmarker, alignZmarker});

//  cout << "Retracted, updating modelworld. " << endl;

//  // Update the position
#if 0
  myBaxter->disablePosControl();
  mlr::wait(5.);
  arr new_q;
  updatedJointPose(joint, new_q);
  myBaxter->setRealWorld(new_q);
  myBaxter->enablePosControl();
#endif
  q0(myBaxter->getKinematicWorld().getJointByName(joint_to_ors_joint.at(joint))->qIndex) = desired;


  // Retracting without alignment.
  cout << "Retracting. " << endl;

  str.clear();
//  str << "map=pos ref1=endeffL ref2=" << marker_name << " vec2=[0 0 0.2] PD=[1., 1, 1., 1.]";

  str << "map=pos ref1=endeffL ref2=base_footprint PD=[1, 1, 1, 1]";
  retract = myBaxter->task("clear", GRAPH(str));
  retract->map.phi(retract->y, NoArr, myBaxter->getModelWorld());
  retract->y_ref = retract->y + ARR(-0.2, 0, 0);

  myBaxter->waitConv({retract});
  myBaxter->stop({retract});

  // Done, move home.
  moveHome(true);

//  locked_joints.removeValue(joint);
  return true;
}

void Lockbox::moveHome(const bool stopAllOtherTasks)
{
  if (stopAllOtherTasks)
  {
    cout << "Stopping all other tasks. " << endl;
    for (auto task : joint_fixed_tasks)
    {
      cout << "Stopping: " << task->name << endl;
      myBaxter->stop({task});
    }
    joint_fixed_tasks.clear();
    myBaxter->stopAll();
    mlr::wait(2.);
//    CtrlTask* limits = myBaxter->task("limits", new LimitsConstraint());
//    limits->setGainsAsNatural(1, 1);
  }

  if (q0.N == 0)
  {
//    myBaxter->grip(0, !mlr::getParameter<bool>("useRos", false));
    auto endL   = myBaxter->task(GRAPH("map=pos ref1=endeffL ref2=base_footprint target=[0.675 0.45 1.4] PD=[.5, 1., 2., 10.]"));
    auto endR   = myBaxter->task(GRAPH("map=pos ref1=endeffR ref2=base_footprint target=[0.6 -0.8 1.5] PD=[.5, 1., 2., 10.]"));
    auto alignX = myBaxter->task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[1 0 0] vec2=[1 0 0] target=[1] PD=[1., 1., 1., 1.]"));
    auto alignY = myBaxter->task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 1 0] vec2=[0 0 -1] target=[1] PD=[1., 1., 1., 1.]"));
    auto alignZ = myBaxter->task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 0 1] vec2=[0 1 0] target=[1] PD=[1., 1., 1., 1.]"));
    auto wrist   = myBaxter->task(GRAPH("map=vecAlign ref1=wristL ref2=base_footprint vec1=[0 1 0] vec2=[0 0 1] target=[1] PD=[1., 1., 1., 1.]"));
    auto elbow   = myBaxter->task(GRAPH("map=vecAlign ref1=elbowL ref2=base_footprint vec1=[0 1 0] vec2=[0 0 1] target=[1] PD=[1., 1., 1., 1.]"));
    myBaxter->waitConv({alignX, alignY, alignZ, endR, endL});
    if (usingRos)
      myBaxter->testRealConv({alignX, alignY, alignZ, endR, endL}, 10);

    myBaxter->stop({alignX, alignY, alignZ, wrist, elbow, endR, endL});
    q0 = myBaxter->getModelWorld().q;
    grip(false);
  }
  else
  {
    auto home = myBaxter->task(GRAPH("name=home map=qItself PD=[.5, 1., 2., 10.]"));
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
