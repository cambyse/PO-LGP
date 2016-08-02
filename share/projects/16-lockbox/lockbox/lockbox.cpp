#define REPORT 1
#define SIM 1

#include "lockbox.h"
#include <unordered_map>
#include <Motion/komo.h>


Lockbox::Lockbox(MyBaxter* baxter) : Module("lockbox", -1),
    object_database(this, "object_database", true),
    data_collector(!mlr::getParameter<bool>("useRos", false))
{
  threadOpenModules(true);
  myBaxter = baxter;
  usingRos = mlr::getParameter<bool>("useRos", false);
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
  }
}

void Lockbox::fixJoint(const uint joint, const bool fix)
{
//  ors::Joint *j = myBaxter->getModelWorld().getJointByName(joint_to_ors_joint.at(joint));
//  j->X
//  this->myBaxter->task(STRING(joint_to_ors_joint.at(joint) << fix),            
}

void Lockbox::moveJoint(const uint joint, const double radians)
{
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
  komo.setTiming(1, 1, 5., 1, false);
  komo.setSquaredQVelocities();
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

  ors::Joint* model_jt = myBaxter->getModelWorld().getJointByName(STRING("lockbox_" << joint_name.at(joint_num)));

  if (!alvar)
    return false;

//  cout << "Name: " << alvar_name << endl;
//  cout << "Desired tf: " << alvar->X << endl;
//  cout << "Desired Q: " << myBaxter->getModelWorld().q(model_jt->qIndex) << endl;
//  cout << "Current tf: " << my_copy.getShapeByName(lockbox_marker_name)->X << endl;
//  cout << "Current Q: " << my_copy.q(jt->qIndex) << endl;

//  cout << "Full Q before: " << my_copy.q << endl;
//  cout << "Model world Q: " << myBaxter->getModelWorld().q << endl;

  komo.setPosition(1., 1., lockbox_marker_name, NULL, sumOfSqrTT, alvar->X.pos.getArr());
  komo.setAlign(1., 1., lockbox_marker_name, ARR(1, 0, 0), NULL, alvar->X.rot.getX().getArr(), sumOfSqrTT);
  komo.setAlign(1., 1., lockbox_marker_name, ARR(0, 1, 0), NULL, alvar->X.rot.getY().getArr(), sumOfSqrTT);
  komo.setAlign(1., 1., lockbox_marker_name, ARR(0, 0, 1), NULL, alvar->X.rot.getZ().getArr(), sumOfSqrTT);

  komo.reset();
  komo.run();

//  cout << "Full Q after: " << komo.x << endl;

  // New Q vector is in komo.x
  new_q = myBaxter->getKinematicWorld().q;


  new_q(jt->qIndex) = komo.x(jt->qIndex);

//  cout << "Q with modified joint: " << new_q << endl;

//  cout << "Joint. Original: " << myBaxter->getKinematicWorld().q(jt->qIndex)
//       << " Modelworld: " << myBaxter->getModelWorld().q(jt->qIndex)
//       << " min: " << jt->limits(0) << "\tCurrent: " << komo.x(jt->qIndex) << "\t Max: " << jt->limits(1) << endl;


  return true;
}
