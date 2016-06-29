#define REPORT 1
#define SIM 1

#include "lockbox.h"
#include <unordered_map>

void Lockbox::step(){

}

void Lockbox::open(){
  this->listenTo(*object_database.var);
}

void Lockbox::close(){
  this->stopListenTo(*object_database.var);
}

void Lockbox::initializeJoints()
{
  joint_to_ors_joint.clear();

  joint_to_handle.insert(std::make_pair(1, "door_handle"));
  joint_to_ors_joint.insert(std::make_pair(1, "lockbox_door"));

  joint_to_handle.insert(std::make_pair(2, "bar_handle"));
  joint_to_ors_joint.insert(std::make_pair(2, "lockbox_bar"));

  joint_to_handle.insert(std::make_pair(3, "wheel_handle"));
  joint_to_ors_joint.insert(std::make_pair(3, "lockbox_wheel"));

  joint_to_handle.insert(std::make_pair(4, "bar2_handle"));
  joint_to_ors_joint.insert(std::make_pair(4, "lockbox_bar2"));

  joint_to_handle.insert(std::make_pair(5, "door2_handle"));
  joint_to_ors_joint.insert(std::make_pair(5, "lockbox_door2"));
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
