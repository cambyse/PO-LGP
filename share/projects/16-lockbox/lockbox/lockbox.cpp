#define REPORT 1

#include "lockbox.h"
#include <unordered_map>

bool Lockbox::getJointPosition(const uint joint, double& position){
  ors::Transformation diff;
  if ( !getJointTransform(joint, diff) )
    return false;

  // for now, just calculate the linear difference between positions.
  position = diff.pos.length();
  return true;
}

bool Lockbox::getJointTransform(const uint joint, ors::Transformation &diff)
{
  if ( joint_positions.count(joint) == 0 )
  {
    std::cout << "Joint: " << joint << " doesn't exist!" << std::endl;
    return false;
  }

  diff = joint_origins.at(joint);
  diff.appendInvTransformation(joint_positions.at(joint));
  return true;
}

bool Lockbox::getAbsoluteJointTransform(const uint joint, ors::Transformation &diff)
{
  if ( joint_positions.count(joint) == 0 )
  {
    std::cout << "Joint: " << joint << " doesn't exist!" << std::endl;
    return false;
  }

  diff = joint_positions.at(joint);
  return true;
}

void Lockbox::readJointPositions(){
  object_database.readAccess();
  joint_positions.clear();
  FilterObjects filter_objects = object_database();

  for (FilterObject* fo : filter_objects)
  {
      if (fo->type == FilterObject::FilterObjectType::alvar)
      {
        Alvar* av = dynamic_cast<Alvar*>(fo);
        ors::Transformation tf = av->frame;
        tf.appendTransformation(av->transform);
        joint_positions.insert(std::make_pair(av->id, tf));
      }
  }
  object_database.deAccess();

  if (joint_origins.empty() && !joint_positions.empty())
  {
    joint_origins = joint_positions;
  }
}

void Lockbox::step(){
  readJointPositions();
}

void Lockbox::open(){
  this->listenTo(*object_database.var);
}

void Lockbox::close(){
  this->stopListenTo(*object_database.var);
}
