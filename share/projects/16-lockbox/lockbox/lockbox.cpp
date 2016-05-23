#define REPORT 1

#include "lockbox.h"
#include <unordered_map>

bool Lockbox::getJointPosition(const uint joint, double position){
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
    return false;

  diff = joint_positions.at(joint);
  return true;
}

void Lockbox::readJointPositions(){
  joint_positions.clear();
  object_database.readAccess();
  FilterObjects filter_objects = object_database();

  for (FilterObject* fo : filter_objects)
  {
      if (fo->type == FilterObject::FilterObjectType::alvar)
      {
        Alvar* av = dynamic_cast<Alvar*>(fo);
        joint_positions.at(av->id) = av->transform;
      }
  }
  object_database.deAccess();
}
