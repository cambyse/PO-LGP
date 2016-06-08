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
  if ( joint_tfs.count(joint) == 0 )
  {
    std::cout << "Joint: " << joint << " not visible." << std::endl;
    return false;
  }

  diff = joint_origins.at(joint);
  diff.appendInvTransformation(joint_tfs.at(joint));
  return true;
}

bool Lockbox::getAbsoluteJointTransform(const uint joint, ors::Transformation &tf)
{
  if ( joint_tfs.count(joint) == 0 )
  {
    std::cout << "Joint: " << joint << " doesn't exist!" << std::endl;
    return false;
  }

  tf = joint_tfs.at(joint);
  return true;
}

bool Lockbox::getOriginalJointTransform(const uint joint, ors::Transformation &orig)
{
  if ( joint_origins.count(joint) == 0 )
  {
    std::cout << "Joint: " << joint << " doesn't exist!" << std::endl;
    return false;
  }

  orig = joint_origins.at(joint);
  return true;
}

void Lockbox::readJointPositions(){
  if (!update)
    return;

  object_database.readAccess();
//  joint_tfs.clear();
  FilterObjects filter_objects = object_database();

  for (FilterObject* fo : filter_objects)
  {
      if (fo->type == FilterObject::FilterObjectType::alvar)
      {
        Alvar* av = dynamic_cast<Alvar*>(fo);
        ors::Transformation tf = av->frame;
        tf.appendTransformation(av->transform);
        joint_tfs.erase(av->id);
        joint_tfs.insert(std::make_pair(av->id, tf));
      }
  }
  object_database.deAccess();

  if (joint_origins.empty() && !joint_tfs.empty())
  {
    joint_origins = joint_tfs;
    for (uint i = 1; i < 6; ++i)
    {
      if (joint_origins.count(i))
        std::cout << "Origin: " << i << ' ' << joint_origins.at(i).pos << std::endl;
    }
  }
}

bool Lockbox::calculateStubOffset(const uint joint, const ors::Transformation& alvar_tf, ors::Transformation& newStub)
{
  ors::Transformation tf;
  tf.setZero();
  tf.addRelativeTranslation(offsets.at(joint)(0), offsets.at(joint)(1), offsets.at(joint)(2));
//  newStub.setZero();
//  if (! getJointTransform(joint, newStub) )
//    return false;
//  tf.appendTransformation(newStub);
  tf.appendTransformation(alvar_tf);
  newStub = tf;
//  ors::Vector offset = ors::Vector( offsets.at(joint)(0), offsets.at(joint)(1), offsets.at(joint)(2) );
//  newStub.addRelativeTranslation(offset);
  std::cout << "New Stub: " << newStub.pos << std::endl;
  return true;
}

bool Lockbox::moveToJointStub(const uint joint, const ors::Transformation& offset, const double speed, const double prec)
{
  std::cout << "Move to joint: " << joint << " offset: " << offset << std::endl;
  // Calculate stub position.
  ors::Transformation newPosition;
  if (! calculateStubOffset(joint, joint_tfs.at(joint), newPosition) )
    return false;

  newPosition.appendTransformation(offset);
  return moveToAlvar(joint, newPosition, speed, prec);
}

bool Lockbox::moveToAlvar(const uint joint, const ors::Transformation& offset, const double speed, const double prec)
{
  ors::Transformation newPosition = joint_tfs.at(joint);
  newPosition = offset * newPosition;
  return moveAbsolute(newPosition.pos, speed, prec);
}

bool Lockbox::moveRelative(const ors::Vector& offset, const double speed, const double prec)
{
  CtrlTask* currentPositionTask = myBaxter->task(
                      "rel",
                      new DefaultTaskMap(posTMT, myBaxter->getKinematicWorld(), "endeffL", NoVector, "base_footprint"), //map
                      1., speed, 3., 3.);
  currentPositionTask->map.phi(currentPositionTask->y, NoArr, myBaxter->getKinematicWorld()); //get the current value

  arr pos = currentPositionTask->y;

  myBaxter->stop({currentPositionTask});

  return moveAbsolute(pos + ARR(offset.x, offset.y, offset.z));
}

bool Lockbox::moveAbsolute(const ors::Vector& offset, const double speed, const double prec)
{
  std::cout << "Move Absolute: " << offset << std::endl;
  CtrlTask* currentPositionTask = myBaxter->task(
                      "rel",
                      new DefaultTaskMap(posTMT, myBaxter->getKinematicWorld(), "endeffL", NoVector, "base_footprint"), //map
                      1., speed, 3., 3.);
  currentPositionTask->map.phi(currentPositionTask->y, NoArr, myBaxter->getKinematicWorld()); //get the current value

  myBaxter->modifyTarget(currentPositionTask, ARR(offset.x, offset.y, offset.z));
  currentPositionTask->prec = ARR(prec);
  myBaxter->waitConv({currentPositionTask});
  myBaxter->stop({currentPositionTask});
  return true;
}

void Lockbox::step(){
  readJointPositions();
}

void Lockbox::open(){
  this->listenTo(*object_database.var);

  ors::Quaternion quat;
  quat.setRpy(0, 0, -MLR_PI / 2);
  end_orientations.insert( std::make_pair(1, quat) );

  quat.setRpy(0, 0, 0);
  end_orientations.insert( std::make_pair(2, quat) );

  quat.setRpy(0, 0, 0);
  end_orientations.insert( std::make_pair(3, quat) );

  quat.setRpy(MLR_PI/2, 0, 0);
  end_orientations.insert( std::make_pair(4, quat) );

  quat.setRpy(-MLR_PI/4, 0, 0);
  end_orientations.insert( std::make_pair(5, quat) );
}

void Lockbox::close(){
  this->stopListenTo(*object_database.var);
}

void Lockbox::moveJointToPosition(const uint joint, const double position)
{
  // Assume position is between 0 and 1.
  myBaxter->grip(0);

  auto alignR = myBaxter->task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[1 0 0] vec2=[0 0 1] target=[0] PD=[1., .8, 1., 5.] prec=[1]"));
  auto alignR2 = myBaxter->task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 1 0] vec2=[0 0 -1] target=[1] PD=[1., .8, 1., 5.] prec=[1]"));
  auto alignR3 = myBaxter->task(GRAPH("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 0 1] vec2=[1 0 0] target=[0] PD=[1., .8, 1., 5.] prec=[1]"));

  ors::Transformation offset_tf;
  offset_tf.setZero();
  offset_tf.addRelativeTranslation(-0.2, 0.05, 0);
  moveToAlvar(joint, offset_tf, 1.2, 50);
  myBaxter->waitConv({alignR, alignR2, alignR3});

  myBaxter->disablePosControl();
  mlr::wait(1.);
  update = false;
  myBaxter->enablePosControl();
  mlr::wait(1.);

  offset_tf.setZero();
  offset_tf.addRelativeTranslation(-0.08, 0, 0);
  moveToJointStub(joint, offset_tf, 1, 50);
  moveToJointStub(joint, offset_tf, 1, 0.1);
  myBaxter->waitConv({alignR, alignR2, alignR3});

  offset_tf.setZero();
  moveToJointStub(joint, offset_tf, 1, 50);
  moveToJointStub(joint, offset_tf, 1, 1);
  myBaxter->waitConv({alignR, alignR2, alignR3});

  //myBaxter->grip(1);

  myBaxter->stop({alignR, alignR2, alignR3});

  ors::Quaternion rot = end_orientations.at(joint);

//  uint steps = 1000;
//  for (uint i = 0; i < steps; i++)
//  {
    rot = end_orientations.at(joint);
    ors::Quaternion initial; initial.setRpy(-MLR_PI/2, 0, 0);
    rot = rot * initial;

    rot.setInterpolate(position, initial, rot);
    //    rot.setInterpolate(i / steps, initial, rot);

    ors::Vector xVec = rot.getMatrix() * ors::Vector(1, 0, 0);
    ors::Vector yVec = rot.getMatrix() * ors::Vector(0, 1, 0);
    ors::Vector zVec = rot.getMatrix() * ors::Vector(0, 0, 1);


    auto alignX = myBaxter->task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[1 0 0] vec2=["
                                           << xVec.x << ' ' << xVec.y << ' ' << xVec.z
                                           << "] target=[1] PD=[1., 1., .8, 1.] prec=[1000]")));

    auto alignY = myBaxter->task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 1 0] vec2=["
                                           << yVec.x << ' ' << yVec.y << ' ' << yVec.z
                                           << "] target=[1] PD=[1., 1., .8, 1.] prec=[1000]")));

    auto alignZ = myBaxter->task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 0 1] vec2=["
                                           << zVec.x << ' ' << zVec.y << ' ' << zVec.z
                                           << "] target=[1] PD=[1., 1., .8, 1.] prec=[1000]")));

    ors::Vector pos = end_offsets.at(joint);
//    pos.setLength(pos.length() * (i / 1000));
    pos.setLength(pos.length() * (position));

    moveRelative(pos, 0.8, 1000);

    alignX->prec = ARR(50);
    alignY->prec = ARR(50);
    alignZ->prec = ARR(50);
    myBaxter->waitConv({alignX, alignY, alignZ});
//  }
  myBaxter->grip(0);
//        mlr::wait(2.);

  ors::Vector away = rot * ors::Vector(-.1, 0, 0);
  moveRelative(away, 0.8, 100);
}

