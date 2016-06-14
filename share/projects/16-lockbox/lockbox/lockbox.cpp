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

  if (simulate)
  {
    ors::Transformation tf; tf.setZero();
    joint_origins.insert(std::make_pair(1, tf));

    return;
  }


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

//bool Lockbox::moveToAlvar(const uint joint, const ors::Transformation& offset, const double speed, const double prec)
//{
//  cout << "Moving alvar: " << joint << endl;
//  ors::Transformation newPosition = joint_tfs.at(joint);
//  newPosition.setInverse(newPosition);
//  newPosition = newPosition * offset;
//  newPosition.setInverse(newPosition);

//  return moveAbsolute(newPosition.pos, speed, prec);
//}

/**/bool Lockbox::moveRelative(const ors::Vector& offset, const double speed, const double prec)
{
  cout << "Moving relative: " << offset << endl;
  CtrlTask* currentPositionTask = myBaxter->task(
                      "rel",
                      new DefaultTaskMap(posTMT, myBaxter->getKinematicWorld(), "endeffL", NoVector, "base_footprint"), //map
                      1., speed, 1., 1.);
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
                      1., speed, 1., 1.);
  currentPositionTask->map.phi(currentPositionTask->y, NoArr, myBaxter->getKinematicWorld()); //get the current value

  myBaxter->modifyTarget(currentPositionTask, ARR(offset.x, offset.y, offset.z));
  currentPositionTask->prec = ARR(prec);
  myBaxter->waitConv({currentPositionTask});
  myBaxter->stop({currentPositionTask});
  return true;
}

bool Lockbox::moveAbsolute(const ors::Transformation& tf, const double speed, const double prec)
{
  std::cout << "Move Absolute: " << tf << std::endl;
  CtrlTask* currentPositionTask = myBaxter->task(
                      "rel",
                      new DefaultTaskMap(posTMT, myBaxter->getKinematicWorld(), "endeffL", NoVector, "base_footprint"), //map
                      1., speed, 1., 1.);
  currentPositionTask->map.phi(currentPositionTask->y, NoArr, myBaxter->getKinematicWorld()); //get the current value

  myBaxter->modifyTarget(currentPositionTask, ARR(tf.pos.x, tf.pos.y, tf.pos.z));
  currentPositionTask->prec = ARR(prec);

  ors::Vector rotX = tf.rot.getX(); rotX.normalize();
  ors::Vector rotY = tf.rot.getY(); rotY.normalize();
  ors::Vector rotZ = tf.rot.getZ(); rotZ.normalize();

  auto alignX = myBaxter->task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 0 1] vec2=[" <<
                                            rotX.x << ' ' << rotX.y << ' ' << rotX.z << "] target=[1] PD=[1., .8, 1., 1.] prec=[100]")));
  auto alignY = myBaxter->task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 1 0] vec2=[" <<
                                            rotY.x << ' ' << rotY.y << ' ' << rotY.z << "] target=[1] PD=[1., .8, 1., 1.] prec=[100]")));
  auto alignZ = myBaxter->task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[-1 0 0] vec2=[" <<
                                            rotZ.x << ' ' << rotZ.y << ' ' << rotZ.z << "] target=[1] PD=[1., .8, 1., 1.] prec=[100]")));

  myBaxter->waitConv({currentPositionTask, alignX, alignY, alignZ});
  myBaxter->stop({currentPositionTask, alignX, alignY, alignZ});
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

  quat.setZero();
  end_orientations.insert( std::make_pair(2, quat) );

  quat.setRpy(MLR_PI/2, 0, 0);
  end_orientations.insert( std::make_pair(3, quat) );

  quat.setZero();
  end_orientations.insert( std::make_pair(4, quat) );

  quat.setRpy(-MLR_PI/4, 0, 0);
  end_orientations.insert( std::make_pair(5, quat) );
}

void Lockbox::close(){
  this->stopListenTo(*object_database.var);
}

bool Lockbox::moveToJoint(const uint joint, const ors::Transformation& offset_in_base, const double speed, const double prec)
{
  cout << "Move to joint: " << joint << " offset: " << offset_in_base << endl;
  // Calculate stub position.
  ors::Transformation newPosition;

  if (joint_tfs.count(joint) == 0)
    return false;

  newPosition = joint_tfs.at(joint);
  newPosition.setInverse(newPosition);
  newPosition.appendTransformation(offset_in_base);
  newPosition.setInverse(newPosition);

  if (! calculateStubOffset(joint, joint_tfs.at(joint), newPosition) )
    return false;

  newPosition.appendTransformation(offset_in_base);
  //  return moveToAlvar(joint, newPosition, speed, prec);
  return moveAbsolute(newPosition.pos, speed, prec);
}

void Lockbox::moveJointInterpolate(const uint joint, const double position)
{
  // Assume position is between 0 and 1.
  myBaxter->grip(0);

  ors::Transformation alvar = joint_tfs.at(joint);
  alvar.setInverse(alvar);
  alvar.addRelativeTranslation(0.2, -0.05, 0);
  alvar.setInverse(alvar);
  moveAbsolute(alvar, 1, 10);

  myBaxter->disablePosControl();
  mlr::wait(1.);
  update = false;
  myBaxter->enablePosControl();
  mlr::wait(1.);

  // Now we have an accurate read of the alvar marker.
  // Add the joint offset to get to the stub
  alvar = joint_tfs.at(joint);
  update = true;

  alvar.setInverse(alvar);
  alvar.addRelativeTranslation(-offsets.at(joint));
  alvar.setInverse(alvar);

  // Add an offset from the stub of 5cm
  alvar.addRelativeTranslation(0, 0, 0.05);

  moveAbsolute(alvar, 1, 10);

  cout << "Pregrasp " << endl;

  alvar.addRelativeTranslation(0, 0, -0.05);
  moveAbsolute(alvar, 1, 50);

  cout << "Grasp " << endl;

  myBaxter->grip(1);
  myBaxter->grip(0);

  ors::Quaternion rot = end_orientations.at(joint) * alvar.rot;
  ors::Quaternion initial; initial.setRpy(-MLR_PI/2, 0, 0);

  rot.setInterpolate(position, initial, rot);

  ors::Quaternion current_rot;

  ors::Vector end_pos = alvar.pos + end_offsets.at(joint) * position;

  uint steps = 10;
  for (uint i = 1; i <= steps; i++)
  {
    current_rot.setInterpolate(i / steps, initial, rot );

    ors::Transformation tf;
    tf.rot = current_rot;

    CtrlTask* currentPositionTask = myBaxter->task(
                        "pos",
                        new DefaultTaskMap(posTMT, myBaxter->getKinematicWorld(), "endeffL", NoVector, "base_footprint"), //map
                        1., 1, 1., 1.);
    currentPositionTask->map.phi(currentPositionTask->y, NoArr, myBaxter->getKinematicWorld()); //get the current value
    arr pos = currentPositionTask->y;
    myBaxter->stop({currentPositionTask});

    // Now, scale from the current position to the end position
    ors::Vector new_pos = pos + (end_pos - pos) * (1 / (steps - i + 1));

    tf.pos = new_pos;
    moveAbsolute(tf, 1, 10);

  }
  myBaxter->grip(0);

  exit(0);
  ors::Vector away = rot * ors::Vector(-.1, 0, 0);
  moveRelative(away, 0.8, 100);
}

void Lockbox::moveJointToPosition(const uint joint, const double position)
{
  // Assume position is between 0 and 1.
  myBaxter->grip(0);

  ors::Transformation alvar = joint_tfs.at(joint);
  alvar.setInverse(alvar);
  alvar.addRelativeTranslation(0.2, -0.05, 0);
  alvar.setInverse(alvar);
  moveAbsolute(alvar, 1, 10);

//  ors::Transformation tf; tf.setZero(); tf.addRelativeTranslation(0.2, -0.05, 0);
//  moveToJoint(joint, tf );

  myBaxter->disablePosControl();
  mlr::wait(1.);
  update = false;
  myBaxter->enablePosControl();
  mlr::wait(1.);

  // Now we have an accurate read of the alvar marker.
  // Add the joint offset to get to the stub
  alvar = joint_tfs.at(joint);
  alvar.setInverse(alvar);
  alvar.addRelativeTranslation(-offsets.at(joint));
  alvar.setInverse(alvar);

  // Add an offset from the stub of 5cm
  alvar.addRelativeTranslation(0, 0, 0.05);

  moveAbsolute(alvar, 1, 10);

  cout << "Pregrasp " << endl;

  alvar.addRelativeTranslation(0, 0, -0.05);
  moveAbsolute(alvar, 1, 50);

  cout << "Grasp " << endl;

  myBaxter->grip(1);
  myBaxter->grip(0);

  ors::Quaternion rot = end_orientations.at(joint) * alvar.rot;

//  uint steps = 1000;
//  for (uint i = 0; i < steps; i++)
//  {
//    rot = end_orientations.at(joint);
//    ors::Quaternion initial; initial.setRpy(-MLR_PI/2, 0, 0);
//    rot = rot * initial;

//    rot.setInterpolate(position, initial, rot);
    //    rot.setInterpolate(i / steps, initial, rot);

    ors::Vector xVec = rot.getX(); //rot.getMatrix() * ors::Vector(1, 0, 0);
    ors::Vector yVec = rot.getY(); //rot.getMatrix() * ors::Vector(0, 1, 0);
    ors::Vector zVec = rot.getZ(); //rot.getMatrix() * ors::Vector(0, 0, 1);


    auto alignX2 = myBaxter->task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 0 1] vec2=["
                                           << xVec.x << ' ' << xVec.y << ' ' << xVec.z
                                           << "] target=[1] PD=[1., 1., .8, 1.] prec=[100]")));

    auto alignY2 = myBaxter->task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 1 0] vec2=["
                                           << yVec.x << ' ' << yVec.y << ' ' << yVec.z
                                           << "] target=[1] PD=[1., 1., .8, 1.] prec=[100]")));

    auto alignZ2 = myBaxter->task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[-1 0 0] vec2=["
                                           << zVec.x << ' ' << zVec.y << ' ' << zVec.z
                                           << "] target=[1] PD=[1., 1., .8, 1.] prec=[100]")));

    ors::Vector pos = end_offsets.at(joint);
//    pos.setLength(pos.length() * (i / 1000));
//    pos.setLength(pos.length() * (position));

    moveRelative(pos, 0.8, 100);

    alignX2->prec = ARR(100);
    alignY2->prec = ARR(100);
    alignZ2->prec = ARR(100);
    myBaxter->waitConv({alignX2, alignY2, alignZ2});
    myBaxter->stop({alignX2, alignY2, alignZ2});
//  }
  myBaxter->grip(0);
//        mlr::wait(2.);

  exit(0);
  ors::Vector away = rot * ors::Vector(-.1, 0, 0);
  moveRelative(away, 0.8, 100);
}

/*void Lockbox::moveJointToPosition(const uint joint, const double position)
{
  // Assume position is between 0 and 1.
  myBaxter->grip(0);


  ors::Transformation alvar = joint_tfs.at(joint);
  alvar.setInverse(alvar);
  alvar.addRelativeTranslation(0.2, -0.05, 0);
  alvar.setInverse(alvar);

//  ors::Quaternion relRot;
//  relRot.setRpy(MLR_PI/2, 0, MLR_PI/2);
//  alvar.addRelativeRotation(relRot);

  ors::Vector rotX = alvar.rot.getX(); rotX.normalize();
  ors::Vector rotY = alvar.rot.getY(); rotY.normalize();
  ors::Vector rotZ = alvar.rot.getZ(); rotZ.normalize();

  auto alignX = myBaxter->task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 0 1] vec2=[" <<
                                            rotX.x << ' ' << rotX.y << ' ' << rotX.z << "] target=[1] PD=[1., .8, 1., 1.] prec=[100]")));
  auto alignY = myBaxter->task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 1 0] vec2=[" <<
                                            rotY.x << ' ' << rotY.y << ' ' << rotY.z << "] target=[1] PD=[1., .8, 1., 1.] prec=[100]")));
  auto alignZ = myBaxter->task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[-1 0 0] vec2=[" <<
                                            rotZ.x << ' ' << rotZ.y << ' ' << rotZ.z << "] target=[1] PD=[1., .8, 1., 1.] prec=[100]")));


  moveAbsolute(alvar.pos, 1, 10);

//  moveToAlvar(joint, offset_tf, 1.2, 50);
  myBaxter->waitConv({alignX, alignY, alignZ});

  myBaxter->disablePosControl();
  mlr::wait(1.);
  update = false;
  myBaxter->enablePosControl();
  mlr::wait(1.);

  myBaxter->stop({alignX, alignY, alignZ});

  alvar = joint_tfs.at(joint);
  rotX = alvar.rot.getX(); rotX.normalize();
  rotY = alvar.rot.getY(); rotY.normalize();
  rotZ = alvar.rot.getZ(); rotZ.normalize();

  alignX = myBaxter->task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 0 1] vec2=[" <<
                                            rotX.x << ' ' << rotX.y << ' ' << rotX.z << "] target=[1] PD=[1., .8, 1., 1.] prec=[100]")));
  alignY = myBaxter->task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 1 0] vec2=[" <<
                                            rotY.x << ' ' << rotY.y << ' ' << rotY.z << "] target=[1] PD=[1., .8, 1., 1.] prec=[100]")));
  alignZ = myBaxter->task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[-1 0 0] vec2=[" <<
                                            rotZ.x << ' ' << rotZ.y << ' ' << rotZ.z << "] target=[1] PD=[1., .8, 1., 1.] prec=[100]")));

  mlr::wait(10.);
  alvar.setInverse(alvar);
  alvar.addRelativeTranslation(-offsets.at(joint));
  alvar.setInverse(alvar);
  alvar.addRelativeTranslation(0, 0, 0.05);

  moveAbsolute(alvar.pos, 1, 10);
  myBaxter->waitConv({alignX, alignY, alignZ});

  cout << "Pregrasp " << endl;
  moveRelative(alvar.rot * ors::Vector(0.0, 0, -0.05), 1, 50);
  cout << "Grasp " << endl;

  myBaxter->waitConv({alignX, alignY, alignZ});

  myBaxter->grip(1);
  myBaxter->grip(0);

  myBaxter->stop({alignX, alignY, alignZ});

  ors::Quaternion rot = end_orientations.at(joint) * alvar.rot;

//  uint steps = 1000;
//  for (uint i = 0; i < steps; i++)
//  {
//    rot = end_orientations.at(joint);
//    ors::Quaternion initial; initial.setRpy(-MLR_PI/2, 0, 0);
//    rot = rot * initial;

//    rot.setInterpolate(position, initial, rot);
    //    rot.setInterpolate(i / steps, initial, rot);

    ors::Vector xVec = rot.getX(); //rot.getMatrix() * ors::Vector(1, 0, 0);
    ors::Vector yVec = rot.getY(); //rot.getMatrix() * ors::Vector(0, 1, 0);
    ors::Vector zVec = rot.getZ(); //rot.getMatrix() * ors::Vector(0, 0, 1);


    auto alignX2 = myBaxter->task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 0 1] vec2=["
                                           << xVec.x << ' ' << xVec.y << ' ' << xVec.z
                                           << "] target=[1] PD=[1., 1., .8, 1.] prec=[100]")));

    auto alignY2 = myBaxter->task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[0 1 0] vec2=["
                                           << yVec.x << ' ' << yVec.y << ' ' << yVec.z
                                           << "] target=[1] PD=[1., 1., .8, 1.] prec=[100]")));

    auto alignZ2 = myBaxter->task(GRAPH(STRING("map=vecAlign ref1=endeffL ref2=base_footprint vec1=[-1 0 0] vec2=["
                                           << zVec.x << ' ' << zVec.y << ' ' << zVec.z
                                           << "] target=[1] PD=[1., 1., .8, 1.] prec=[100]")));

    ors::Vector pos = end_offsets.at(joint);
//    pos.setLength(pos.length() * (i / 1000));
//    pos.setLength(pos.length() * (position));

    moveRelative(pos, 0.8, 100);

    alignX2->prec = ARR(100);
    alignY2->prec = ARR(100);
    alignZ2->prec = ARR(100);
    myBaxter->waitConv({alignX2, alignY2, alignZ2});
    myBaxter->stop({alignX2, alignY2, alignZ2});
//  }
  myBaxter->grip(0);
//        mlr::wait(2.);

  exit(0);
  ors::Vector away = rot * ors::Vector(-.1, 0, 0);
  moveRelative(away, 0.8, 100);
}
*/

//double Lockbox::calculateJointPosition(const uint joint){}
