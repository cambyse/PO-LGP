#include "mobject.h"

MObject::MObject(ors::KinematicWorld *_world,MT::String _name, ObjectType _objectType, double _stepLength, const arr &_direction) {
  world=_world;
  name = _name;
  objectType = _objectType;
  direction = _direction;
  stepLength = _stepLength;
  cout << "Loaded MObject: " << name << endl;

  world->kinematicsPos(position,NoArr, world->getBodyByName(name)->index);
  positionHistory.append(~position);

  world->kinematicsVec(orientation,NoArr, world->getBodyByName(name)->index);
  orientationHistory.append(~orientation);
}

MObject::~MObject() {

}


void MObject::predict(uint _T) {
  prediction = position + (direction*stepLength)*double(_T);
}

void MObject::setPosition(const arr& _position) {
  world->kinematicsPos(position,NoArr, world->getBodyByName(name)->index);
  positionHistory.append(~position);
  world->getBodyByName(name)->X.pos = _position;
  position = _position;
}

void MObject::setOrientation(const arr& _orientation) {
  positionHistory.append(~orientation);
  orientation = _orientation;
}

void MObject::move() {
  world->kinematicsPos(position,NoArr, world->getBodyByName(name)->index);
  positionHistory.append(~position);
  position = position + (direction*stepLength);
  world->getBodyByName(name)->X.pos = position;
}

void MObject::move(const arr& _offset) {
  world->kinematicsPos(position,NoArr, world->getBodyByName(name)->index);
  positionHistory.append(~position);
  position = position + _offset;
  world->getBodyByName(name)->X.pos = position;
}

void MObject::rotate(const arr& _offset) {
  orientationHistory.append(~orientation);
  orientation = orientation + _offset;
  world->getBodyByName(name)->X.rot.setDeg( world->getBodyByName(name)->X.rot.getDeg()+0.7,_offset);
}

