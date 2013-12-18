#include "mobject.h"

MObject::MObject(ors::KinematicWorld *_ors,MT::String _name, ObjectType _objectType, double _stepLength, const arr &_direction) {
  ors=_ors;
  name = _name;
  objectType = _objectType;
  direction = _direction;
  stepLength = _stepLength;
  cout << "Loaded MObject: " << name << endl;

  ors->kinematicsPos(position, NoArr, ors->getBodyByName(name)->index);
  positionHistory.append(~position);

  ors->kinematicsVec(orientation, NoArr, ors->getBodyByName(name)->index);
  orientationHistory.append(~orientation);
}

MObject::~MObject() {

}


void MObject::predict(uint _T) {
  prediction = position + (direction*stepLength)*double(_T);
}

void MObject::setPosition(const arr& _position) {
  ors->kinematicsPos(position, NoArr, ors->getBodyByName(name)->index);
  positionHistory.append(~position);
  ors->getBodyByName(name)->X.pos = _position;
  position = _position;
}

void MObject::setOrientation(const arr& _orientation) {
  positionHistory.append(~orientation);
  orientation = _orientation;
}

void MObject::move() {
  ors->kinematicsPos(position, NoArr, ors->getBodyByName(name)->index);
  positionHistory.append(~position);
  position = position + (direction*stepLength);
  ors->getBodyByName(name)->X.pos = position;
}

void MObject::move(const arr& _offset) {
  ors->kinematicsPos(position, NoArr, ors->getBodyByName(name)->index);
  positionHistory.append(~position);
  position = position + _offset;
  ors->getBodyByName(name)->X.pos = position;
}

void MObject::rotate(const arr& _offset) {
  orientationHistory.append(~orientation);
  orientation = orientation + _offset;
}
