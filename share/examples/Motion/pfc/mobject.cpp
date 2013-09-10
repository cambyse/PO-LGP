#include "mobject.h"

MObject::MObject(ors::Graph *_ors,MT::String _name, ObjectType _objectType, double _stepLength, const arr &_direction) {
  ors=_ors;
  name = _name;
  objectType = _objectType;
  direction = _direction;
  stepLength = _stepLength;
  cout << name << endl;

  ors->kinematicsPos(position, ors->getBodyByName(name)->index);
  positionHistory.append(~position);
  predict(10);
}

MObject::~MObject() {

}


void MObject::predict(uint _T) {
  prediction = position + (direction*stepLength)*double(_T);
}


void MObject::move() {
  ors->kinematicsPos(position, ors->getBodyByName(name)->index);
  positionHistory.append(~position);
  ors->getBodyByName(name)->X.pos = position + (direction*stepLength);
}

