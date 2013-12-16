#ifndef MOBJECT_H
#define MOBJECT_H

#include <Core/util.h>
#include <Core/array.h>
#include <Ors/ors.h>


struct MObject {
  enum ObjectType {OBSTACLE, GOAL};

  ors::KinematicWorld *ors;
  MT::String name;
  ObjectType objectType;
  double stepLength;

  arr position;
  arr orientation;
  arr direction;
  arr prediction;
  arr positionHistory; // save positions of object
  arr orientationHistory;

  MObject(ors::KinematicWorld *_ors,MT::String _name, ObjectType _objectType, double _stepLength=0.001, const arr& _direction = ARRAY(1.,0.,0.));
  ~MObject();

  void predict(uint _T); // predict _T time steps ahead
  void move();  // simulate object for one time step along direction
  void move(const arr& _offset);  // simulate object for one time step
  void rotate(const arr& _offset);
  void drawPrediction();

  // Getter
  void setPosition(const arr& _position);
  void setOrientation(const arr& _orientation);


};

#endif // MOBJECT_H
