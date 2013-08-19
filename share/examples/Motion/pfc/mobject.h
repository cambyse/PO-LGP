#ifndef MOBJECT_H
#define MOBJECT_H

#include <Core/util.h>
#include <Core/array.h>
#include <Ors/ors.h>


struct MObject {
  enum ObjectType {OBSTACLE, GOAL};

  ors::Graph *ors;
  MT::String name;
  ObjectType objectType;
  double stepLength;

  arr position;
  arr direction;
  arr prediction;
  arr positionHistory; // save positions of object

  MObject(ors::Graph *_ors,MT::String _name, ObjectType _objectType, double _stepLength=0.001, const arr& _direction = ARRAY(1.,0.,0.));
  ~MObject();

  void predict(uint _T); // predict _T time steps ahead
  void move();  // simulate object for one time step
  void drawPrediction();

  // Getter
  void pose(arr& _position) {_position = position;};


};

#endif // MOBJECT_H
