#ifndef OBJECTGENERATOR_H
#define OBJECTGENERATOR_H

#include <Core/array.h>
#include <Ors/ors.h>

struct Object {

  ors::KinematicWorld& world;
  ors::Body* b;
  ors::Shape* s;

  Object(ors::KinematicWorld& world);

  void generateObject();
  arr sampleFromObject();
};

#endif // OBJECTGENERATOR_H
