#ifndef OBJECTGENERATOR_H
#define OBJECTGENERATOR_H

#include <Core/array.h>
#include <Kin/kin.h>

struct Object {

  mlr::KinematicWorld& world;
  mlr::Body* b;
  mlr::Shape* s;

  Object(mlr::KinematicWorld& world);

  void generateObject(const char* name = "b", double xScale = 0.2, double yScale = 0.2, double zScale = 0.2, double xPos = 0.0, double yPos = 0.0, double zPos = 0.0, bool contact = true);
  arr sampleFromObject();
  void generateMeshObject(const char* name, mlr::Mesh mesh, double xPos, double yPos, double zPos);
};

#endif // OBJECTGENERATOR_H
