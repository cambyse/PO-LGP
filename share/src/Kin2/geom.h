#ifndef GEOM_H
#define GEOM_H

#include <Core/array.h>

struct Geom{
  struct GeomStore& G; //every geom is uniquely owned (listed) by a geom store
  int ID; //its ID equals its indes in the list

  arr vertices;  //(nx3)
  intA triangles; //(Tx3): triangles; (0): point cloud; (Tx2): lines
  arr normals; //(nx3)
  arr color;     //(nx3|4): color per vertex, (3|4): color of whole geom (to allow colors per triangle you need to triple the vertices)
  //texture
  double radius; //if positive, this is a sphere-swept convex geometry, where 'verticies' describe the convex core; the true geom is larger

  Geom createColorsPerTriangle(const arr& colors); //returns a new geom with colors per triangle [asserts colors.dim(0)==triangles.dim(0)]
  Geom createSphereSweptVisual(int sphereApproxLevel=3); //asserts radius>0; returns a geom with radius==0 that is a visual of this geom
};

#endif // GEOM_H

