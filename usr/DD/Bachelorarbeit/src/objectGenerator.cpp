#include "objectGenerator.h"
#include <Core/util.h>

Object::Object(ors::KinematicWorld& world) : world(world) {
  rnd.clockSeed();
}

void Object::generateObject(const char* name, double xScale, double yScale, double zScale, double xPos, double yPos, double zPos, bool contact) {
  b = new ors::Body(world);
  b->type = ors::BodyType::staticBT;
  b->name = name;
  b->X.pos = ors::Vector(xPos, yPos, zPos);
  s = new ors::Shape(world,*b);
  s->type = ors::ShapeType::meshST;
  s->name = name;
  arr color = ARR(0.5,0.5,0.);
  memmove(s->color, color.p, 3*sizeof(double));
  ors::Mesh me;

  ScalarFunction torus = [](arr&,arr&, const arr& X){
      double x=X(0), y=X(1), z=X(2);
      double r=sqrt(x*x + y*y);
      return z*z + (1.-r)*(1.-r) - .1;
  };

  ScalarFunction cub = [](arr&,arr&, const arr& X){
      double x=X(0), y=X(1), z=X(2);
      return sqrt(x*x)*sqrt(x*x)*sqrt(x*x)/**sqrt(x*x)*/ + sqrt(y*y)*sqrt(y*y)*sqrt(y*y)/**sqrt(y*y)*/ + sqrt(z*z)*sqrt(z*z)*sqrt(z*z)/**sqrt(z*z)*/ - 1.0 + 0.1*fabs(x-y);
  };

  /*ScalarFunction sphere=[](arr&,arr&, const arr& X){
      double x=X(0), y=X(1), z=X(2);
      return (x*x +y*y+z*z)-1.;
    };*/

  //me.setRandom(100);
  me.setImplicitSurface(cub, -10.0, 10.0, 300);
  //me.scale(0.15,.2,.05);
  //me.setBox();
  me.scale(xScale,yScale,zScale);
  me.deleteUnusedVertices();
  me.computeNormals();
  me.clean();
  s->mesh = me;
  s->cont = contact;
  world.calc_fwdPropagateFrames();
}

arr Object::sampleFromObject() {
  return s->mesh.V[rnd.num(s->mesh.V.d0)] + conv_vec2arr(s->X.pos);
}

void Object::generateMeshObject(const char* name, ors::Mesh mesh, double xPos, double yPos, double zPos) {
  b = new ors::Body(world);
  b->type = ors::BodyType::staticBT;
  b->name = name;
  b->X.pos = ors::Vector(xPos, yPos, zPos);
  s = new ors::Shape(world,*b);
  s->type = ors::ShapeType::meshST;
  s->name = name;
  arr color = ARR(0.5,0.5,0.);
  memmove(s->color, color.p, 3*sizeof(double));
  s->mesh = mesh;
  world.calc_fwdPropagateFrames();
}

/*void Object::generateEmptyObject(const char* name) {
  b = new ors::Body(world);
  b->type = ors::BodyType::staticBT;
  b->name = name;
  b->X.pos = ors::Vector(xPos, yPos, zPos);
  s = new ors::Shape(world,*b);
  s->type = ors::ShapeType::meshST;
  s->name = name;
  arr color = ARR(0.5,0.5,0.);
  memmove(s->color, color.p, 3*sizeof(double));
  world.calc_fwdPropagateFrames();
}*/
