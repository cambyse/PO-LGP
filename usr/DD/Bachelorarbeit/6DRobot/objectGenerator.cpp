#include "objectGenerator.h"
#include <Core/util.h>

Object::Object(ors::KinematicWorld& world) : world(world) {
  mlr::rnd.clockSeed();
}

void Object::generateObject() {
  b = new ors::Body(world);
  b->type = ors::BodyType::staticBT;
  b->name = "b";
  b->X.pos = ors::Vector(0.5,0.,.65);
  s = new ors::Shape(world,*b);
  s->type = ors::ShapeType::meshST;
  s->name = "b";
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
      return sqrt(x*x)*sqrt(x*x)*sqrt(x*x)*sqrt(x*x) + sqrt(y*y)*sqrt(y*y)*sqrt(y*y)*sqrt(y*y) + sqrt(z*z)*sqrt(z*z)*sqrt(z*z)*sqrt(z*z) - 1.0;
  };

  //me.setRandom(100);
  me.setImplicitSurface(cub);
  me.scale(0.2,.2,.1);
  me.deleteUnusedVertices();
  me.computeNormals();
  me.clean();
  s->mesh = me;
  s->cont = true;
  world.calc_fwdPropagateFrames();
}

arr Object::sampleFromObject() {
  return s->mesh.V[mlr::rnd.num(s->mesh.V.d0)] + conv_vec2arr(s->X.pos);
}
