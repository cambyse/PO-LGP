#include <stdlib.h>
#include <Kin/kin.h>
#include <Gui/opengl.h>

#include "kin_ibds.h"


int main(int argc, char **argv) {
  IbdsModule ibds;
  mlr::KinematicWorld ors;
  OpenGL gl;

  ors.init("test_ibds.ors");
  
#if 0 //randomly add spheres!
  for(uint k=0;k<20;k++){
    mlr::Body *b = new mlr::Body(ors);
    b->X.setRandom();
    b->X.pos.z += 1.;
    b->name <<"rndSphere_" <<k;
    mlr::Shape *s = new mlr::Shape(ors, b);
    s->type=(mlr::ShapeType)1;
    s->size[3]=.1;
  }
  ors.calcShapeFramesFromBodies();
#endif
  
  gl.add(glStandardScene,NULL);
  gl.add(mlr::glDrawGraph,&ors);
  gl.setClearColors(1.,1.,1.,1.);
  gl.camera.setPosition(10.,-15.,8.);
  gl.camera.focus(0,0,1.);
  gl.watch();
  
  ibds.create(ors);
  
  uint t;
  for(t=0;; t++) {
    ibds.step();
    cout <<"\r t = " <<t <<" energy = " <<ors.getEnergy() <<std::flush;
    gl.update();
  }
  
  return 0;
}
