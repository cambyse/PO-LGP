#define MT_IMPLEMENT_TEMPLATES
#include <Gui/opengl.h>
#include <Ors/ors.h>
#include <Ors/ors_physx.h>

#include "graspEvaluation.h"

void evaluate(){
  ors::Graph ors;
  OpenGL gl;
  init(ors, gl, "test.ors");
  gl.update();
  
  GraspEvaluation ge;
  ge.copyGraspFromOrs(ors, "m9", "box");
  ge.getContactPoints();
  ge.gl.watch();
  ge.closeFingers();
  ge.getContactPoints();
  ge.gl.watch();
  ge.simulateInPhysX();
}

int main(int argn, char** argv){
  evaluate();

  return 0;
}
