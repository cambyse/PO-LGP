#define MLR_IMPLEMENT_TEMPLATES
#include <Gui/opengl.h>
#include <Ors/ors.h>
#include <Ors/ors_physx.h>

#include "graspEvaluation.h"

void evaluate(){
  mlr::KinematicWorld ors;
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

int main(int argc, char** argv){
  evaluate();

  return 0;
}
