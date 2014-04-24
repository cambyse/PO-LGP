#include <Ors/ors.h>
#include <Ors/ors_physx.h>
#include <Gui/opengl.h>

bool locked_function(void* data) {
  double angle = ((ors::Joint*) data)->Q.rot.getDeg();
  if (angle > 10 && angle < 60.) {
    return false;
  }
  else {
    return true;
  }
}

void TEST(OrsPhysx) {
  ors::KinematicWorld G(MT::getParameter<MT::String>("orsFile"));
  OpenGL glPh; 

  glPh.add(glStandardScene, NULL);
  glPh.add(glPhysXInterface, &G.physx());
  glPh.setClearColors(1.,1.,1.,1.);
  glPh.camera.setPosition(10.,-15.,8.);
  glPh.camera.focus(0,0,1.);
  glPh.watch();

  ors::Joint *locked = G.getJointByName("locked");
  ors::Joint *locking = G.getJointByName("locking");

  locked->locked_func = *(locked_function);
  locked->locked_data = locking;
  
  ors::Body *door = G.getBodyByName("door1-door");
  G.addForce(ors::Vector(0, 6600, 0), door);
  
  for(uint t=0; t<1500; t++) {
    G.physx().step();
    glPh.update();
    G.watch(false);
  }
}

int MAIN(int argc, char** argv) {
  MT::initCmdLine(argc, argv);
  testOrsPhysx();

  return 0;
}
