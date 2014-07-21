#include <Ors/ors.h>
#include <Ors/ors_locker.h>
#include <Ors/ors_physx.h>
#include <Gui/opengl.h>

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

  ors::JointDependentLocker locker;
  locker.dependent_joint = locking;
  locker.lower_locked_limit = 10;
  locker.upper_locked_limit = 50;
  locked->locker = &locker;
  
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
