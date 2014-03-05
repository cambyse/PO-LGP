#include <Ors/ors.h>
#include <Ors/ors_physx.h>
#include <Gui/opengl.h>

int dir = 1;
double GRAVITY = 9.81;
ors::Vector oldPos(0, 0, 0);

void move_robot(ors::KinematicWorld& G) {
    ors::Body *b = G.getBodyByName("robot");

    double kd = 400;
    double kp = 1000;
  
    ors::Vector pos = G.getBodyByName("robot")->X.pos;
    double dPos = 1 - pos.z;
    double dVel = oldPos.z - pos.z;
    oldPos = G.getBodyByName("robot")->X.pos;
    
    G.addForce(ors::Vector(0, 10, b->mass * GRAVITY + kp * dPos + kd * dVel), b);
    //if (b->X.pos.y > 1.3) dir = -1;
    //b->X.pos.y += dir * .01;
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
  
  oldPos = G.getBodyByName("robot")->X.pos;
  for(uint t=0; t<1500; t++) {
    cout <<"\r t=" <<t <<std::flush;
    G.physx().step();
    glPh.update();
    G.watch(false);
    move_robot(G);
  }
}

int MAIN(int argc, char** argv) {
  MT::initCmdLine(argc, argv);
  testOrsPhysx();

  return 0;
}
