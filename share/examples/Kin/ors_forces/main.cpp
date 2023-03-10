#include <Kin/kin.h>
#include <Kin/kin_physx.h>
#include <Gui/opengl.h>

int dir = 1;
double GRAVITY = 9.81;

void move_robot(mlr::KinematicWorld& G, int time) {
    mlr::Body *b = G.getBodyByName("robot");

    double kd = 400;
    double kp = 1000;
  
    mlr::Vector pos = G.getBodyByName("robot")->X.pos;
    mlr::Vector vel = G.getBodyByName("robot")->X.vel;
    double dPos = 1 - pos.z;
    double dVel = 0 - vel.z;

    double dyPos = 1. - pos.y;
    double dyVel = 0 - vel.y;
    
    G.addForce(mlr::Vector(0, kp * dyPos + kd * dyVel, b->mass * GRAVITY + kp * dPos + kd * dVel), b);
    //if (b->X.pos.y > 1.3) dir = -1;
    //b->X.pos.y += dir * .01;
}
void TEST(OrsPhysx) {
  mlr::KinematicWorld G(mlr::getParameter<mlr::String>("orsFile"));
  OpenGL glPh;

  glPh.add(glStandardScene, NULL);
  glPh.add(glPhysXInterface, &G.physx());
  glPh.setClearColors(1.,1.,1.,1.);
  glPh.camera.setPosition(10.,-15.,8.);
  glPh.camera.focus(0,0,1.);
  glPh.watch();
  
  for(uint t=0; t<1500; t++) {
    cout <<"\r t=" <<t <<std::flush;
    G.physx().step();
    glPh.update();
    G.watch(false);
    move_robot(G, t);
  }
}

int MAIN(int argc, char** argv) {
  mlr::initCmdLine(argc, argv);
  testOrsPhysx();

  return 0;
}
