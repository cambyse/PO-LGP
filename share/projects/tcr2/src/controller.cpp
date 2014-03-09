#include "controller.h"

#include <Core/util.h>
#include <Ors/ors.h>
#include <Ors/ors_physx.h>
#include <Gui/opengl.h>

#include <ros/ros.h>

Controller::Controller(ros::NodeHandle& n, const char* endeff) : 
        n(n), 
        G(MT::getParameter<MT::String>("orsFile")),
        endeff(endeff),
        integrate(0, 0, 0) { 
  Kp = MT::getParameter<double>("Kp");
  Kd = MT::getParameter<double>("Kd");
  Ki = MT::getParameter<double>("Ki");

  bindOrsToOpenGL(G, gl);
}

void Controller::step() {
  ors::Body *robot = G.getBodyByName(endeff);
  ors::Vector pos = robot->X.pos;
  ors::Vector vel = robot->X.vel;

  ors::Vector dPos = goal - pos;
  ors::Vector dVel = - vel;
  integrate += dPos;
  
  ors::Vector force = Kp * dPos + Kd * dVel + Ki * integrate;

  //G.addForce(force, robot);
}

void Controller::run() {
  while (ros::ok()) {
    step();
    G.physx().step();
    gl.update();
  }
    
}
int main(int argc, char** argv) {
  MT::initCmdLine(argc, argv);
  ros::init(argc, argv, "Controller");

  ros::NodeHandle n;

  Controller c(n, "endeff");
  c.run();
}
