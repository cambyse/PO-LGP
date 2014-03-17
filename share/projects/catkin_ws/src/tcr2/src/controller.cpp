#include "controller.h"
#include "util.h"

#include "tcr2/SetGoal.h"

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
  G.physx();
}

void Controller::step() {
  ors::Body *robot = G.getBodyByName(endeff);
  ors::Vector pos = robot->X.pos;
  ors::Vector vel = robot->X.vel;

  ors::Vector dPos = goal - pos;
  ors::Vector dVel = - vel;
  integrate += dPos;
  
  ors::Vector force = Kp * dPos + Kd * dVel + Ki * integrate;

  G.addForce(force, robot);
}

bool Controller::set_goal(tcr2::SetGoal::Request &req,
                          tcr2::SetGoal::Response &res) {
  goal = ros_to_ors_vector(req.pose.position);
  double eps = 10e-5;
  while ((G.getBodyByName(endeff)->X.pos - goal).length() > eps &&
         G.getBodyByName(endeff)->X.vel.length() > eps) {
    ros::Duration(.5).sleep();
  }
}

void Controller::run() {
  goal = G.getBodyByName(endeff)->X.pos;
  while (ros::ok()) {
    this->step(); 
    G.physx().step();
    gl.update();
    ros::spinOnce();
  }
    
}
int main(int argc, char** argv) {
  MT::initCmdLine(argc, argv);
  ros::init(argc, argv, "Controller");

  ros::NodeHandle n;
  Controller c(n, "robot");

  ros::ServiceServer service = n.advertiseService("set_goal", &Controller::set_goal, &c);

  c.run();
}
