#include "pr2DynamicSimulation.h"

void DynamicSimulation::setWorld(ors::KinematicWorld *world) {
  this->world = world;
}

void DynamicSimulation::setGravity(bool gravity) {
  this->gravity = gravity;
}

void DynamicSimulation::startSimulation(bool start) {
  this->threadLoop();
  mlr::wait(1.0);
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
  cout << "%        start dynamic simulation        %" << endl;
  cout << "%             and LOOOOOOP!              %" << endl;
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
}

void DynamicSimulation::initializeSimulation(ors::KinematicWorld *world, bool gravity) {
  this->world = world;
  this->gravity = gravity;
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
  cout << "%        init dynamic simulation         %" << endl;
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
  world->meldFixedJoints();
  //arr qStart = ARR(0.0,0.0,0.0,-0.3,0.0,-0.3,0.0);
  world->setJointState(this->world->getJointState(), zeros(this->world->getJointStateDimension()));
  //world->watch(true, "Simulation");
  arr q, qDot;
  world->getJointState(q,qDot);
  this->ctrl_obs.set()->q = q;
  this->ctrl_obs.set()->qdot = qDot;
  this->ctrl_obs.set()->fL = zeros(6);
  this->ctrl_obs.set()->fR = zeros(6);

  this->ctrl_ref.set()->Kp = zeros(world->getJointStateDimension(),world->getJointStateDimension());
  this->ctrl_ref.set()->Kd = zeros(world->getJointStateDimension(),world->getJointStateDimension());
  this->ctrl_ref.set()->u_bias = zeros(world->getJointStateDimension());
  this->ctrl_ref.set()->q = zeros(world->getJointStateDimension());
  this->ctrl_ref.set()->qdot = zeros(world->getJointStateDimension());
}


void DynamicSimulation::step() {
  arr Kp, Kd, u0, qRef, qDotRef;

  //this->ctrl_ref.readAccess();
  //this->ctrl_ref.waitForNextRevision();
  Kd = this->ctrl_ref.get()->Kd;
  Kp = this->ctrl_ref.get()->Kp;
  u0 = this->ctrl_ref.get()->u_bias;
  qRef = this->ctrl_ref.get()->q;
  qDotRef = this->ctrl_ref.get()->qdot;
  //this->ctrl_ref.deAccess();

  arr u;
  arr q, qDot;
  world->getJointState(q, qDot);

  u = u0 + Kp*(qRef - q) + Kd*(qDotRef - qDot);
  double tau = 0.01;

  //world->setJointState(q, qDot);
  world->stepDynamics(u, tau, 0.0, this->gravity);
  world->getJointState(q,qDot);

  //this->ctrl_obs.writeAccess();
  this->ctrl_obs.set()->q = q;
  this->ctrl_obs.set()->qdot = qDot;
  //this->ctrl_obs.deAccess();

  //world->watch(false);
  mlr::wait(tau);
}

REGISTER_MODULE(DynamicSimulation)

