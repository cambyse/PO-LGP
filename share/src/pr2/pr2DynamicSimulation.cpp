#include "pr2DynamicSimulation.h"
#include <Motion/taskMaps.h>

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
  //world->meldFixedJoints();
  //arr qStart = ARR(0.0,0.0,0.0,-0.3,0.0,-0.3,0.0);
  world->setJointState(this->world->getJointState(), zeros(this->world->getJointStateDimension()));
  //world->watch(true, "Simulation");
  arr q, qDot;
  world->getJointState(q,qDot);
  //this->ctrl_obs.writeAccess();
  this->ctrl_obs.set()->q = q;
  this->ctrl_obs.set()->qdot = qDot;
  this->ctrl_obs.set()->fL = zeros(6);
  this->ctrl_obs.set()->fR = zeros(6);
  this->ctrl_obs.set()->u_bias = zeros(q.d0);
//this->ctrl_obs.deAccess();
  //this->ctrl_ref.readAccess();
  this->ctrl_ref.set()->Kp = zeros(world->getJointStateDimension(),world->getJointStateDimension());
  this->ctrl_ref.set()->Kd = zeros(world->getJointStateDimension(),world->getJointStateDimension());
  this->ctrl_ref.set()->u_bias = zeros(world->getJointStateDimension());
  this->ctrl_ref.set()->q = zeros(world->getJointStateDimension());
  this->ctrl_ref.set()->qdot = zeros(world->getJointStateDimension());
//this->ctrl_ref.deAccess();
  this->ftErrInt = zeros(1);
}


void DynamicSimulation::step() {
  arr Kp, Kd, u0, qRef, qDotRef, J_ft_inv, fLObs, fLRef, KiFt;

  //this->ctrl_ref.readAccess();
  //this->ctrl_ref.waitForNextRevision();
  Kd = this->ctrl_ref.get()->Kd;
  Kp = this->ctrl_ref.get()->Kp;
  u0 = this->ctrl_ref.get()->u_bias;
  qRef = this->ctrl_ref.get()->q;
  qDotRef = this->ctrl_ref.get()->qdot;

  J_ft_inv = this->ctrl_ref.get()->J_ft_inv;
  double gamma = this->ctrl_ref.get()->gamma;
  fLRef = this->ctrl_ref.get()->fL;
  KiFt = this->ctrl_ref.get()->KiFT;
  //this->ctrl_ref.deAccess();

  //cout << u0 << endl;

  arr u;
  arr q, qDot;
  world->getJointState(q, qDot);

  u = u0 + Kp*(qRef - q) + Kd*(qDotRef - qDot);

#if 0
  DefaultTaskMap t(posTMT, *this->world, "endeffL");
  arr y, J;
  t.phi(y, J, *this->world);
  double height = 0.6;
  arr f = zeros(3);
  double D = 100.0;
  if(y(2) < height) {
    f(2) = D*(height - y(2));
    u += ~J*f;
  }

  ors::Shape *ftShape = this->world->getShapeByName("endeffForceL");
  arr J_ft;
  this->world->kinematicsPos_wrtFrame(NoArr, J_ft, ftShape->body, ftShape->rel.pos, this->world->getShapeByName("l_ft_sensor"));
  fLObs = -~J_ft*f;
  this->ctrl_obs.set()->fL = fLObs;

  if(KiFt.N) {
    this->ftErrInt *= gamma;
    arr fEndEff = J_ft_inv*fLObs;
    /*
    for(uint i = 0; i < fEndEff.N; i++) {
      if(fLRef(i) < 0) {
        if(fEndEff(i) < fLRef(i)) {
          cout << "fEndEff(i) < fLRef(i)" << endl;
          this->ftErrInt(i) += fLRef(i) - fEndEff(i);
        }
      } else {
        if(fEndEff(i) > fLRef(i)) {
          //cout << "fEndEff(i) > fLRef(i)" << endl;
          //cout << -(fLRef(i) - fEndEff(i)) << endl;
          this->ftErrInt(i) += fEndEff(i) - fLRef(i);
          u += KiFt * this->ftErrInt;
        }
      }
    }*/

    for(uint i = 0; i < fEndEff.N; i++) {
      this->ftErrInt(i) = 0.0;
      if(fEndEff(i) < fLRef(i)) {
        this->ftErrInt(i) = fLRef(i) - fEndEff(i);
      }
    }

    u += KiFt * this->ftErrInt;
    //cout << this->ftErrInt << endl;
  }
#endif

  double tau = 0.01;

  //world->setJointState(q, qDot);
  world->stepDynamics(u, tau, 0.0, this->gravity);
  world->getJointState(q,qDot);

  //this->ctrl_obs.writeAccess();
  this->ctrl_obs.set()->q = q;
  this->ctrl_obs.set()->qdot = qDot;
  this->ctrl_obs.set()->u_bias = u;
  //this->ctrl_obs.deAccess();

  //world->watch(false);
  mlr::wait(tau);
}

REGISTER_MODULE(DynamicSimulation)

