#include "RTControllerSimulation.h"
#include <Motion/taskMaps.h>

void RTControllerSimulation::open() {
  world = new ors::KinematicWorld(modelWorld.get());
  arr q, qDot;
  world->getJointState(q,qDot);
  this->ctrl_obs.writeAccess();
  this->ctrl_obs().q = q;
  this->ctrl_obs().qdot = qDot;
  this->ctrl_obs().fL = zeros(6);
  this->ctrl_obs().fR = zeros(6);
  this->ctrl_obs().u_bias = zeros(q.d0);
  this->ctrl_obs.deAccess();
//  mlr::wait(1.0);
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
  cout << "%        start dynamic simulation        %" << endl;
  cout << "%             and LOOOOOOP!              %" << endl;
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
}


void RTControllerSimulation::step() {
  this->ctrl_ref.readAccess();
  arr Kd = this->ctrl_ref().Kd;
  arr Kp = this->ctrl_ref().Kp;
  arr u0 = this->ctrl_ref().u_bias;
  arr qRef = this->ctrl_ref().q;
  arr qDotRef = this->ctrl_ref().qdot;
//  J_ft_inv = this->ctrl_ref().J_ft_inv;
//  double gamma = this->ctrl_ref().gamma;
//  fLRef = this->ctrl_ref().fL;
//  KiFt = this->ctrl_ref().KiFT;
  this->ctrl_ref.deAccess();

  arr u;
  arr q, qDot;
  world->getJointState(q, qDot);

  if(qRef.N==q.N){ //TODO: use exactly same conditions as in RT controller
    u = u0 + Kp*(qRef - q) + Kd*(qDotRef - qDot);

    //TODO: the real RT controller does a lot more: checks ctrl limits, etc. This should be simulated as well

    world->stepDynamics(u, tau, 0.0, this->gravity);
    world->getJointState(q,qDot);
  }

  this->ctrl_obs.writeAccess();
  this->ctrl_obs().q = q;
  this->ctrl_obs().qdot = qDot;
  this->ctrl_obs().u_bias = u;
  this->ctrl_obs.deAccess();

  //world->watch(false);
  mlr::wait(tau); //TODO: why does this change something??? FISHY!
}



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
