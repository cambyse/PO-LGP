#include <Ors/roboticsCourse.h>
#include <Core/array.h>

void TEST(Dynamics){
  Simulator S("pin_balancing.ors");
  
  arr q,qdot,qddot;
  arr M,Minv,F,u;
  S.getJointAngles(q);
  qdot.resizeAs(q);
  qdot.setZero();
  u=qdot;
  double tau = .01; //duration of one time step = .01sec
  
  arr Kp,Kd;
  Kp = eye(q.d0)*100.;
  Kd = eye(qdot.d0)*10.;

  arr qRef = ARR(3.14/2.,3.14,3.14,0.0,0.0,0.0,0.0,0.0);

  for(uint i=0;i<1000;i++){
    mlr::wait(0.01);
    u = -(Kp*(q-qRef) + Kd*qdot);
    //u = -0.1 * qdot; //a little friction
    S.stepDynamics(u, tau);
    S.watch(false);
    S.getJointAnglesAndVels(q, qdot);
  }
  
  S.watch();
}

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  testDynamics();
  return 0;
}
