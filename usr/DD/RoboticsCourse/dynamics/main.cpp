#include <stdlib.h>
#include <Ors/roboticsCourse.h>

void holdSteady(uint mode){
  Simulator S("pegArm.ors");
  S.setDynamicSimulationNoise(.0);
  S.setDynamicGravity(true);
  
  arr q,qDot;
  arr M,F,u;
  S.getJointAngles(q);
  q(2) = 1.;
  qDot.resizeAs(q);
  qDot = 0.;
  S.setJointAnglesAndVels(q,qDot);
  u.resizeAs(q);
  u = 0.;

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  S.watch();        //pause and watch initial posture
  
  double tau = .01; //duration of one time step = .01sec
  uint T=1000;

  arr integral;
  integral = zeros(q.d0);

  arr qRef = zeros(q.d0);
  arr qDotRef = zeros(q.d0);

  arr Kp,Kd,Ki;

  for(uint t=0;t<=T;t++){
    mlr::wait(0.01);
    S.getDynamics(M,F);

    //-- implementation of the controller --//
    if(mode == 1) {
      Kp = eye(q.d0)*100.0;
      Kd = eye(q.d0)*10.0;
      Kp(0,0) = 800.0;
      Kp(1,1) = 800.0;
      Kp(2,2) = 800.0;
      Kd(0,0) = 200.0;
      Kd(0,0) = 200.0;
      u = Kp*(qRef-q) + Kd*(qDotRef-qDot);
    }

    if(mode == 2) {
      integral = integral + tau*(qRef-q);
      Kp = eye(q.d0)*100.0;
      Kd = eye(q.d0)*10.0;
      Kp(0,0) = 1300.0;
      Kp(1,1) = 1300.0;
      Kp(2,2) = 1000.0;
      Kd(0,0) = 200.0;
      Kd(0,0) = 200.0;
      Ki = eye(q.d0)*1000.0;
      u = Kp*(qRef-q) + Kd*(qDotRef-qDot) + Ki*integral;
    }

    if(mode == 3) {
      Kp = eye(q.d0)*100.0;
      Kd = eye(q.d0)*20.0;
      u = M*(Kp*(qRef-q) + Kd*(qDotRef-qDot)) + F;
    }
    // ....



    //--------------------------------------//

    //dynamic simulation (simple Euler integration of the system dynamics, look into the code)
    S.stepDynamics(u, tau);
    S.watch(false);
    S.getJointAnglesAndVels(q, qDot);

    cout  <<" t=" <<tau*t  <<"sec E=" <<S.getEnergy()  <<"  q = " <<q <<endl;
  }
}

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  holdSteady(3);
  return 0;
}
