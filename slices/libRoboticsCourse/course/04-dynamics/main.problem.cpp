#include <stdlib.h>
#include <MT/roboticsCourse.h>
#include <MT/plot.h>

void followReferenceTrajectory(){
  Simulator S("../02-pinInAHole/pin_in_a_hole.ors");
  S.setDynamicSimulationNoise(0.);
  S.setDynamicGravity(true);

  double tau=.01;
  uint T=500;
  
  arr q,qdot;
  arr M,F,u(7);
  
  S.getJointAnglesAndVels(q, qdot);
  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  S.watch();
  
  for(uint t=0;t<=T;t++){
    //get system equation
    S.getDynamics(M,F);

    //no controller torques
    u = 0.;

    //dynamic simulation (simple Euler integration of the system dynamics, look into the code)
    S.stepDynamic(u, tau);
    S.getJointAnglesAndVels(q, qdot);

    cout  <<" t=" <<tau*t  <<"sec E=" <<S.getEnergy()  <<"  q = " <<q <<endl;
  }
  S.watch();
}

int main(int argc,char **argv){

  followReferenceTrajectory();
  
  return 0;
}
