#include <stdlib.h>
#include <Ors/roboticsCourse.h>
#include <Gui/opengl.h>
#include <Algo/kalman.h>

#include "racer.h"

void testDraw(){
  Racer s;
  s.gl.watch();
}

void TestMove(){
  Racer R;
  for (uint t=0; t<400000; t++){
    R.stepDynamics(0.0);
    R.gl.text.clear() <<t <<" ; " <<R.q(0) << " ; " <<R.q(1);
    R.gl.update();
    R.getEnergy();
  }
}

void CheckGradients(){
  Racer R;
  for (uint t=0; t<20; t++){
    rndUniform(R.q, -.1, .1);
    rndUniform(R.q_dot, -.1, 1.);
    R.u = 1.; //rnd.uni(-.1,.1);
    checkJacobian(R, cat(R.q, R.q_dot).reshape(2,2), 1e-4);
  }
}

void TestControl(){
  ofstream data("z.data");
  data <<"iteration time control x th x_dot th_dot x_ddot th_ddot y0 y1 y2 E(x) E(th) E(x_dot) E(th_dot) V(x) V(th) V(x_dot) V(th_dot)" <<endl;
  Racer R;

  arr A,a,B;
  R.q.setZero();
  R.getDynamicsAB(A,a,B);
  cout <<"*** ZERO LQG: \nA=" <<A <<"\na=" <<a <<"\nB=" <<B <<endl;

  R.q(1)=.5;
  R.noise_dynamics = 1.;

  Kalman K;
  K.initialize(cat(R.q, R.q_dot),1.*eye(4)); //correct initialization...

  for (uint t=0; t<1000; t++){
#if 0 //heuristic
    double th_ref = -0.1*R.q(0)-0.1*R.q_dot(0);
    double u = 1.*(R.q(1)-th_ref) + .1*R.q_dot(1);
#else //CARE
//    arr K = ARR(0.31623,   1.99224,   0.47460,   0.42549); //medium u_costs
    arr k = ARR(0.10000,   0.90218,   0.15766,   0.15744); //strong u_costs
//    double u = scalarProduct(k, cat(R.q, R.q_dot));
    double u = scalarProduct(k, K.b_mean);
#endif

    //-- dynamics
    arr A,a,B;
    R.getDynamicsAB(A,a,B);

    R.stepDynamics(u);
    R.gl.text.clear() <<t <<" ; " <<R.q(0) << " ; " <<R.q(1);
    R.gl.update();

    //-- observations
    arr y,C,c,W;
    R.getObservation(y, C, c, W);
    rndGauss(y, 1., true);

    //-- state estimation
    K.stepPredict(eye(4)+R.tau*A, R.tau*(a+B*ARR(u)), .0001*eye(4));
    K.stepObserve(y, C, c, 1.*eye(3));

//    checkJacobian(R.getObs(), cat(R.q, R.q_dot).reshape(2,2), 1e-4);

    MT::arrayBrackets="  ";
    data <<t <<' ' <<t*R.tau <<' ' <<R.u <<' '
        <<R.q <<R.q_dot <<R.q_ddot
       <<y
      <<K.b_mean <<K.b_mean+2.*::sqrt(getDiag(K.b_var)) <<endl;

  }
}

int main(int argc,char **argv){
//  testDraw();
//  TestMove();
//  CheckGradients();
  TestControl();

  return 0;
}
