#include <stdlib.h>
#include <Ors/roboticsCourse.h>
#include <Gui/opengl.h>
#include <Algo/kalman.h>
#include <Motion/motion.h>

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
  arr x(2,2);
  for (uint t=0; t<20; t++){
    rndUniform(x, -.1, .1);
    R.u = 1.; //rnd.uni(-.1,.1);
    checkJacobian(R.dynamicsFct(), x, 1e-4);
    checkJacobian(R.observationFct(), x, 1e-4);
  }
}

void TestControl(){
  ofstream data("z.data");
  data <<"iteration time u u_acc x th x_dot th_dot x_ddot th_ddot B_q B_qdot B_qddot y0 y1 y2 y4 E(x) E(th) E(x_dot) E(th_dot) V(x) V(th) V(x_dot) V(th_dot)" <<endl;
  Racer R;

  arr A,a,B;
  R.q.setZero();
  R.getDynamicsAB(A,a,B);
  cout <<"*** ZERO LQG: \nA=" <<A <<"\na=" <<a <<"\nB=" <<B <<endl;

  R.q(1)=.5;
  R.noise_dynamics = 0;//.1;

  Kalman K;
  K.initialize(cat(R.q, R.q_dot),1.*eye(4)); //correct initialization...

  for (uint t=0; t<1000; t++){
    double x_ref=(t/200)&1;
#if 0 //heuristic
    double th_ref = -0.1*(R.q(0)-x_ref)-0.1*R.q_dot(0);
    double u = 1.*(R.q(1)-th_ref) + .1*R.q_dot(1);
    double u_acc = 200.*(R.q(1)-th_ref) + .1*R.q_dot(1);
#else //CARE1
    arr k = ARR(3.1623,   8.6762,   2.6463,   2.0126);
    arr x = cat(R.q, R.q_dot);
    x = K.b_mean;
    x(0) -= x_ref;
    double u = scalarProduct(k, x);
    double u_acc = 0.;
#endif

    //-- get dynamics (for later filtering)
    arr A,a,B;
    R.getDynamicsAB(A,a,B);

    //-- dynamics
    R.stepDynamics(u);
    R.gl.text.clear() <<t <<" ; " <<R.q(0) << " ; " <<R.q(1);
    R.gl.update();

    //-- observations
    arr y,C,c,W;
    R.getObservation(y, C, c, W);
    y(0) += R.noise_accel*rnd.gauss();
    y(1) += R.noise_accel*rnd.gauss();
    y(2) += R.noise_gyro*rnd.gauss();
    y(3) += R.noise_enc*rnd.gauss();

    //-- state estimation
    K.stepPredict(eye(4)+R.tau*A, R.tau*(a+B*ARR(u)),  diag(ARR(1e-6, 1e-6, 1., 1.)));
    K.stepObserve(y, C, c, W);

    MT::arrayBrackets="  ";
    data <<t <<' ' <<t*R.tau <<' ' <<R.u <<' ' <<u_acc <<' '
        <<R.q <<R.q_dot <<R.q_ddot
       <<~R.B * R.q <<' ' <<~R.B * R.q_dot <<' ' <<~R.B * R.q_ddot <<' '
       <<y
      <<K.b_mean <<K.b_mean+2.*::sqrt(getDiag(K.b_var)) <<endl;
  }
}

void FollowSignal(){
  ofstream data("z.data");
  data <<"iteration time u u_acc x th x_dot th_dot x_ddot th_ddot B_q B_qdot B_qddot ref ref_v y0 y1 y2 y4 E(x) E(th) E(x_dot) E(th_dot) V(x) V(th) V(x_dot) V(th_dot)" <<endl;
  Racer R;
  arr ctrl,ctrl_v;
  MT::load(ctrl,"ctrl_ref");
  ctrl = (~ctrl)[1];
  ctrl.reshape(ctrl.N,1);
  ctrl *= .001;
  getVel(ctrl_v, ctrl, 0.03);
  ctrl.reshape(ctrl.N);
  ctrl_v.reshape(ctrl.N);

  R.noise_dynamics = 0;//.1;
  R.tau = 0.03;

  arr B = ARR(1./R.r, -1.);

  for (uint t=0; t<ctrl.N; t++){
    double u_acc = 1000.*(ctrl(t) - (~B*R.q).scalar()) + 50.*(ctrl_v(t) - (~B*R.q_dot).scalar());

    //-- dynamics
    R.stepDynamicsAcc(u_acc);
    R.gl.text.clear() <<t <<" ; " <<R.q(0) << " ; " <<R.q(1);
    R.gl.update();

    MT::arrayBrackets="  ";
    data <<t <<' ' <<t*R.tau <<' ' <<R.u <<' ' <<u_acc <<' '
        <<R.q <<R.q_dot <<R.q_ddot
       <<~R.B * R.q <<' ' <<~R.B * R.q_dot <<' ' <<~R.B * R.q_ddot <<' '
      <<ctrl(t) <<' ' <<ctrl_v(t) <<' '
     <<endl;
  }
}

void FollowIMU(){
  ofstream data("z.data");
  data <<"iteration time E(x) E(th) E(x_dot) E(th_dot) sin(E_th) cos(E_th) Y0 Y1 Y2 Y4 y0 y1 y2 y4" <<endl;

  Racer R;

  arr imu;
  MT::load(imu,"nogit-data/imu-02.dat");
  uint T=imu.d0;
  imu = ~imu;
  arr times;
  times = imu[0];
  imu[0] = imu[1]*(1./(1<<14));
  imu[1] = imu[3]*(1./(1<<14));
  imu[2] = imu[5]*(1./(1<<13));
  imu[3] = 0.;

//  R.c4 = sum(imu.sub(2,2,0,199))/200.;
//  cout <<"gyro_off = c4 = " <<R.c4 <<endl;

  //for(uint t=1;t<imu.d1;t++) imu(3,t) = imu(3,t-1) + (times(t)-times(t-1))*imu(2,t-1);

  imu.resizeCopy(4, imu.d1);
  imu = ~imu;
  MT::arrayBrackets="  ";
  imu >>FILE("02-imu.dat");
  ~(~times) >>FILE("02-times.dat");

  R.q(1)=MT_PI/2.;

  Kalman K;
  K.initialize(cat(R.q, R.q_dot),1.*eye(4)); //correct initialization...
  for (uint t=1; t<T; t++){
    //-- state estimation
    arr y_pred, C, c, W;
    R.getObservation(y_pred, C, c, W);
    arr y_true = imu[t];

    double tau=times(t)-times(t-1);
    arr K_old = K.b_mean;

    arr A,a,B;
    R.getDynamicsAB(A,a,B);
    K.stepPredict(eye(4)+tau*A, tau*a, diag(ARR(1e-6, 1e-6, 1., 1.)));
    K.stepObserve(y_true, C, c, W);

    R.q = K.b_mean.sub(0,1);
    R.q_dot = K.b_mean.sub(2,3);
    R.gl.text.clear() <<t <<' ' <<times(t) <<" ; " <<R.q(0) << " ; " <<R.q(1);
    if(!(t%10)) R.gl.update();

    MT::arrayBrackets="  ";

    data <<t <<' ' <<times(t) <<' '
        <<K_old <<' ' <<sin(K_old(1)) <<' ' <<cos(K_old(1)) <<' '
       <<y_pred
      <<y_true
     <<endl;
  }
}

int main(int argc,char **argv){
//  testDraw();
//  TestMove();
//  CheckGradients();
//  TestControl();

//  FollowSignal();
  FollowIMU();

  return 0;
}
