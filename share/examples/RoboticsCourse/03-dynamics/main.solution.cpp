#include <stdlib.h>
#include <Ors/roboticsCourse.h>
#include <Gui/plot.h>

void TestPD(double xi, double waveLength){
  double mass = 1.0;
  double tau = 0.01;
  int T = 500;
  double Target = 1;
  arr x(T),v(T),a(T);
  double u;

  double lambda = waveLength/MT_2PI;
  double Kp = 1./(lambda*lambda);
  double Kd = 2.*xi/lambda;
  
  for (int i = 0; i < T-1; i++){
    //control
    u  = Kp*(Target - x(i)) - Kd*v(i);

    //simulate
    a(i+1) = u/mass;
    v(i+1) = v(i) + a(i+1)*tau;
    x(i+1) = x(i) + v(i+1)*tau;
  }
  gnuplot(x);
  MT::wait();
}


//for 500 steps gravity effects, than control back to home position
void testDynamicSimulation(){
  Simulator S("../02-pegInAHole/pegInAHole.ors");
  S.setDynamicSimulationNoise(2.);
  S.setDynamicGravity(true);
  
  arr q,qdot,qddot;
  arr M,Minv,F,u;
  S.getJointAngles(q);
  q(2) = 1.;
  S.setJointAngles(q);
  qdot.resizeAs(q);
  qdot = 0.;
  u=qdot;
  
  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  S.watch();        //pause and watch initial posture
  
  double tau = .01; //duration of one time step = .01sec
  bool control = 1;
  
  for(uint i=0;i<1000;i++){
    //** CONTROLLER part
    //get system equation
    S.getDynamics(M,F);
    if(!control){
      //pure friction
      u = -.1 * qdot; //pure friction
      if(i > 500)control = 1;
    }else{
      //separate PDs in each joint separately:
      double lambda = .5/MT_2PI; //corresponds to 1 second period
      double xi = .2;
      double Kp,Kd;
      Kp = 1./(lambda*lambda);
      Kd = 2.*xi/lambda;
      arr qddot_desired = Kp * (0. - q) + Kd * (0. - qdot);
      //compute coordinated u so that each joint behaves like a PD:
      //(the u given above is now the desired qddot; see slide 21)
      u = M*qddot_desired + F;
    }

    S.stepDynamic(u, tau);
    S.getJointAnglesAndVels(q, qdot);
    //output
    cout <<" t = " <<.01*i
    <<" E = " <<S.getEnergy()
    <<" q = " <<q <<endl;
  }
}


void sineProfile(arr& q, const arr& q0, const arr& qT,uint T){
  q.resize(T+1,q0.N);
  for(uint t=0; t<=T; t++) q[t] = q0 + .5 * (1.-cos(MT_PI*t/T)) * (qT-q0);
}

void getVel(arr& v, const arr& q, double tau){
  uint T=q.d0-1;
  v.resizeAs(q);
  for(uint t=0; t<T; t++)  v[t] = (q[t+1] - q[t])/tau;
  v[T] = 0.;
}
  
void getAcc(arr& a, const arr& q, double tau){
  uint T=q.d0-1;
  a.resizeAs(q);
  a[0] = 0.;
  for(uint t=1; t<T; t++)  a[t] = (q[t+1] + q[t-1] - 2.*q[t])/(tau*tau);
  a[T] = 0.;
}


void followReferenceTrajectory(){
  Simulator S("../02-pegInAHole/pegInAHole.ors");
  S.setDynamicSimulationNoise(.0);
  S.setDynamicGravity(true);

  double tau=.01;
  uint T=100; //we simulate for 5 seconds

  //-- compute a stupid reference trajectory
  arr q0(7); q0.setZero();
  arr qT = ARRAY(0.945499, 0.431195, -1.97155, 0.623969, 2.22355, -0.665206, -1.48356);
  arr q_ref,v_ref,a_ref;
  sineProfile(q_ref, q0, qT, T);
  getVel(v_ref,q_ref,tau);
  getAcc(a_ref,q_ref,tau);

  arr q,qdot; //current state
  arr M,F,u;
  S.getJointAnglesAndVels(q, qdot);
  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  //initial perturbation
  q(2) = .3;
  S.setJointAnglesAndVels(q, qdot);

  S.watch();        //pause and watch initial posture

  arr plot;
  
  for(uint t=0;t<=T;t++){
    //get system equation
    S.getDynamics(M,F);

    //PD parameters
    double lambda = .2/MT_2PI; //corresponds to 1 second wavelength
    double xi = .2; //oscillatory or critically damped
    double Kp,Kd;
    Kp = 1./(lambda*lambda);
    Kd = 2.*xi/lambda;

    //follow trajectory (slide 05:21)
    arr qddot = a_ref[t] + Kp * (q_ref[t] - q) + Kd * (v_ref[t] - qdot);
    arr u = M*qddot + F;

    //PD heuristic, hardly works
    //u = 1e1 * (q_ref[t] - q) - 1e+0 * qdot;

    //dynamic simulation (simple Euler integration of the system dynamics, look into the code)
    S.stepDynamic(u, tau);
    S.getJointAnglesAndVels(q, qdot);

    cout  <<" t = " <<tau*t  <<"sec   q = " <<q <<endl;
    plot.append(q(2));
  }
  gnuplot(plot);
  S.watch();
}

int main(int argc,char **argv){
  int mode=5;
  if(argc>1) mode = atoi(argv[1]);
  switch(mode){
    case 1:  TestPD(1., .5);  break;
    case 2:  TestPD(5., .5);  break;
    case 3:  TestPD(.1, .5);  break;
    case 4:  testDynamicSimulation();  break;
    case 5:  followReferenceTrajectory();  break;
  }
  if(true)
  
  
  return 0;
}
