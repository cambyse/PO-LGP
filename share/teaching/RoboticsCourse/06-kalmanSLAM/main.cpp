#include <stdlib.h>
#include <Ors/roboticsCourse.h>
#include <Gui/opengl.h>
#include <Gui/plot.h>
#include <Algo/kalman.h>


//car state change with respect to control
arr GetJacoB(CarSimulator & S, const arr & u,const arr & x){
  arr J(3,2);
  J(0,0) = cos(x(2));
  J(1,0) = sin(x(2));
  J(2,0) = tan(u(1))/S.L;
  J(0,1) = 0.;
  J(1,1) = 0.;
  J(2,1) = u(0)/S.L*(1./(cos(u(1))*cos(u(1))));
  J *= S.tau;
  return J;
}

//ok, works
void testKalman(){
  double theta = 0.3;
  
  CarSimulator Sim;
  Sim.gl->watch();
  
  arr u(2),y_meassured;

  Kalman K;
  K.initialize(ARR(Sim.x, Sim.y, Sim.theta), 0.1*eye(3));
  
  arr W(4,4);  W.setDiag(Sim.observationNoise);
  arr Q(3,3);  Q.setDiag(Sim.dynamicsNoise);
  
  arr A(3,3),a;
  A.setDiag(1);
  
  for(uint t=0;t<10000;t++){
    u = ARR(.1, .2); //control signal
    Sim.step(u);
    Sim.getRealNoisyObservation(y_meassured);
    
    //get the linear observation model
    arr C,c,y_mean;
    Sim.getObservationJacobianAtState(C, K.b_mean);
    Sim.getMeanObservationAtState(y_mean, K.b_mean);
    c = y_mean - C*K.b_mean;

    arr B = GetJacoB(Sim, u, K.b_mean);
    a = B*u;
    
    //Kalman filter
    K.step(A,a,Q,y_meassured,C,c,W);
    
    Sim.gaussiansToDraw.resize(1);
    Sim.gaussiansToDraw(0).A=K.b_var.sub(0,1,0,1);
    Sim.gaussiansToDraw(0).a=K.b_mean.sub(0,1);
    //sanity check of the linear observation model -- don't use this code in the solution!
    cout << "estim error " <<maxDiff(K.b_mean, ARR(Sim.x, Sim.y, Sim.theta)) << endl;
  }
}



int main(int argc,char **argv){
  testKalman();
  return 0;
}
