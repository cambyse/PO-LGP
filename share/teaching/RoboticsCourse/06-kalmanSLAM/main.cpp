#include <stdlib.h>
#include <Ors/roboticsCourse.h>
#include <Gui/opengl.h>
//#include <MT/gauss.h>
#include <Gui/plot.h>


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
void Kalman(){
  double theta = 0.3;
  
  CarSimulator Sim;
  Sim.gl->watch();
  
  arr u(2),y_meassured;
  
  arr s(3),S(3,3); //kalman estimates
  s(0) = Sim.x; s(1) = Sim.y; s(2) = Sim.theta;//initial at true state..
  S.setDiag(0.1);//which nosy constant
  arr shat,Shat,dW;
  
  arr W(4,4);  W.setDiag(Sim.observationNoise);
  arr Q(3,3);  Q.setDiag(Sim.dynamicsNoise);
  
  arr A(3,3),a;
  A.setDiag(1);
  
  for(uint t=0;t<10000;t++){
    u = ARR(.1, .2); //control signal
    Sim.step(u);
    Sim.getRealNoisyObservation(y_meassured);
    
    //get the linear observation model
    arr C,c,y_mean,Cinv;
    Sim.getObservationJacobianAtState(C, s);
    Sim.getMeanObservationAtState(y_mean, s);
    c = y_mean - C*s;
    inverse(Cinv,C);

    arr B = GetJacoB(Sim, u, s);
    a = B*u;
    
    //Kalman filter
    arr Shat = Q + A*S*~A;     //term from integral
    arr shat = a + A*s;
    arr What = Cinv*W*~Cinv;   //term from observation
    arr WSinv = inverse(What + Shat);
    s = Shat*WSinv*Cinv*(y_meassured-c) + What*WSinv*shat;
    S = What*WSinv*Shat;
    //s = shat;
    //S = Shat;
    
    Sim.gaussiansToDraw.resize(1);
    Sim.gaussiansToDraw(0).A=S.sub(0,1,0,1);
    Sim.gaussiansToDraw(0).a=s.sub(0,1);
    //sanity check of the linear observation model -- don't use this code in the solution!
    cout << "estim error " <<maxDiff(s, ARR(Sim.x, Sim.y, Sim.theta)) << endl;
  }
}



int main(int argc,char **argv){
  Kalman();
  return 0;
}
