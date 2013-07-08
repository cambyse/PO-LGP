#include <Core/array.h>
#include <Gui/plot.h>
#include <Core/util.h>

void generateData(){
  arr X;
  arr x;
  arr A,a;
  double tau=0.02;
  A=ARR(1.,tau,0.,1.); A.reshape(2,2);
  a=ARR(0,-tau);
  x=ARR(0.,1.);
  
  for(uint t=0;t<100;t++){
    x = A*x + a;
    rndGauss(x,.01,true); //transition noise
    X.append(x);
  }
  X.reshape(X.N/2,2);
  
  ofstream fil1("dataReal");
  X.write(fil1," ","\n"," \n");
  fil1.close();

  rndGauss(X,0.1,true); //observation noise
  
  ofstream fil("data");
  X.write(fil," ","\n"," \n");
  fil.close();

  //gnuplot("plot 'data' us 0:1");  MT::wait();
}


void kalmanFilter(){
  //--load the data
  arr X;
  ifstream fil("data");
  X.read(fil);
  fil.close();
  uint T=X.d0, n=X.d1;

  //-- setup system matrices
  arr A,a,C,Cinv,c,W,Q;
  double tau=0.02;
  A=ARR(1.,tau,0.,1.); A.reshape(2,2); //transition matrix
  a=ARR(0,-tau);      //transition offset
  Q.setDiag(1e-4,n);  //transition covariance
  C.setId(n);         //observation matrix
  c=ARR(0,0);         //observation offset
  W.setDiag(1e-2,n);  //observation covariance
  Cinv = inverse(C);

  //-- setup filter
  arr Xfilter(T,n),s,S;
  s = ARR(0.,1.);     //initial guess
  S.setDiag(1e-4,n);  //initial variance
  Xfilter[0] = s;     //init

  //-- Kalman filter
  for(uint t=1;t<100;t++){
    arr What = Cinv*W*~Cinv;   //term from observation
    arr Shat = Q + A*S*~A;     //term from integral
    arr WSinv = inverse(What + Shat);
    s = Shat*WSinv*Cinv*(X[t]-c) + What*WSinv*(a + A*s);
    S = What*WSinv*Shat;
    Xfilter[t] = s;
  }

  //-- save and display
  ofstream fil2("dataFilter");
  Xfilter.write(fil2," ","\n"," \n");
  fil2.close();
  gnuplot("plot 'dataFilter' us 0:1,'data' us 0:1,'dataReal' us 0:1");
  MT::wait();
}

int main(int argc, char** argv){
  generateData();
  kalmanFilter();
  return 0;
}
