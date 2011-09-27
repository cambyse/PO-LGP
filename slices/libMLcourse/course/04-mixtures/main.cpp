#include<MT/array.h>
#include<MT/util.h>
#include<MT/plot.h>

using namespace std;

#include <MT/MLcourse.h>
#include <MT/ann.h>
#include <MT/util.h>
#include <MT/plot.h>


void generateArtificialGauss(){
  arr X(1000,2);
  rndGauss(X);
  arr V = ARR(0.70711, 0.70711, -.70711, 0.70711);  V.reshape(2,2);
  arr D = ARR( .5, 0, 0, 2); D.reshape(2,2);
  X = X * D * ~V;
  arr mu=ARR(3,5);
  X += ones(X.d0,1)*~mu;
  MT::save(X,"gauss.txt");
}

void generateArtificialMixData(){
  arr X,y;
  rnd.clockSeed();
  artificialData_GaussianMixture(X, y);
  MT::save(X,"mixture.txt");
}

void exercise1(){
  arr X;
  MT::load(X,"gauss.txt");
  uint n=X.d0;
  arr mu=(1./n)*sum(X,0);
  arr C = (1./n)*~X*X - mu*~mu;
  arr U,d,V;
  svd(U,d,V,C);
  cout <<"mu=" <<mu
       <<"\nC=" <<C
       <<"\nV=" <<V
       <<"\nd=" <<d
       <<"\nU=" <<U
       <<endl;
}

double NNinv(const arr& a,const arr& b,const arr& Cinv){
  double d=sqrDistance(Cinv,a,b);
  double norm = ::sqrt(lapack_determinantSymPosDef((1./MT_2PI)*Cinv));
  return norm*::exp(-.5*d);
}
double NN(const arr& a,const arr& b,const arr& C){
  arr Cinv;
  inverse_SymPosDef(Cinv,C);
  return NNinv(a,b,Cinv);
}

void exercise2(){
  arr X;
  MT::load(X,"mixture.txt");
  gnuplot("plot 'mixture.txt' w p",NULL,true);
  
  uint n=X.d0;
  uint d=X.d1;
  uint K=3;
  uint k,i,t;
  arr q(K,n),w(K,n),mu(K,d),S(K,d,d);
  //initialize
  for(k=0;k<K;k++){
    mu[k]() = X[rnd(n)];
    S[k]() = eye(d);
  }
  //EM
  for(t=0;t<100;t++){
    //E-step
    for(k=0;k<K;k++) for(i=0;i<n;i++) q(k,i) = NN(X[i],mu[k],S[k]);
    arr tmp = sum(q,0);
    for(k=0;k<K;k++) for(i=0;i<n;i++) q(k,i) /= tmp(i);

    //M-step
    arr qsum = sum(q,1);
    for(k=0;k<K;k++) for(i=0;i<n;i++) w(k,i) = q(k,i)/qsum(k);
    mu = w*X;
    for(k=0;k<K;k++){
      S[k]() = (~X)*diag(w[k])*X - (mu[k]^mu[k]);
    }
    
    plotClear();
    plotPoints(X);
    plotPoints(mu);
    for(k=0;k<K;k++)  plotCovariance(mu[k],S[k]);
    plot();
  }
}

int main(int argc, char *argv[]){
  MT::initCmdLine(argc,argv);

  //generateArtificialGauss();
  //generateArtificialMixData();
  exercise1();
  exercise2();
  
  return 0;
}

