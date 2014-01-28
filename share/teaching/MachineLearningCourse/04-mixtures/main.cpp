#include <Core/array.h>
#include <Core/util.h>
#include <Algo/ann.h>
#include <Gui/plot.h>
#include <Algo/MLcourse.h>

using namespace std;



void generateArtificialGauss(){
  arr X(1000,2);
  rndGauss(X);
  arr V = ARR(0.70711, 0.70711, -.70711, 0.70711);  V.reshape(2,2);
  arr D = ARR( .5, 0, 0, 2); D.reshape(2,2);
  X = X * D * ~V;
  arr mu=ARR(3,5);
  X += ones(X.d0,1)*~mu;
  X >>FILE("gauss.txt");
}

void generateArtificialMixData(){
  arr X,y;
  rnd.clockSeed();
  artificialData_GaussianMixture(X, y);
  X >>FILE("mixture.txt");
}

void exercise1(){
  arr X;
  X <<FILE("gauss.txt");
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
  arr lines(4,2);
  lines[0] = mu;
  lines[1] = mu + sqrt(d(0))*U[0];
  lines[2] = mu;
  lines[3] = mu + sqrt(d(1))*U[1];
  lines >>FILE("z.lines");
  gnuplot("plot './gauss.txt' with points,'./z.lines' with lines lw 5", true, true);
}

double NN(const arr& a,const arr& b,const arr& C){
  arr Cinv;
  inverse_SymPosDef(Cinv,C);
  double d = (~(a-b)*Cinv*(a-b)).scalar();
  double norm = ::sqrt(lapack_determinantSymPosDef((1./MT_2PI)*Cinv));
  return norm*::exp(-.5*d);
}

void gaussianMixture(const arr& X){
  uint n=X.d0;
  uint d=X.d1;
  uint K=3;
  uint k,i,t;
  arr q(n,K),w(K,n),pi(K),mu(K,d),S(K,d,d);

  bool exerb=false;

  //initialize
  pi = 1./K;
  cout <<"initialization points ";
  for(k=0;k<K;k++){
    uint r=rnd(n);
    cout <<' ' <<r;
    mu[k]() = X[r];
    S[k]() = eye(d);
  }
  if(exerb){
    q=0.;
    for(i=0;i<n;i++)  q(i,rnd(K)) = 1.;
  }
  cout <<endl;

  double LL, LL_last=0.;
  
  //EM
  for(t=0;t<100;t++){
    if(t || !exerb){
    //E-step
    for(k=0;k<K;k++)  for(i=0;i<n;i++)  q(i,k) = pi(k)*NN(X[i],mu[k],S[k]);
    arr tmp = sum(q,1);
    for(k=0;k<K;k++)  for(i=0;i<n;i++)  q(i,k) /= tmp(i); //normalize q w.r.t. k
    }

    //M-step
    pi = sum(q,0)/(double)n;
    for(k=0;k<K;k++)  for(i=0;i<n;i++)  w(k,i) = q(i,k)/(n*pi(k)); //intuitively: the weighing of the i-th data point w.r.t. the k-th cluster
    mu = w*X; //update reinterpreted as a matrix multiplication!
    for(k=0;k<K;k++)  S[k]() = (~X)*diag(w[k])*X - (mu[k]^mu[k]); //again: realized via matrix multiplications
    
    //log-likelihood:
    LL=0.;
    for(i=0;i<n;i++){
      double li=0.;
      for(k=0;k<K;k++)  li += pi(k) * NN(X[i], mu[k], S[k]); //likelihood of the i-th datum
      LL += log(li);
    }
    cout <<"** iteration " <<t <<endl;
    cout <<"log-likelihood = " <<LL <<" (per point: ll=" <<LL/n << " like=" <<exp(LL/n) <<")" <<endl;
    cout <<"pi=" <<pi <<"\nmu=" <<mu <<"\nsig=" <<S <<endl;

    //stopping criterion:
    if(LL_last) CHECK(LL+1e-10 > LL_last,"likelihood is not increasing! BUG ALERT!");
    if(LL_last && LL-LL_last < 1e-6) break;
    LL_last = LL;

    //plot
    plotClear();
    plotPoints(X);
    plotPoints(mu);
    for(k=0;k<K;k++)  plotCovariance(mu[k],S[k]);
    plot();
  }

  //plot
  plotClear();
  plotPoints(X);
  plotPoints(mu);
  for(k=0;k<K;k++)  plotCovariance(mu[k],S[k]);
  plot();
}

void exercise2(){
  arr X;
  X <<FILE("mixture.txt");
  gnuplot("plot 'mixture.txt' w p", false, true);
  for(uint k=0;k<10;k++)
    gaussianMixture(X);
}


int main(int argc, char *argv[]){
  MT::initCmdLine(argc,argv);
  MT::arrayBrackets="  ";

  //generateArtificialGauss();
  //generateArtificialMixData();
  //exercise1();
  //MT::wait();
  exercise2();
  
  return 0;
}

