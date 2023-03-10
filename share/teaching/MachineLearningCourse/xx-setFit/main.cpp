#include <Core/array.h>
#include <Core/util.h>
#include <Plot/plot.h>

using namespace std;

#include "../01-linearRegression/code.h"



void TEST(SetFit){
  //random data:
  rnd.clockSeed();
  uint n=100;
  uint d=2;
  arr beta,x(d),beta_;
  arr Z,X=x;
  X.reshape(1,d);
  makeFeatures(Z, X, X);
  beta.resize(Z.d1);
  rndGauss(beta,1.);
  beta_=beta.sub(1,-1);
  
  X.resize(n,d);
  for(uint i=0;i<n;i++){
    rndGauss(x,1.);
    x -= beta_ * (  (beta(0)+scalarProduct(x,beta_)) / sumOfSqr(beta_) );
    X[i]=x;
  }
  write(LIST<arr>(X),"data");
}


int main(int argc, char *argv[]){
  mlr::initCmdLine(argc,argv);
  
  switch(mlr::getParameter<uint>("mode",1)){
    case 1:  testSetFit();  break;
  }
  
  return 0;
}

