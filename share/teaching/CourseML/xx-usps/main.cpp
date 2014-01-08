#include <Core/array.h>
#include <Core/util.h>
#include <Gui/plot.h>

using namespace std;

#include <Algo/MLcourse.h>

void loadUSPS(arr& X,arr& y,const char* filename){
  X.readTagged(filename,"usps");
  X/=128.;
  X-=1.;
  
  X.reshape(X.N/2560,10,256);
  arr X2(X.d0,2,256);
  for(uint i=0;i<X.d0;i++){
    X2.subDim(i,0)() = X.subDim(1,1);
    X2.subDim(i,1)() = X.subDim(1,7);
  }
  X.reshape(X.N/256,256);
  X2.reshape(X2.N/256,256);
  
  uint n=X.d0,M=10;
  y.resize(n,M);
  y.setZero();
  for(uint i=0;i<n;i++) y(i,(i+1)%M)=1.;
  
  arr y2(X2.d0);
  for(uint i=0;i<y2.d0;i++) y2(i)=i%2;
  
  X=X2;  y=y2;
  //cout <<X <<y <<endl;
}

void testUsps(){
  rnd.seed(1);
  arr X,Phi,y;
  arr beta;

  loadUSPS(X,y,"usps.arr");

  arr X_test,y_test;
  X_test = X.sub(X.d0-100,-1,0,-1);
  X.delRows(X.d0-100,100);
  y_test = y.sub(y.d0-100,-1);
  y.resizeCopy(y.d0-100);
  
  makeFeatures(Phi,X,X);
  //cout <<Phi <<endl;
  logisticRegression2Class(beta, Phi, y, MT::getParameter<double>("ridge",1e-10));
  //logisticRegressionMultiClass(beta, Phi, y, MT::getParameter<double>("ridge",1e-10));

  cout <<beta <<endl;
  
  arr p_pred = Phi*beta;
  for(uint i=0;i<p_pred.N;i++){
    p_pred(i) = 1./(1.+exp(-p_pred(i)));
  }
  write(LIST<arr>(y, p_pred), "train");

  makeFeatures(Phi,X_test,X);
  p_pred = Phi*beta;
  for(uint i=0;i<p_pred.N;i++){
    p_pred(i) = 1./(1.+exp(-p_pred(i)));
  }
  write(LIST<arr>(y_test, p_pred), "test");
}


int main(int argc, char *argv[]){
  MT::initCmdLine(argc,argv);

  testUsps();
  
  return 0;
}

