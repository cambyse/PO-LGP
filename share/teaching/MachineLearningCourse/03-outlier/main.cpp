#include <Core/array.h>
#include <Core/util.h>
#include <Gui/plot.h>

using namespace std;

#include <Algo/MLcourse.h>

void TEST(Outlier){
  arr X,Phi,y,y_pred;
  arr beta;
  
  rnd.clockSeed();
  artificialData(X, y, linearOutlier);
  makeFeatures(Phi,X,X);

  arr X_test,Phi_test,y_test;
  X_test.setGrid(1,-3,3,100);
  makeFeatures(Phi_test,X_test,X);

  uint n = y.N;
  double p_prior=.5;
  arr sigma(2);
  sigma(0) = MT::getParameter<double>("sigma",1.);
  sigma(1) = MT::getParameter<double>("outlierSigma",10.);
  arr q(n),w(n),e(n);
  q=MT::getParameter<double>("outlierRate",.1); w=1.;
  for(uint r=0;r<10;r++){
    ridgeRegression(beta, Phi, y, MT::getParameter<double>("ridge",1e-10), &w);
    
    y_pred = Phi*beta;
    y_test = Phi_test*beta;
    write(LIST<arr>(X, y), "train_pure");
    write(LIST<arr>(X, y, y_pred, q, e, w), "train");
    write(LIST<arr>(X_test, y_test), "model");
    //y_test = Phi_test*beta_true;  write(LIST<arr>(X_test, y_test), "true");
    gnuplot("plot 'train' us 1:2 w p, 'model' us 1:2 w l, 'train' us 1:2:(3*$4) w p pt 6 ps variable", false, true, "z.pdf");
    MT::wait();

    for(uint i=0;i<n;i++){
      e(i) = MT::sqr(y(i) - y_pred(i));
      double q0=NormalSdv(y(i), y_pred(i), sigma(0)) * (1.-p_prior);
      double q1=NormalSdv(y(i), y_pred(i), sigma(1)) * p_prior;
      q(i) = q1/(q0+q1);
      w(i) = q(i)*MT::sqr(1./sigma(1)) + (1.-q(i))*MT::sqr(1./sigma(0));
    }
  }
}

int main(int argc, char *argv[]){
  MT::initCmdLine(argc,argv);
  
  switch(MT::getParameter<uint>("mode",1)){
    case 1:  testOutlier();  break;
    default: HALT("");
  }
  
  return 0;
}

