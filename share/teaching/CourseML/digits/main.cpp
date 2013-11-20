#include <Core/array.h>
#include <Core/util.h>
#include <Gui/plot.h>
#include <stdlib.h>
using namespace std;

#include <Algo/MLcourse.h>
#include <Core/util.h>


void digits() {
  arr X,y;
  ifstream digit_pca("digit_pca.txt");
  X.read(digit_pca);
  ifstream digit_label("digit_label.txt");
  y.read(digit_label);
  y.reshape(y.N);


  struct myCV:public CrossValidation {
    void  train(const arr& X, const arr& y, double param, arr& beta) {
      //ridgeRegression(beta, X, y, param);
      logisticRegression2Class(beta, X, y, param);
    }
    double test(const arr& X, const arr& y, const arr& beta) {
      uint n=X.d0;
      arr f = X*beta;
      arr p(n);
      double logLike=0.;
      for(uint i=0; i<X.d0; i++) {
	p(i) = 1./(1.+exp(-f(i)));
	logLike += y(i)*log(p(i))+(1.-y(i))*log(1.-p(i));
      }
      return -logLike/n;
    }
  } cv;

  arr Phi;
  arr beta;
  makeFeatures(Phi,X,X);
  
  cv.crossValidateMultipleLambdas(Phi, y, ARR(1e-2,1e-1,1e0,1e1,1e2,1e3,1e4,1e5), 10, false);
  cv.plot();
  cout <<"10-fold CV:\n  costMeans= " <<cv.scoreMeans <<"\n  costSDVs= " <<cv.scoreSDVs <<endl;

}



int main(int argc, char *argv[]) {
  MT::initCmdLine(argc,argv);
  
  digits();

  return 0;
}

