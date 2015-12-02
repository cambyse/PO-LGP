#include <Algo/MLcourse.h>
#include <Gui/plot.h>
#include <Optim/optimization.h>

struct NeuralNet{
  uintA h; ///< # of neurons in each layer, including output layer
  double lambda; ///< regularization

  mlr::Array<arr> w;
  mlr::Array<arr> g;
  double Error;
  uint missClass;

  NeuralNet(const uintA& h, double lambda):h(h), lambda(lambda){
    w.resize(h.N-1);
    g.resize(h.N-1);
    for(uint l=0;l<h.N-1;l++){
      w(l).resize(h(l+1), h(l));
      g(l).resize(h(l+1), h(l));
    }
    for(arr& wl:w) rndGauss(wl);
    missClass = 0;
    Error = 0.;
    for(arr& wl:w) Error += lambda*sumOfSqr(wl);
    for(uint l=0;l<w.N;l++) g(l) = (2.*lambda)*w(l);
  }

  arr f(const arr& X_input, const arr& y_target=NoArr, bool resetEgrad=true){
    CHECK(X_input.d1==h(0),"");

    //forward
    mlr::Array<arr> x(h.N);
    x(0) = ~X_input;
    for(uint l=1;l<h.N-1;l++) x(l) = sigm( w(l-1)*x(l-1) );
    x(h.N-1) = w(h.N-2)*x(h.N-2);

    if(&y_target){
      for(auto& y:y_target) CHECK(y==-1. || y==1.,"");//if(y==0.) y= -1.; //we want target +/-1
      CHECK(y_target.d0==X_input.d0 && y_target.d1==h.last(),"");

      if(resetEgrad){
        missClass = 0;
        Error = 0.;
        for(arr& wl:w) Error += lambda*sumOfSqr(wl);
        for(uint l=0;l<w.N;l++) g(l) = (2.*lambda)*w(l);
      }

      //error
      arr E(y_target.d0), dE(y_target.d0);
      for(uint i=0;i<y_target.d0;i++){
        double y_f = x.last()(0,i)*y_target(i,0);
        E(i) = 1.-y_f;
        if(E(i)<0.){ E(i)=0.; dE(i)=0.; } else dE(i)=-1;
        if(y_f<1e-10) missClass++;
      }

      //backward & gradient
      mlr::Array<arr> del(h.N);
      del.last() = dE % y_target;
      for(uint l=h.N-1;l--;)     del(l) = (del(l+1)*w(l)) % ~(x(l) % (1.-x(l)));
      for(uint l=0;l<h.N-1;l++)  g(l) += ~(x(l) * del(l+1));

      Error += sum(E);
    }

    return ~x.last();
  }

  ScalarFunction fn(const arr& X, const arr& y){
    return [this, X, y](arr& g, arr& H, const arr& x) -> double {
      if(&H) NIY;
      this->setParams(x);
      this->f(X,y);
      if(&g) g = this->getGrads();
      return this->Error;
    };
  }

  //-- two ugly methods, overwriting parameters and reading out gradients
  arr getWeights(){
    arr W;
    for(arr& wl:w) W.append(wl);
    return W;
  }
  arr getGrads(){
    arr G;
    for(arr& gl:g) G.append(gl);
    return G;
  }
  void setParams(const arr& W){
    uint i=0;
    for(arr& wl:w){
      CHECK(i+wl.N<=W.N, "");
      memmove(wl.p, W.p+i, wl.N*wl.sizeT);
      i += wl.N;
    }
    CHECK(i==W.N,"");
  }

};

//===========================================================================

void TEST(NN) {

  ofstream fil("z.NN");
  arr X,y;
  artificialData_Hasties2Class(X, y);
  arr Phi = makeFeatures(X, linearFT);
//  arr Phi = X;
  y.reshape(X.d0,1);
  for(auto& yi:y) if(yi==0.) yi=-1.;

  NeuralNet NN({3u, 10, 1}, 1e-3);

  arr W = NN.getWeights();
  rndGauss(W, .1);
//  checkGradient(NN.fn(Phi, y), W, 1e-4); return;

#if 0
  optRprop(W, NN.fn(Phi,y), OPT(verbose=2, stopTolerance=1e-4, stopIters=10000, stopEvals=10000));
//  optGradDescent(W, NN.fn(Phi,y), OPT(verbose=2, stopTolerance=1e-6, stopIters=10000, stopEvals=10000));
#else
  double eps=1e-2;
  for(uint k=0;k<10000;k++){
    NN.f(Phi, y);
    if(!(k%100)) cout <<"k=" <<k <<"  E=" <<NN.Error << " missClass=" <<(double)NN.missClass/Phi.d0 <<endl;
    fil <<k <<' ' <<NN.Error <<' ' <<(double)NN.missClass/Phi.d0 <<endl;
    for(uint i=0;i<NN.w.N;i++) NN.w(i) -= eps*NN.g(i);
  }
#endif

  NN.f(Phi, y);
  cout <<"miss classification rate=" <<(double)NN.missClass/Phi.d0 <<endl;

  arr X_grid;
  X_grid.setGrid(X.d1, -2, 3, (X.d1==1?500:50));
  Phi = makeFeatures(X_grid, linearFT);
//  Phi = X_grid;

  arr f_grid = sigm(NN.f(Phi));
  FILE("z.train") <<catCol(X, y);
  FILE("z.model") <<f_grid.reshape(51,51);
  gnuplot("load 'plt.contour'; pause mouse", false, true, "z.pdf");
  gnuplot("load 'plt.contour2'; pause mouse", false, true, "z.pdf");

}

//===========================================================================

int main(int argc, char *argv[]) {
  mlr::initCmdLine(argc,argv);

  testNN();
  
  return 0;
}

