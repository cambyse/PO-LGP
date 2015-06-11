#include <Algo/MLcourse.h>
#include <Gui/plot.h>
#include <Optim/optimization.h>

struct NeuralNet{
  uintA h; ///< # of neurons in each hidden layer
  double lambda; ///< regularization

  MT::Array<arr> w;
  MT::Array<arr> g;

  NeuralNet(const uintA& h, double lambda):h(h), lambda(lambda){
    w.resize(h.N-1);
    g.resize(h.N-1);
    for(uint l=0;l<h.N-1;l++){
      w(l).resize(h(l+1), h(l));
      g(l).resize(h(l+1), h(l));
    }
    for(arr& wl:w) rndGauss(wl);
    for(arr& gl:g) gl.setZero();
  }

  void resetGradients(){
    for(arr& gl:g) gl.setZero();
  }

  double f(const arr& x0, double y_target, double* y_output=0){
    CHECK(x0.N==h(0),"");
    MT::Array<arr> z(h.N),x(h.N);
    x(0) = x0;
    if(y_target==0.) y_target = -1.; //we want target +/-1

    //forward
    for(uint l=1;l<h.N;l++){
      z(l) = w(l-1)*x(l-1);
      if(l<h.N-1) x(l) = sigm(z(l));
      else x(l) = z(l);
    }

    //error
    double E=1.-x.last().scalar()*y_target;
    if(E<0.) E=0.;

    //gradient
    if(E>0.){
      MT::Array<arr> del(h.N);
      del.last() = ARR(-y_target);
      for(uint l=h.N-1;l--;){
        del(l) = (del(l+1)*w(l)) % (x(l) % (1.-x(l)));
      }
      for(uint l=0;l<h.N-1;l++){
        g(l) += ~del(l+1) * ~x(l);
      }
    }

    if(y_output) *y_output = x.last().scalar();
    return E;
  }

  void setParams(const arr& W){
    uint i=0;
    for(arr& wl:w){
      CHECK(i+wl.N<=W.N, "");
      memmove(wl.p, W.p+i, wl.N*wl.sizeT);
      i += wl.N;
    }
  }
  arr getGrads(){
    arr G;
    for(arr& gl:g) G.append(gl);
    G.reshape(G.N);
    return G;
  }

  double applyOnData(arr& grad, const arr& X, const arr& y){
    resetGradients();
    double E=0.;
    for(arr& wl:w) E += lambda*sumOfSqr(wl);
    for(uint i=0;i<X.d0;i++) E += f(X[i],y(i));
    if(&grad){
      for(uint l=0;l<w.N;l++) g(l) += (2.*lambda)*w(l);
      grad = getGrads();
    }
    return E;
  }

  arr evaluate(const arr& X){
    arr y(X.d0);
    for(uint i=0;i<X.d0;i++) f(X[i], 0., &y(i));
    return sigm(y);
  }

  ScalarFunction fn(const arr& X, const arr& y){
    return [this, X, y](arr& g, arr& H, const arr& x) -> double {
      if(&H) NIY;
      this->setParams(x);
      return this->applyOnData(g, X, y);
    };
  }

};

//===========================================================================

void testNN() {

  arr X,y;
  artificialData_Hasties2Class(X, y);
  arr Phi = makeFeatures(X, linearFT);

  NeuralNet NN({3u, 100, 1}, 1e-2);
  double eps=1e-2;

  arr W = NN.getGrads();
  rndGauss(W);
//  checkGradient(NN.fn(Phi, y), W, 1e-4);

#if 1
  optRprop(W, NN.fn(Phi,y), OPT(verbose=2));
//  optGradDescent(W, NN.fn(Phi,y), OPT(verbose=2, stopTolerance=1e-4));
#else
  for(uint k=0;k<10;k++){
    double E=NN.applyOnData(NoArr, X,y);
    cout <<"E=" <<E <<endl;
    for(uint i=0;i<NN.w.N;i++) NN.w(i) -= eps*NN.g(i);
  }
#endif

  arr X_grid;
  X_grid.setGrid(X.d1, -2, 3, (X.d1==1?500:50));
  Phi = makeFeatures(X_grid, linearFT);

  arr f_grid = NN.evaluate(Phi);
  FILE("z.train") <<catCol(X, y);
  FILE("z.model") <<f_grid.reshape(51,51);
  gnuplot("load 'plt.contour'; pause mouse", false, true, "z.pdf");
  gnuplot("load 'plt.contour2'; pause mouse", false, true, "z.pdf");

}

//===========================================================================

int main(int argc, char *argv[]) {
  MT::initCmdLine(argc,argv);

  testNN();
  
  return 0;
}

