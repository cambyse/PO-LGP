#include "gaussianProcess.h"

/** prior of 0 */
double const_0(const arr &x, const void *p){return 0.;}

GaussianProcess::GaussianProcess(){
  kernelP=priorP=NULL;
  mu_func=const_0;
  mu=0.;
}

/*! set Gauss cov function, its parameters, and GP prior
 */

void GaussianProcess::setGaussKernelGP(
  void *_kernelP,
  double _mu){
  mu = _mu;
  mu_func = const_0;
  priorP = NULL;
  kernelP=_kernelP;
  kernelF=GaussKernel;
  dkernelF=dGaussKernel;
  kernelD1=GaussKernelD1;
  kernelD2=GaussKernelD2;
}

/*! set Gauss cov function, its parameters, and GP prior
 */
void GaussianProcess::setGaussKernelGP(
  void *_kernelP,
  double(*_mu)(const arr&, const void*),
  void *_priorP){
  mu_func = _mu;
  priorP = _priorP;
  mu = 0;
  kernelP=_kernelP;
  kernelF=GaussKernel;
  dkernelF=dGaussKernel;
  kernelD1=GaussKernelD1;
  kernelD2=GaussKernelD2;
}


void GaussianProcess::recompute(const arr& _XX, const arr& _YY){
  X.referTo(_XX);
  Y.referTo(_YY);
  recompute();
}

void GaussianProcess::recompute(){
  uint i, j, N=Y.N, dN=dY.N;
  arr gram, xi, xj, Mu_func;
  gram.resize(N+dN, N+dN);
  if(!gram.N) return;
  for(i=0; i<N; i++){
    xi.referToSubDim(X, i);
    gram(i, i) = kernelF(kernelP, xi, xi);
    Mu_func.append(mu_func(xi, priorP));
  }
  for(i=1; i<N; i++){
    xi.referToSubDim(X, i);
    for(j=0; j<i; j++){
      xj.referToSubDim(X, j);
      gram(i, j) = gram(j, i) = kernelF(kernelP, xi, xj);
    }
  }
  if(dN){ //derivative observations
    for(i=0; i<dN; i++){ xi.referToSubDim(dX, i); gram(N+i, N+i) = kernelD2(dI(i), dI(i), kernelP, xi, xi); }
    for(i=0; i<dN; i++){
      xi.referToSubDim(dX, i);
      for(j=0; j<N; j++){
        xj.referToSubDim(X, j);
        gram(N+i, j) = gram(j, N+i) = kernelD1(dI(i), kernelP, xj, xi);
      }
      for(j=0; j<i; j++){
        xj.referToSubDim(dX, j);
        gram(N+i, N+j) = gram(N+j, N+i) = kernelD2(dI(i), dI(j), kernelP, xi, xj);
      }
    }
  }
  inverse_SymPosDef(Ginv, gram);
  if(!dN){
    if(N) GinvY = Ginv * (Y-Mu_func-mu); else GinvY.clear();
  }else{
    arr Yfull; Yfull.append(Y-Mu_func-mu); Yfull.append(dY);
    GinvY = Ginv * Yfull;
  }
  //cout  <<"gram="  <<gram  <<" Ginv="  <<Ginv  <<endl;
}

void GaussianProcess::appendObservation(const arr& x, double y){
  uint i, N=X.d0;
  
  static arr k, m, M, xi;
  //update of inverse Gram matrix:
  if(false & N){
    double mu;
    k.resize(N); m.resize(N); M.resize(N, N); xi.referToSubDim(X, 0);
    for(i=0; i<N; i++){ xi.referToSubDim(X, i); k(i)=kernelF(kernelP, x, xi); }
    innerProduct(m, Ginv, k);
    mu=1./(kernelF(kernelP, x, x) - scalarProduct(k, m));
    m *= -mu;
    M = Ginv;
    M += (1./mu) * m^m;
    
    Ginv.setBlockMatrix(M, m, m, ARR(mu));
    
    X.append(x); //append it to the data
    Y.append(y);
  }else{
    X.reshape(N, x.N);
    Y.reshape(N);
    X.append(x); //append it to the data
    Y.append(y);
  }
#if MT_GP_DEBUG
  arr iG=Ginv;
  recompute();
  double err=maxDiff(iG, Ginv);
  CHECK(err<1e-6, "mis-updated inverse Gram matrix"  <<err  <<endl  <<iG  <<Ginv);
#endif
}

void GaussianProcess::appendDerivativeObservation(const arr& x, double y, uint i){
  uint N=dX.d0;
  dX.reshape(N, x.N);
  dY.reshape(N);
  dI.reshape(N);
  dX.append(x); //append it to the data
  dY.append(y);
  dI.append(i);
}

void GaussianProcess::appendGradientObservation(const arr& x, const arr& nabla){
  for(uint i=0; i<nabla.N; i++) appendDerivativeObservation(x, nabla(i), i);
}

double GaussianProcess::max_var(){
  return kernelF(kernelP, ARR(0), ARR(0));
}

void GaussianProcess::evaluate(const arr& x, double& y, double& sig){
  uint i, N=Y.N, dN=dY.N;
  static arr k, xi, Ginvk;
  if(N+dN==0){ //no data
    y = mu_func(x, priorP) + mu;
    sig=::sqrt(kernelF(kernelP, x, x));
    return;
  }
  if(k.N!=N+dN) k.resize(N+dN);
  for(i=0; i<N; i++){ xi.referToSubDim(X, i); k(i)=kernelF(kernelP, x, xi); }
  //derivative observations
  for(i=0; i<dN; i++){ xi.referToSubDim(dX, i); k(N+i)=kernelD1(dI(i), kernelP, x, xi); }
  
  y = scalarProduct(k, GinvY) + mu_func(x, priorP) + mu;
  innerProduct(Ginvk, Ginv, k);
  sig = kernelF(kernelP, x, x) - scalarProduct(k, Ginvk);
  if(sig<0) sig=0.; else sig = ::sqrt(sig);
}

/* \nabla f(x) (i) =  \frac{
 *      \partial (  k *  K^{-1} * Y_all ) }{
 *      \partial x_i }
 *
 * where k is a vector 1..N+dN with k(j)
 * j \in {1...N}            % function value observations
 *  \frac{
 *    \partial k(x, xj) }{
 *    \partial x_i }
 * j \in {N...dN}           % derivative observations
 *  \frac{
 *    \partial^2 k(x, xj) }{
 *    \partial x_i  \partial x_l }
 *    where x_l is the the same argument in which the derivative in xj has been
 *    observed
 */
void GaussianProcess::gradient(arr& grad, const arr& x){
  CHECK(X.N || dX.N , "can't recompute gradient without data");
  CHECK((X.N && x.N==X.d1) || (dX.N && x.N==dX.d1), "dimensions don't match!");
  uint i, d, N=Y.N, dN=dY.N, dim;
  dim = X.d1?X.d1:dX.d1;
  arr dk(dim);
  static arr xi, dxi;
  grad.resize(x.N);
  grad.setZero();
  // take the gradient in the function valu observations
  for(i=0; i<N; i++){
    xi.referToSubDim(X, i);
    dkernelF(dk, kernelP, x, xi);
    grad += GinvY(i) * dk;
  }
  // derivative observations
  for(i=0; i<dN; i++){//FIXME!!!!
    dxi.referToSubDim(dX, i);
    dk.setZero();
    for(d=0; d<dim; ++d){
      dk(d) = kernelD2(d, dI(i), kernelP, x, dxi);
    }
    grad += GinvY(i+N) * dk;
  }
}

void GaussianProcess::evaluate(const arr& X, arr& Y, arr& S){
  uint i;
  static arr xi;
  Y.resize(X.d0); S.resize(X.d0);
  for(i=0; i<X.d0; i++){ xi.referToSubDim(X, i); evaluate(xi, Y(i), S(i)); }
}
