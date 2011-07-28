#ifndef MT_gaussianProcess_h
#define MT_gaussianProcess_h

#include "array.h"
#include "MT/util.h"

#define MT_GP_DEBUG 0
#include "plot.h"

//===========================================================================
//
//!@name Gaussian Process code
//

struct GaussianProcess{
  arr X,Y;   //!< data
  arr dX,dY; //!< derivative data
  uintA dI;  //!< derivative data (derivative indexes)
  arr Ginv,GinvY,ig2;  //!< inverse gram matrix (and a second buffer for push/pop)

  //--prior function
  double mu; //!< const bias of the GP
  double (*mu_func)(const arr &x, const void *param); //!< prior of the GP (variable bias)
  void *priorP;
  
  //-- kernel function
  double (*kernelF)(void *P,const arr& x,const arr& y); //!< pointer to kernel function
  void (*dkernelF) (arr& grad,void *P,const arr& x,const arr& y);
  double (*kernelD1)(uint i,void *P,const arr& x,const arr& y);
  double (*kernelD2)(uint i,uint j,void *P,const arr& x,const arr& y);
  double (*kernelD3)(uint i,uint j, uint k, void *P,const arr& x,const arr& y);
  void *kernelP;                      //!< pointer to parameters (a struct or so) passed to the kernel function

  GaussianProcess();
      
  void clear(){ X.clear(); Y.clear(); dX.clear(); dY.clear(); dI.clear(); Ginv.clear(); GinvY.clear(); ig2.clear(); }

  void copyFrom(GaussianProcess &f){
  X=f.X;Y=f.Y;dX=f.dX;dY=f.dY; dI=f.dI;
  Ginv=f.Ginv;GinvY=f.GinvY;ig2=f.ig2;
  mu=f.mu; mu_func=f.mu_func;priorP=f.priorP;
  kernelF=f.kernelF; kernelD1=f.kernelD1; kernelD2=f.kernelD2; kernelP=f.kernelP; }

  /*! set an arbitrary covariance function,
      P is a pointer to parameters (a struct) that is passed
      everytime when the _kernelF is called */
  void setKernel(double (*_kernelF)(void *P,const arr& x,const arr& y),void *_kernelP){
    kernelP=_kernelP;
    kernelF=_kernelF;
    dkernelF=NULL;
  }
  void setKernel(double (*_kernelF)(void *P,const arr& x,const arr& y),void (*_dkernelF)(arr& grad,void *P,const arr& x,const arr& y),void *_kernelP){
    kernelP=_kernelP;
    kernelF=_kernelF;
    dkernelF=_dkernelF;
  }
  /*! set a covariance function,
   * gradient of the cov,
   * cov of function value and
   * derivative,
   * cov of two derivatives.
   * cov function parameters (SDV rather than variance)
   */
  void setKernel(
      double (*_kernelF)(void *P,const arr& x,const arr& y),
      void (*_dkernelF)(arr& grad,void *P,const arr& x,const arr& y),
      double (*_kernelD1)(uint i,void *P,const arr& x,const arr& y),
      double (*_kernelD2)(uint i,uint j,void *P,const arr& x,const arr& y),
      double (*_kernelD3)(uint i,uint j, uint k, void *P,const arr& x,const arr& y),
      void *_kernelP){
    kernelP=_kernelP;
    kernelF=_kernelF;
    dkernelF=_dkernelF;
    kernelD1=_kernelD1;
    kernelD2=_kernelD2;
    kernelD3=_kernelD3;
  }
  void setGaussKernelGP( void *_kernelP, double (*_mu)(const arr&, const void*), void*);
  void setGaussKernelGP( void *_kernelP, double _mu);

  void recompute(const arr& X,const arr&Y);              //!< calculates the inv Gram matrix for the given data
  void recompute();                                      //!< recalculates the inv Gram matrix for the current data
  void appendObservation(const arr& x,double y);     //!< add a new datum to the data and updates the inv Gram matrix
  void appendDerivativeObservation(const arr& x,double dy,uint i);
  void appendGradientObservation(const arr& x,const arr& dydx);
  void evaluate(const arr& x,double& y,double& sig);   //!< evaluate the GP at some point - returns y and sig (=standard deviation)
  void evaluate(const arr& X,arr& Y,arr& S);   //!< evaluate the GP at some array of points - returns all y's and sig's
  double max_var(); // the variance when no data present
  void gradient(arr& grad,const arr& x);           //!< evaluate the gradient dy/dx of the mean at some point
  void hessian(arr& hess,const arr& x);           //!< evaluate the hessian dy/dx1dx2 of the mean at some point

  void push(const arr& x,double y){ ig2=Ginv; appendObservation(x,y); recompute(); }
  void pop(){ Ginv=ig2; X.resizeCopy(X.d0-1,X.d1); Y.resizeCopy(Y.N-1); }
};


//===========================================================================
//
//!@name standard Gaussian covariance function
//

struct GaussKernelParams{
  double obsVar,priorVar,widthVar,derivVar;
  GaussKernelParams(){ obsVar=.01; priorVar=.01; widthVar=.04; derivVar=.01; }
  GaussKernelParams(double _noiseSDV,double _priorSDV,double _widthSDV,double _derivSDV){ obsVar=_noiseSDV*_noiseSDV; priorVar=_priorSDV*_priorSDV; widthVar=_widthSDV*_widthSDV; derivVar=_derivSDV*_derivSDV;}
};

//! you can also pass a double[3] as parameters
inline double GaussKernel(void *P,const arr& x1,const arr& x2){
  GaussKernelParams& K = *((GaussKernelParams*)P);
  if( (&x1==&x2) || operator==(x1,x2)) return K.priorVar+K.obsVar;
  double d;
  if(x1.N!=1) d=sqrDistance(x1,x2); else{ d=x2(0)-x1(0); d=d*d; }
  return K.priorVar*::exp(-.5 * d/K.widthVar);
}

/*! \brief return gradient, i.e.
  for i \in {vector dimensions}: \frac { \parital k(x, x2) }{ \partial x_i  }
  you can also pass a double[3] as parameters */
inline void dGaussKernel(arr& grad,void *P,const arr& x1,const arr& x2){
  GaussKernelParams& K = *((GaussKernelParams*)P);
  if(&x1==&x2){ grad.resizeAs(x1); grad.setZero(); return; }
  double gauss=GaussKernel(P,x1,x2), gamma=1./K.widthVar;
  grad = gamma * (x2-x1) * gauss; // SD: Note the (x2 - x1) swap cancles the leading minus 
  //MT_MSG("gamma="<<gamma<<"; x2-x1"<<x2 -x1<<"; gauss="<<gauss<<"; grad="<<grad);
}

/*! \brief \frac { \parital k(x_p, x) }{ \partial x_i  }
  you can also pass a double[3] as parameters */
inline double GaussKernelD1(uint i,void *P,const arr& x_point,const arr& x_deriv){
  GaussKernelParams& K = *((GaussKernelParams*)P);
  if(&x_point==&x_deriv){ HALT("this shouldn't happen, I think"); return K.obsVar; }
  double gauss=GaussKernel(P,x_point,x_deriv), gamma=1./K.widthVar;
  double di=x_point(i)-x_deriv(i);
  return gamma * di * gauss;
}

/*! \brief \frac { \parital^2 k(x, x2) }{ \partial x_i \partial x_j  }
  you can also pass a double[3] as parameters */
inline double GaussKernelD2(uint i,uint j,void *P,const arr& x1,const arr& x2){
  GaussKernelParams& K = *((GaussKernelParams*)P);
  if(&x1==&x2) return K.priorVar/K.widthVar + K.derivVar;
  double gauss=GaussKernel(P,x1,x2), gamma=1./K.widthVar;
  double di=x1(i)-x2(i), dj=x1(j)-x2(j);
  return gamma * ((i==j?1.:0.) - gamma*di*dj) * gauss;
}

/*! \brief \( \frac { \partial^3 k(\vec{x}, \vec{x2}) }{ \partial x_i \partial x_j \partial x_k  } \)
  you can also pass a double[3] as parameters */
inline double GaussKernelD3(uint i,uint j, uint k, void *P,const arr& x1,const arr& x2){
  uint i2,j2,k2;
  GaussKernelParams& K = *((GaussKernelParams*)P);
  if(&x1==&x2) return K.priorVar/K.widthVar + K.derivVar; //TODO: kerneld3(x,x)
  double gauss=GaussKernel(P,x1,x2), gamma=1./K.widthVar;
  double di=x2(i)-x1(i), dj=x2(j)-x1(j), dk=x2(k)-x1(k); 
  /*
  if (i!=j  && i!=k && j!=k ) 
    return gamma*gamma*gamma*di*dj*dk*gauss; //TODO check signs here and below
  else if (i!=j  && j==k ) 
    return gamma*gamma*(di - gamma*di*dj*dk) * gauss;
  else if (i!=j  && i==k ) 
    return gamma*gamma*(dj - gamma*di*dj*dk) * gauss;
  else if (i==j && i!=k)
    return gamma*gamma*(dk - gamma*di*dj*dk) * gauss;
  else if (i==j && i==k)
    return gamma*gamma*(3*dk - gamma*di*dj*dk) * gauss;
  */
  
  i2=1<<i; j2=1<<j; k2=1<<k;
  if ( i2&j2&k2 ) // nonzero => all the same
    return gamma*gamma*(3*dk - gamma*di*dj*dk) * gauss;  //TODO check signs here and below
  else if ( (i2|j2|k2) == 7 ) // all different
    return gamma*gamma*gamma*di*dj*dk*gauss; 
  else
    return gamma*gamma*( i==j?dk:(i==k?dj:di) - gamma*di*dj*dk) * gauss;

}

inline double maximizeGP(GaussianProcess& gp,arr& x){
  NIY;
  /*
  CHECK(gp.Y.N,"");
  if(!x.N){ x.resize(gp.X.d1); x=0.; } //rndGauss(x,1.); }
  arr dx;
  Rprop rp;
  rp.init(x.N,.1);
  uint t=0;
  do{
    gp.gradient(dx,x);
    dx *= -1.;
    rp.learn(x,dx);
    t++;
    if(t>100){ cout <<" couldn't stop!"; break; }
  }while(!rp.fine());
  double y,s;
  gp.evaluate(x,y,s);
  cout <<"GP maximization: " <<t <<"iterations, x=" <<x <<" y=" <<y <<" s=" <<s <<endl;
  return y;
  */
}

inline void plotBelief(GaussianProcess& gp,double lo,double hi, bool pause=true){
  arr X,Y,Z,S;
  uint dim;
  //there should be at least 1 observation to guess the dimensionality from
  dim = gp.X.d1 ? gp.X.d1 : gp.dX.d1;
  CHECK(dim > 0, "still no data here. I have no clue about dimensionality!?!");

  X.setGrid(dim,lo,hi,100);
  gp.evaluate(X,Y,S);
  plotClear();
  switch (dim){
  case 1:
    plotFunctionPrecision(X,Y,Y+S,Y-S);
    //plotFunction(X,Y);
    //plotFunction(X,Y+S);
    //plotFunction(X,Y-S);
    plotPoints(gp.X,gp.Y);
    plotPoints(gp.dX,gp.dY);
    break;
  case 2:
    plotFunction(X,Y );
    plotFunction(X,Y+S );
    plotFunction(X,Y-S );
    plotPoints(gp.X,gp.Y);
    plotPoints(gp.dX,gp.dY);
    break;
  default :
    HALT("Space is either 0- or higher than 3-dimensional. Tell me how to plot that!")
    break;
  }
  plot(pause);
}

inline void plotKernel1D(GaussianProcess& gp,double lo,double hi){
  arr X,K,KD1,KD2;
  X.setGrid(1,lo,hi,1000);
  K.resize(X.d0);
  KD1.resize(X.d0);
  KD2.resize(X.d0);
  arr null=ARR(0);
  for(uint i=0;i<X.d0;i++){
    K(i) = gp.kernelF(gp.kernelP,null,X[i]);
    KD1(i) = gp.kernelD1(0,gp.kernelP,null,X[i]);
    KD2(i) = gp.kernelD2(0,0,gp.kernelP,null,X[i]);
  }
  plotClear();
  plotFunction(X,K);
  plotFunction(X,KD1);
  plotFunction(X,KD2);
  plot(true);
}

inline void plotKernel2D(GaussianProcess& gp,double lo,double hi){
  arr X,K,KD1,KD2;
  X.setGrid(2,lo,hi,1000);
  K.resize(X.d0,X.d1);
  KD1.resize(X.d0,X.d1);
  KD2.resize(X.d0,X.d1);
  arr null=ARR(0);
  for(uint i=0;i<X.d0;i++){
    for(uint j=0;j<X.d1;j++){
      K(i,j) = gp.kernelF(gp.kernelP,null,X[i]);
      KD1(i,j) = gp.kernelD1(0,gp.kernelP,null,X[i]);
      KD2(i,j) = gp.kernelD2(0,0,gp.kernelP,null,X[i]);
    }
  }
  plotClear();
  plotSurface(K);
  plotSurface(KD1);
  plotSurface(KD2);
  plot(true);
}

inline void randomFunction(GaussianProcess& gp,arr& Xbase,bool illustrate,bool fromPosterior=false){
  GaussKernelParams& K = *((GaussKernelParams*)gp.kernelP);
  double orgObsVar=K.obsVar;
  K.obsVar=1e-6;

  arr x;
  double y,sig;
  uint i;
  //gp.setKernel(&stdKernel,GaussKernel);
  if(!fromPosterior) gp.clear();

//  Xbase.randomPermute();
  //gp.X.resize(0,1); gp.Y.resize(0); //clear current data
  for(i=0;i<Xbase.d0;i++){
    if(illustrate && i>0 && !(i%10)){ //display
      plotBelief(gp,Xbase.min(),Xbase.max());
    }
      
    x.referToSubDim(Xbase, i); //get next input point
    gp.evaluate(x,y,sig);      //sample it from the GP itself
    y+=sig*rnd.gauss();        //with standard deviation..
    gp.appendObservation(x,y);
    gp.recompute();
  }

  K.obsVar=orgObsVar;
}



#ifdef  MT_IMPLEMENTATION
#  include "gaussianProcess.cpp"
#endif

#endif
