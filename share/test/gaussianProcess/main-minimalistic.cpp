#define MT_IMPLEMENTATION

#include<MT/array.h>
#include<MT/plot.h>


//===========================================================================
//
/// @name Gaussian Process code
//

struct GaussianProcess{
  /// pointer to kernel function
  double (*kernel)(const doubleA& x,const doubleA& y);
  /// pointers to data
  doubleA *X,*Y;
  /// inverse gram matrix
  doubleA invGram;
};

void calculate(GaussianProcess &gp,doubleA &X,doubleA &Y){
  gp.X=&X; gp.Y=&Y;
  uint i,j,N=Y.N;
  doubleA gram(N,N),xi,xj;
  for(i=0;i<N;i++){ xi.referToSubDim(X,i); gram(i,i) = gp.kernel(xi,xi); }
  for(i=1;i<N;i++){
    xi.referToSubDim(X,i);
    for(j=0;j<i;j++){
      xj.referToSubDim(X,j);
      gram(i,j) = gram(j,i) = gp.kernel(xi,xj);
    }
  }
  inverse(gp.invGram,gram);
}

void evaluate(GaussianProcess &gp,const doubleA& x,double& y,double& sig){
  uint i,N=gp.Y->N;
  doubleA k(N);
  for(i=0;i<N;i++) k(i) = gp.kernel(x,(*gp.X)[i]);
  y = scalarProduct(gp.invGram,k,*gp.Y);
  sig = gp.kernel(x,x) - scalarProduct(gp.invGram,k,k);
  if(sig<0) sig=0.; else sig = ::sqrt(sig);
}
void evaluate(GaussianProcess &gp,const doubleA& X,doubleA& Y,doubleA& S){
  uint i;
  Y.resize(X.d0); S.resize(X.d0);
  for(i=0;i<X.d0;i++) evaluate(gp,X[i],Y(i),S(i));
}

//
//===========================================================================


double noise=.1,priorVar=1.,kernelRange=.2;

void randomData(doubleA& X,doubleA& Y){
  X.setGrid(1,-4.,4.,5);
  Y=sin(X);
  Y.reshape(Y.N);
  rndGauss(Y,noise,true);
}

double kernel(const doubleA& x,const doubleA& y){
  if(&x==&y) return priorVar+noise*noise;
  double d=sqrDistance(x,y);
  return priorVar*::exp(-.5 * d/(kernelRange*kernelRange));
}

void testGP(){
  cout <<"*** test GP on sinus functions" <<endl;
  noise=.1;
  priorVar=1.;
  kernelRange=1.;
  
  doubleA X,Y,Xp,Yp,Sp;
  GaussianProcess gp; gp.kernel=kernel;

  randomData(X,Y);
  calculate(gp,X,Y);

  Xp.setGrid(1,-5.,5.,1000);
  evaluate(gp,Xp,Yp,Sp);

  plotPoints(X,Y);
  plotFunction(Xp,Yp);
  plotFunction(Xp,Yp+Sp);
  plotFunction(Xp,Yp-Sp);
  plot(true);
}

void randomFunctions(){
  cout <<"*** generate random functions by sampling from the GP itself..." <<endl;
  noise=.001;
  priorVar=1.;
  kernelRange=.2;
  
  doubleA X,Y,Xp,Yp,Sp;
  doubleA Xall,x;
  double y,sig;
  uint i,k;

  Xall.setGrid(1,-1.,1.,50);
  Xp.setGrid(1,-1.,1.,100);
  Xall.permuteRandomly();

  GaussianProcess gp; gp.kernel=kernel;
  //plotGnuplot();

  //do it a couple of times:
  for(k=0;k<10;k++){
    X.resize(0,1); Y.resize(0); //clear current data
    for(i=0;i<Xall.d0;i++){
      calculate(gp,X,Y); //calculate GP for current data

      if(!(i%10)){ //display
	evaluate(gp,Xp,Yp,Sp);
	plotClear();
	plotPoints(X,Y);
	plotFunction(Xp,Yp);
	plotFunction(Xp,Yp+Sp);
	plotFunction(Xp,Yp-Sp);
	plot(true);
      }
      
      x=Xall[i]; //get next input point
      evaluate(gp,x,y,sig); //sample it from the GP itself
      y+=sig*rnd.gauss(); //with variance..
      X.append(x); //append it to the data
      Y.append(y);
    }
  }
}

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

  testGP();
  randomFunctions();

  return 0;
}
