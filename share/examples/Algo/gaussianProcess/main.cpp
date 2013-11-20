#define MT_IMPLEMENT_TEMPLATES

#include <Core/array.h>
#include <Gui/plot.h>
#include <Algo/gaussianProcess.h>

bool pltPause=true;

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

void TEST(GP) {
  cout <<"*** test GP on sinus functions" <<endl;

  doubleA X,Y,Xp,Yp,Sp;
  GaussianProcess gp;
  GaussKernelParams gpp(1., 1., .1);
  gp.obsVar = 0.05;
  gp.setKernel(GaussKernel,&gpp);

  randomData(X,Y);
  gp.recompute(X,Y);
  plotBelief(gp,-5.,5., pltPause);
}

void TEST(DerivativeObservations) {
  cout <<"*** test derivative observations" <<endl;

  GaussianProcess gp;
  //GaussKernelParams gpp(10., .3, .05);
  GaussKernelParams gpp(10., .3, .05);
  gp.obsVar = 0.05;
  gp.setKernel(GaussKernel,&gpp);
  gp.covF_D = GaussKernelF_D;   //for GP
  gp.covDD_F = GaussKernelDD_F; //for plotKernel1D
  gp.covD_D = GaussKernelD_D; //for GP

  plotKernel1D(gp,-3.,3., pltPause);

  gp.appendDerivativeObservation(ARR(0),1,0);
  gp.appendObservation(ARR(0),0);
  gp.recompute();
  plotBelief(gp,-5.,5., pltPause);
  gp.appendObservation(ARR(1),1);
  gp.recompute();
  plotBelief(gp,-5.,5., pltPause);
  gp.appendObservation(ARR(2),0);
  gp.recompute();
  plotBelief(gp,-5.,5., pltPause);
  gp.appendObservation(ARR(.5),2);
  gp.recompute();
  plotBelief(gp,-5.,5., pltPause);
  gp.appendDerivativeObservation(ARR(2),0,0);
  gp.recompute();
  plotBelief(gp,-5.,5., pltPause);
}

void TEST(RandomFunctions) {
  cout <<"*** generate random functions by sampling from the GP itself..." <<endl;
  GaussianProcess gp;
  GaussKernelParams gpp(1., .2, .001);
  gp.obsVar = .1;
  gp.setKernel(GaussKernel,&gpp);

  uint k;
  doubleA Xbase;
  Xbase.setGrid(1,-1.,1.,50);
  Xbase.permuteRandomly();
  for(k=0;k<10;k++){
    randomFunction(gp,Xbase,pltPause);
  }
  /*
    byteA img(H,W);
    img = 0 255;....
    write_ppm("z.ppm",img);
  */
}

int MAIN(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  if(MT::checkCmdLineTag("test")) pltPause=false;

  testGP();
  
  testDerivativeObservations();

  testRandomFunctions();

  return 0;
}
