#include <Core/array.h>
#include <Gui/plot.h>

void TEST(Plot) {
  //plotGnuplot();
  plotOpengl();
  doubleA mean(2),A;

  A.resize(10,10);
  rndUniform(A,-.1,.1,false);
  plotSurface(A);
  rndUniform(A,-.1,.1,false);
  plotSurface(A);
  plot();

  mean=0.;
  A.setDiag(1.,2); A(0,1)=A(1,0)=-.5; //A(1,1)=2.;
  plotCovariance(mean,A);
  plot();

  A.resize(10);
  rndUniform(A,-1,1,false);
  plotFunction(A);
  plot();

  A.resize(20,2);
  rndUniform(A,-1,1,false);
  plotPoints(A);
  plot();

}

int MAIN(int argc,char** argv){
  testPlot();

  return 0;
}
