#include <Core/array.h>
#include <Algo/MLcourse.h>


int main(int argn, char** argv){
  arr D;
  D <<FILE("data1.output");

  uint range=10;
  arr y;
  y = (D);

  arr X;
  X.setGrid(1,-0.001*range,0.,range-1);
  X.reshape(X.N,1);

  cout << repmat(~X,3,1) << endl;
  return 0;


  arr Phi,Phit,I,gram;
  double lambda = 1e-10;
  makeFeatures(Phi, X, quadraticFT);
  transpose(Phit, Phi);
  I.setDiag(lambda, Phi.d1);
  I(0, 0)=1e-10; //don't regularize beta_0 !!
  lapack_inverseSymPosDef(gram,Phit*Phi+I);
  gram=gram*Phit;
  ofstream fil("z.y");

  uint t=0;
  for(;t<range;t++) fil <<y[t] <<endl; //just copy the first 10 pts

  arr y_sub = y.subRange(0,range-1);
  cout << y.d0-1 << endl;
  for(;t<y.d0-1;t++){

    y_sub = ~y_sub;
    y_sub.shift(-1);
    y_sub = ~y_sub;
    y_sub[y_sub.d0-1] = y[t];

    arr beta;
    beta = gram*y_sub;
    double yd_fd = (y(t,0)-y(t-1,0))/0.001;

    fil << beta(0,0) <<' ' <<beta(1,0) <<' '<< yd_fd  <<endl;
  }
  fil.close();
  gnuplot("plot 'z.y' us 3,'z.y' us 2,'z.y' us 1", false, true);

  return 0;
}
