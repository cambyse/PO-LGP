#include <Core/array.h>
#include <Algo/MLcourse.h>


int main(int argn, char** argv){
  arr D;
  D <<FILE("data1.output");

  uint range=20;
  arr y;
  y = (~D)[0];
  arr X;
  X.setGrid(1,-1.,0.,range);
  X.reshape(X.N,1);
//  X.append(ones(1,X.N));
//  X=~X;
  arr Phi;
  makeFeatures(Phi, X, quadraticFT);
//  cout <<y <<' ' <<X <<Phi <<endl;

  ofstream fil("z.y");

  uint t=0;
  for(;t<range;t++) fil <<y(t) <<endl; //just copy the first 10 pts
  for(;t<y.N;t++){
    arr y_sub = y.sub(t-range,t);
    arr beta;

    ridgeRegression(beta, Phi, y_sub);
    double y_pred = scalarProduct(beta, Phi[Phi.d0-1]);
    fil <<y_pred <<' ' <<beta(1) <<endl;
  }
  fil.close();

  gnuplot("plot [:200] 'data1.output' us 1, 'z.y' us 1, 'z.y' us 2", false, true);


  return 0;
}
