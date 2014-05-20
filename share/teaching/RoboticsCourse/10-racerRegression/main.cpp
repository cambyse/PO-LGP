#include <Core/array.h>
#include <Core/util.h>
#include <Gui/plot.h>
#include <stdlib.h>
using namespace std;

#include <Algo/MLcourse.h>

void linReg() {
  arr data = FILE("imuKalman-01.dat");
  CHECK(data.d1 = 2+4+2+4+4,"");

  arr X = data.sub(0,-1,2,7); //2,11 versus 2,7
  arr Y = data.sub(0,-1,12,15);
  arr modelPred = data.sub(0,-1,8,11);
  arr Phi, beta;
  
  Phi = makeFeatures(X);
  beta = ridgeRegression(Phi, Y);

  arr Y_pred = Phi*beta;
  cout <<"MSE = " <<sumOfSqr(Y_pred-Y)/Y.N <<endl;
  cout <<"MSE of model predicted = " <<sumOfSqr(modelPred-Y)/Y.N <<endl;
  write(LIST<arr>(X, Y, Y_pred), "z.train");
  beta >>FILE("z.beta");
}


int main(int argc, char *argv[]) {
  MT::initCmdLine(argc,argv);
  system("cat USAGE");

  linReg();
  
  return 0;
}

