#include <Core/array.h>
#include <MT/util.h>
#include <MT/plot.h>
#include <stdlib.h>
using namespace std;

#include <MT/MLcourse.h>
#include <MT/util.h>

void testLinReg() {
  arr X,Phi,y;
  arr beta;
  
  rnd.clockSeed();
  artificialData(X, y);
  makeFeatures(Phi, X, X);
  ridgeRegression(beta, Phi, y);
  cout <<"estimated beta = "<< beta <<endl;
  if(beta.N==beta_true.N){
    cout <<"max-norm beta-beta_true = " <<maxDiff(beta, beta_true) <<endl;
  }

  write(LIST<arr>(X, y), "z.train");
  
  arr X_grid,y_grid;
  X_grid.setGrid(X.d1,-3,3, (X.d1==1?100:30));
  makeFeatures(Phi,X_grid,X);
  y_grid = Phi*beta;

  if(X.d1==1){
    write(LIST<arr>(X_grid, y_grid), "z.model");
    gnuplot("plot [-3:3] 'z.train' us 1:2 w p,'z.model' us 1:2 w l", false, true,"z.pdf");
  }else{
    y_grid.reshape(31,31);
    write(LIST<arr>(y_grid), "z.model");
    gnuplot("splot [-3:3][-3:3] 'z.train' w p, 'z.model' matrix us ($1/5-3):($2/5-3):3 w l; pause mouse", false, true, "z.pdf");
  }
}

void test2Class() {
  rnd.clockSeed();
  //rnd.seed(1);

  arr X,Phi,y;
  arr beta;
  //artificialData_HastiesMultiClass(X, y);  y = ~((~y)[1]);  y.reshape(y.N);
  artificialData_Hasties2Class(X, y);
  
  makeFeatures(Phi,X,X);
  logisticRegression2Class(beta, Phi, y);
  
  arr X_grid,y_grid,p_grid;
  X_grid.setGrid(2,-2,3,50);
  makeFeatures(Phi,X_grid,X);
  y_grid = Phi*beta;
  
  p_grid=exp(y_grid);
  for(uint i=0; i<p_grid.N; i++) p_grid(i) = p_grid(i)/(p_grid(i)+1.);
  
  y_grid.reshape(51,51);
  p_grid.reshape(51,51);
  
  write(LIST<arr>(X, y), "z.train");
  write(LIST<arr>(p_grid), "z.model");
  gnuplot("load 'plt.contour'; pause mouse", false, true, "z.pdf");
  gnuplot("load 'plt.contour2'; pause mouse", false, true, "z.pdf");
}

void testMultiClass() {
  //rnd.seed(1);
  rnd.clockSeed();

  arr X,Phi,y;
  arr beta;
  artificialData_HastiesMultiClass(X, y);
  
  makeFeatures(Phi,X,X);
  logisticRegressionMultiClass(beta, Phi, y);
  
  arr p_pred,label(Phi.d0);
  p_pred = exp(Phi*beta);
  for(uint i=0; i<label.N; i++) {
    p_pred[i]() /= sum(p_pred[i]);
    label(i) = y[i].maxIndex();
  }
  write(LIST<arr>(X, label, y, p_pred), "z.train");
  
  arr X_grid,p_grid;
  X_grid.setGrid(2,-2,3,50);
  makeFeatures(Phi,X_grid,X);
  p_grid = exp(Phi*beta);
  for(uint i=0; i<p_grid.d0; i++) p_grid[i]() /= sum(p_grid[i]);
  p_grid = ~p_grid;
  p_grid.reshape(p_grid.d0,51,51);
  
  write(LIST<arr>(p_grid[0]), "z.model1");
  write(LIST<arr>(p_grid[1]), "z.model2");
  if(y.d1==3){
    write(LIST<arr>(p_grid[2]), "z.model3");
    gnuplot("load 'plt.contourMulti'; pause mouse", false, true, "z.pdf");
    gnuplot("load 'plt.contourMulti2'; pause mouse", false, true, "z.pdf");
  }
  if(y.d1==4){
    write(LIST<arr>(p_grid[2]), "z.model3");
    write(LIST<arr>(p_grid[3]), "z.model4");
    gnuplot("load 'plt.contourM4'; pause mouse", false, true, "z.pdf");
    gnuplot("load 'plt.contourM4_2'; pause mouse", false, true, "z.pdf");
  }
}

void testCV() {

  struct myCV:public CrossValidation {
    void  train(const arr& X, const arr& y, double param, arr& beta) {
      ridgeRegression(beta, X,y,param);
    };
    double test(const arr& X, const arr& y, const arr& beta) {
      arr y_pred = X*beta;
      return sumOfSqr(y_pred-y)/y.N;
    };
  } cv;
  
  rnd.clockSeed();
  arr X,Phi,y;
  artificialData(X, y);
  makeFeatures(Phi,X,X);
  write(LIST<arr>(X, y), "z.train");

  cv.crossValidateMultipleLambdas(Phi, y, ARR(1e-3,1e-2,1e-1,1e0,1e1,1e2,1e3,1e4,1e5), 10, false);
  cv.plot();
  cout <<"10-fold CV:\n  costMeans= " <<cv.scoreMeans <<"\n  costSDVs= " <<cv.scoreSDVs <<endl;
}

void exercise1() {
  arr X,Phi,y;
  arr beta;

  //load the data, split in input and output
  X.read("./dataQuadReg2D_noisy.txt");
  y = (~X)[X.d1-1];    //last row of transposed X
  X.delColumns(X.d1-1);
  
  //compute optimal beta
  makeFeatures(Phi,X,X);
  ridgeRegression(beta, Phi, y);
  cout <<"estimated beta = "<< beta <<endl;

  //compute mean squared error
  arr y_pred = Phi*beta;
  cout <<"error = "<<sumOfSqr(y_pred-y)/y.N <<endl;

  //predict on grid
  arr X_grid,y_grid;
  X_grid.setGrid(X.d1,-3,3,30);
  makeFeatures(Phi,X_grid,X);
  y_grid = Phi*beta;

  //save and plot
  write(LIST<arr>(X, y), "z.train");
  if(X.d1==1) {
    write(LIST<arr>(X_grid, y_grid), "z.model");
    gnuplot("plot 'z.train' us 1:2 w p,'z.model' us 1:2 w l", false, true, "z.pdf");
  } else {
    y_grid.reshape(31,31);
    write(LIST<arr>(y_grid), "z.model");
    gnuplot("splot [-3:3][-3:3] 'z.train' w p, 'z.model' matrix us ($1/5-3):($2/5-3):3 w l", false, true, "z.pdf");
  }
}

void exercise2() {
  arr X,Phi,y;
  arr beta;

  //provide virtual train and test routines for CV
  struct myCV:public CrossValidation {
    void  train(const arr& X, const arr& y, double param, arr& beta) {
      ridgeRegression(beta, X, y, param); //returns optimal beta for training data
    };
    double test(const arr& X, const arr& y, const arr& beta) {
      arr y_pred = X*beta;
      return sumOfSqr(y_pred-y)/y.N; //returns MSE on test data
    };
  } cv;

  //load the data, split in input and output
  X.read("./dataQuadReg2D_noisy.txt");
  y = (~X)[X.d1-1];    //last row of transposed X
  X.delColumns(X.d1-1);

  //make data less
  //COMMENT: the data has too little noise: when going down to 3 data points one sees the regularization need
//  uint n=5;
//  y.resizeCopy(n);
//  X.resizeCopy(n,X.d1);

  //cross valide
  makeFeatures(Phi,X,X);
  cv.crossValidateMultipleLambdas(Phi, y,
				  ARR(1e-3,1e-2,1e-1,1e0,1e1,1e2,1e3,1e4,1e5),
				  10, false);
  cv.plot();
  cout <<"10-fold CV:\n  costMeans= " <<cv.scoreMeans <<"\n  costSDVs= " <<cv.scoreSDVs <<endl;
}


int main(int argc, char *argv[]) {
  MT::initCmdLine(argc,argv);
  
  system("cat USAGE");
  
  switch(MT::getParameter<uint>("mode",1)) {
    case 1:  testLinReg();  break;
    case 2:  test2Class();  break;
    case 3:  testMultiClass();  break;
    case 4:  testCV();  break;
    case 5:  exercise1();  break;
    case 6:  exercise2();  break;
      break;
  }
  
  return 0;
}

