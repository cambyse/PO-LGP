#include <Algo/MLcourse.h>
#include <Gui/plot.h>

//===========================================================================

void testLinReg(const char *datafile=NULL) {
  if(!datafile){ //store artificial data to a file
    datafile="z.train";
    arr X,y;
    artificialData(X, y);

    FILE(datafile) <<catCol(X,y);
  }

  //-- load data from a file
  arr X,y;
  X <<FILE(datafile);
  y = (~X)[X.d1-1];    //last row of transposed X
  X.delColumns(X.d1-1);

  //-- generate features
  arr Phi = makeFeatures(X);

  //-- compute optimal parameters
  arr Sigma;
  arr beta = ridgeRegression(Phi, y, -1., Sigma);
  cout <<"estimated beta = "<< beta <<endl;
  if(beta.N==beta_true.N) cout <<"max-norm beta-beta_true = " <<maxDiff(beta, beta_true) <<endl; //beta_true is global and generated during artificialData
  double sigma = sqrt(sumOfSqr(Phi*beta-y)/(X.d0-1));
  cout <<"Mean error (sdv) = " <<sigma <<endl;

  //-- evaluate model on a grid
  arr X_grid,y_grid;
  X_grid.setGrid(X.d1,-3,3, (X.d1==1?500:30));
  Phi = makeFeatures(X_grid, readFromCfgFileFT, X);
  y_grid = Phi*beta;
  arr s_grid = sqrt(evaluateBayesianRidgeRegressionSigma(Phi, Sigma)/*+MT::sqr(sigma)*/);

  if(X.d1==1){
    plotGnuplot();
    plotFunctionPrecision(X_grid, y_grid, y_grid+s_grid, y_grid-s_grid);
    plotPoints(X,y);
    plot(true);
  }

  //-- gnuplot
  MT::arrayBrackets="  ";
//  if(X.d1==1){
//    FILE("z.model") <<catCol(X_grid, y_grid);
//    gnuplot(STRING("plot [-3:3] '" <<datafile <<"' us 1:2 w p,'z.model' us 1:2 w l"), false, true,"z.pdf");
//  }
  if(X.d1==2){
    FILE("z.model") <<y_grid.reshape(31,31);
    FILE("z.model_s") <<(y_grid+s_grid).reshape(31,31);
    FILE("z.model__s") <<(y_grid-s_grid).reshape(31,31);
    gnuplot(STRING("splot [-3:3][-3:3] '" <<datafile <<"' w p ps 2 pt 4,\
                   'z.model' matrix us ($1/5-3):($2/5-3):3 w l,\
                   'z.model_s' matrix us ($1/5-3):($2/5-3):3 w l,\
                   'z.model__s' matrix us ($1/5-3):($2/5-3):3 w l; pause mouse"), false, true, "z.pdf");
  }
}

//===========================================================================

void testKernelReg(const char *datafile=NULL) {
  if(!datafile){ //store artificial data to a file
    datafile="z.train";
    arr X,y;
    artificialData(X, y);

    FILE(datafile) <<catCol(X,y);
  }

  //-- load data from a file
  arr X,y;
  X <<FILE(datafile);
  y = (~X)[X.d1-1];    //last row of transposed X
  X.delColumns(X.d1-1);

  KernelRidgeRegression f(X,y);
  cout <<"Mean error (sdv) = " <<f.sigma <<endl;

  //-- evaluate model on a grid
  arr X_grid, s_grid;
  X_grid.setGrid(X.d1,-3,3, (X.d1==1?500:30));
  arr y_grid = f.evaluate(X_grid, s_grid);
  s_grid = sqrt(s_grid);

  if(X.d1==1){
    plotGnuplot();
    plotFunctionPrecision(X_grid, y_grid, y_grid+s_grid, y_grid-s_grid);
    plotPoints(X,y);
    plot(true);
  }

  //-- gnuplot
  MT::arrayBrackets="  ";
//  if(X.d1==1){
//    FILE("z.model") <<catCol(X_grid, y_grid);
//    gnuplot(STRING("plot [-3:3] '" <<datafile <<"' us 1:2 w p,'z.model' us 1:2 w l"), false, true,"z.pdf");
//  }
  if(X.d1==2){
    FILE("z.model") <<y_grid.reshape(31,31);
    FILE("z.model_s") <<(y_grid+s_grid).reshape(31,31);
    FILE("z.model__s") <<(y_grid-s_grid).reshape(31,31);
    gnuplot(STRING("splot [-3:3][-3:3] '" <<datafile <<"' w p ps 2 pt 4,\
                   'z.model' matrix us ($1/5-3):($2/5-3):3 w l,\
                   'z.model_s' matrix us ($1/5-3):($2/5-3):3 w l,\
                   'z.model__s' matrix us ($1/5-3):($2/5-3):3 w l;\
                    pause mouse"), false, true, "z.pdf");
  }
}

//===========================================================================

void test2Class() {
//  rnd.seed(1);

  arr X,y;
  //artificialData_HastiesMultiClass(X, y);  y = ~((~y)[1]);  y.reshape(y.N);
  artificialData_Hasties2Class(X, y);
  
  arr Phi = makeFeatures(X);
  arr Sigma;
  arr beta = logisticRegression2Class(Phi, y, -1., Sigma);
  
  arr X_grid;
  X_grid.setGrid(X.d1,-3,3, (X.d1==1?500:50));
  Phi = makeFeatures(X_grid,readFromCfgFileFT, X);
  arr y_grid = Phi*beta;
  arr s_grid = evaluateBayesianRidgeRegressionSigma(Phi, Sigma);
  arr ybay_grid = y_grid/ sqrt(1.+s_grid*MT_PI/8.); //bayesian logistic regression: downscale discriminative function
  s_grid=sqrt(s_grid);

  arr p_grid=exp(y_grid); p_grid /= p_grid+1.;
  arr pup_grid=exp(y_grid+s_grid); pup_grid /= pup_grid+1.;
  arr pdn_grid=exp(y_grid-s_grid); pdn_grid /= pdn_grid+1.;
  arr pba_grid=exp(ybay_grid); pba_grid /= pba_grid+1.;

  if(X.d1==1){
    plotGnuplot();
    plotFunctionPrecision(X_grid, p_grid, pup_grid, pdn_grid);
    plotFunction(X_grid, pba_grid);
    plotPoints(X,y);
    plot(true);
  }

  MT::arrayBrackets="  ";
//  if(X.d1==1){
//    FILE("z.train") <<catCol(X, y);
//    FILE("z.model") <<catCol(X_grid, p_grid);
//    gnuplot(STRING("plot [-3:3] 'z.train' us 1:2 w p,'z.model' us 1:2 w l"), false, true, "z.pdf");
//  }
  if(X.d1==2){
    FILE("z.train") <<catCol(X, y);
    FILE("z.model") <<p_grid.reshape(51,51);
    gnuplot("load 'plt.contour'; pause mouse", false, true, "z.pdf");
    gnuplot("load 'plt.contour2'; pause mouse", false, true, "z.pdf");
  }
}

//===========================================================================

void testKernelLogReg() {
//  rnd.seed(1);

  arr X,y;
  artificialData_Hasties2Class(X, y);

  KernelLogisticRegression klr(X,y);

  arr X_grid;
  X_grid.setGrid(X.d1,-3,3, (X.d1==1?500:50));
  arr p_ba,p_hi,p_lo;
  arr p_grid = klr.evaluate(X_grid, p_ba, p_hi, p_lo);

  if(X.d1==1){
    plotGnuplot();
    plotFunctionPrecision(X_grid, p_grid, p_hi, p_lo);
    plotFunction(X_grid, p_ba);
    plotPoints(X,y);
    plot(true);
  }
  if(X.d1==2){
    MT::arrayBrackets="  ";
    FILE("z.train") <<catCol(X, y);
    FILE("z.model") <<p_grid.reshape(51,51);
    gnuplot("load 'plt.contour'; pause mouse", false, true, "z.pdf");
    gnuplot("load 'plt.contour2'; pause mouse", false, true, "z.pdf");
  }
}

//===========================================================================

void testMultiClass() {
  //rnd.seed(1);
  rnd.clockSeed();

  arr X,y;
  artificialData_HastiesMultiClass(X, y);
  
  arr Phi = makeFeatures(X);
  arr beta = logisticRegressionMultiClass(Phi, y);
  
  arr p_pred,label(Phi.d0);
  p_pred = exp(Phi*beta);
  for(uint i=0; i<label.N; i++) {
    p_pred[i]() /= sum(p_pred[i]);
    label(i) = y[i].maxIndex();
  }
  MT::arrayBrackets="  ";
  FILE("z.train") <<catCol(X, label, y, p_pred);
  
  arr X_grid,p_grid;
  X_grid.setGrid(2,-2,3,50);
  Phi = makeFeatures(X_grid,readFromCfgFileFT,X);
  p_grid = exp(Phi*beta);
  for(uint i=0; i<p_grid.d0; i++) p_grid[i]() /= sum(p_grid[i]);
  p_grid = ~p_grid;
  p_grid.reshape(p_grid.d0,51,51);
  
  FILE("z.model1") <<p_grid[0];
  FILE("z.model2") <<p_grid[1];
  if(y.d1==3){
    FILE("z.model3") <<p_grid[2];
    gnuplot("load 'plt.contourMulti'; pause mouse", false, true, "z.pdf");
    gnuplot("load 'plt.contourMulti2'; pause mouse", false, true, "z.pdf");
  }
  if(y.d1==4){
    FILE("z.model3") <<p_grid[2];
    FILE("z.model4") <<p_grid[3];
    gnuplot("load 'plt.contourM4'; pause mouse", false, true, "z.pdf");
    gnuplot("load 'plt.contourM4_2'; pause mouse", false, true, "z.pdf");
  }
}

void testCV() {

  struct myCV:public CrossValidation {
    void  train(const arr& X, const arr& y, double param, arr& beta) {
      beta = ridgeRegression(X,y,param);
    };
    double test(const arr& X, const arr& y, const arr& beta) {
      arr y_pred = X*beta;
      return sumOfSqr(y_pred-y)/y.N;
    };
  } cv;
  
  rnd.clockSeed();
  arr X,y;
  artificialData(X, y);
  arr Phi = makeFeatures(X);
  FILE("z.train") <<catCol(X, y);

  cv.crossValidateMultipleLambdas(Phi, y, ARR(1e-3,1e-2,1e-1,1e0,1e1,1e2,1e3,1e4,1e5), 10, false);
  cv.plot();
  cout <<"10-fold CV:\n  costMeans= " <<cv.scoreMeans <<"\n  costSDVs= " <<cv.scoreSDVs <<endl;
}

void exercise1() {
  arr X,y;

  //load the data, split in input and output
  X <<FILE("./dataQuadReg2D_noisy.txt");
  y = (~X)[X.d1-1];    //last row of transposed X
  X.delColumns(X.d1-1);
  
  //compute optimal beta
  arr Phi = makeFeatures(X);
  arr beta = ridgeRegression(Phi, y);
  cout <<"estimated beta = "<< beta <<endl;

  //compute mean squared error
  arr y_pred = Phi*beta;
  cout <<"error = "<<sumOfSqr(y_pred-y)/y.N <<endl;

  //predict on grid
  arr X_grid,y_grid;
  X_grid.setGrid(X.d1,-3,3,30);
  Phi = makeFeatures(X_grid,readFromCfgFileFT,X);
  y_grid = Phi*beta;

  //save and plot
  MT::arrayBrackets="  ";
  FILE("z.train") <<catCol(X, y);
  if(X.d1==1) {
    FILE("z.model") <<catCol(X_grid, y_grid);
    gnuplot("plot 'z.train' us 1:2 w p,'z.model' us 1:2 w l", false, true, "z.pdf");
  } else {
    y_grid.reshape(31,31);
    FILE("z.model") <<y_grid;
    gnuplot("splot [-3:3][-3:3] 'z.train' w p, 'z.model' matrix us ($1/5-3):($2/5-3):3 w l", false, true, "z.pdf");
  }
}

void exercise2() {
  arr X,Phi,y;
  arr beta;

  //provide virtual train and test routines for CV
  struct myCV:public CrossValidation {
    void  train(const arr& X, const arr& y, double param, arr& beta) {
      beta = ridgeRegression(X, y, param); //returns optimal beta for training data
    };
    double test(const arr& X, const arr& y, const arr& beta) {
      arr y_pred = X*beta;
      return sumOfSqr(y_pred-y)/y.N; //returns MSE on test data
    };
  } cv;

  //load the data, split in input and output
  X <<FILE("./dataQuadReg2D_noisy.txt");
  y = (~X)[X.d1-1];    //last row of transposed X
  X.delColumns(X.d1-1);

  //make data less
  //COMMENT: the data has too little noise: when going down to 3 data points one sees the regularization need
//  uint n=5;
//  y.resizeCopy(n);
//  X.resizeCopy(n,X.d1);

  //cross valide
  Phi = makeFeatures(X);
  cv.crossValidateMultipleLambdas(Phi, y,
				  ARR(1e-3,1e-2,1e-1,1e0,1e1,1e2,1e3,1e4,1e5),
				  10, false);
  cv.plot();
  cout <<"10-fold CV:\n  costMeans= " <<cv.scoreMeans <<"\n  costSDVs= " <<cv.scoreSDVs <<endl;
}

//===========================================================================

int main(int argc, char *argv[]) {
  MT::initCmdLine(argc,argv);
  
//  rnd.clockSeed();

  switch(MT::getParameter<uint>("mode",1)) {
    case 1:  testLinReg();  break;
    case 2:  test2Class();  break;
    case 3:  testMultiClass();  break;
    case 4:  testCV();  break;
    case 5:  testKernelReg();  break;
    case 6:  testKernelLogReg();  break;
      break;
  }
  
  return 0;
}

