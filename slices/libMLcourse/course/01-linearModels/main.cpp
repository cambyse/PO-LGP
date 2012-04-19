#include<MT/array.h>
#include<MT/util.h>
#include<MT/plot.h>

using namespace std;

#include <MT/MLcourse.h>
#include <MT/util.h>


void testLinReg(){
  arr X,Phi,y;
  arr beta;

  rnd.clockSeed();
  artificialData(X,y);
  makeFeatures(Phi,X,X);
  ridgeRegression(beta, Phi, y, MT::getParameter<double>("ridge",1e-10));
  cout <<"estimated beta = "<< beta <<endl;

  arr X_test,y_test;
  X_test.setGrid(X.d1,-3,3,100);
  makeFeatures(Phi,X_test,X);
  y_test = Phi*beta;

  write(LIST(X, y), "z.train");
  write(LIST(X_test, y_test), "z.model");
  gnuplot("plot 'z.train' us 1:2 w p,'z.model' us 1:2 w l","z.pdf",true);
  //MT::wait();
}

void test2Class(){
  rnd.seed(1);
  arr X,Phi,y;
  arr beta;
  //artificialData_HastiesMultiClass(X, y);  y = ~((~y)[1]);  y.reshape(y.N);
  artificialData_Hasties2Class(X, y);
  
  makeFeatures(Phi,X,X);
  logisticRegression2Class(beta, Phi,y,MT::getParameter<double>("ridge",1e-10));
  
  arr X_test,y_test,p_test;
  X_test.setGrid(2,-2,3,100);
  makeFeatures(Phi,X_test,X);
  y_test = Phi*beta;

  p_test=exp(y_test);
  for(uint i=0;i<p_test.N;i++) p_test(i) = p_test(i)/(p_test(i)+1.);

  y_test.reshape(101,101);
  p_test.reshape(101,101);
  
  write(LIST(X, y), "z.train");
  write(LIST(y_test), "z.model");
  write(LIST(p_test), "z.model2");
  gnuplot("load 'plt.contour'","z.pdf",true);
}

void testMultiClass(){
  rnd.seed(1);
  arr X,Phi,y;
  arr beta;
  artificialData_HastiesMultiClass(X, y);
  
  makeFeatures(Phi,X,X);
  logisticRegressionMultiClass(beta, Phi,y,MT::getParameter<double>("ridge",1e-10));

  arr p_pred,label(Phi.d0);
  p_pred = exp(Phi*beta);
  for(uint i=0;i<label.N;i++){
    p_pred[i]() /= sum(p_pred[i]);
    label(i) = y[i].maxIndex();
  }
  write(LIST(X, label, y, p_pred), "z.train");
  
  arr X_test,p_test;
  X_test.setGrid(2,-2,3,50);
  makeFeatures(Phi,X_test,X);
  p_test = exp(Phi*beta);
  for(uint i=0;i<p_test.d0;i++) p_test[i]() /= sum(p_test[i]);
  p_test = ~p_test;
  p_test.reshape(p_test.d0,51,51);
  
  write(LIST(p_test[0]), "z.model");
  write(LIST(p_test[1]), "z.model2");
  if(p_test.d1>2) write(LIST(p_test[2]), "z.model3");
  gnuplot("load 'plt.contourMulti'","z.pdf",true);
}

void testCV(){
  
  struct myCV:public CrossValidation{
    void  train(const arr& X, const arr& y, double param, arr& beta){
      ridgeRegression(beta, X,y,param);

      //arr y_pred = X*beta;
      //write(LIST(X, y, y_pred), "data");
      //gnuplot("plot 'data' us 2:3 w p,'data' us 2:4 w l");
      //MT::wait();
    };
    double test(const arr& X, const arr& y, const arr& beta){
      arr y_pred = X*beta;
      return sumOfSqr(y_pred-y)/y.N;
    };
  } cv;

  rnd.clockSeed();
  arr X,Phi,y;
  artificialData(X,y,sinusData); //,20,10);
  makeFeatures(Phi,X,X);

  //cv.crossValidate(Phi,y,0.,10);
  cv.crossValidateMultipleLambdas(Phi, y, ARR(1e-3,1e-2,1e-1,1e0,1e1,1e2,1e3,1e4,1e5), 10, false);
  cv.plot();
  cout <<"10-fold CV: costMeans= " <<cv.scoreMeans <<" costSDVs= " <<cv.scoreSDVs <<endl;
  
}

void exercise(){
  arr X,Phi,y;
  arr beta;

  ifstream fil("./dataQuadReg2D.txt");
  X.read(fil);
  y = (~X)[X.d1-1];
  X.delColumns(X.d1-1);
  //cout <<X <<y;

  makeFeatures(Phi,X,X);
  ridgeRegression(beta, Phi,y,MT::getParameter<double>("ridge",1e-10));
  cout <<"estimated beta = "<< beta <<endl;

  arr y_pred = Phi*beta;
  cout <<"error = "<<sumOfSqr(y_pred-y)/y.N <<endl;

  arr X_test,y_test;
  X_test.setGrid(X.d1,-3,3,50);
  makeFeatures(Phi,X_test,X);
  y_test = Phi*beta;

  write(LIST(X, y), "z.train");
  write(LIST(X_test, y_test), "z.model");
  if(X.d1==1){
    gnuplot("plot 'z.train' us 1:2 w p,'z.model' us 1:2 w l","z.pdf",true);
  }else{
    gnuplot("splot 'z.train' w p,'z.model' w l","z.pdf",true);
  }
  //MT::wait();
}


int main(int argc, char *argv[]){
  MT::initCmdLine(argc,argv);
  
  switch(MT::getParameter<uint>("mode",1)){
    case 1:  testLinReg();  break;
    case 2:  test2Class();  break;
    case 3:  testMultiClass();  break;
    case 4:  testCV();  break;
    case 5:  exercise();  break;
  }
  
  return 0;
}

