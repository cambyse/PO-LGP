#include <Core/array.h>
#include <Core/util.h>
#include <Gui/plot.h>

using namespace std;

#include <Algo/MLcourse.h>

void load_data(arr& X,const char* filename){
  ifstream is;
  MT::open(is,filename);
  MT::Array<MT::String> strs;
  if(!MT::contains("0123456789.-+",MT::peerNextChar(is))){
    //read line of strings
    MT::String str;
    for(;;){
      str.read(is," \"\t\r"," \"\t\r\n",false);
      if(!str.N) break;
      strs.append(str);
    }
    cout <<"header: " <<strs <<endl;
  }
  X.clear();
  X.read(is);
  for(uint i=0;i<X.d0;i++) X(i,3)/=360.; //rescale steer!!
  cout <<"data stats:"
       <<"\n  data entries    n=" <<X.d0
       <<"\n  entry dimension d=" <<X.d1
       <<"\n  stats: [# 'name' mean sdv]" <<endl;
  arr mean = sum(X,0);  mean /= (double)X.d0;
  arr var = ~X*X;       var /= (double)X.d0;
  var -= mean^mean;
  for(uint j=0;j<X.d1;j++){
    cout <<j <<' ';
    if(strs.N) cout <<strs(j) <<' ';
    cout <<mean(j) <<' ' <<sqrt(var(j,j)) <<endl;
  }

  //-- whiten the data
  if(MT::getParameter<bool>("whitenData",0)){
    for(uint i=0;i<X.d0;i++) for(uint j=0;j<X.d1;j++){
      X(i,j) /= sqrt(var(j,j));
    }
  }
}

void prepare_features(arr& Phi, arr& y, const arr& X, uint k, uint D){
  uint i,n=X.d0;
  const uint t=0,O=1,V=2,S=3,B=4,T=5;
  //-- build y
  y.resize(n);
  for(i=0;i<n;i++){
    uint a=(i>D)?i-D:0;
    uint b=(i+D<n)?i+D:n-1;
    y(i) = (X(b,V) - X(a,V))/(X(b,t) - X(a,t));
  }

  //-- build history features
  arr Z(n,4+3*k);
  for(i=0;i<n;i++){
    for(uint j=0;j<4;j++){ //copy V S B T
      Z(i,j) = X(i,2+j);
    }
    for(uint j=0;j<k;j++){ //history of S B T
      uint h=(i>j+1)?i-1-j:0;
      Z(i,4    +j) = X(h,S);
      Z(i,4+  k+j) = X(h,B);
      Z(i,4+2*k+j) = X(h,T);
    }
  }

  //-- build regression features
  makeFeatures(Phi, Z, Z);
}

void exercise(){
  arr X,Phi,y,beta;
  arr zScores;
  load_data(X,"autonomous.txt");
  prepare_features(Phi,y,X, MT::getParameter<uint>("k",2), MT::getParameter<uint>("D",5));
  ridgeRegression(beta, Phi, y, MT::getParameter<double>("ridge",1e-10), NULL, &zScores);
  cout <<"estimated beta = "<< beta <<endl;
  cout <<"z-scores= " <<zScores <<endl;

  arr y_pred = Phi*beta;
  cout <<"error = "<<sumOfSqr(y_pred-y)/y.N <<endl;

  write(LIST<arr>(X, y, y_pred), "z.pred");
  gnuplot("plot 'z.pred' us 1:7 w l title 'data','z.pred' us 1:8 w l title 'prediction'", false, true, "z.pdf");

  //-- cross validation
  struct myCrossValidation:public CrossValidation{
    arr beta;
    void  train(const arr& X, const arr& y,double lambda){
      ridgeRegression(beta, X, y, lambda);
    }
    double test(const arr& X, const arr& y){
      arr y_pred = X*beta;
      return sumOfSqr(y_pred-y)/y.N;
    }
  } CV;
  
  //CV.crossValidate(Phi, y, MT::getParameter<double>("ridge",1e-10), 10, MT::getParameter<bool>("permute",false), &mean, &svd, &train);
  CV.crossValidate(Phi, y, MT::getParameter<arr>("ridges"), 10, MT::getParameter<bool>("permute",false));
}



int main(int argc, char *argv[]){
  MT::initCmdLine(argc,argv);

  exercise();
  
  return 0;
}

