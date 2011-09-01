#ifndef MT_data_h
#define MT_data_h

#include "array.h"

struct Data {
  arr X, Y, Z;
  arr Xtest, Ytest;
  MT::String tag;
  
  uint N()    const { return X.d0; }
  uint Xdim() const { return X.d1; }
  uint Ydim() const { return Y.d1; }
  
  double costMaxClassError(){
    uint E=0;
    for(uint i=0; i<N(); i++) if(Y[i].maxIndex()!=Z[i].maxIndex()) E++;
    return double(E)/N();
  }
  double costHammingDist(){
    uint E=0;
    for(uint i=0; i<N(); i++){
      for(uint j=0; j<Ydim(); j++) if(Y(i, j)>.5){ if(Z(i, j)<=0.) E++; }else{ if(Z(i, j)>=0.) E++; }
    }
    return double(E)/(N()*Ydim());
  }
  double costNegLogLike(){
    double C=0.;
    uint i, j;
    for(i=0; i<N(); i++){
      for(j=0; j<Ydim(); j++) C -= (2.*Y(i, j)-1.) * Z(i, j) - log(2.*cosh(Z(i, j))); //NEG log-likelihood
    }
    return C/N();
  }
  void dump(){
    for(uint i=0; i<N(); i++){
      cout  <<" x="  <<X[i]  <<"\n y="  <<Y[i]  <<"\n z="  <<Z[i]  <<endl;
    }
  }
  void loadSpecific(bool test);
  void loadToyData(const char *filename, uint labels);
  void loadSvmMultiLabelData(const char *filename, bool inSharePath=true);
  void loadUSPS(const char *filename, bool inSharePath=true);
  void displayInput(uint i, uint height=16);
  
  void displayAllInputs();
  void reduce(uint dx, uint dy);
  void splitTest(Data &test);
};

#ifdef  MT_IMPLEMENTATION
#  include "data.cpp"
#endif

#endif