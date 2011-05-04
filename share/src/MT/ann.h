#ifndef MT_ann_h
#define MT_ann_h

#include <MT/array.h>

//===========================================================================
//
// Approximate Nearest Neighbor Search (kd-trees, etc)
//

struct sANN; //private space

struct ANN{
  sANN *s;

  arr X;       //the data set for which a ANN tree is build
  uint buffer; //a tree is only rebuild if there are more than 'buffer' new points appended [default: 20]

  ANN();
  ~ANN();

  void clear();              //clears the tree and X
  void setX(const arr& _X);  //set X
  void append(const arr& x); //append to X
  void calculate();          //compute a tree for all of X

  uint getNN(const arr& x,double eps=.0,bool verbose=false);
  void getNN(intA& idx           ,const arr& x,uint k,double eps=.0,bool verbose=false);
  void getNN(arr& dists,intA& idx,const arr& x,uint k,double eps=.0,bool verbose=false);
  void getNN(arr& xx             ,const arr& x,uint k,double eps=.0,bool verbose=false);

  void map(arr& y,const arr& x,const arr& Y); //given a data set Y of outputs for each X, do a NN regression to predict y at x
};

#ifdef MT_IMPLEMENTATION
#include "ann.cpp"
#endif

#endif
