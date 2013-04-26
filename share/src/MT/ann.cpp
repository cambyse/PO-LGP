/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


#include "ann.h"
#include "algos.h"

#ifdef MT_ANN

#include<ANN/ANN.h>

struct sANN {
  ANNbd_tree *tree;
  //PartialLeastSquares pls;
  MT::Array<double*> cpointers;
  uint treeSize;   //for how many entries in X have we build the tree?
  void clear() { if(tree) delete tree;   tree=NULL;  cpointers.clear();  treeSize=0; }
};

ANN::ANN() {
  bufferSize = 1 <<10;
  s = new sANN;
  s->tree = NULL;
  s->treeSize = 0;
}

ANN::~ANN() {
  s->clear();
  delete s;
  //annClose(); //mt09-07-29 this would close down the ANN lib completely
}

void ANN::clear() {
  s->clear();
  X.clear();
}

void ANN::setX(const arr& _XX) {
  s->clear();
  X=_XX;
}

void ANN::append(const arr& x) {
  double *p=X.p;
  X.append(x);
  if(X.N==x.d0) X.reshape(1, x.d0);
  if(X.p!=p) s->clear(); //when the memory location changed clear the tree! (implies recomputation)
}

void ANN::calculate() {
  if(s->treeSize == X.d0) return;
  s->clear();
  X.getCarray(s->cpointers);
  s->tree = new ANNbd_tree(s->cpointers.p, X.d0, X.d1);
  s->treeSize = X.d0;
}

void ANN::getNN(arr& dists, intA& idx, const arr& x, uint k, double eps, bool verbose) {
  CHECK(X.d0>=k, "data has less (" <<X.d0 <<") than k=" <<k <<" points");
  CHECK(x.N==X.d1, "query point has wrong dimension");
  
  if(X.d0-s->treeSize>bufferSize) {
    if(verbose) std::cout <<"ANN recomputing: X.d0=" <<X.d0 <<" treeSize=" <<s->treeSize <<std::endl;
    calculate();
  }
  uint restStartsAt;
  if(s->treeSize>=k) {
    dists.resize(k);
    idx.resize(k);
    s->tree->annkSearch(x.p, k, idx.p, dists.p, eps);
    restStartsAt=s->treeSize;
  } else {
    dists.clear();
    idx.clear();
    restStartsAt=0;
  }
  
  //now check if in the rest of X there are even nearer points
  for(uint i=restStartsAt; i<X.d0; i++) {
    for(uint j=0; j<=idx.N && j<k; j++) {
      double d=sqrDistance(X[i], x);
      if(j==idx.N || d < dists(j)) {
        idx.insert(j, i);
        dists.insert(j, d);
        break;
      }
    }
  }
  if(idx.N>k) {
    idx.resizeCopy(k);
    dists.resizeCopy(k);
  }
  
  if(verbose) {
    std::cout
        <<"ANN query:"
        <<"\n data size = " <<X.d0 <<"  data dim = " <<X.d1 <<"  treeSize = " <<s->treeSize
        <<"\n query point " <<x
        <<"\n found neighbors:\n";
    for(uint i=0; i<idx.N; i++) {
      std::cout <<' '
                <<i <<' '
                <<idx(i) <<'\t'
                <<sqrt(dists(i)) <<'\t'
                <<X[idx(i)] <<std::endl;
    }
  }
}

uint ANN::getNN(const arr& x, double eps, bool verbose) {
  intA idx(1);
  arr dists(1);
  getNN(dists, idx, x, 1, eps, verbose);
  return idx(0);
}

void ANN::getNN(intA& idx, const arr& x, uint k, double eps, bool verbose) {
  arr dists(k);
  getNN(dists, idx, x, k, eps, verbose);
}

void ANN::getNN(arr& xx             , const arr& x, uint k, double eps, bool verbose) {
  intA idx(k);
  arr dists(k);
  getNN(dists, idx, x, k, eps, verbose);
  xx.resize(idx.N, X.d1);
  for(uint i=0; i<idx.N; i++) xx[i]=X[idx(i)];
}

void ANN::map(arr& y, const arr& x, const arr& Y) {
  //if(x.nd==2){ y.resize(x.d0, Y.d1); for(uint k=0;k<x.d0;k++) map(x[k], y[k]()); return; }
  if(!X.N) return;
  uint k=20;
  if(k>X.d0) k=X.d0;
  arr dists(k);
  intA idx(k);
  getNN(dists, idx, x, k);
  
  LinearStatistics S;
  uint i;
  for(i=0; i<k; i++) S.learn(X[idx(i)]-x);
  //S.computeZeroMean();
  double d=.1*S.variance(), w;
  if(!d) d=1.;
  //pls.S.forget();
  S.forget();
  for(i=0; i<k; i++) {
    //w=::exp(-.5*dists(i)/d);
    w=(k-i)*(k-i); //1./(k+i);
    S.learn(Y[idx(i)], w);
    //pls.learn(X(idx(i))->x, X(idx(i))->y, w);
  }
  S.compute();
  y=S.MeanX;
  //pls.map(x, y);
}

#else //MT_ANN
ANN::ANN() { NICO }
void ANN::append(const arr& x) { NICO }
void ANN::getNN(arr& dists, intA& idx, const arr& x, uint k, double eps, bool verbose) { NICO }
uint ANN::getNN(const arr& x, double eps, bool verbose) { NICO }
void ANN::getNN(arr& xx             , const arr& x, uint k, double eps, bool verbose) { NICO }
void ANN::setX(const arr& _XX) { NICO }
ANN::~ANN() { NICO }
#endif
