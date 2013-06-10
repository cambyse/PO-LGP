/*  ---------------------------------------------------------------------
    Copyright 2013 Marc Toussaint
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



#ifndef MT_ann_h
#define MT_ann_h

#include "array.h"

//===========================================================================
//
// Approximate Nearest Neighbor Search (kd-tree)
//

struct ANN {
  struct sANN *s;
  
  arr X;       //the data set for which a ANN tree is build
  uint bufferSize; //a tree is only rebuild if there are more than 'buffer' new points appended [default: 20]
  
  ANN();
  ~ANN();
  
  void clear();              //clears the tree and X
  void setX(const arr& _X);  //set X
  void append(const arr& x); //append to X
  void calculate();          //compute a tree for all of X
  
  uint getNN(const arr& x, double eps=.0, bool verbose=false);
  void getNN(intA& idx, const arr& x, uint k, double eps=.0, bool verbose=false);
  void getNN(arr& sqrDists, intA& idx, const arr& x, uint k, double eps=.0, bool verbose=false);
  void getNN(arr& xx, const arr& x, uint k, double eps=.0, bool verbose=false);
  
  void map(arr& y, const arr& x, const arr& Y); //given a data set Y of outputs for each X, do a NN regression to predict y at x
};

#endif
