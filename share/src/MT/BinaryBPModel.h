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


#include "BinaryBP.h"
#include "data.h"
#include "optimization_obsolete.h"

struct BinaryBPNetModel:virtual public OptimizationProblem {
  Data *data;
  BinaryBPNet net;
  uint ITER;
  double regJ, regT;
  uintA layers;
   
  void initLayered(Data *_data);
  void randomInit();
  
  void beliefs(arr& b, const arr& w);
  void d_beliefs(arr& dbdw, const arr& w);
  
  double bethe();
  void d_bethe(arr& grad);
  
  void model(arr& output, const arr& input, const arr& w);
  void model(Data& data, const arr& w);
  
  double logLikelihood(const arr& w, const Data& data, arr* grad=NULL);
  
  double loss(arr& output, const arr& target, const arr& input, const arr& w, arr *grad); //loss on the output
  double loss(const arr& w, const uintA& range, const Data& data, arr *grad, double *err);
  double totalLoss(const arr& w, const Data& data, arr *grad, double *err);
  
  void EMtrain(const arr& w, const Data& data);
  
  virtual double loss(const arr& w, uint i, arr *grad, double *err){ arr output; if(err)(*err)=-1.; return loss(output, data->Y[i], data->X[i], w, grad); }
  virtual double totalLoss(const arr& w, arr *grad, double *err){   return totalLoss(w, *data, grad, err); }
};

#ifdef  MT_IMPLEMENTATION
#  include "BinaryBPModel.cpp"
#endif
