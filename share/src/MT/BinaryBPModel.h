#include "BinaryBP.h"
#include "data.h"
#include "optimization.h"

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
