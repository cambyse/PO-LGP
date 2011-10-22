#ifndef MT_linearRegression_h
#define MT_linearRegression_h

#include "array.h"

//===========================================================================
//
// basic regression and classification methods
//

void linearRegression(arr& beta, const arr& X, const arr& y, const arr* weighted=NULL);
void ridgeRegression(arr& beta, const arr& X, const arr& y, double lambda, const arr* weighted=NULL, arr* zScores=NULL);
void logisticRegression2Class(arr& beta, const arr& X, const arr& y, double lambda);
void logisticRegressionMultiClass(arr& beta, const arr& X, const arr& y, double lambda);


//===========================================================================
//
// cross validation
//

struct CrossValidation {
  arr scoreMeans, scoreSDVs, scoreTrains, lambdas;
  
  virtual void  train(const arr& X, const arr& y, double lambda) = 0;
  virtual double test(const arr& X, const arr& y) = 0;
  
  void crossValidate(const arr& X, const arr& y, double lambda, uint k_fold, bool permute, double *scoreMean=NULL, double *scoreSDV=NULL, double *scoreTrain=NULL);
  void crossValidate(const arr& X, const arr& y, const arr& lambdas, uint k_fold, bool permute);
  void plot();
};


//===========================================================================
//
// constructing features from data
//

enum FeatureType { readFromCfgFileFT=0, linearFT=1, quadraticFT, cubicFT, rbfFT=4, piecewiseConstantFT=5, piecewiseLinearFT=6 };
void makeFeatures(arr& Phi, const arr& X, const arr& Xtrain, FeatureType featureType=readFromCfgFileFT);


//===========================================================================
//
// artificial test data & data load routines
//

extern arr beta_true;
enum ArtificialDataType { readFromCfgFileDT=0, linearData, sinusData, linearOutlier };

void artificialData(arr& X, arr& y, ArtificialDataType dataType=readFromCfgFileDT);
void artificialData_Hasties2Class(arr& X, arr& y);
void artificialData_HastiesMultiClass(arr& X, arr& y);
void artificialData_GaussianMixture(arr& X, arr& y);
void load_data(arr& X, const char* filename, bool whiten);


//===========================================================================
//
// helper
//

double NormalSdv(const double& a, const double& b, double sdv);

#ifdef MT_IMPLEMENTATION
#  include "MLcourse.cpp"
#endif

#endif
