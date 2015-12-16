#include "mf_strategy.h"

MF_strategy::MF_strategy(uint nParam_,arr &paramLimit_,mlr::String folder,mlr::String taskName):nParam(nParam_),paramLimit(paramLimit_)
{
  if (!(ep = engOpen(""))) {
    fprintf(stderr, "\nCan't start MATLAB engine\n");
  }
  buffer[BUFSIZE] = '\0';
  engOutputBuffer(ep, buffer, BUFSIZE);

  mxArray *nParam_M = mxCreateDoubleScalar(nParam);
  engPutVariable(ep, "nParam", nParam_M);

  mxArray *paramLim_M = mxCreateDoubleMatrix(paramLimit.d1, paramLimit.d0, mxREAL);
  memcpy((void *)mxGetPr(paramLim_M), (void *)paramLimit.p, paramLimit.d0*paramLimit.d1*sizeof(double));
  engPutVariable(ep, "paramLimit", paramLim_M);
  engEvalString(ep, "paramLimit = paramLimit'");

  write(LIST<arr>(paramLimit),STRING(folder<<"Limit.dat"));

  engEvalString(ep,STRING("run matlab_interface/" << taskName << "Defs.m"));
  printf("%s", buffer);
}

MF_strategy::~MF_strategy()
{
  engClose(ep);
}

void MF_strategy::addDatapoint(arr x, arr y, arr ys)
{
  X.append(~x);
  Y.append(~y);
  YS.append(~ys);
}

void MF_strategy::evaluate(arr &x_exp)
{
  // send datapoints to matlab
  mxArray *X_M = mxCreateDoubleMatrix(X.d1, X.d0, mxREAL);
  memcpy((void *)mxGetPr(X_M), (void *)X.p, X.d0*X.d1*sizeof(double));
  engPutVariable(ep, "X", X_M); engEvalString(ep, "X=X'");

  mxArray *Y_M = mxCreateDoubleMatrix(Y.d1, Y.d0, mxREAL);
  memcpy((void *)mxGetPr(Y_M), (void *)Y.p, Y.d0*Y.d1*sizeof(double));
  engPutVariable(ep, "Y", Y_M); engEvalString(ep, "Y=Y'");

  mxArray *YS_M = mxCreateDoubleMatrix(YS.d1, YS.d0, mxREAL);
  memcpy((void *)mxGetPr(YS_M), (void *)YS.p, YS.d0*YS.d1*sizeof(double));
  engPutVariable(ep, "YS", YS_M); engEvalString(ep, "YS=YS'");

  mxDestroyArray(X_M);
  mxDestroyArray(Y_M);
  mxDestroyArray(YS_M);

  // evaluate gp for next datapoint
  engEvalString(ep, "run matlab_interface/constraintExploration.m");
  engEvalString(ep, "run matlab_interface/constraintExplorationPlot.m");

  // get next datapoint from matlab
  x_exp.resize(nParam);
  mxArray *result = engGetVariable(ep,"x_exp");
  x_exp.p = mxGetPr(result);

  printf("%s", buffer);
}

void MF_strategy::load(mlr::String folder)
{
  X << FILE(STRING(folder<<"X.dat"));
  Y << FILE(STRING(folder<<"Y.dat"));
  YS << FILE(STRING(folder<<"YS.dat"));
}


void MF_strategy::save(mlr::String folder)
{
  write(LIST<arr>(X),STRING(folder<<"X.dat"));
  write(LIST<arr>(Y),STRING(folder<<"Y.dat"));
  write(LIST<arr>(YS),STRING(folder<<"YS.dat"));
}
