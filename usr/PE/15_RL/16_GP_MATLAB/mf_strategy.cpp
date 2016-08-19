#include "mf_strategy.h"

MF_strategy::MF_strategy(uint nParam_,arr &paramLim_):nParam(nParam_),paramLim(paramLim_)
{
  if (!(ep = engOpen(""))) {
    fprintf(stderr, "\nCan't start MATLAB engine\n");
  }
  buffer[BUFSIZE] = '\0';
  engOutputBuffer(ep, buffer, BUFSIZE);

  mxArray *nParam_M = mxCreateDoubleScalar(nParam);
  engPutVariable(ep, "nParam", nParam_M);

  mxArray *paramLim_M = mxCreateDoubleMatrix(paramLim.d1, paramLim.d0, mxREAL);
  memcpy((void *)mxGetPr(paramLim_M), (void *)paramLim.p, paramLim.d0*paramLim.d1*sizeof(double));
  engPutVariable(ep, "paramLim", paramLim_M);

  engEvalString(ep, "run matlab_interface/init_interface");
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

void MF_strategy::evaluate(arr &x_n)
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
  engEvalString(ep, "run matlab_interface/evaluate");


  // get next datapoint from matlab
  x_n.resize(nParam);
  engEvalString(ep, "x_n=x_n'");
  mxArray *result = engGetVariable(ep,"x_n");
  x_n.p = mxGetPr(result);

  printf("%s", buffer);
}
