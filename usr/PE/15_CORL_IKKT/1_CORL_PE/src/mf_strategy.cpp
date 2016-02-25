#include "mf_strategy.h"

MF_strategy::MF_strategy(uint nParam_,arr &pLimit_,mlr::String folder,mlr::String taskName):nParam(nParam_),pLimit(pLimit_) {
  if (!(ep = engOpen(""))) {
    fprintf(stderr, "\nCan't start MATLAB engine\n");
  }
  buffer[BUFSIZE] = '\0';
  engOutputBuffer(ep, buffer, BUFSIZE);

  /// init optimization options
  engEvalString(ep, "addpath('matlab/gpml/');");
  engEvalString(ep, "addpath('matlab/util/');");
  engEvalString(ep, "addpath('matlab/');");
  engEvalString(ep, "startup;");
  engEvalString(ep, "seed = 1");
  engEvalString(ep, "e=0;");
  engEvalString(ep, "bOffset = 0.1;");
  engEvalString(ep, "verbose = 1;");
  engEvalString(ep, "optM = 2;");

  engEvalString(ep, STRING("ellC = "<<mlr::getParameter<double>("ellC")<<";"));
  engEvalString(ep, STRING("sfC = "<<mlr::getParameter<double>("sfC")<<";"));
  engEvalString(ep, STRING("ellR = "<<mlr::getParameter<double>("ellR")<<";"));
  engEvalString(ep, STRING("sfR = "<<mlr::getParameter<double>("sfR")<<";"));
  engEvalString(ep, STRING("snR = "<<mlr::getParameter<double>("snR")<<";"));
  engEvalString(ep, STRING("n = "<<nParam<<";"));
  engEvalString(ep, "m = 200;");

  switch (nParam) {
    case 1:
      engEvalString(ep, STRING("t = ndgrid(linspace("<<pLimit(0,0)<<","<<pLimit(0,1)<<",m));"));
      break;
    case 2:
      engEvalString(ep, STRING("[t1 t2] = ndgrid(linspace("<<pLimit(0,0)<<","<<pLimit(0,1)<<",m),linspace("<<pLimit(1,0)<<","<<pLimit(1,1)<<",m));"));
      engEvalString(ep, "t = [t1(:) t2(:)];");
      break;
    case 3:
      engEvalString(ep, STRING("[t1 t2 t3] = ndgrid(linspace("<<pLimit(0,0)<<","<<pLimit(0,1)<<",m),linspace("<<pLimit(1,0)<<","<<pLimit(1,1)<<",m),linspace("<<pLimit(2,0)<<","<<pLimit(2,1)<<",m));"));
      engEvalString(ep, "t = [t1(:) t2(:) t3(:)];");
      break;
    default:
      NIY;
      break;
  }

  engEvalString(ep, "cbo = conBOpt(n,t,e,bOffset,verbose,optM,ellC,sfC,ellR,sfR,snR)");
  write(LIST<arr>(pLimit),STRING(folder<<"Limit.dat"));
  printf("%s", buffer);
}

MF_strategy::~MF_strategy(){
  engClose(ep);
}

void MF_strategy::addDatapoint(arr x, arr y, arr ys){
  sendArrToMatlab(x,"x");
  sendArrToMatlab(y,"y");
  sendArrToMatlab(ys,"ys");
  engEvalString(ep, "cbo.addDataPoint(x,y,ys);");
  if (X.d0>0) engEvalString(ep, "cbo.plot();");
  engEvalString(ep, "cbo.stats()");
  cout << buffer << endl;

  X.append(~x);
  Y.append(~y);
  YS.append(~ys);
}

void MF_strategy::evaluate(arr &x){
  engEvalString(ep, "x = cbo.selectNextPoint();");
  getArrFromMatlab(x,"x");
  x.flatten();
}

void MF_strategy::load(mlr::String folder){
  X << FILE(STRING(folder<<"X.dat"));
  Y << FILE(STRING(folder<<"Y.dat"));
  YS << FILE(STRING(folder<<"YS.dat"));
  sendArrToMatlab(X,"cbo.X");
  sendArrToMatlab(Y,"cbo.Y");
  sendArrToMatlab(YS,"cbo.YS");
  engEvalString(ep, "cbo.stats()");
  cout << buffer << endl;
}

void MF_strategy::save(mlr::String folder) {
  write(LIST<arr>(X),STRING(folder<<"X.dat"));
  write(LIST<arr>(Y),STRING(folder<<"Y.dat"));
  write(LIST<arr>(YS),STRING(folder<<"YS.dat"));
  engEvalString(ep, STRING("save('"<<folder<<"conBOpt.mat');"));
}

void MF_strategy::sendArrToMatlab(arr& x, mlr::String name) {
  engEvalString(ep, STRING(std::setprecision(16)<<name<<"=["<<x<<"];"));
}

void MF_strategy::getArrFromMatlab(arr& x, mlr::String name) {
  x.clear();
  mxArray *x_mat = engGetVariable(ep,name);
  x.resize(mxGetN(x_mat),mxGetM(x_mat));
  memcpy((void *)x.p,(void *)mxGetPr(x_mat), mxGetN(x_mat)*mxGetM(x_mat)*sizeof(double));
  x=~x;
}
