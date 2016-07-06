#include "conbopt.h"

ConBOpt::ConBOpt(uint nParam_,arr &pLimit_,mlr::String folder_,mlr::String name_):nParam(nParam_),pLimit(pLimit_),folder(folder_),name(name_) {
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
  engEvalString(ep, "bOffset = 0.15;");
  engEvalString(ep, "verbose = 1;");
  engEvalString(ep, "optM = 2;");

  engEvalString(ep, STRING("ellC = "<<mlr::getParameter<double>(STRING(name<<"/ellC"))<<";"));
  engEvalString(ep, STRING("sfC = "<<mlr::getParameter<double>(STRING(name<<"/sfC"))<<";"));
  engEvalString(ep, STRING("ellR = "<<mlr::getParameter<double>(STRING(name<<"/ellR"))<<";"));
  engEvalString(ep, STRING("sfR = "<<mlr::getParameter<double>(STRING(name<<"/sfR"))<<";"));
  engEvalString(ep, STRING("snR = "<<mlr::getParameter<double>(STRING(name<<"/snR"))<<";"));
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
  x0 = zeros(nParam);

  engEvalString(ep, "cbo = conBOpt(n,t,e,bOffset,verbose,optM,ellC,sfC,ellR,sfR,snR)");
  write(LIST<arr>(pLimit),STRING(folder<<"Limit.dat"));
  printf("%s", buffer);
}

ConBOpt::~ConBOpt(){
  engClose(ep);
}

void ConBOpt::addDatapoint(arr x, arr y, arr ys){
  if (ys(0)==0) ys = ARR(-1);
  sendArrToMatlab(x,"x");
  sendArrToMatlab(y,"y");
  sendArrToMatlab(ys,"ys");
  cout << buffer << endl;
  engEvalString(ep, "cbo.addDataPoint(x,y,ys);");
  if (X.d0>0) engEvalString(ep, "cbo.plot();");
  engEvalString(ep, "cbo.stats()");
  cout << buffer << endl;

  X.append(~x);
  Y.append(~y);
  YS.append(~ys);
}

void ConBOpt::evaluate(arr &x) {
  if (X.N==0) {x = x0; return;}
  engEvalString(ep, "x = cbo.selectNextPoint();");
  getArrFromMatlab(x,"x");
  x.flatten();
}

void ConBOpt::load(int id) {
  X << FILE(STRING(folder<<"/"<<id<<"_"<<name<<"_CBO_X.dat"));
  Y << FILE(STRING(folder<<"/"<<id<<"_"<<name<<"_CBO_Y.dat"));
  YS << FILE(STRING(folder<<"/"<<id<<"_"<<name<<"_CBO_YS.dat"));
//  sendArrToMatlab(X,"cbo.X");
//  sendArrToMatlab(Y,"cbo.Y");
//  sendArrToMatlab(YS,"cbo.YS");
  engEvalString(ep, STRING("load('"<<folder<<"/"<<id<<"_"<<name<<"_CBO.mat');"));
//  engEvalString(ep, STRING("cbo=c;"));

  engEvalString(ep, "cbo.stats()");
  engEvalString(ep, "cbo");
  cout << buffer << endl;
}

void ConBOpt::save(int id) {
  write(LIST<arr>(X),STRING(folder<<"/"<<id<<"_"<<name<<"_CBO_X.dat"));
  write(LIST<arr>(Y),STRING(folder<<"/"<<id<<"_"<<name<<"_CBO_Y.dat"));
  write(LIST<arr>(YS),STRING(folder<<"/"<<id<<"_"<<name<<"_CBO_YS.dat"));
  engEvalString(ep, STRING("save('"<<folder<<"/"<<id<<"_"<<name<<"_CBO.mat');"));

  // save max index
  arr Ytmp = Y;
  Ytmp += -fabs(Y%(YS-1.))*1e5;
  uint i = Ytmp.maxIndex();
  CHECK(YS(i,0)==1,"");
  i =i+1; //< file indices start with 1
  write(LIST<arr>(ARR(i)),STRING(folder<<"/bestIdx_"<<name<<"_CBO.dat"));
}

void ConBOpt::sendArrToMatlab(arr& x, mlr::String name) {
  engEvalString(ep, STRING(std::setprecision(16)<<name<<"=["<<x<<"];"));
}

void ConBOpt::getArrFromMatlab(arr& x, mlr::String name) {
  x.clear();
  mxArray *x_mat = engGetVariable(ep,name);
  x.resize(mxGetN(x_mat),mxGetM(x_mat));
  memcpy((void *)x.p,(void *)mxGetPr(x_mat), mxGetN(x_mat)*mxGetM(x_mat)*sizeof(double));
  x=~x;
}
