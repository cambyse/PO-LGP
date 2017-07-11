#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <iomanip>


#include "/usr/local/MATLAB/R2015a/extern/include/engine.h"
#define  BUFSIZE 1000

/// documentation:
/// http://de.mathworks.com/help/matlab/calling-matlab-engine-from-c-c-and-fortran-programs.html

arr reward(arr &x) {
//  return ARR(1.-exp(-x(0)*x(0))); // prob1.m
  return ARR(exp(-x(0)*x(0) - x(1)*x(1))); // prob2.m
}

arr success(arr &x) {
//  if (x(0)<0.8 && x(0)>-0.6) { // prob1.m
  if (fabs(x(0))<3. && fabs(x(1))<4.) { // prob2.m;
    return ARR(1.);
  } else {
    return ARR(-1.);
  }
}

void sendArrToMatlab(Engine *e, arr &x, mlr::String name) {
  engEvalString(e, STRING(std::setprecision(16)<<name<<"=["<<x<<"];"));
}

void getArrFromMatlab(Engine *e, arr &x, mlr::String name) {
  x.clear();
  mxArray *x_mat = engGetVariable(e,name);
  x.resize(mxGetN(x_mat),mxGetM(x_mat));
  memcpy((void *)x.p,(void *)mxGetPr(x_mat), mxGetN(x_mat)*mxGetM(x_mat)*sizeof(double));
  x=~x;
}

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  /// init matlab engine
  Engine *ep;
  char buffer[BUFSIZE+1];
  if (!(ep = engOpen(""))) {
    fprintf(stderr, "\nCan't start MATLAB engine\n");
  }
  buffer[BUFSIZE] = '\0';
  engOutputBuffer(ep, buffer, BUFSIZE);


  /// init optimization options
  engEvalString(ep, "addpath('../matlab/gpml/');");
  engEvalString(ep, "addpath('../matlab/util/');");
  engEvalString(ep, "addpath('../matlab/');");
  engEvalString(ep, "startup;");
  engEvalString(ep, "seed = 1");
  engEvalString(ep, "e=0;");
  engEvalString(ep, "bOffset = 0.1;");
  engEvalString(ep, "verbose = 1;");
  engEvalString(ep, "optM = 2;");
  engEvalString(ep, "ellC = 0.7; sfC = 1e1; ellR = 0.7; sfR = 1e0; snR = 0.11;");
  uint nIter = 100;


  /// init algorithm
  arr x = ARR(2.,2.); // start point
  arr y = reward(x);
  arr ys = success(x);
  sendArrToMatlab(ep,x,"x");
  sendArrToMatlab(ep,y,"y");
  sendArrToMatlab(ep,ys,"ys");
  engEvalString(ep, "n = length(x);");
  engEvalString(ep, "m = 200;");
  engEvalString(ep, "[t1 t2] = ndgrid(linspace(-5,5,m),linspace(-5,5,m));");
  engEvalString(ep, "t = [t1(:) t2(:)];");
  engEvalString(ep, "cbo = conBOpt(n,t,e,bOffset,verbose,optM,ellC,sfC,ellR,sfR,snR)");

  engEvalString(ep, "cbo.addDataPoint(x,y,ys);");

  /// run algorithm
  for (uint i=0;i<nIter;i++) {
    engEvalString(ep, "x = cbo.selectNextPoint();");
    getArrFromMatlab(ep,x,"x");
    x.flatten();
    arr y = reward(x);
    arr ys = success(x);

    sendArrToMatlab(ep,x,"x");
    sendArrToMatlab(ep,y,"y");
    sendArrToMatlab(ep,ys,"ys");
    engEvalString(ep, "cbo.addDataPoint(x,y,ys);");
    engEvalString(ep, "cbo.plot();");
    engEvalString(ep, "cbo.stats()");
    cout << buffer << endl;
  }


  /// show results
  engEvalString(ep, "cbo.verbose = 1;");
  engEvalString(ep, "cbo.stats();");
  engEvalString(ep, "cbo.plot();");

  engClose(ep);

  return 0;
}
