#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <iomanip>


//#include "/usr/local/MATLAB/R2015a/extern/include/engine.h"
#include "/usr/local/MATLAB/R2016a/extern/include/engine.h"
#define  BUFSIZE 512

/// documentation:
/// http://de.mathworks.com/help/matlab/calling-matlab-engine-from-c-c-and-fortran-programs.html

void TEST(Matlab) {
  Engine *ep;
  mxArray *A_mat = NULL, *B_mat = NULL;
  char buffer[BUFSIZE+1];

  /// Call engOpen with a NULL string. This starts a MATLAB process
  if (!(ep = engOpen(""))) {
    fprintf(stderr, "\nCan't start MATLAB engine\n");
  }

  buffer[BUFSIZE] = '\0';
  engOutputBuffer(ep, buffer, BUFSIZE);

  arr A_ors = randn(3,5);
  cout << "A" << A_ors << endl;

  /// copy matrix A to matlab (NOTE: in MATLAB columns and rows are switched)
  A_mat = mxCreateDoubleMatrix(A_ors.d1, A_ors.d0, mxREAL);
  memcpy((void *)mxGetPr(A_mat), (void *)A_ors.p, A_ors.d0*A_ors.d1*sizeof(double));
  engPutVariable(ep, "A", A_mat);

  /// do some MATLAB operations
  engEvalString(ep, "A=A + 2.");
  printf("%s", buffer);

  /// copy matrix A back to C++
  B_mat = engGetVariable(ep,"A");
  arr B_ors;
  B_ors.resize(mxGetN(B_mat),mxGetM(B_mat));
  memcpy((void *)B_ors.p,(void *)mxGetPr(B_mat), mxGetN(B_mat)*mxGetM(B_mat)*sizeof(double));
  cout << "B: " << B_ors << endl;

  engClose(ep);
}

void TEST(MatlabScript) {
  Engine *ep;
  char buffer[BUFSIZE+1];

  if (!(ep = engOpen(""))) {
    fprintf(stderr, "\nCan't start MATLAB engine\n");
  }

  buffer[BUFSIZE] = '\0';
  engOutputBuffer(ep, buffer, BUFSIZE);

  engEvalString(ep, "testScript");
  mlr::wait();

  engClose(ep);
}

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);
  testMatlab();
  testMatlabScript();
  return 0;
}
