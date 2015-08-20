#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Perception/videoEncoder.h>
#include <iomanip>


#include "/usr/local/MATLAB/R2015a/extern/include/engine.h"
#define  BUFSIZE 512

void TEST(Matrices) {
  ors::KinematicWorld G("chain.ors");
  MotionProblem MP(G);

  arr y_fL, J_fL;
  MP.world.kinematicsPos(y_fL, J_fL, MP.world.getShapeByName("endeff")->body);
  MP.world.getBodyByName("target")->X.rot.setRandom();
  arr dir = {2.,1.,0.};
  ors::Quaternion quat;
  quat.setDiff(ors::Vector(1.,0.,0.),dir);
  arr R = ~quat.getArr();
  cout << "R: " << R << endl;
  arr J_fL0 = R*J_fL;
  J_fL0[0]=0.;

  arr JJ = ~J_fL*inverse(J_fL * ~J_fL)*J_fL0 ;
  arr dq;
  G.getJointState(dq);
  dq = 0.*dq+0.1;
  arr dy = ~R*J_fL*JJ*dq;
  cout << JJ*dq << endl;
  cout << "dy: " << dy << endl;

//  cout << ~dy*R[0] << endl;
//  cout << ~dy*R[1] << endl;
//  cout << ~dy*R[2] << endl;

  cout << R<< endl;
  R.reverseRows();
  cout << R << endl;
  cout << R.col(2).max() << endl;
}

void TEST(GradCheck) {
  ors::KinematicWorld G("chain.ors");
  MotionProblem MP(G);

  arr y_fL, J_fL;
  MP.world.kinematicsPos(y_fL, J_fL, MP.world.getShapeByName("endeff")->body);
  MP.world.getBodyByName("target")->X.rot.setRandom();
  arr dir = {2.,1.,0.};
  ors::Quaternion quat;
  quat.setDiff(ors::Vector(1.,0.,0.),dir);
  arr R = ~quat.getArr();
  cout << "R: " << R << endl;
  arr J_fL0 = R*J_fL;
  J_fL0[0]=0.;



  arr JJ = ~J_fL*inverse(J_fL * ~J_fL)*J_fL0 ;
  arr dq;
  G.getJointState(dq);
  dq = 0.*dq+0.1;
  arr dy = ~R*J_fL*JJ*dq;
  cout << JJ*dq << endl;
  cout << "dy: " << dy << endl;

  cout << ~dy*R[0] << endl;
  cout << ~dy*R[1] << endl;
  cout << ~dy*R[2] << endl;
}


//  arr phi, grad;
//  TermTypeA tt;
//  ConstrainedProblemMix v = Convert(MPF);
//  ConstrainedProblemMix UCP = Convert(MPF);
//  UCP(phi, grad, tt, x);
//  grad = unpack(grad);
//  cout << ~phi*phi << endl;
//  cout << "phi" << phi << endl;
//  cout << "grad " << grad << endl;
//  cout << ~grad*grad << endl;
//  cout << "c " << 4.*~phi*grad*~grad*phi << endl;


// std::ofstream fil;
//   MT::open(fil, filename);
// fil.precision(30);
//   catCol(X).write(fil, ELEMSEP, LINESEP, BRACKETS, dimTag, binary);
// fil.close();

void TEST(Matlab) {
  Engine *ep;
  mxArray *T = NULL, *result = NULL;
  char buffer[BUFSIZE+1];
  double time[10] = { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0 };

  /*
           * Call engOpen with a NULL string. This starts a MATLAB process
       * on the current host using the command "matlab".
           */
  if (!(ep = engOpen(""))) {
    fprintf(stderr, "\nCan't start MATLAB engine\n");
  }

  arr A = randn(3,5);
  cout << A << endl;

  uint nRow = A.d0;
  uint nCol = A.d1;

//  T = mxCreateDoubleMatrix(nRow, nCol, mxREAL);
//  memcpy((void *)mxGetPr(T), (void *)time, sizeof(time));

//  double *xValues = mxGetPr(T);

//  for(uint row = 0; row < nRow; row++) {
//    for(uint col = 0; col < nCol; col++) {
//      xValues[nRow * col + row] = A(row,col);
//    }
//  }
  T = mxCreateDoubleMatrix(nCol, nRow, mxREAL);
  memcpy((void *)mxGetPr(T), (void *)A.p, A.d0*A.d1*sizeof(double));
  engPutVariable(ep, "T", T);
  engEvalString(ep, "T=T'");

  buffer[BUFSIZE] = '\0';
  engOutputBuffer(ep, buffer, BUFSIZE);
  engEvalString(ep, "testMC");
  printf("Hit return to continue\n\n");
  fgetc(stdin);

  printf("%s", buffer);

  arr B;
  B.resizeAs(A);
  engEvalString(ep, "T=T'");
  result = engGetVariable(ep,"T");
  B.p = mxGetPr(result);


  cout << B << endl;

//  mxSetPr(T,A.p);
  /*
  engPutVariable(ep, "T", T);

  buffer[BUFSIZE] = '\0';
  engOutputBuffer(ep, buffer, BUFSIZE);

  engEvalString(ep, "testMC");
  engEvalString(ep, "T=T'");

  result = engGetVariable(ep,"T");
  double *rValues = mxGetPr(result);

  arr B;
  B.resizeAs(A);

  B.p = mxGetPr(result);
  /*
  B.resizeAs(A);
  for (uint i =0;i<B.d0;i++){
    for (uint j =0;j<B.d1;j++){
      B(i,j) = rValues[j*B.d0+i];
    }
  }*/
//  cout << B << endl;
  /*

  for(uint row = 0; row < nRow; row++) {
      for(uint col = 0; col < nCol; col++) {

          printf("%lf", xValues[nRow * col + row]);
      }
      printf("\n");
  }*/

//  printf("%s", buffer);
}

void TEST(MatlabGP) {
  Engine *ep;
  mxArray *T = NULL, *result = NULL;
  char buffer[BUFSIZE+1];

  if (!(ep = engOpen(""))) {
    fprintf(stderr, "\nCan't start MATLAB engine\n");
  }


  buffer[BUFSIZE] = '\0';
  engOutputBuffer(ep, buffer, BUFSIZE);

  engEvalString(ep, "testMC");
  MT::wait(10.);

  printf("%s", buffer);
}

void TEST(Transformation) {
  ors::KinematicWorld world("trans.ors");
  world.gl().resize(800,800);


  ors::Transformation marker = world.getJointByName("tm")->B;
  cout << world.getJointByName("tm")->B.pos << endl;
  cout << world.getJointByName("tm")->B.rot << endl;
  cout << marker << endl;

  ors::Body *trans1 = world.getBodyByName("trans1");
  ors::Body *trans2 = world.getBodyByName("trans2");

  arr m = trans2->X.rot.getArr();

//  ors::Vector v = trans2->X.pos + trans2->X.rot*marker.pos;
//  ors::Quaternion r = trans2->X.rot*marker.rot;

//  world.getBodyByName("marker2")->X.pos = v;
//  world.getBodyByName("marker2")->X.rot = r;
//  world.getBodyByName("marker2")->X = trans2->X*marker;

  ors::Transformation T;
  T.setZero();
  T.addRelativeRotationDeg(90.,0.,1.,0.);
  world.getBodyByName("marker2")->X = trans1->X*T*marker;

  world.calc_fwdPropagateFrames();

  world.watch(true);
}

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);
//  testMatrices();
//  testGradCheck();
  testMatlab();
//  testMatlabGP();
//  testTransformation();
  return 0;
}

//    cout << "Enter result:  success [1] or failure [0]: "<<endl;
//    std::cin >> result;

