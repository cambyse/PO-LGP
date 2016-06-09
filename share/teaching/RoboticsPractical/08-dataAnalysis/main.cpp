#include <Core/util.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Algo/MLcourse.h>

// =================================================================================================

void printJointIndices(){
  ors::KinematicWorld W("rawbaxter.ors");
  for(ors::Joint *j:W.joints){
    cout <<j->qIndex <<' ' <<j->qDim() <<' ' <<j->name <<endl;
  }
}

// =================================================================================================

void agumentDataWithF(mlr::String filename){
  arr D = FILE(filename);
  uint n=D.d0;
  CHECK_EQ(D.d1, 2*17,"");

  ors::KinematicWorld W("rawbaxter.ors");

  filename <<"_Faugmented";
  ofstream fil(filename);
  cout <<"output = '" <<filename <<"'" <<endl;

  for(uint i=0;i<n;i++){
    if(!(i%10)) cout <<i <<endl;
    const arr& Di = D[i];
    arr q = Di.refRange(0,16);
    arr u = Di.refRange(17,-1);
    W.setJointState(q);
    W.gl().update();
    arr M,F;
    W.equationOfMotion(M, F);

    fil <<q <<' ' <<F <<' ' <<u <<endl;
  }
  fil.close();
}

// =================================================================================================

void display(const char* filename){
  arr D = FILE(filename);
  uint n=D.d0;
  CHECK_EQ(D.d1, 3*17,"");

  uintA cols={4u,6,8,10,12,14};
  arr X = D.sub(0,-1,cols);
  arr F = D.sub(0,-1,cols+17u);
  arr Y = D.sub(0,-1,cols+34u);

  arr Xp,v,W;
  pca(Xp, v, W, X, 1);

  for(uint i=0;i<n;i++){
    cout <<Xp[i] <<' ' <<F[i] <<' ' <<Y[i] <<endl;
  }
}

// =================================================================================================

void fitModel(const char* filename){
  arr D = FILE(filename);
  CHECK_EQ(D.d1, 3*17,"");

//  D.reshape(10, D.d0/10, D.d1);
  D.reshape(150, 10, 51);
  D = sum(D, 1) / 10.;
  D.reshape(150, 51);

  uintA cols={4u,6,8,10,12,14};
  arr X = D.sub(0,-1,cols);
  arr F = D.sub(0,-1,cols+17u);
  arr Y = D.sub(0,-1,cols+34u);
  X = catCol(X,F);

  arr Phi0 = makeFeatures(X, constFT);
  RidgeRegression R0(Phi0, Y, 0.);
  arr s0=sqrt( R0.getMultiOutputSquaredErrors(Phi0, Y) );

  arr Phi1 = makeFeatures(X, quadraticFT);
  RidgeRegression R1(Phi1, Y, 1e-4);
  arr s1=sqrt( R1.getMultiOutputSquaredErrors(Phi1, Y) );

  cout <<"\nRelative Errors: " <<s1/s0 <<endl;

  cout <<"Output Sdv = " <<sqrt(R0.sigmaSqr) <<endl;
  cout <<"Relative error (sdv) = " <<sqrt(R1.sigmaSqr)/sqrt(R0.sigmaSqr) <<endl;

//  return;
  cout <<"----- CV -------" <<endl;
  //-- CV
  struct myCV:CrossValidation {
    virtual void  train(const arr& X, const arr& y, double lambda, arr& beta){
      RidgeRegression R(X, y, lambda, NoArr, 0);
      beta = R.beta;
    }

    virtual double test(const arr& X, const arr& y, const arr& beta){
      return ( sumOfSqr(X*beta-y)/y.N );
    }
  } CV;

  CV.crossValidateMultipleLambdas(Phi1, Y, {1., 1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6}, 10, true);
  CV.plot();

}

// =================================================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  mlr::String filename="../data/dataTrainingNew.txt_Faugmented";
  if(mlr::argc>1) filename=mlr::argv[1];
  cout <<"Processing file '" <<filename <<"'" <<endl;

  //================================================================

//  printJointIndices();
//  agumentDataWithF(filename);
//  display(filename);
  fitModel(filename);

  return 0;
}
