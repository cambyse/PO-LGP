#include <Core/util.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>

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

  ors::KinematicWorld W("rawbaxter.ors");

  filename <<"_Faugmented";
  ofstream fil(filename);
  cout <<"output = '" <<filename <<"'" <<endl;

  for(uint i=0;i<D.d0;i++){
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

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  mlr::String filename="../data/dataTraining_100x10.txt_Faugmented";
  if(mlr::argc>1) filename=mlr::argv[1];
  cout <<"Processing file '" <<filename <<"'" <<endl;

  //================================================================

  printJointIndices();
//  augmentDataWithF(filename);
//  display(filename);

  return 0;
}
