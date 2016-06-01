#include <Core/util.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>

// =================================================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  mlr::String filename="../data/dataTraining_100x10.txt";
  if(argc>1) filename=argv[1];
  cout <<"Processing file '" <<filename <<"'" <<endl;

  arr D = FILE(filename);
//  cout <<D;

  ors::KinematicWorld W("rawbaxter.ors");

  filename <<"_Faugmented";
  ofstream fil(filename);
  cout <<"output = '" <<filename <<"'" <<endl;

  for(uint i=0;i<D.d0;i++){
    if(!(i%10)) cout <<i <<endl;
    const arr& Di = D[i];
    arr q = Di.subRef(0,16);
    arr u = Di.subRef(17,-1);
    W.setJointState(q);
    W.gl().update();
    arr M,F;
    W.equationOfMotion(M, F);

    fil <<q <<' ' <<F <<' ' <<u <<endl;
  }
  fil.close();

  return 0;
}
