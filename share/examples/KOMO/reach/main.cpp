#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Motion/komo.h>


//===========================================================================

void TEST(Specs){
  KOMO komo(Graph("specs.g"));

  for(;;){
    komo.init(Graph("specs.g"));
    komo.run();
    for(uint i=0;i<2;i++) komo.displayTrajectory();
  }
}

//===========================================================================

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  testSpecs();

  return 0;
}




