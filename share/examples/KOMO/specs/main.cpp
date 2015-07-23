#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Motion/komo.h>

//===========================================================================

void TEST(Specs){
  const char* specsfile="specs.g";
  if(MT::argc>1) specsfile=MT::argv[1];

  Graph specs(specsfile);
  KOMO komo(specs);

  for(;;){
    komo.init(Graph(specsfile));
    komo.run();
    komo.displayTrajectory();
  }
}

//===========================================================================

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  cout <<MT::String(FILE("USAGE"));

  testSpecs();

  return 0;
}




