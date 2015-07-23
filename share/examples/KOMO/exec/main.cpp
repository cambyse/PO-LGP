#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Motion/komo.h>

//===========================================================================

void TEST(Specs){
  const char* specsfile="specs.g";
  if(MT::argc>1) specsfile=MT::argv[1];

  Graph specs(specsfile);
  KOMO komo(specs);

  int repeats=specs.get<double>("repeats", -1.);
  for(int r=0;repeats<0. || r<repeats; r++){
    komo.init(Graph(specsfile));
    komo.run();
    FILE("z.output.g") <<komo.getReport();
    komo.displayTrajectory(); //repeats<0.);
  }
}

//===========================================================================

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  cout <<MT::String(FILE("USAGE"));

  testSpecs();

  return 0;
}




