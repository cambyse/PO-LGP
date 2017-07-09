#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <KOMO/komo.h>


//===========================================================================

void TEST(Specs){
  KOMO komo(Graph("specs.g"));

  for(;;){
    komo.init(Graph("specs.g"));
    komo.run();
    komo.checkGradients();
    for(uint i=0;i<2;i++){
      komo.displayTrajectory();
      // arr y;
      // komo.MP->tasks.last()->map.phi(y, NoArr, komo.world, komo.MP->tau);
      komo.world.reportProxies();
      komo.world.gl().watch();
    }
  }
}

//===========================================================================

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);

  testSpecs();

  return 0;
}




